// Retained schedule slots, fired locally off the SNTP clock. The element label
// in the topic is ignored (single relay).
// Topic format and payload fields: see FIRMWARE_B_REFERENCE.md §8.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "cJSON.h"
#include "scheduler.h"
#include "mqtt_app_handler.h" // replay commands through the normal handler

#define MAX_REPEATED 5
#define MAX_ONCE 3
#define SCHED_CMD_LEN 384 // stored command[] array JSON
#define ONCE_STALE_S 60   // a once-moment older than this when first seen is not fired

static const char *TAG = "SCHEDULER";

typedef struct {
    bool active;
    // repeated fields
    uint8_t days_mask;   // bit d set for ISO day d (1=Mon..7=Sun)
    int start_h, start_m;
    bool has_end;
    int end_h, end_m;    // local HH:MM
    int last_start_yday; // yday the START last fired (-1 = never)
    int last_end_yday;   // yday the END last fired
    // once fields
    time_t fires_at;     // epoch (UTC) of the one-time start
    time_t off_at;       // epoch of the one-time off (0 if none)
    bool once_on_done;
    bool once_off_done;
    bool once_checked;   // stale check done since the slot was (re)stored
    // the command[] array, stored verbatim as JSON text
    char command[SCHED_CMD_LEN];
} sched_slot_t;

static sched_slot_t s_repeated[MAX_REPEATED];
static sched_slot_t s_once[MAX_ONCE];
static SemaphoreHandle_t s_lock;

static void scheduler_task(void *pv);

void scheduler_init(void) {
    ESP_LOGI(TAG, "Initializing Scheduler (slot-topic / retained model)...");
    memset(s_repeated, 0, sizeof(s_repeated));
    memset(s_once, 0, sizeof(s_once));
    for (int i = 0; i < MAX_REPEATED; i++) { s_repeated[i].last_start_yday = -1; s_repeated[i].last_end_yday = -1; }
    if (s_lock == NULL) s_lock = xSemaphoreCreateMutex();
    // 6 KB: the task calls process_message() at fire time, which parses JSON and
    // drives the relay GPIO path.
    xTaskCreate(scheduler_task, "scheduler_task", 6144, NULL, 5, NULL);
}

void scheduler_clear_all(void) {
    if (s_lock == NULL) return;   // scheduler not started yet - nothing stored
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_REPEATED; i++) s_repeated[i].active = false;
    for (int i = 0; i < MAX_ONCE; i++) s_once[i].active = false;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "All schedule slots cleared (retained resend will rebuild them).");
}

// --- Helpers ---------------------------------------------------------------

// Days since the Unix epoch for a Gregorian Y-M-D (Howard Hinnant's algorithm).
static long days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    long era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097L + (long)doe - 719468L;
}

// Parse an ISO-8601 UTC instant "2026-05-30T15:45:00(.sss)Z" to epoch seconds.
static time_t iso_utc_to_epoch(const char *iso) {
    int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0;
    if (sscanf(iso, "%d-%d-%dT%d:%d:%d", &Y, &Mo, &D, &H, &Mi, &S) < 5) return 0;
    return (time_t)(days_from_civil(Y, Mo, D) * 86400L + H * 3600L + Mi * 60L + S);
}

// Replay a stored command[] array through the normal command handler. When
// turn_off is true, each command's "state" is forced to "off".
static void fire_command(const char *command_json, bool turn_off) {
    cJSON *arr = cJSON_Parse(command_json);
    if (!cJSON_IsArray(arr)) { cJSON_Delete(arr); return; }
    int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n; i++) {
        cJSON *item = cJSON_GetArrayItem(arr, i);
        if (!cJSON_IsObject(item)) continue;
        if (turn_off) {
            if (cJSON_GetObjectItem(item, "state"))
                cJSON_ReplaceItemInObject(item, "state", cJSON_CreateString("off"));
            else
                cJSON_AddStringToObject(item, "state", "off");
        }
        // commandId makes the handler emit feedback too, so the app updates.
        static unsigned sched_seq = 0;
        char cid[24];
        snprintf(cid, sizeof(cid), "sched-%u", ++sched_seq);
        if (cJSON_GetObjectItem(item, "commandId"))
            cJSON_ReplaceItemInObject(item, "commandId", cJSON_CreateString(cid));
        else
            cJSON_AddStringToObject(item, "commandId", cid);

        char *s = cJSON_PrintUnformatted(item);
        if (s) {
            mqtt_app_handler_process_message(NULL, s); // topic NULL = not a schedule
            free(s);
        }
    }
    cJSON_Delete(arr);
}

// Fill a slot struct from a parsed schedule JSON object.
static void populate_slot(sched_slot_t *slot, cJSON *root, bool is_once) {
    memset(slot, 0, sizeof(*slot));
    slot->last_start_yday = -1;
    slot->last_end_yday = -1;

    // command[] — stored verbatim.
    cJSON *cmd = cJSON_GetObjectItem(root, "command");
    if (cJSON_IsArray(cmd)) {
        char *s = cJSON_PrintUnformatted(cmd);
        if (s) { strncpy(slot->command, s, SCHED_CMD_LEN - 1); free(s); }
    }

    // endTime "HH:MM" or null.
    cJSON *et = cJSON_GetObjectItem(root, "endTime");
    if (cJSON_IsString(et) && sscanf(et->valuestring, "%d:%d", &slot->end_h, &slot->end_m) == 2) {
        slot->has_end = true;
    }

    if (is_once) {
        cJSON *fa = cJSON_GetObjectItem(root, "firesAt");
        if (cJSON_IsString(fa)) slot->fires_at = iso_utc_to_epoch(fa->valuestring);
        if (slot->has_end && slot->fires_at > 0) {
            // off-moment = endTime on the local date of firesAt (next day if <= start).
            struct tm t;
            localtime_r(&slot->fires_at, &t);
            t.tm_hour = slot->end_h; t.tm_min = slot->end_m; t.tm_sec = 0;
            time_t off = mktime(&t);
            if (off <= slot->fires_at) off += 86400;
            slot->off_at = off;
        }
    } else {
        cJSON *st = cJSON_GetObjectItem(root, "startTime");
        if (cJSON_IsString(st)) sscanf(st->valuestring, "%d:%d", &slot->start_h, &slot->start_m);
        cJSON *days = cJSON_GetObjectItem(root, "days");
        if (cJSON_IsArray(days)) {
            int dn = cJSON_GetArraySize(days);
            for (int i = 0; i < dn; i++) {
                int d = cJSON_GetArrayItem(days, i)->valueint; // 1=Mon..7=Sun
                if (d >= 1 && d <= 7) slot->days_mask |= (1 << d);
            }
        }
    }
    slot->active = true;
}

// --- Incoming slot message -------------------------------------------------

void scheduler_handle_slot_message(const char *topic, const char *payload) {
    if (topic == NULL) return;
    const char *p = strstr(topic, "/schedule/");
    if (p == NULL) return;
    p += strlen("/schedule/");

    // The path after "/schedule/" is one or more '/'-separated segments. The
    // backend addresses each schedule per element:
    //     repeated: sig/{id}/schedule/{element}/{slot}
    //     once:     sig/{id}/schedule/once/{element}/{slot}
    // (the grouped forms .../schedule/{slot} and .../schedule/once/{slot} are
    // also accepted). Walk the segments: "once" selects the one-time table, a
    // bare number is the slot, and any other token (e.g. "relay1") is an element
    // label — ignored here, since this 1-gang device has a single relay.
    bool is_once = false;
    long slot = -1;
    while (*p) {
        size_t n = 0;
        while (p[n] && p[n] != '/') n++;
        char seg[24];
        if (n > 0 && n < sizeof(seg)) {
            memcpy(seg, p, n);
            seg[n] = '\0';
            if (strcmp(seg, "once") == 0) {
                is_once = true;
            } else {
                char *end = NULL;
                long v = strtol(seg, &end, 10);
                if (end != seg && *end == '\0') slot = v;
            }
        }
        p += n;
        if (*p == '/') p++;
    }

    if (slot < 0) {
        ESP_LOGW(TAG, "No slot number in schedule topic: %s", topic);
        return;
    }

    int max = is_once ? MAX_ONCE : MAX_REPEATED;
    if (slot < 1 || slot > max) {
        ESP_LOGW(TAG, "Schedule slot %ld out of range on %s", slot, topic);
        return;
    }
    sched_slot_t *dst = is_once ? &s_once[slot - 1] : &s_repeated[slot - 1];

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (payload == NULL || payload[0] == '\0') {
        // Tombstone — clear the slot.
        dst->active = false;
        ESP_LOGI(TAG, "Cleared %s slot %ld (tombstone).", is_once ? "once" : "repeated", slot);
    } else {
        cJSON *root = cJSON_Parse(payload);
        if (root) {
            populate_slot(dst, root, is_once);
            cJSON_Delete(root);
            ESP_LOGI(TAG, "Stored %s slot %ld.", is_once ? "once" : "repeated", slot);
        } else {
            ESP_LOGE(TAG, "Bad schedule JSON on %s", topic);
        }
    }
    xSemaphoreGive(s_lock);
}

// --- Firing task -----------------------------------------------------------

static void scheduler_task(void *pv) {
    bool warned_no_time = false;
    while (1) {
        time_t now;
        struct tm t;
        time(&now);
        localtime_r(&now, &t);

        if (t.tm_year < (2020 - 1900)) { // SNTP not synced yet
            if (!warned_no_time) { ESP_LOGW(TAG, "Waiting for SNTP time before running schedules..."); warned_no_time = true; }
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        warned_no_time = false;

        int iso_today = (t.tm_wday == 0) ? 7 : t.tm_wday; // ISO: Mon=1..Sun=7

        xSemaphoreTake(s_lock, portMAX_DELAY);

        // Repeated slots — weekly by days_mask at start/end HH:MM.
        for (int i = 0; i < MAX_REPEATED; i++) {
            sched_slot_t *s = &s_repeated[i];
            if (!s->active) continue;

            // START on a selected day.
            if ((s->days_mask & (1 << iso_today)) &&
                t.tm_hour == s->start_h && t.tm_min == s->start_m && s->last_start_yday != t.tm_yday) {
                ESP_LOGI(TAG, "Repeated slot %d START — applying command.", i + 1);
                fire_command(s->command, false);
                s->last_start_yday = t.tm_yday;
            }
            // END: an endTime not after startTime belongs to the NEXT day, so it is
            // checked against yesterday's day in days[] (e.g. Fri 22:00 -> Sat 06:00).
            if (s->has_end) {
                bool overnight = (s->end_h * 60 + s->end_m) <= (s->start_h * 60 + s->start_m);
                int end_day = overnight ? (iso_today == 1 ? 7 : iso_today - 1) : iso_today;
                if ((s->days_mask & (1 << end_day)) &&
                    t.tm_hour == s->end_h && t.tm_min == s->end_m && s->last_end_yday != t.tm_yday) {
                    ESP_LOGI(TAG, "Repeated slot %d END — turning off.", i + 1);
                    fire_command(s->command, true);
                    s->last_end_yday = t.tm_yday;
                }
            }
        }

        // Once slots — fire at firesAt, off at off_at, then wait for tombstone.
        for (int i = 0; i < MAX_ONCE; i++) {
            sched_slot_t *s = &s_once[i];
            if (!s->active) continue;

            // Retained slots are resent on every reconnect, which resets the done
            // flags. A moment already well in the past has fired before (or was
            // missed) - don't fire it again.
            if (!s->once_checked) {
                s->once_checked = true;
                if (s->fires_at > 0 && now > s->fires_at + ONCE_STALE_S) {
                    s->once_on_done = true;
                    if (!s->has_end || (s->off_at > 0 && now > s->off_at + ONCE_STALE_S))
                        s->once_off_done = true;
                    ESP_LOGI(TAG, "Once slot %d already past - not re-firing.", i + 1);
                }
            }

            if (!s->once_on_done && s->fires_at > 0 && now >= s->fires_at) {
                ESP_LOGI(TAG, "Once slot %d START — applying command.", i + 1);
                fire_command(s->command, false);
                s->once_on_done = true;
            }
            if (s->once_on_done && s->has_end && !s->once_off_done && s->off_at > 0 && now >= s->off_at) {
                ESP_LOGI(TAG, "Once slot %d END — turning off.", i + 1);
                fire_command(s->command, true);
                s->once_off_done = true;
            }
        }

        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 s precision
    }
}
