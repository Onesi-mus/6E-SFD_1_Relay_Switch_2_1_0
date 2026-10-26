/*
//SECTIONS
-IOT JOB
-MESSAGE HANDLING
-NVS HELPERS
-OTA
-PUBLIC API
*/


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "cJSON.h"

#include "ota.h"
#include "app_config.h"
#include "mesh.h"

//Internal states and declerations
//JOB topic template
#define TOPIC_JOBS_NOTIFY_NEXT       "$aws/things/%s/jobs/notify-next"
#define TOPIC_JOBS_GET_NEXT          "$aws/things/%s/jobs/$next/get"
#define TOPIC_JOBS_GET_NEXT_ACCEPTED "$aws/things/%s/jobs/$next/get/accepted"
#define TOPIC_JOBS_GET_NEXT_REJECTED "$aws/things/%s/jobs/$next/get/rejected"
#define TOPIC_JOBS_UPDATE            "$aws/things/%s/jobs/%s/update"

//NVS bookkeeping for tracking the active OTA jobId across reboots, so we can
//report SUCCEEDED or FAILED to AWS once the new image confirms its boot or
//the bootloader rolls back. Without this, a silently rolled-back device leaves
//the AWS Job execution stuck IN_PROGRESS forever.
#define NVS_OTA_NS              "ota"
#define NVS_KEY_PENDING_JOB     "pending_job"

static esp_mqtt_client_handle_t s_client = NULL;
static char s_thing_name[48] = {0};
static char s_topic_notify_next[96] = {0};  //Topic for live OTA when device in online
static char s_topic_get_next[96] = {0};  //Asking job for any update
static char s_topic_get_next_accepted[96] = {0};  //topic when there is ota available
static char s_topic_get_next_rejected[96] = {0};  //topic when there is no ota
static char s_active_job_id[64] = {0};

static bool s_ota_in_progress = false;

//Deferred report of the previous boot's OTA outcome. Set by ota_init based on
//the running partition state, sent by ota_handler_on_connected once MQTT is up.
typedef enum {
    PENDING_REPORT_NONE = 0,
    PENDING_REPORT_SUCCEEDED,
    PENDING_REPORT_FAILED,
} pending_report_t;

static pending_report_t s_pending_report = PENDING_REPORT_NONE;
static char s_pending_job_id[64] = {0};

typedef struct{
    char job_id[64];
    char url[512];
    char version[32];
} ota_task_arg_t;




static const char *O = "OTA";

//Forward declarations
void ota_init(esp_mqtt_client_handle_t client, const char *thing_name);
static void build_topic(void);
void ota_handler_on_connected(void);
bool check_job_topic(const char *topic);
static void report_job_status(const char *job_id, const char *status, const char *detail);
static bool version_is_newer(const char *remote_ver, const char *local_ver);
static void ota_task(void *pv);
void ota_process_message(const char *topic, const char *payload);
static void handle_job_execution(const cJSON *execution);
static bool nvs_save_pending_job(const char *job_id);
static bool nvs_load_pending_job(char *out, size_t out_size);
static void nvs_clear_pending_job(void);



//
//IOT JOB
//
static void build_topic(void){
    snprintf(s_topic_notify_next,       sizeof(s_topic_notify_next),       TOPIC_JOBS_NOTIFY_NEXT,       s_thing_name);
    snprintf(s_topic_get_next,          sizeof(s_topic_get_next),          TOPIC_JOBS_GET_NEXT,          s_thing_name);
    snprintf(s_topic_get_next_accepted, sizeof(s_topic_get_next_accepted), TOPIC_JOBS_GET_NEXT_ACCEPTED, s_thing_name);
    snprintf(s_topic_get_next_rejected, sizeof(s_topic_get_next_rejected), TOPIC_JOBS_GET_NEXT_REJECTED, s_thing_name);
}

//Function to check does the topic is related to the job
bool check_job_topic(const char *topic){
    if (!topic) return false;
    char prefix[96];
    snprintf(prefix, sizeof(prefix), "$aws/things/%s/jobs/", s_thing_name);
    return strncmp(topic, prefix, strlen(prefix)) == 0;
}

//Function to check the version of the OTA file
static bool version_is_newer(const char *remote_ver, const char *local_ver){
    if (!remote_ver || !local_ver){
        ESP_LOGE(O, "Error to get the version number...");
        return false;
    }
    int r1 = 0, r2 = 0, r3 = 0;
    int l1 = 0, l2 = 0, l3 = 0;
    sscanf(remote_ver, "%d.%d.%d", &r1, &r2, &r3);
    sscanf(local_ver, "%d.%d.%d", &l1, &l2, &l3);
    if (r1 != l1) return r1 > l1;
    if (r2 != l2) return r2 > l2;
    return r3 > l3;
}

//Function to report the job status to backend
static void report_job_status(const char *job_id, const char *status, const char *detail){
    if (!s_client || !job_id || !job_id[0]){
        ESP_LOGE(O, "Failed to create json to report job status wrong parameters passed...");
        return;
    }
    char topic[128];
    snprintf(topic, sizeof(topic), TOPIC_JOBS_UPDATE, s_thing_name, job_id);
    cJSON *root = cJSON_CreateObject();
    if (!root){
        ESP_LOGE(O, "Failed to created json for reproting job status...");
        return;
    }
    cJSON_AddStringToObject(root, "status", status);
    if (detail){
        cJSON *details = cJSON_CreateObject();
        if (!details){
            ESP_LOGE(O, "Failed to create object for detail in report job status...");
            cJSON_Delete(root);
            return;
        }
        cJSON_AddStringToObject(details, "reason", detail);
        cJSON_AddItemToObject(root, "statusDetails", details);
    }
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body){
        ESP_LOGE(O, "Failed to create body when converting the root for report job status...");
        return;
    }
    esp_mqtt_client_publish(s_client, topic, body, 0, 1, 0);
    ESP_LOGI(O, "Job status reported : %s", body);
    free(body);
}



//
//MESSAGE HANDLING
//
void ota_process_message(const char *topic, const char *payload){
    if (!topic || !payload) return;
    bool is_notify = strcmp(topic, s_topic_notify_next) == 0;
    bool is_accepted = strcmp(topic, s_topic_get_next_accepted) == 0;
    bool is_rejected = strcmp(topic, s_topic_get_next_rejected) == 0;
    if (is_rejected){
        ESP_LOGE(O, "job rejected the request : %s", payload);
        return;
    }
    if (!is_notify && !is_accepted){
        ESP_LOGW(O, "Some other job topic : error...");
        return;
    }
    cJSON *root = cJSON_Parse(payload);
    if (!root){
        ESP_LOGE(O, "Failed to parse the payload...");
        return;
    }
    const cJSON *execution = cJSON_GetObjectItem(root, "execution");
    handle_job_execution(execution);
    cJSON_Delete(root);
}

//Function to handle the object execution
static void handle_job_execution(const cJSON *execution){
    if (!cJSON_IsObject(execution)){
        ESP_LOGI(O, "No pending job...");
        return;
    }
    const cJSON *job_id = cJSON_GetObjectItem(execution, "jobId");
    const cJSON *job_doc = cJSON_GetObjectItem(execution, "jobDocument");
    if (!cJSON_IsString(job_id) || !cJSON_IsObject(job_doc)){
        ESP_LOGE(O, "Missing job id and doc in the json...");
        return;
    }
    const cJSON *url = cJSON_GetObjectItem(job_doc, "url");
    const cJSON *ver = cJSON_GetObjectItem(job_doc, "version");
    if (!cJSON_IsString(url) || !cJSON_IsString(ver)){
        ESP_LOGE(O, "Job json missing url & version name");
        report_job_status(job_id->valuestring, "FAILED", "bad jobDocument");
        return;
    }
    if (s_ota_in_progress){
        ESP_LOGW(O, "OTA already in progress for  job id : %s, so ignoring job id : %s", s_active_job_id, job_id->valuestring);
        return;
    }
    if (!version_is_newer(ver->valuestring, firmware_version)){
        ESP_LOGW(O, "No new version available to update \nAvailable version : %s, existing version : %s \nUp to date",
            ver->valuestring, firmware_version);
        report_job_status(job_id->valuestring, "SUCCEEDED", "already up to date");
        return;
    }
    ota_task_arg_t *arg = calloc(1, sizeof(*arg));
    if (!arg){
        ESP_LOGE(O, "Out of memory when allocating memory for OTA task...");
        report_job_status(job_id->valuestring, "FAILED", "Out of memory");
        return;
    }
    strncpy(arg->job_id, job_id->valuestring, sizeof(arg->job_id) - 1);
    strncpy(arg->url, url->valuestring, sizeof(arg->url) - 1);
    strncpy(arg->version, ver->valuestring, sizeof(arg->version) - 1);
    s_ota_in_progress = true;
    strncpy(s_active_job_id, arg->job_id, sizeof(s_active_job_id) - 1);
    if (xTaskCreate(ota_task, "ota_task", 8192, arg, 5, NULL) != pdPASS){
        ESP_LOGE(O, "Failed to create OTA task..");
        report_job_status(arg->job_id, "FAILED", "task create");
        s_ota_in_progress = false;
        s_active_job_id[0] = '\0';
        free(arg);
    }

}



//
//NVS HELPERS
//
//Function to persist the active OTA jobId before we cross any reboot threshold
static bool nvs_save_pending_job(const char *job_id){
    nvs_handle_t h;
    if (nvs_open(NVS_OTA_NS, NVS_READWRITE, &h) != ESP_OK){
        ESP_LOGE(O, "Failed to open NVS for saving pending job...");
        return false;
    }
    esp_err_t e1 = nvs_set_str(h, NVS_KEY_PENDING_JOB, job_id);
    esp_err_t e2 = nvs_commit(h);
    nvs_close(h);
    if (e1 != ESP_OK || e2 != ESP_OK){
        ESP_LOGE(O, "Failed to save pending job in NVS...");
        return false;
    }
    ESP_LOGI(O, "Saved pending job to NVS : %s", job_id);
    return true;
}

//Function to read the pending OTA jobId from NVS
static bool nvs_load_pending_job(char *out, size_t out_size){
    nvs_handle_t h;
    if (nvs_open(NVS_OTA_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = out_size;
    esp_err_t err = nvs_get_str(h, NVS_KEY_PENDING_JOB, out, &len);
    nvs_close(h);
    return err == ESP_OK;
}

//Function to clear the pending OTA jobId from NVS
static void nvs_clear_pending_job(void){
    nvs_handle_t h;
    if (nvs_open(NVS_OTA_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY_PENDING_JOB);
    nvs_commit(h);
    nvs_close(h);
}



//
//OTA
//
static void ota_task(void *pv){
    ota_task_arg_t *arg =(ota_task_arg_t *) pv;
    ESP_LOGI(O, "HTTPS OTA started, url : %s, version : %s", arg->url, arg->version);

    //Persist jobId BEFORE we cross any reboot threshold so we can report
    //FAILED on next boot if the new image rolls back, or SUCCEEDED if it
    //confirms its boot.
    nvs_save_pending_job(arg->job_id);

    report_job_status(arg->job_id, "IN_PROGRESS", "downloading");

    ESP_LOGI(O, "Heap before OTA: free=%u, min_free=%u, largest_block=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    //The mesh holds tasks, queues and the ESP-NOW driver; free them too.
    bool mesh_was_up = mesh_is_running();
    if (mesh_was_up){
        ESP_LOGI(O, "Stopping mesh for the OTA...");
        mesh_stop();
    }

    //MQTT's TLS session (~40 KB) leaves too little contiguous heap for a second
    //one, so the S3 handshake fails. Free it for the duration of the download.
    if (s_client){
        ESP_LOGI(O, "Stopping MQTT to free heap for the OTA TLS handshake...");
        esp_mqtt_client_stop(s_client);
        vTaskDelay(pdMS_TO_TICKS(500)); //let the TLS session release its heap
        ESP_LOGI(O, "Heap after MQTT stop: free=%u, largest_block=%u",
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }

    //Modem sleep parks the radio between DTIM beacons — the log shows a 409 ms
    //listen interval. Over a marginal link that stalls the TCP window long
    //enough for S3 to reset the connection mid-download. Full power for the
    //duration; restored below if the download fails.
    wifi_ps_type_t saved_ps = WIFI_PS_MIN_MODEM;
    esp_wifi_get_ps(&saved_ps);
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_http_client_config_t http_cfg = {
        .url = arg->url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
        .buffer_size = 4096,       //default 512 — fewer, larger reads
        .buffer_size_tx = 1024,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };
    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err != ESP_OK) esp_wifi_set_ps(saved_ps);   //success reboots anyway
    if (err == ESP_OK){
        ESP_LOGI(O, "OTA download done, rebooting into new firmware (version : %s)", arg->version);
        //SUCCEEDED is reported by the new image after it confirms PENDING_VERIFY,
        //not here. Reporting it now would race with a possible rollback.
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        ESP_LOGE(O, "Failed to finish the OTA, %s", esp_err_to_name(err));
        //Bring MQTT back so the FAILED report can be published.
        if (s_client){
            esp_mqtt_client_start(s_client);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        report_job_status(arg->job_id, "FAILED", esp_err_to_name(err));
        //OTA failed before any reboot, no rollback will happen. Clear the
        //pending job so the next boot doesn't re-report on it.
        nvs_clear_pending_job();
    }
    s_ota_in_progress = false;
    s_active_job_id[0] = '\0';
    free(arg);
    vTaskDelete(NULL);
}


//
//PUBLIC API
//
//Function to initialize the OTA
void ota_init(esp_mqtt_client_handle_t client, const char *thing_name){
    s_client = client;
    strncpy(s_thing_name, thing_name, sizeof(s_thing_name) - 1);
    s_thing_name[sizeof(s_thing_name) - 1] = '\0';
    build_topic();

    //Decide what to do based on the boot scenario
    //  1. Running partition is PENDING_VERIFY -> new firmware booted ok -> mark
    //     valid + defer SUCCEEDED report until MQTT connects.
    //  2. Bootloader rolled back to the previous image -> last OTA failed ->
    //     defer FAILED report until MQTT connects.
    //  3. Neither -> normal boot, nothing to do.
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *invalid = esp_ota_get_last_invalid_partition();

    char saved_job[sizeof(s_pending_job_id)] = {0};
    bool have_saved_job = nvs_load_pending_job(saved_job, sizeof(saved_job));

    esp_ota_img_states_t state;
    bool is_pending_verify = (esp_ota_get_state_partition(running, &state) == ESP_OK
                              && state == ESP_OTA_IMG_PENDING_VERIFY);

    if (is_pending_verify){
        ESP_LOGI(O, "New firmware uploaded and ran successfully, making it valid");
        esp_ota_mark_app_valid_cancel_rollback();
        if (have_saved_job){
            strncpy(s_pending_job_id, saved_job, sizeof(s_pending_job_id) - 1);
            s_pending_report = PENDING_REPORT_SUCCEEDED;
        }
        nvs_clear_pending_job();
    } else if (invalid != NULL && invalid != running && have_saved_job){
        ESP_LOGW(O, "Bootloader rolled back from invalid partition, will report FAILED for job %s", saved_job);
        strncpy(s_pending_job_id, saved_job, sizeof(s_pending_job_id) - 1);
        s_pending_report = PENDING_REPORT_FAILED;
        nvs_clear_pending_job();
    }

    ESP_LOGI(O, "OTA handler initialized for thing : %s, F_version : %s", s_thing_name, firmware_version);
}

//Function to connect ota topic with mqtt client
void ota_handler_on_connected(void){
    if (!s_client){
        ESP_LOGE(O, "No MQTT client exist...");
        return;
    }

    //Send any deferred job status report from the previous boot. This is the
    //thing that prevents a stuck IN_PROGRESS execution on AWS after a rollback.
    if (s_pending_report == PENDING_REPORT_SUCCEEDED){
        ESP_LOGI(O, "Reporting SUCCEEDED for job %s (boot verified)", s_pending_job_id);
        report_job_status(s_pending_job_id, "SUCCEEDED", firmware_version);
        s_pending_report = PENDING_REPORT_NONE;
        s_pending_job_id[0] = '\0';
    } else if (s_pending_report == PENDING_REPORT_FAILED){
        ESP_LOGW(O, "Reporting FAILED for job %s (rolled back after boot failure)", s_pending_job_id);
        report_job_status(s_pending_job_id, "FAILED", "rolled back after boot failure");
        s_pending_report = PENDING_REPORT_NONE;
        s_pending_job_id[0] = '\0';
    }

    esp_mqtt_client_subscribe(s_client, s_topic_notify_next, 1);
    esp_mqtt_client_subscribe(s_client, s_topic_get_next_accepted, 1);
    esp_mqtt_client_subscribe(s_client, s_topic_get_next_rejected, 1);
    esp_mqtt_client_publish(s_client, s_topic_get_next, "", 0, 1, 0);
}
