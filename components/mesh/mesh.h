#ifndef MESH_H
#define MESH_H

#include <stdbool.h>
#include <stdint.h>

// ESP-NOW flooding mesh — 001 ONLINE / 002 PHONE / 003 ISLAND.
// Ported from the tested standalone mesh firmware; state machine, channel
// authority, search phases, hold-off and election are unchanged.
// Protocol and topics: MESH_BACKEND_CONTRACT.md

#define MESH_SERIAL_LEN  24   // "sig10:91:A8:41:FB:54" + NUL
#define MESH_MAX_NODES   16

// A command addressed to this device arrived over the mesh.
typedef void (*mesh_cmd_cb_t)(const char *json);
// Roster membership or the gateway role changed — republish it.
typedef void (*mesh_roster_cb_t)(void);
// A node we forwarded a command to has acked. state_json may be NULL.
typedef void (*mesh_ack_cb_t)(const char *serial, const char *command_id,
                              const char *state_json);
// The 003 WiFi phase found the router. Informational only — the app must NOT
// associate here; only the elected gateway associates, via mesh_uplink_up_cb_t.
typedef void (*mesh_router_found_cb_t)(uint8_t channel);
// Elected 001 gateway: the app should associate on this channel and start MQTT.
// channel 0 means "no hint" (the mesh is stopping and handing the radio back).
typedef void (*mesh_uplink_up_cb_t)(uint8_t channel);
// Lost or yielded the gateway role: the app should stop MQTT and disconnect.
typedef void (*mesh_uplink_down_cb_t)(void);
// Current relay state, reported back inside an ACK.
typedef bool (*mesh_relay_state_cb_t)(void);

void mesh_start(mesh_cmd_cb_t on_cmd, mesh_roster_cb_t on_roster,
                mesh_ack_cb_t on_ack, mesh_router_found_cb_t on_router,
                mesh_relay_state_cb_t get_relay);
void mesh_stop(void);

// Stop the mesh AFTER delay_ms, from a task of its own.
// A "mesh off" command arrives ON the mesh, so tearing the mesh down inside the
// receive callback deletes the very task executing it: the ACK is never sent
// and mesh_stop() never finishes. Use this for every command-driven stop. The
// delay also keeps the node relaying so the command reaches nodes further out.
// First call wins; later calls while one is pending are ignored.
void mesh_stop_deferred(uint32_t delay_ms);

bool mesh_is_running(void);

// Register the uplink callbacks. Call once, before mesh_start(): mesh_start()
// itself takes the STA down through on_down, and mesh_stop() gives it back
// through on_up.
void mesh_set_uplink_cbs(mesh_uplink_up_cb_t on_up, mesh_uplink_down_cb_t on_down);

// Group membership, derived from userId. Empty or NULL clears it.
void mesh_set_group(const char *group_id);
bool mesh_has_group(void);

// Fed by the app.
void mesh_set_cloud(bool up);
void mesh_set_router_ssid(const char *ssid);
void mesh_set_home_channel(uint8_t ch);

bool mesh_is_gateway(void);
uint8_t mesh_state(void);          // 1 = ONLINE, 2 = PHONE, 3 = ISLAND

// target_serial is "sig{MAC}". NULL targets every node (broadcast).
bool mesh_send_command(const char *target_serial, const char *command_id,
                       const char *json);

int  mesh_roster(char out[][MESH_SERIAL_LEN], int max);

// Per-peer link quality. There is no serial console in the field, so this is
// published over MQTT rather than printed.
typedef struct {
    char     serial[MESH_SERIAL_LEN];
    int8_t   rssi;        // most recent frame
    int8_t   min_rssi;    // weakest ever seen
    uint32_t rx_total;    // frames since boot
    uint32_t silent_ms;   // time since the last frame
} mesh_node_info_t;

int     mesh_node_info(mesh_node_info_t *out, int max);
uint8_t mesh_score(void);
uint8_t mesh_channel(void);
void mesh_own_serial(char out[MESH_SERIAL_LEN]);

#endif // MESH_H
