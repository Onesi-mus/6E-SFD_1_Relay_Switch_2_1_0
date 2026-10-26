#ifndef BLE_PROV_H
#define BLE_PROV_H

// On-demand BLE Wi-Fi provisioning. Protocol/UUIDs match the factory firmware.
// See FIRMWARE_B_REFERENCE.md §2 and FRONTEND_WIFI_PROVISIONING.md.

typedef void (*ble_prov_cb_t)(const char *ssid, const char *password);

// Start NimBLE and advertise. Idempotent while already active.
void ble_prov_enter(ble_prov_cb_t cb);

// Stop advertising and de-init NimBLE (frees ~40 KB). Safe if not active.
void ble_prov_exit(void);

// Notify a JSON status string to the connected app. No-op if disconnected.
void ble_prov_send_status(const char *json);

#endif // BLE_PROV_H
