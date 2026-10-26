#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

void wifi_manager_init(void);

// Blocks until connected or the timeout elapses. Safe to call again to switch
// networks (restarts the driver cleanly).
bool wifi_manager_connect(const char *ssid, const char *password);

// Disconnect and STOP the driver. ESP-NOW dies with it — do not use this to
// hand the radio to the mesh; use wifi_manager_set_auto_retry(false).
void wifi_manager_disconnect(void);

// Suppress/resume the automatic reconnect, leaving the driver running.
void wifi_manager_set_auto_retry(bool enabled);

// Bring the Wi-Fi DRIVER up (APSTA) WITHOUT associating. The mesh needs the
// driver running for ESP-NOW and the SoftAP for 002, but only the elected
// gateway may associate — so this deliberately does not connect.
void wifi_manager_start_driver(void);

#endif
