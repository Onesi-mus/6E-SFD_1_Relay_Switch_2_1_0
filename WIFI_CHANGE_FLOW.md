# Wi-Fi change over BLE — firmware flow (app integration guide)

> **Firmware:** 6E-SFD_1 relay switch, version **2.1.4** (ESP32-C3)
> **Source:** `main/main.c` (`wifi_prov_task`, `on_ble_credentials_received`), `components/ble_prov/ble_prov.c`
> **Audience:** mobile app team
>
> This document describes exactly what the firmware does when the app changes the Wi-Fi network.
> It works reliably with nRF Connect, so if the app fails, compare the app's behaviour step by step
> against this document (especially §5 "Common app mistakes").

---

## 1. Overview

```
App                                   Device
 │  (device in provisioning window: LED fast blink)
 │── scan for service 0x00FF ───────────▶│
 │── connect ───────────────────────────▶│
 │── enable notify on FF02 ─────────────▶│
 │── write FF01 {"pin":"123456"} ───────▶│
 │◀── notify {"status":"ok","reason":"Authenticated"}
 │── write FF01 {"ssid":"…","password":"…"} ▶│
 │◀── notify {"status":"ok","reason":"Credentials received"}   ← NOT the result yet
 │                                       │  … tries new Wi-Fi (≤ 25 s)
 │                                       │  … waits for cloud/MQTT (≤ 30 s)
 │◀── notify FINAL RESULT ───────────────│   "connected"  or  "failed to connect"
 │                                       │  ~0.7 s later BLE shuts down → app is disconnected
```

**The app must stay connected until the FINAL result arrives — up to ~60 seconds after sending the credentials.**

---

## 2. Opening the provisioning window (device side)

BLE is **off** during normal operation. It is only turned on in a provisioning window:

| Situation | How the window opens | Window length |
| --- | --- | --- |
| Device has **no Wi-Fi saved** (new / unlinked) | Automatically at boot | **No timeout** — stays open until provisioned |
| Device already has Wi-Fi (changing network) | User flips the rocker so it goes **to GND 3 times within 3 seconds** (about 5 quick flips on a latching rocker) | **2 minutes** |

- While the window is open the status **LED blinks fast** (100 ms on / 100 ms off).
- If nothing is received within 2 minutes, BLE closes and the device keeps its old Wi-Fi.
- The 3-flip gesture also toggles the relay on each flip (expected).

---

## 3. BLE interface

### Advertising
| Item | Value |
| --- | --- |
| Device name (in **scan response**) | `Sig` + base MAC, e.g. `SigDC:B4:D9:E4:BB:4C` |
| Service UUID (in **advertising packet**) | `000000FF-0000-1000-8000-00805F9B34FB` |
| Connectable | Yes. After a disconnect the device advertises again while the window is open |

### Characteristics (service `0x00FF`)
| Characteristic | UUID | Properties | Use |
| --- | --- | --- | --- |
| **Receive** | `0000FF01-0000-1000-8000-00805F9B34FB` | **Write (with response)** | App → device JSON (PIN, credentials) |
| **Status** | `0000FF02-0000-1000-8000-00805F9B34FB` | **Notify** | Device → app JSON status |

- Enable notifications on **FF02** (write the CCCD) **before** sending anything, or the app will miss the replies.
- Use **write with response**. Write-without-response is not supported on FF01.
- The device requests MTU 512. Each JSON must arrive as **one write** (a long write is fine; see §5).

---

## 4. Step-by-step protocol

All payloads are UTF-8 JSON. Keys are **case-sensitive**.

### Step 1 — Authenticate (PIN)

**Write FF01:**
```json
{"pin":"123456"}
```

| Notify on FF02 | Meaning | App action |
| --- | --- | --- |
| `{"status":"ok", "reason":"Authenticated"}` | PIN correct | Go to step 2 **on the same connection** |
| `{"status":"error", "reason":"Invalid PIN"}` | Wrong PIN | Device **disconnects** the app. Reconnect and retry |

- Authentication is **per connection**. If BLE disconnects, the app must send the PIN again.
- Send the PIN **in its own write**. If a write contains a `"pin"` key, the device handles **only** the PIN and ignores every other field in that write.

### Step 2 — Send Wi-Fi credentials

**Write FF01:**
```json
{"ssid":"MyHomeWiFi","password":"secret123"}
```

| Field | Rules |
| --- | --- |
| `ssid` | String, **exactly** the router name. Max **31 characters**. Used byte-for-byte — **no trimming**: a leading/trailing space, or wrong upper/lower case, is a different network |
| `password` | String, max **63 characters**. Used byte-for-byte, no trimming. Must be a string (not `null`) |

Immediate reply:

| Notify on FF02 | Meaning |
| --- | --- |
| `{"status":"ok", "reason":"Credentials received"}` | ⚠️ **Only means the JSON was received.** The connection attempt starts now. **Do not treat this as success.** |
| `{"status":"error", "reason":"Not authenticated"}` | Step 1 was not done on this connection |
| `{"status":"error", "reason":"Missing credentials"}` | `ssid` or `password` missing, or not a string |
| `{"status":"error", "reason":"Invalid JSON"}` | The write was not valid JSON (often a split / partial write — see §5) |

### Step 3 — Device switches network (app waits)

What the firmware does after "Credentials received":

1. Remembers the **current** (old) Wi-Fi as a backup.
2. Disconnects from the current Wi-Fi.
3. Connects to the new Wi-Fi — waits up to **25 s** for an IP address.
4. If Wi-Fi connected: waits up to **30 s** for the **cloud (AWS MQTT)** to connect.
5. **Success = Wi-Fi AND cloud both connected.** Only then are the new credentials **saved**.

### Step 4 — Final result (notify on FF02)

| Notify | Meaning | What the device does next |
| --- | --- | --- |
| `{"status":"ok", "reason":"connected"}` | New Wi-Fi **and** cloud connected | New credentials saved. BLE closes ~0.7 s later |
| `{"status":"error", "reason":"failed to connect"}` | Wi-Fi did not connect in 25 s, **or** Wi-Fi connected but the cloud did not connect in 30 s | Goes **back to the old Wi-Fi** (if it had one) — this takes up to 25 s. New credentials are **not** saved. BLE closes ~0.7 s **after the revert finishes** |

- After the final result, **the device closes BLE itself** — the app will see a disconnect. This is normal.
  On success this is ~0.7 s after "connected"; on failure it is after the revert to the old Wi-Fi (up to ~26 s after "failed to connect").
- To try again after a failure, the user must open a new window (3 rocker flips).
  Exception: a device with **no previous Wi-Fi** reopens BLE automatically (no timeout).

### Timing summary
| Phase | Max time |
| --- | --- |
| New Wi-Fi connect | 25 s |
| Cloud connect | 30 s |
| Revert to old Wi-Fi (failure only) | up to 25 s, **after** the "failed to connect" notify |
| **App should wait for the final notify** | **≥ 60 s** after "Credentials received" |

---

## 5. Common app mistakes (why nRF Connect works but the app may not)

| # | App behaviour | What the device does | What the app sees |
| --- | --- | --- | --- |
| 1 | Treats **"Credentials received"** as success and disconnects / shows "done" | Keeps trying in the background; result never reaches the app | App says success, but Wi-Fi may have failed and reverted |
| 2 | Sends PIN and credentials in **one** JSON `{"pin":…,"ssid":…,"password":…}` | Authenticates only, **ignores ssid/password** | Only "Authenticated", then nothing |
| 3 | Sends credentials on a **new connection** without the PIN | Rejects | "Not authenticated" |
| 4 | **Splits** the JSON into several separate writes (e.g. 20-byte chunks) | Each piece is parsed alone → invalid JSON | "Invalid JSON" |
| 5 | Adds/removes **spaces** or changes case in SSID/password (keyboard autocomplete, `trim()`, auto-capitalise) | Looks for a different network | "Credentials received" → "failed to connect" |
| 6 | Sends credentials **twice** (retry button, double tap, retry on slow reply) | Starts two Wi-Fi switches at once — they interfere | Unpredictable / failure |
| 7 | Does not enable notifications on FF02 first | Replies are sent but not delivered to the app | App waits forever / times out |
| 8 | Uses write-without-response on FF01 | Not supported | No reply |
| 9 | Times out after a few seconds | Device still working (up to ~55 s) | App reports failure while device may succeed |
| 10 | Connects after the 2-minute window closed | BLE is off | Device not found |

For #4: a **long write** (one write larger than the MTU, which iOS `writeValue(..., type: .withResponse)` and Android do automatically as prepare/execute write) is fine — the device receives it as one message. Only **separate** writes are a problem.

---

## 6. Network requirements

- **2.4 GHz** Wi-Fi only (ESP32-C3 has no 5 GHz).
- WPA2 / WPA3 networks. **Open networks (no password) are currently not supported** by this firmware version.
- The network must have **internet access to AWS** — otherwise the step-4 result is "failed to connect" and the device reverts to the old Wi-Fi.

---

## 7. Recommended app flow

1. Ask the user to open the window (3 quick rocker flips) and check the LED is fast-blinking.
2. Scan for service `0x00FF` (or the name `Sig…`), connect.
3. Request a larger MTU (Android: `requestMtu(512)`; iOS negotiates automatically).
4. Enable notifications on FF02 and wait for the CCCD write to complete.
5. Write `{"pin":"123456"}` → wait for `"Authenticated"`.
6. Write `{"ssid":"…","password":"…"}` **once**, as a single write with response, exact strings, no trimming.
7. On `"Credentials received"` → show "Connecting…" (**not** success).
8. Wait up to **60–70 s** for the final notify:
   - `"connected"` → success.
   - `"failed to connect"` → show failure; tell the user the device kept its old Wi-Fi; to retry, open the window again.
9. Expect the device to disconnect BLE right after the final notify.
10. If the BLE link drops before a final result, do not assume failure: check whether the device comes online in the cloud (it publishes `{"status":"online_and_ready",…}` on `sig/Sigtron{MAC}/sub` when it connects).

---

## 8. Debugging with the device serial log

The firmware logs every BLE write and every step. Ask for a serial log when the app fails:

```
I BLE_PROV: BLE client authenticated
I BLE_PROV: Write (NN bytes): {"ssid":"…","password":"…"}     ← exactly what the app sent
I SIGTRON:  Switching to new Wi-Fi network: <ssid>
I WIFI_MANAGER: Connected to WIFI : <ssid>                    ← or: WIFI connect timeout/failed
I IOT_NET: MQTT connected
I SIGTRON:  Connected (Wi-Fi + cloud). New credentials saved.  ← or: Provisioning failed. Reverting…
```

Comparing the `Write (NN bytes)` line from the app with the one from nRF Connect shows the difference immediately.
