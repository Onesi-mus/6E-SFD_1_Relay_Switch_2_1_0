# Relay settings: powerOnState and switchType

Device: **6E-SFD 1-Relay Switch** (firmware 2.1.x, 1-gang)

## 1. Accepted values

### powerOnState: what the relay does when mains power returns

| Value      | Meaning                                                  |
|------------|----------------------------------------------------------|
| `"off"`    | Relay always starts OFF                                  |
| `"on"`     | Relay always starts ON                                   |
| `"memory"` | Relay returns to its last state before the power cut. **Default** |

### switchType: how the physical wall switch drives the relay

| Value         | Switch hardware          | Behaviour                                      |
|---------------|--------------------------|------------------------------------------------|
| `"toggle"`    | Latching rocker (flip)   | Every flip, up or down, inverts the relay. **Default** |
| `"follow"`    | Latching rocker (flip)   | The relay follows the switch position (one position = ON, the other = OFF) |
| `"momentary"` | Push-button (spring-back) | Each press inverts the relay; releasing does nothing |

Values are **case-sensitive** and must be lowercase.

## 2. Name mismatch with the current backend payload

The backend currently uses different names. Please send the firmware names:

| Backend sends now | Send instead  | Current result on the device                         |
|-------------------|---------------|------------------------------------------------------|
| `"last"`          | `"memory"`    | Works only by accident (an unknown value falls back to memory) |
| `"flip"`          | `"toggle"`    | Works only by accident (an unknown value falls back to toggle) |
| `"push"`          | `"momentary"` | **Wrong:** falls back to toggle, so a push-button behaves like a flip switch |

## 3. How to send them

Topic (backend → device): `sig/Sigtron<serial>/pub`
where `<serial>` is the MAC, e.g. `sig/SigtronAA:BB:CC:00:00:01/pub`.

**Power-on state**
```json
{ "type": "relay_status", "mode": "memory" }
```

**Switch type**
```json
{ "type": "switch_type", "mode": "momentary" }
```

Notes:
- Both settings are saved on the device and survive reboots.
- `relay_status` takes effect at the **next power-up**. It does not change the relay now.
- `switch_type` takes effect **immediately**.
- These two commands send **no feedback/ack**, even if a `commandId` is included.
- The device does **not** report these settings back in its state messages. The backend must keep its own copy.

## 4. One relay per device

This device has **one relay**, so it has a single `powerOnState` and a single `switchType`.
In the backend model, only `relay1` applies to this device. `relay2` / `relay3` belong to 2-gang / 3-gang hardware and are ignored here.

Example for this device:
```json
{ "serialNo": "SigtronAA:BB:CC:00:00:01",
  "state": { "relay1": { "state": "on", "powerOnState": "memory", "switchType": "toggle" } } }
```

## 5. Relay on/off command (for reference)

```json
{ "type": "relay1", "state": "on", "commandId": "abc-123" }
```
`"relay"` and `"overall"` are also accepted as the type. State is published on `sig/Sigtron<serial>/sub`:
```json
{ "relay1": true, "relay": true }
```
plus feedback `{ "commandId": "abc-123", "status": "SUCCESS" }` when a `commandId` is given.
