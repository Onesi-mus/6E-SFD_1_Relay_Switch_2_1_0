#ifndef SCHEDULER_H
#define SCHEDULER_H

// Backend-pushed, retained schedule slots fired locally.
// Topics, payload fields and timing rules: FIRMWARE_B_REFERENCE.md §8.

// Clear all slots and start the 1 s firing task. Call once SNTP is initialised.
void scheduler_init(void);

// Handle one sig/{id}/schedule/... slot message. Empty payload = tombstone.
// The slot is taken from the topic, not the payload.
void scheduler_handle_slot_message(const char *topic, const char *payload);

// Forget every stored slot. Call on every MQTT (re)connect, just before
// subscribing: the retained resend then rebuilds exactly the current set, so a
// slot deleted while offline (its tombstone never arrived) stops firing.
void scheduler_clear_all(void);

#endif // SCHEDULER_H
