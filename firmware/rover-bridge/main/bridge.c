/* See bridge.h. */
#include "bridge.h"

#include <string.h>

#define SERVO_BROADCAST 0xFE
#define SERVO_SYNC_WRITE 0x83
#define ECHO_FORGET_MS 20 /* an echo that has not come back by now is not coming */

bool bridge_init(bridge_t *b, const bridge_config_t *cfg, uint32_t now_ms)
{
    if (b == NULL || cfg == NULL || cfg->n_wheels == 0 || cfg->n_wheels > BRIDGE_MAX_WHEELS ||
        cfg->speed_size == 0 || cfg->speed_size > 4 || cfg->watchdog_ms == 0) {
        return false;
    }
    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;
    /* As if the hub had last spoken a whole watchdog period ago: the first tick stops the wheels. (Unsigned
     * arithmetic: this is right whatever now_ms is, including just after the counter has wrapped.) */
    b->last_word_ms = now_ms - cfg->watchdog_ms;
    b->last_stop_ms = now_ms;
    return true;
}

size_t bridge_stop_packet(const bridge_config_t *cfg, uint8_t *out, size_t cap)
{
    /* FF FF | FE | LEN | 83 | addr | size | (id, zeros...) per wheel | CHK      LEN = params + 2 */
    size_t params = 2 + (size_t)cfg->n_wheels * (1 + cfg->speed_size);
    size_t total = 4 + 1 + params + 1;
    if (cap < total) {
        return 0;
    }
    size_t k = 0;
    out[k++] = 0xFF;
    out[k++] = 0xFF;
    out[k++] = SERVO_BROADCAST;
    out[k++] = (uint8_t)(params + 2);
    out[k++] = SERVO_SYNC_WRITE;
    out[k++] = cfg->speed_addr;
    out[k++] = cfg->speed_size;
    for (uint8_t w = 0; w < cfg->n_wheels; w++) {
        out[k++] = cfg->wheel_ids[w];
        for (uint8_t i = 0; i < cfg->speed_size; i++) {
            out[k++] = 0;
        }
    }
    uint32_t sum = 0;
    for (size_t i = 2; i < k; i++) { /* over ID, LEN, instruction and the parameters */
        sum += out[i];
    }
    out[k++] = (uint8_t)(~sum & 0xFF);
    return k;
}

size_t bridge_from_hub(bridge_t *b, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap)
{
    if (n == 0) {
        return 0;
    }
    if (n > cap) {
        n = cap; /* the caller reads the link in pieces no larger than its buffer; this is only a guard */
    }
    memcpy(out, data, n);
    b->last_word_ms = now_ms; /* the hub is there */
    b->stopped = false;
    b->hub_bytes += (uint32_t)n;
    if (b->cfg.strip_echo) {
        b->echo_pending += (uint32_t)n;
        b->echo_since_ms = now_ms;
    }
    return n;
}

size_t bridge_from_bus(bridge_t *b, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap)
{
    if (b->echo_pending && (uint32_t)(now_ms - b->echo_since_ms) > ECHO_FORGET_MS) {
        b->echo_pending = 0;
    }
    if (b->echo_pending) { /* the first bytes back are what was just sent */
        size_t drop = n < b->echo_pending ? n : b->echo_pending;
        data += drop;
        n -= drop;
        b->echo_pending -= (uint32_t)drop;
        b->echo_dropped += (uint32_t)drop;
    }
    if (n > cap) {
        n = cap;
    }
    if (n) {
        memcpy(out, data, n);
        b->bus_bytes += (uint32_t)n;
    }
    return n; /* what the servos say is not word from the hub: it does not feed the watchdog */
}

size_t bridge_tick(bridge_t *b, uint32_t now_ms, uint8_t *out, size_t cap)
{
    bool quiet = (uint32_t)(now_ms - b->last_word_ms) >= b->cfg.watchdog_ms;
    if (!quiet) {
        return 0;
    }
    bool first = !b->stopped;
    bool again = b->stopped && b->cfg.repeat_ms && (uint32_t)(now_ms - b->last_stop_ms) >= b->cfg.repeat_ms;
    if (!first && !again) {
        return 0;
    }
    size_t n = bridge_stop_packet(&b->cfg, out, cap);
    if (n == 0) {
        return 0;
    }
    if (first) {
        b->trips++;
    }
    b->stopped = true;
    b->last_stop_ms = now_ms;
    b->stops_sent++;
    if (b->cfg.strip_echo) {
        b->echo_pending += (uint32_t)n;
        b->echo_since_ms = now_ms;
    }
    return n;
}
