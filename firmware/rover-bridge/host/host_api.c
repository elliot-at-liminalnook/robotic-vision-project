/* The rover board's logic built for a PC: main/bridge.c unchanged, behind a C interface the simulation loads
 * (sim/so101sim/rover_fw.py). One bridge per handle, so several rovers can run at once. */
#include <stdlib.h>

#include "../main/bridge.c"

typedef struct {
    bridge_t b;
} fw_t;

void *fw_new(const uint8_t *wheel_ids, int n_wheels, int speed_addr, int speed_size, uint32_t watchdog_ms, uint32_t repeat_ms,
             int strip_echo, uint32_t now_ms)
{
    if (n_wheels < 0 || n_wheels > BRIDGE_MAX_WHEELS) {
        return NULL;
    }
    bridge_config_t cfg = {.n_wheels = (uint8_t)n_wheels, .speed_addr = (uint8_t)speed_addr, .speed_size = (uint8_t)speed_size,
                           .watchdog_ms = watchdog_ms, .repeat_ms = repeat_ms, .strip_echo = strip_echo != 0};
    for (int i = 0; i < n_wheels; i++) {
        cfg.wheel_ids[i] = wheel_ids[i];
    }
    fw_t *f = calloc(1, sizeof(*f));
    if (f == NULL || !bridge_init(&f->b, &cfg, now_ms)) {
        free(f);
        return NULL;
    }
    return f;
}

void fw_free(void *f) { free(f); }

size_t fw_from_hub(void *f, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap)
{
    return bridge_from_hub(&((fw_t *)f)->b, data, n, now_ms, out, cap);
}

size_t fw_from_bus(void *f, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap)
{
    return bridge_from_bus(&((fw_t *)f)->b, data, n, now_ms, out, cap);
}

size_t fw_tick(void *f, uint32_t now_ms, uint8_t *out, size_t cap) { return bridge_tick(&((fw_t *)f)->b, now_ms, out, cap); }

/* which: 0 trips, 1 stops sent, 2 bytes hub->bus, 3 bytes bus->hub, 4 echo bytes dropped, 5 stopped (0/1) */
uint32_t fw_count(void *f, int which)
{
    const bridge_t *b = &((fw_t *)f)->b;
    switch (which) {
    case 0: return b->trips;
    case 1: return b->stops_sent;
    case 2: return b->hub_bytes;
    case 3: return b->bus_bytes;
    case 4: return b->echo_dropped;
    case 5: return b->stopped ? 1u : 0u;
    default: return 0;
    }
}
