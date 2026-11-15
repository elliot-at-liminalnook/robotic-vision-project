/* The rover's board, between the hub and the rover's servo bus.
 *
 * Bytes from the hub (over Wi-Fi) go to the bus; bytes from the bus go back to the hub. The board does not read
 * them. The one thing it does by itself: when the hub has said nothing for a while, it stops the wheels, because
 * a servo in motor mode keeps turning at the last speed it was given, and a rover whose link has dropped must
 * not keep driving.
 *
 * This file and bridge.c are plain C with no operating system under them, so the same code runs on the board
 * (rover_bridge_main.c) and inside the simulation (../host, sim/so101sim/rover_fw.py).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BRIDGE_MAX_WHEELS 4
#define BRIDGE_STOP_PACKET_MAX (8 + BRIDGE_MAX_WHEELS * 5)

typedef struct {
    uint8_t wheel_ids[BRIDGE_MAX_WHEELS]; /* the servos that are wheels (motor mode) */
    uint8_t n_wheels;
    uint8_t speed_addr;      /* the register a wheel's speed is written to */
    uint8_t speed_size;      /* its size in bytes (1..4); all zero bytes is "stopped" */
    uint32_t watchdog_ms;    /* silence from the hub for this long stops the wheels */
    uint32_t repeat_ms;      /* while the silence lasts, the stop is sent again this often (0: never) */
    bool strip_echo;         /* the bus interface repeats back what is sent to it: do not pass that on to the hub */
} bridge_config_t;

typedef struct {
    bridge_config_t cfg;
    uint32_t last_word_ms;   /* when the hub last sent anything */
    uint32_t last_stop_ms;   /* when the wheels were last told to stop by this board */
    bool stopped;            /* the watchdog has tripped and the hub has not spoken since */
    uint32_t echo_pending;   /* bytes just written to the bus that will come back as echo */
    uint32_t echo_since_ms;
    /* counters, for the status line and for tests */
    uint32_t trips;          /* times the watchdog stopped the wheels */
    uint32_t stops_sent;     /* stop packets sent, repeats included */
    uint32_t hub_bytes;      /* bytes passed hub -> bus */
    uint32_t bus_bytes;      /* bytes passed bus -> hub */
    uint32_t echo_dropped;
} bridge_t;

/* Start. The board cannot know what the wheels were last told (it may have restarted while they turned), so the
 * first bridge_tick stops them. Returns false if the configuration cannot be used. */
bool bridge_init(bridge_t *b, const bridge_config_t *cfg, uint32_t now_ms);

/* Bytes that arrived from the hub. Returns how many to write to the bus, copied to `out` (never more than `n`). */
size_t bridge_from_hub(bridge_t *b, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap);

/* Bytes that arrived from the bus. Returns how many to send to the hub, copied to `out`. */
size_t bridge_from_bus(bridge_t *b, const uint8_t *data, size_t n, uint32_t now_ms, uint8_t *out, size_t cap);

/* Call at least every few milliseconds. Returns the length of a packet to write to the bus (the stop), or 0. */
size_t bridge_tick(bridge_t *b, uint32_t now_ms, uint8_t *out, size_t cap);

/* The packet that stops the wheels: one broadcast SYNC_WRITE of zero to every wheel's speed register. It draws
 * no reply, so nothing unexpected reaches the hub. Returns its length, or 0 if `cap` is too small. */
size_t bridge_stop_packet(const bridge_config_t *cfg, uint8_t *out, size_t cap);
