# rover-bridge: the rover's board

The rover's six servos are on one bus. The hub is somewhere else, on Wi-Fi. This firmware sits between them on
an ESP32-P4 board fixed to the chassis (docs/BOM.md; DECISIONS MD43, MD48):

```
hub  ──TCP over Wi-Fi──▶  rover_bridge_main.c  ──UART, 1 Mbps──▶  BusLinker (UART mode)  ──▶  six HX-30HM
                               │
                           bridge.c        passes bytes both ways, unread;
                                           stops the wheels when the hub has said nothing for 0.4 s
```

To the hub the board is a serial port that happens to be far away (`hub/ports.py`): it speaks the servos'
own protocol through it, and the board does not interpret a byte.

**The one thing the board does by itself** is stop the wheels. A servo in motor mode keeps turning at the last
speed it was given, so a rover whose link drops would drive on. When nothing has come from the hub for
`CONFIG_ROVER_BRIDGE_WATCHDOG_MS` (400), the board writes a speed of zero to both wheels: one broadcast
SYNC_WRITE, which draws no reply, so nothing unexpected reaches the hub when it comes back. While the silence
lasts it sends the same packet again every second, in case the first was lost. It also does this once at
start-up, because it cannot know what the wheels were last told. What the servos say back is not word from the
hub and does not keep the wheels going.

| File | What |
|---|---|
| `main/bridge.c`, `bridge.h` | Everything the board decides. Plain C, no operating system: the same file runs on the board and in the simulation |
| `main/rover_bridge_main.c` | What needs the board: Wi-Fi (station), one TCP connection from the hub, the UART, a status line every 10 s |
| `main/Kconfig.projbuild` | Network name and password, port, pins, wheel ids, speed register, watchdog time |
| `host/host_api.c` | `bridge.c` built for a PC behind a C interface (`sim/so101sim/rover_fw.py` loads it) |

## In the simulation

```
python sim/scripts/run_system.py --scene open_small --rover-fw
```

`World(scene, rover_firmware=True)` runs this firmware between the simulated radio link and the emulated servos,
in place of the simulation's own model of the board. `sim/tests/test_rover_fw.py` checks the core by itself
(bytes pass unchanged in any chunking; the stop packet is one the servos accept; it trips once per silence,
re-arms when the hub speaks, and survives its millisecond counter wrapping) and in the loop (wheels left
turning stop within the watchdog time; the hub drives normally through it; it behaves as the simulation's
model did).

By hand: `cc -std=c99 -Wall -Wextra -Werror -shared -fPIC host/host_api.c -o rover_bridge.so`

## Not yet built or flashed

`bridge.c` is compiled and tested on a PC. `rover_bridge_main.c` has not been compiled: there is no ESP-IDF
on the machine it was written on. To check on the first build and first power-up:

1. **Wi-Fi on the P4.** The chip has no radio; the board's ESP32-C6 is reached through `esp_wifi_remote` and
   `esp_hosted` (`main/idf_component.yml`, versions unpinned). Their Kconfig must match the board's wiring
   between the two chips. The ESP32-P4-WIFI6-M in the parts list was out of stock when priced.
2. **Pins.** `CONFIG_ROVER_BRIDGE_TX_GPIO` and `RX_GPIO` are placeholders.
3. **Echo.** Whether the BusLinker in UART mode repeats back what is sent to it. If it does, the hub sees its
   own packets ahead of every reply: set `CONFIG_ROVER_BRIDGE_STRIP_ECHO`.
4. **Latency.** The hub allows a reply 50 ms longer over this link than on a wire (`RadioPort.reply_time_s`).
   The loop here adds up to one tick (1 ms) each way; Wi-Fi adds the rest. Measure it.
5. **The watchdog on the bench**, wheels off the ground: start a drive, pull the hub's network cable, and see
   the wheels stop within half a second.
6. **Who else is on the bus.** The stop packet is written when the hub has been silent for 0.4 s, so the bus is
   idle. If something else ever shares the bus, that no longer follows.

Nothing here handles the rover's two cameras: they are their own boards with their own links.
