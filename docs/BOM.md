# Parts

Both online orders were placed on 2026-11-10 (Hiwonder and DigiKey). Prices are as shown in the carts that
day, before shipping. The fuse, clamps and splice supplies are still to buy locally (last section). The
cameras, ESP32-P4 boards and ribbon cables were already on hand (`PLAN.md`, "Parts on hand").

Follower arm only. No leader arm, so no HX-10HM servos (decided 2026-11-10).

## Hiwonder: $129.93

| Qty | Part | Unit | Total | Link |
|---:|---|---:|---:|---|
| 6 | HX-30HM bus servo. Each includes two horns, screws and a 20 cm bus cable | $19.99 | $119.94 | [hiwonder.com](https://www.hiwonder.com/products/hx-30hm?variant=42135141253207) |
| 1 | BusLinker V3.0 servo driver board. Includes a USB-C cable. Barrel jack and 2-pin screw terminal, marked 14 V max | $9.99 | $9.99 | [hiwonder.com](https://www.hiwonder.com/products/hx-30hm?variant=44074494459991) |

## DigiKey: $182.69 (about $230.74 with estimated tariff $39.56 and FedEx Ground $8.49)

| Qty | Part | DigiKey no. | Unit | Total | Link |
|---:|---|---|---:|---:|---|
| 1 | Mean Well GST280A12-C6P desktop supply, 12 V 21 A (252 W). 6-pin rectangular output plug. Estimated tariff $32.83 | 1866-2119-ND | $89.10 | $89.10 | [digikey.com](https://www.digikey.com/en/products/detail/mean-well-usa-inc/GST280A12-C6P/7703681) |
| 1 | Power cord, NEMA 5-15P to IEC C13, 6 ft | 116-PC-ABK006F-ND | $5.88 | $5.88 | [digikey.com](https://www.digikey.com/en/products/detail/cinch-connectivity-solutions/PC-ABK006F/15777826) |
| 2 | Molex PicoBlade 4-circuit cable, socket to socket, 150 mm (0151340402). For the camera boards' MX1.25 "USB" header | WM15258-ND | $4.26 | $8.52 | [digikey.com](https://www.digikey.com/en/products/detail/molex/0151340402/6198148) |
| 2 | Adafruit 4448: USB-A plug to four jumper leads, 30 cm | 1528-4448-ND | $1.95 | $3.90 | [digikey.com](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4448/11569137) |
| 2 | Adam Tech USB 2.0 extension, A plug to A receptacle, 6 ft | 2057-CA-USB-AM-AF-6FT-ND | $5.55 | $11.10 | [digikey.com](https://www.digikey.com/en/products/detail/adam-tech/CA-USB-AM-AF-6FT/9830213) |
| 1 | Adafruit 4255: brass M3 × 4 mm heat-set inserts, bag of 50 | 1528-4255-ND | $5.95 | $5.95 | [digikey.com](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4255/10244656) |
| 100 | M3 × 8 pan-head Phillips machine screw (minimum order 100) | H743-ND | $0.0517 | $5.17 | [digikey.com](https://www.digikey.com/en/products/detail/bfirst-industrial/MPMS-003-0008-PH/274954) |
| 100 | M3 × 12 pan-head Phillips machine screw (minimum order 100) | H744-ND | $0.1805 | $18.05 | [digikey.com](https://www.digikey.com/en/products/detail/bfirst-industrial/MPMS-003-0012-PH/274955) |
| 100 | M3 hex nut, 5.5 mm across flats (minimum order 100) | H762-ND | $0.1142 | $11.42 | [digikey.com](https://www.digikey.com/en/products/detail/bfirst-industrial/MHNZ-003/274973) |
| 1 | Creality Hyper PLA, black, 1.75 mm, 1 kg | 4095-HYPERPLABLACK1KG-ND | $23.60 | $23.60 | [digikey.com](https://www.digikey.com/en/products/detail/creality-3d/HYPER-PLA-BLACK-1KG/21806109) |

**Parts total: $312.62** before shipping and tariff.

## What each part is for

| Part | Use |
|---|---|
| Servos, driver board | The six joints and the USB-to-bus adapter |
| 12 V supply, cord | Servo power (DECISIONS F1). See "Power wiring" below |
| PicoBlade cable + USB-A lead | One camera cable per board: splice them per the pinout in `firmware/p4-uvc-camera/README.md` |
| USB extensions | The spliced camera cables are only ~45 cm long |
| Inserts, M3 × 8, M3 × 12, nuts | Wrist bracket and camera holder (`hardware/wrist_mount/README.md`); spares for the scene-camera stand |
| PLA | Printing the arm and mounts |

## Power wiring

```
wall ── power cord ── supply brick ── output cable ──[10 A fuse]── BusLinker screw terminal (+ / −)
                                                                        │
                                                     3-pin servo cable (power + data)
                                                                        │
                                           servo 1 ── servo 2 ── … ── servo 6   (daisy-chained)
```

1. Cut the 6-pin plug off the supply's output cable. Several wires are +12 V and several are ground: identify
   them from the supply's datasheet or with a multimeter, not by colour.
2. Twist the +12 V wires together and the ground wires together.
3. Splice an **inline blade-fuse holder with a 10 A fuse** into the + lead, near the supply. Solder both joints
   and cover them with heat-shrink.
4. Clamp + and − into the BusLinker's screw terminal, matching the polarity printed on the board. Reverse-polarity
   protection on the board is unconfirmed.
5. The board passes 12 V onto the servo bus. Each servo has two bus sockets, so one 3-pin cable per hop carries
   power and data. Splitting the six servos across two of the board's bus sockets halves the current in the first cable.

Why the fuse: the supply can deliver 21 A, but the servo leads are thin and their connectors small. A short
would overheat the wire before the supply shut down. With the fuse, the usable current is 10 A, not 21 A.

## Not checked

- That the board's "MX1.25" header takes a PicoBlade plug. The pitch matches; the fit is unconfirmed.
- The bracket's 4.0 mm pilot hole against the insert's recommended hole size.
- The BusLinker's screw terminal and the servo bus connectors are small. The supply can deliver 21 A, but the
  wiring between it and the servos has not been rated for that.

## Still to get, locally

Not bought yet. Stock at these stores is unchecked.

| Item | Ask for | Where (Greensboro, NC) |
|---|---|---|
| Inline fuse holder | "Inline ATC blade fuse holder", 14 or 16 gauge leads | Auto parts store, e.g. AutoZone, 4407 W Gate City Blvd (address from a directory listing; confirm) |
| Fuses | 10 A ATC blade fuses, pack of five | Same |
| 2 table clamps | 6-inch bar clamps or C-clamps | Harbor Freight, 3709 Farmington Dr, or any hardware store |
| Heat-shrink tubing, solder | Small assortment; thin rosin-core solder | Same |
| Jaw pads | Open decision (`PLAN.md`) | |
