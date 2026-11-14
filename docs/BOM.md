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
- **Camera cable length on the arm.** The simulated wires (`sim/README.md`, "Wires") put the wrist camera's
  USB cable at about 0.76 m from the board to the table (260 + 160 + 340 mm between its clips). The
  spliced lead is about 0.45 m (150 mm PicoBlade lead + 300 mm USB-A lead), so its USB-A plug and the
  extension's socket would end up on the upper arm. Either splice in a longer lead or accept the join there.
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

---

# Rover parts (not ordered)

For the rover of `hardware/rover/` (design: `MULTI_DEVICE_PLAN.md`, 5.6). Prices and stock are what each
store's page showed on 2026-11-14, before shipping and tax. **Nothing here has been bought.** DigiKey's pages
could not be read by the search, so every DigiKey row is unconfirmed.

**Confirmed total for one rover: $416.27.** Of that, $240.81 is more of what the project already uses and
$175.46 is new kinds of part. A few small parts are not in the total (last table).

## More of what the project already uses: $240.81

| Qty | Part | Store | SKU | Unit | Total | Stock | Link |
|---:|---|---|---|---:|---:|---|---|
| 6 | HX-30HM bus servo: two wheels (motor mode), pan, lift, tilt, camera spacing | Hiwonder | 21100237 | $19.99 | $119.94 | Purchasable; no count shown | [hiwonder.com](https://www.hiwonder.com/products/hx-30hm) |
| 2 | Raspberry Pi Camera Module 3 Wide | CanaKit | RSP-CAM-V3-WIDE (SC0874) | $37.95 | $75.90 | "Add to Cart" shown; out of stock at Adafruit and PiShop.us | [canakit.com](https://www.canakit.com/raspberry-pi-camera-module-3.html) |
| 3 | Waveshare ESP32-P4-WIFI6-M: one per camera, one as the rover's controller | Waveshare | 31647 | $14.99 | $44.97 | **Out of stock** at Waveshare and at PiShop.us ($20.95) | [waveshare.com](https://www.waveshare.com/esp32-p4-wifi6.htm?sku=31647) |

## New kinds of part: $175.46

| Qty | Part | Store | SKU | Unit | Total | Stock | Link |
|---:|---|---|---|---:|---:|---|---|
| 1 | Gens ace 5000 mAh 3S 60C LiPo, XT60 (decided, DECISIONS RV1) | genstattu | GEA503S60X6GT | $70.09 | $70.09 | > 100 | [genstattu.com](https://genstattu.com/gens-ace-g-tech-5000mah-3s-60c-11-1v-lipo-battery-pack-with-xt60-plug) |
| 1 | Gens ace iMars mini 60 W balance charger with USB-C supply: 2 to 4 cells, 1 to 5 A | genstattu | GEA60WC1 | $38.99 | $38.99 | > 100 | [genstattu.com](https://genstattu.com/gens-ace-imars-mini-g-tech-60w-rc-battery-charger-with-usb-c-power-supply-adapter/) |
| 1 | LiPo charging bag, 180 × 70 × 70 mm | genstattu | GEA001 | $12.99 | $12.99 | Not shown | [genstattu.com](https://genstattu.com/gea001.html) |
| 20 | O-ring 3 mm × 54 mm inside diameter, Buna-N: a tyre for each 60 mm wheel (two needed; the store's minimum order is $10) | The O-Ring Store | N3.00X054 | $0.50 | $10.00 | 1640 | [theoringstore.com](https://www.theoringstore.com/store/index.php?main_page=product_info&products_id=24158) |
| 1 | Hiwonder BusLinker V3.0, on the rover: turns the controller board's serial pins into the one-wire servo bus | Hiwonder | 21090270 | $9.99 | $9.99 | Purchasable | [hiwonder.com](https://www.hiwonder.com/products/buslinker) |
| 1 | Pololu Big MOSFET Slide Switch with reverse-voltage protection, HP: the rover's on/off, about 16 A at 4.5 to 40 V | Pololu | 2815 | $8.49 | $8.49 | 270 | [pololu.com](https://www.pololu.com/product/2815) |
| 2 | M3 × 16 and M3 × 20 pan-head screws, 25-packs | Pololu | 2698, 2699 | $3.31 | $6.62 | 56, 90 | [pololu.com](https://www.pololu.com/product/2698) |
| 1 | SparkFun BabyBuck 5 V regulator (AP63357): 6 to 32 V in, 3 A at most, 16 × 13 mm, for the three boards | SparkFun | COM-21256 | $5.95 | $5.95 | In stock | [sparkfun.com](https://www.sparkfun.com/sparkfun-babybuck-regulator-breakout-5v-ap63357.html) |
| 2 | Camera ribbon, 22-pin 0.5 mm to 15-pin 1 mm, 200 mm | Adafruit | 5818 | $2.70 | $5.40 | In stock | [adafruit.com](https://www.adafruit.com/product/5818) |
| 1 | XT60 harness, one male to two female, 14 AWG, 150 mm (cut for leads) | genstattu | GEAX6M2X6F | $3.99 | $3.99 | > 100 | [genstattu.com](https://genstattu.com/gens-ace-xt60-male-to-2x-xt60-female-adapter-awg-14-150mm/) |
| 1 | Pololu ball caster, 3/8 in metal ball, 10 to 15 mm tall | Pololu | 951 | $2.95 | $2.95 | 135 | [pololu.com](https://www.pololu.com/product/951) |

## Needed, price not confirmed (not in the total)

| Qty | Part | Where | Note |
|---:|---|---|---|
| 1 | Inline blade-fuse holder and 10 A fuses, at the battery | Littelfuse FHAC0001ZXJ at DigiKey (about $7.37 from a search-indexed page), or the auto parts store as for the arm | Wire gauge of that holder unconfirmed. Fuses not sourced |
| 8 | M2 × 8 screws for the camera modules | DigiKey (APM Hexseal RM2X8MM-2701) | The camera's four holes are 2.2 mm on a 21 × 12.5 mm pattern, so M2; M2.5 will not pass |
| a few | Molex 5264 3-position housings and crimp terminals, to make servo leads longer than the 20 cm that ships | DigiKey (housing 50-37-5033) | Terminal part number not verified. Hiwonder sells no longer lead; it says other lengths are "available upon request" |

## What the search settled

- **The HX-30HM does run as a wheel motor.** Hiwonder's listing: "Mode 2: Speed Closed-Loop Motor Mode – Automatically
  compensates for load changes, ensuring stable and constant speed." Speed range 0 to 3400 steps/s. And it is
  multi-turn in position mode: "Up to 7 turns of absolute position control and feedback. After power loss, the
  turn count is cleared." Both as the design assumed (DECISIONS MD40, MD42). Still to be seen on a real servo.
- **No lighter servo for the head.** Nothing lighter both speaks this protocol and runs from 9 to 12.6 V: the HX-10HM
  is the same body and weight; the HX-12H (36 g) is 115200 baud with 0 to 1000 positions, so another protocol; the
  HX-08LCM (20 g) is 6 to 8.4 V. So HX-30HM throughout (answers the plan's open question O6).
- **BusLinker in UART mode** is the choice for the rover's bus interface: Hiwonder documents it with the HX-30HM at
  1 Mbps and gives an ESP32 wiring example. A single buffer IC would be smaller but rests on the bus's logic level,
  which no Hiwonder document found states.

## Not checked, or contradicting what the project assumes

- **The ESP32-P4 board cannot be bought today** from the stores checked. The rover needs three more.
- **Waveshare's wiki says the ESP32-P4 "can only support cameras with a maximum of 2 million pixels"** and names
  only the OV5647 and SC2336. The Camera Module 3's IMX708 is not listed. This project's two boards already
  stream from the IMX708 (`PLAN.md`), so the statement is about Waveshare's own support, but it is on record.
- **The caster is taller than drawn.** `build_rover.py` draws a 12 mm ball under a plate 9 mm above the
  surface; the Pololu 951 is 10 to 15 mm tall. The chassis's caster boss needs redrawing to the part.
- **XT60 naming.** The battery's listing calls its plug "male" and says a "female" mate is in the box. Stores differ
  on which half is which: use the included mate or buy a pair, not a single lead named by gender.
- **The 3 A regulator has no margin** if three boards and their cameras ever draw near 3 A at 5 V (never measured).
  Pololu's D30V30F5 (3.4 A, $29.95) is the step up.
- **The switch** was chosen over a rocker because the rocker's 10 A rating is for mains AC, with no DC rating given.
- **Servo connector pitch** is 2.50 mm (Molex 5264), not 2.54 mm. Its contacts are rated 3 A, the stall current
  of one servo: do not chain all six servos through one lead.
- **Not found in Hiwonder's documents:** the output spline (25T), the bus's logic level, and the protocol manual
  the BusLinker manual refers to. The listing also has copy errors ("HX-10HM" in places).
- **Wheels:** no 60 mm wheel for this spline is sold; the design prints its own and fits the O-ring.

