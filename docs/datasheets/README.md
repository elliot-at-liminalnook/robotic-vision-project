# Component datasheets

Retrieved 2026-11-08. Files are unmodified; verify with `shasum -a 256 -c SHA256SUMS` from this directory. No redistribution
license is stated for any of them. They are manufacturer documents kept for reference.

Files prefixed `ALT-` come from a retailer (Akizuki Denshi, Japan), not the
manufacturer, because Feetech doesn't host them.

## Feetech STS3215 servo: `feetech-sts3215/`

The SO-101 uses the 7.4V variants: follower 6× C001 (1/345); leader 1× C001,
2× C044 (1/191), 3× C046 (1/147). C018 is the optional 12V follower upgrade.

| File | Variant | Source |
|---|---|---|
| `Feetech_ST-3215-C001_datasheet.pdf` | C001, 7.4V, 1/345. Spec A/0, 2020-04-10, 8 pp | [feetechrc.com](https://www.feetechrc.com/Data/feetechrc/upload/file/20260622/6391772523943436695270694.pdf) |
| `Feetech_ST-3215-C018_12V_datasheet.pdf` | C018, 12V 30kg·cm, 1/345. Spec A/0, 2023-07-20, 8 pp | [feetechrc.com](https://www.feetechrc.com/Data/feetechrc/upload/file/20260622/6391772519923113075854851.pdf) |
| `Feetech_STS3215_drawing.png` | Dimension drawing, predates the Dec 2025 housing change | [feetechrc.com](https://www.feetechrc.com/Data/feetechrc/upload/image/20200611/6372749964088601844071289.png) |
| `ALT-Akizuki_ST-3215-C044_datasheet.pdf` | C044, 7.4V, 1/191. Spec A/0, 2025-04-15, 13 pp | [akizukidenshi.com](https://akizukidenshi.com/goodsaffix/ST-3215-C044%20SEPC.pdf) |
| `ALT-Akizuki_ST-3215-C046_datasheet.pdf` | C046, 7.4V, 1/147. Spec A/0, 2025-04-15, 14 pp. The title line wrongly says 1:191; the spec table says 1/147 | [akizukidenshi.com](https://akizukidenshi.com/goodsaffix/ST-3215-C046%20SEPC.pdf) |
| `ALT-Akizuki_STS3215-C001_datasheet.pdf` | Earlier C001 copy (adds "EMC Yes") | [akizukidenshi.com](https://akizukidenshi.com/goodsaffix/STS3215-C001%20SEPC.pdf) |
| `ALT-Akizuki_STS3215_design_change_20251229.pdf` | 2025-12-29 notice: rear housing slot → hole (image only) | [akizukidenshi.com](https://akizukidenshi.com/goodsaffix/STS3215_design_change_20251229.pdf) |

### Key specs (quoted from the datasheets)

| | C001 | C044 | C046 | C018 (12V) |
|---|---|---|---|---|
| Gear ratio | 1/345 | 1/191 | 1/147 | 1/345 |
| Input voltage | 4V–7.4V | 5V–8.4V | 5V–8.4V | 4V–14V |
| Stall torque | 16.5 kg·cm @6V, 19.5 @7.4V | 12.5 @6V, 16 @7.4V | 9.2 @6V, 14.4 @7.4V | 30 kg·cm @12V |
| Stall current | 2A @6V, 2.5A @7.4V | 3.2A / 3.8A | 2.6A / 3.3A | 2.7A |
| Voltage protection | >7.4V or <4V | >16V or <5V | >16V or <5V | >14V or <4V |

All variants: baud 38400 bps–1 Mbps (factory default 1 Mbps); over-current
protection at >2A for 2s; over-temperature >70℃; connector 5264-3P (1 GND, 2 Vcc, 3 TTL signal).

Caveats:
- Feetech's C001 *web page* says 6–7.4V, 19.5kg·cm@6V and an aluminium case; the PDF says
  4–7.4V, 16.5kg·cm@6V and PA+GF. The table follows the PDF.
- C044/C046 have under-voltage protection below 5V, and the upstream BOM uses a 5V supply
  for the leader, so any sag may trip it.

## Feetech bus protocol: `feetech-protocol/`

Also applies to Hiwonder's HX servos (see `third_party/hiwonder-so-arm101/README.md`).

| File | What | Source |
|---|---|---|
| `Feetech_protocol_manual_magnetic_encoder.pdf` | Official protocol manual, magnetic-encoder version (Chinese, 9 pp, rev. 2024-02-21) | [feetechrc.com](https://www.feetechrc.com/Data/feetechrc/upload/file/20240702/%E8%88%B5%E6%9C%BA%E5%8D%8F%E8%AE%AE%E6%89%8B%E5%86%8C-%E7%A3%81%E7%BC%96%E7%A0%81%E7%89%88%E6%9C%AC.pdf) |
| `Feetech_memory_table_magnetic_encoder.xlsx` | Official SMS/STS register map + bit definitions (Chinese, firmware 2.43) | [feetechrc.com](https://www.feetechrc.com/Data/feetechrc/upload/file/20240702/%E8%88%B5%E6%9C%BA%E5%8D%8F%E8%AE%AE%E5%86%85%E5%AD%98%E8%A1%A8-%E7%A3%81%E7%BC%96%E7%A0%81%E7%89%88%E6%9C%AC.xlsx) |
| `Feetech_wiki_STS_memory_table_EN.md` | English STS register table (derived from SDK `SMS_STS.h`) | [ftservo/ftservo-wiki@5637b79](https://github.com/ftservo/ftservo-wiki/blob/5637b7985bffb9baa3cda9d0f266be9c1714eb84/docs/en/reference/parameter/memory-sts.md) |
| `Feetech_wiki_FT-SCS_protocol_EN.md` | English protocol reference | [ftservo/ftservo-wiki@5637b79](https://github.com/ftservo/ftservo-wiki/blob/5637b7985bffb9baa3cda9d0f266be9c1714eb84/docs/en/reference/protocol/ft-scs.md) |

Frame: `0xFF 0xFF ID Len Instr Params Checksum`, checksum `~(ID+Len+Instr+Params)`, broadcast ID `0xFE`.
Instructions: PING 0x01, READ 0x02, WRITE 0x03, REG WRITE 0x04, ACTION 0x05, RECOVERY 0x06,
RESET 0x0A, SYNC READ 0x82, SYNC WRITE 0x83.

Key registers (from the xlsx, low byte first): 5 ID · 6 baud index (0 = 1 Mbps) · 33 mode ·
40 torque enable (128 = set current position as 2048) · 41 acceleration · 42 goal position ·
46 goal speed · 48 torque limit · 55 EEPROM lock · 56 present position · 58 speed · 60 load ·
62 voltage (0.1V) · 63 temperature · 69 current (6.5 mA units).
The English wiki differs from the xlsx on the default of 55 and the meaning of address 7.

## Waveshare Bus Servo Adapter (A): `waveshare-bus-servo-adapter-a/`

The USB servo driver board in the upstream BOM. Manual = web pages:
[wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) ·
[docs](https://docs.waveshare.com/Bus_Servo_Adapter_A).

| File | What | Source |
|---|---|---|
| `Waveshare_Bus_Servo_Adapter_A_schematic_docs.pdf` | Schematic, newer revision (2026-09-20) | [files.waveshare.com](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus_Servo_Adapter_A.pdf) |
| `Waveshare_Bus_Servo_Adapter_A_schematic_wiki.pdf` | Schematic, older revision (2024-08-12) | [files.waveshare.com](https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/Bus%20Servo%20Adapter%20(A)-Sch.pdf) |
| `Waveshare_Bus_Servo_Driver_Circuit_schematic_docs.pdf` | Generic bus-servo control circuit reference (rev 1.0, 2022-05-31) | [files.waveshare.com](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/Bus%20Servo%20Driver%20Circuit%20Schematic.pdf) |

From the Waveshare pages:
- **Input voltage:** the wiki says 9–12.6V; the docs site says 5–8.4V also works.
- **No regulation:** output equals input, so the supply must match the servo voltage.
- **Connectors:** a 5.5×2.1 mm DC jack plus a screw terminal.
- **Current:** the FAQ says max 5A, but the schematic labels the servo headers "MAX:6A".
- **USB-serial chip:** CH343, default 1 Mbps.
- **Jumper:** position A = UART, B = USB.

## Not available
- No Feetech-hosted C044/C046 datasheets (Feetech lists only C001 and C018).
- No English official protocol manual or memory table; only the wiki transcriptions above.
- No STEP model or post-Dec-2025 drawing for the STS3215.
- Power supply: generic barrel-jack adapter, no datasheet.
