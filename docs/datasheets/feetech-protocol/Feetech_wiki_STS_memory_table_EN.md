# Magnetically Encoded STS Servo - Memory Table

For STS half-duplex TTL series servos. Two-byte fields are transmitted with the **low byte first**.

SMS servos (RS485) use different address definitions; see [SMS Memory Table](memory-sms.md).

<style>
  .md-typeset table:not([class]) { display: table; width: 100%; table-layout: fixed; }
  .md-typeset table:not([class]) th { min-width: 0; padding: .6em .4em; }
  .md-typeset table:not([class]) td { min-width: 0; padding: .6em .4em; }
  .md-typeset table:not([class]) td, .md-typeset table:not([class]) th { overflow-wrap: anywhere; }
  .md-typeset table:not([class]) th:nth-child(1), .md-typeset table:not([class]) td:nth-child(1) { width: 46px; }
  .md-typeset table:not([class]) th:nth-child(2), .md-typeset table:not([class]) td:nth-child(2) { width: 46px; }
  .md-typeset table:not([class]) th:nth-child(3), .md-typeset table:not([class]) td:nth-child(3) { width: 54px; }
  .md-typeset table:not([class]) th:nth-child(4), .md-typeset table:not([class]) td:nth-child(4) { width: 40px; }
  .md-typeset table:not([class]) th:nth-child(5), .md-typeset table:not([class]) td:nth-child(5) { width: 72px; }
  .md-typeset table:not([class]) th:nth-child(6), .md-typeset table:not([class]) td:nth-child(6) { width: 30px; }
  .md-typeset table:not([class]) th:nth-child(7), .md-typeset table:not([class]) td:nth-child(7) { width: 50px; }
  .md-typeset table:not([class]) th:nth-child(8), .md-typeset table:not([class]) td:nth-child(8) { width: 56px; }
  /* §4 workflow tables: column widths by content, not the 9-column memory table widths */
  .md-typeset .ft-t4 table:not([class]) th:nth-child(1), .md-typeset .ft-t4 table:not([class]) td:nth-child(1) { width: 8%; }
  .md-typeset .ft-t4 table:not([class]) th:nth-child(3), .md-typeset .ft-t4 table:not([class]) td:nth-child(3) { width: 22%; }
  .md-typeset .ft-t3a table:not([class]) th:nth-child(1), .md-typeset .ft-t3a table:not([class]) td:nth-child(1) { width: 28%; }
  .md-typeset .ft-t3a table:not([class]) th:nth-child(3), .md-typeset .ft-t3a table:not([class]) td:nth-child(3) { width: 18%; }
  .md-typeset .ft-t3b table:not([class]) th:nth-child(1), .md-typeset .ft-t3b table:not([class]) td:nth-child(1) { width: 14%; }
  .md-typeset .ft-t3b table:not([class]) th:nth-child(2), .md-typeset .ft-t3b table:not([class]) td:nth-child(2) { width: 34%; }
  @media screen and (max-width: 40em) {
    .md-typeset table:not([class]) { table-layout: auto; }
    .md-typeset table:not([class]) th, .md-typeset table:not([class]) td { width: auto; }
  }
</style>

## 1 Servo Communication Protocol

Servos use the FT-SCS proprietary protocol. Factory serial defaults: the STS default baud rate is 1 Mbps over a TTL single-bus, 8 data bits, no parity, 1 stop bit; configurable baud rate range 38400~1 Mbps, default communication address (station number) 1.

For the full frame format and instruction set, see [FT-SCS Protocol](../protocol/ft-scs.md).

## 2 Servo Memory Table Definition

If a function address uses two bytes of data, the low byte is at the leading address and the high byte at the following address

**Notes:**

- Initial values and ranges vary by model (shown as – in the table); refer to the data for your model.
- "Minimum resolution angle" here is the smallest angle the encoder can resolve: the series divides a full turn of 360° into 4096 parts, each 360° ÷ 4096 ≈ 0.0879°, which is the value shown in the Unit column; position and angle-limit registers count in this unit, per your model's data.
- There are two speed units: 0.732RPM and 0.0146RPM (the latter is 1/50 of the former — it only gives finer command granularity). The smallest physical speed change is still 0.732RPM (one step = 0.0879° every 20 ms): in the fine unit you must write 50 to get one physical step, and writing 1~49 produces no actual speed change.

### 2.1 Version Information

| Address (DEC) | Address (HEX) | Function | Bytes | Initial value | Access | Range | Unit | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="reg-0"></a>0 | 0x00 | Firmware major version | 1 | – | R |  |  |  |
| <a id="reg-1"></a>1 | 0x01 | Firmware minor version | 1 | – | R |  |  |  |
| <a id="reg-2"></a>2 | 0x02 | END | 1 | 0 | R |  |  | 0 indicates little-endian storage structure |
| <a id="reg-3"></a>3 | 0x03 | Servo major version | 1 | – | R |  |  |  |
| <a id="reg-4"></a>4 | 0x04 | Servo minor version | 1 | – | R |  |  |  |

### 2.2 EEPROM Configuration

| Address (DEC) | Address (HEX) | Function | Bytes | Initial value | Access | Range | Unit | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="reg-5"></a>5 | 0x05 | Servo ID | 1 | 1 | R/W | 0~253 | ID | Unique main ID on the bus |
| <a id="reg-6"></a>6 | 0x06 | Baud rate | 1 | 0 | R/W | 0~7 | None | 0-7 correspond to baud rates: 1000000(0), 500000(1), 250000(2), 128000(3), 115200(4), 76800(5), 57600(6), 38400(7) |
| <a id="reg-7"></a>7 | 0x07 | Reserved | 1 | – | R/W | – | – |  |
| <a id="reg-8"></a>8 | 0x08 | Response status level | 1 | 1 | R/W | 0~1 | None | 0: no response packet for instructions other than READ and PING; 1: response packet for all instructions |
| <a id="reg-9"></a>9 | 0x09 | Minimum angle limit | 2 | 0 | R/W | 0~4094 | 0.0879° | Sets the minimum running angle limit; the value must be smaller than the maximum angle limit; minimum angle limit = maximum angle limit = 0 enters multi-turn absolute position control mode |
| <a id="reg-11"></a>11 | 0x0B | Maximum angle limit | 2 | 4095 | R/W | 0~4095 | 0.0879° | Sets the maximum running angle limit; the value must be larger than the minimum angle limit; minimum angle limit = maximum angle limit = 0 enters multi-turn absolute position control mode |
| <a id="reg-13"></a>13 | 0x0D | Maximum temperature limit | 1 | 70 | R/W | 0~100 | °C | Temperature threshold of the overheat protection |
| <a id="reg-14"></a>14 | 0x0E | Maximum input voltage | 1 | – | R/W | 0~254 | 0.1V | Voltage threshold of the over-voltage protection |
| <a id="reg-15"></a>15 | 0x0F | Minimum input voltage | 1 | 40 | R/W | 0~254 | 0.1V | Voltage threshold of the under-voltage protection (the default suits low-voltage models; raise it for higher-voltage models) |
| <a id="reg-16"></a>16 | 0x10 | Maximum torque | 2 | 1000 | R/W | 0~1000 | 0.1% | Written to address 48 (torque limit) at power-on |
| <a id="reg-18"></a>18 | 0x12 | Phase | 1 | – | R/W | 0~254 | None | Special function byte; do not modify unless specifically required |
| <a id="reg-19"></a>19 | 0x13 | Unload condition | 1 | – | R/W | 0~254 | None | Set bit to 1 to enable the corresponding protection, set bit to 0 to disable it; bit definitions in [3.3 Unload Condition](#33-unload-condition) |
| <a id="reg-20"></a>20 | 0x14 | LED alarm condition | 1 | – | R/W | 0~254 | None | Set bit to 1 to enable the flashing LED alarm, set bit to 0 to disable the flashing LED alarm; bit definitions in [3.4 LED Alarm Condition](#34-led-alarm-condition) |
| <a id="reg-21"></a>21 | 0x15 | Position loop P (proportional) coefficient | 1 | – | R/W | 0~254 | None | Proportional coefficient controlling the motor (1/4) |
| <a id="reg-22"></a>22 | 0x16 | Position loop D (derivative) coefficient | 1 | – | R/W | 0~254 | None | Derivative coefficient controlling the motor (1/8) |
| <a id="reg-23"></a>23 | 0x17 | Position loop I (integral) coefficient | 1 | 0 | R/W | 0~254 | None | Integral coefficient controlling the motor |
| <a id="reg-24"></a>24 | 0x18 | Minimum startup torque | 1 | – | R/W | 0~254 | 0.1% | Sets the minimum output startup torque of the servo |
| <a id="reg-25"></a>25 | 0x19 | Integral limit | 1 | 0 | R/W | 0~254 | None | Maximum integral = limit × 4; 0 disables the integral limit; effective in position mode 0 and mode 4 (mode 4 is a custom build, only on special models) |
| <a id="reg-26"></a>26 | 0x1A | Positive deadband | 1 | 1 | R/W | 0~16 | 0.0879° | The minimum unit is one minimum resolution angle |
| <a id="reg-27"></a>27 | 0x1B | Negative deadband | 1 | 1 | R/W | 0~16 | 0.0879° | The minimum unit is one minimum resolution angle |
| <a id="reg-28"></a>28 | 0x1C | Protection current | 2 | 511 | R/W | 0~2047 | 6.5mA | The maximum settable current is 500 * 6.5mA = 3250mA (a suggested upper limit; follow your actual model) |
| <a id="reg-30"></a>30 | 0x1E | Angle resolution | 1 | 1 | R/W | 1~128 | None | Magnification factor of the sensor's minimum resolution angle: set to K, one count = K × 0.0879° and the full 0~4095 range covers K turns; a larger K means more turns but a coarser step |
| <a id="reg-31"></a>31 | 0x1F | Position offset | 2 | 0 | R/W | 0~8191 | 0.0879° | 0~2047 represents 0~2047; 2048~4095 represents 0~-2047; 4096~6143 represents 2048~4095; 6144~8191 represents -2048~-4095 (that is, an offset of -4095~4095) |
| <a id="reg-33"></a>33 | 0x21 | Operating mode | 1 | 0 | R/W | 0~3 | None | 0: position servo mode; 1: constant-speed motor mode; 2: PWM open-loop speed-control mode; 3: step mode |
| <a id="reg-34"></a>34 | 0x22 | Holding torque | 1 | 20 | R/W | 0~254 | 1% | Torque output after entering overload protection; for example 20 means 20% of the maximum torque |
| <a id="reg-35"></a>35 | 0x23 | Protection time | 1 | 200 | R/W | 0~254 | 10ms | Duration for which the load output exceeds the overload torque and is held; for example 200 means 2 seconds, maximum 2.54 seconds |
| <a id="reg-36"></a>36 | 0x24 | Overload torque | 1 | 80 | R/W | 0~254 | 1% | Maximum torque threshold that starts the overload protection timer; for example 80 means 80% of the maximum torque |
| <a id="reg-37"></a>37 | 0x25 | Velocity loop P (proportional) coefficient | 1 | – | R/W | 0~254 | None | Speed-loop proportional coefficient in constant-speed motor mode (mode 1) |
| <a id="reg-38"></a>38 | 0x26 | Overcurrent protection time | 1 | 200 | R/W | 0~254 | 10ms | Maximum settable 254 * 10ms = 2540ms |
| <a id="reg-39"></a>39 | 0x27 | Velocity loop I (integral) coefficient | 1 | – | R/W | 0~254 | None | Speed-loop integral coefficient in constant-speed motor mode (mode 1) |

### 2.3 SRAM Control

| Address (DEC) | Address (HEX) | Function | Bytes | Initial value | Access | Range | Unit | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="reg-40"></a>40 | 0x28 | Torque switch | 1 | 0 | R/W | 0~2 | None | Write 0: torque output off; write 1: torque output on; write 2: damping output (torque limit (48) controls the damping force); write 128: calibrate the current position to 2048 |
| <a id="reg-41"></a>41 | 0x29 | Acceleration | 1 | 0 | R/W | 0~254 | 8.79°/s² | Run acceleration/deceleration of the servo; 0 means maximum acceleration |
| <a id="reg-42"></a>42 | 0x2A | Goal position | 2 | 0 | R/W | -32767~32767 | 0.0879° | Absolute position control; the maximum corresponds to the maximum effective angle; BIT15 is the direction bit; ±32767 is the conceptual range — what you actually write is the magnitude (0~32767), and set BIT15 to 1 for the reverse direction |
| <a id="reg-44"></a>44 | 0x2C | PWM open-loop speed | 2 | 1000 | R/W | -1000~1000 | 0.1% | Effective in PWM open-loop speed mode, where it is the PWM duty cycle; write the magnitude (0~1000) and set BIT10 to 1 for the reverse direction |
| <a id="reg-46"></a>46 | 0x2E | Running speed | 2 | Factory default maximum speed | R/W | -32767~32767 | 0.732RPM/0.0146RPM | Controls the maximum running speed of the motor; BIT15 is the direction bit; 0 means the maximum speed by default and can be set to mean stop via the phase setting; the unit follows the phase setting, either 0.732RPM or 0.0146RPM (decided by phase BIT2); 0.0146RPM simply splits the 0.732RPM physical step into 50 command levels, so the physical resolution stays 0.732RPM |
| <a id="reg-48"></a>48 | 0x30 | Torque limit | 2 | Maximum torque (16), default 1000 | R/W | 0~1000 | 0.1% | This value can be modified in software to control the stall torque output |
| 50~54 | 0x32~0x36 | Undefined | 5 | – | R/W | – | – |  |
| <a id="reg-55"></a>55 | 0x37 | Lock flag | 1 | 1 | R/W | 0~1 | None | Write 0 to close the write lock: values written to EEPROM addresses persist after power-off; write 1 to open the write lock: values written to EEPROM addresses do not persist |

### 2.4 SRAM Feedback

| Address (DEC) | Address (HEX) | Function | Bytes | Initial value | Access | Range | Unit | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| <a id="reg-56"></a>56 | 0x38 | Present position | 2 | – | R | – | 0.0879° | Absolute position feedback of the servo; BIT15 is the direction bit; in step mode 3 it returns the step difference between the present position and the goal position, with BIT15 as the direction bit |
| <a id="reg-58"></a>58 | 0x3A | Present speed | 2 | – | R | – | 0.732RPM/0.0146RPM | Rotational speed of the motor; the unit follows the phase setting; BIT15 is the direction bit |
| <a id="reg-60"></a>60 | 0x3C | Present load | 2 | – | R | – | 0.1% | Duty cycle of the output driving the motor; BIT10 is the direction bit |
| <a id="reg-62"></a>62 | 0x3E | Present voltage | 1 | – | R | – | 0.1V | Current working voltage of the servo |
| <a id="reg-63"></a>63 | 0x3F | Present temperature | 1 | – | R | – | °C | Current internal working temperature of the servo |
| <a id="reg-64"></a>64 | 0x40 | Async write flag | 1 | 0 | R | – | None | Flag used with the async write instruction: set to 1 as soon as the async write instruction (0x04, unicast or broadcast) is accepted, cleared to 0 as soon as the execution instruction (0x05) is accepted; a direct write (0x03) does not change it; it is a RAM bit and clears at power-off. |
| <a id="reg-65"></a>65 | 0x41 | Servo status | 1 | 0 | R | – | None | A bit set to 1 indicates the corresponding error |
| <a id="reg-66"></a>66 | 0x42 | Moving flag | 1 | 0 | R | – | None | 1 while the servo is moving, 0 when the target is reached and the servo stops; stays 0 when no new goal position is written |
| <a id="reg-67"></a>67 | 0x43 | Goal position | 2 | 0 | R | – | 0.0879° | Current goal position |
| <a id="reg-69"></a>69 | 0x45 | Present current | 2 | – | R | – | 6.5mA | Motor phase current feedback; BIT15 is the current direction bit |
| <a id="reg-71"></a>71 | 0x47 | Undefined | 2 | – | R | – | – |  |

### 2.5 Factory Parameters

| Address (DEC) | Address (HEX) | Function | Bytes | Initial value | Access | Range | Unit | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 80 | 0x50 | Moving speed threshold | 1 | – | R | – | – |  |
| 81 | 0x51 | DTs(ms) | 1 | – | R | – | – |  |
| 82 | 0x52 | Speed unit coefficient | 1 | – | R | – | – |  |
| 83 | 0x53 | Minimum speed limit | 1 | – | R | – | – | Unit 0.732rpm (follows phase BIT2) |
| 84 | 0x54 | Maximum speed limit | 1 | – | R | – | – | Unit 0.732rpm (follows phase BIT2) |
| 85 | 0x55 | Acceleration limit | 1 | – | R | – | – |  |
| 86 | 0x56 | Acceleration multiplier | 1 | – | R | – | – | The acceleration multiplier takes effect when acceleration is 0; when both the multiplier and the acceleration are 0 the servo responds at the highest speed |

## 3 Special Byte Explanation

### 3.1 Servo Phase

**Bit (weight)**: Description

- BIT0 (1): Drive direction phase; (0) forward, (1) reverse
- BIT1 (2): set to 1 on STSBL servos; has no effect on STSCL servos
- BIT2 (4): Speed unit, (0) 0.732RPM, (1) 0.0146RPM
- BIT3 (8): Speed mode, (0) speed 0 means stop, (1) speed 0 means the highest speed
- BIT4 (16): Angle feedback mode, (0) single-turn angle feedback, (1) full-angle feedback
- BIT5 (32): Driver bridge configuration, (0) independent H-bridge, (1) integrated H-bridge/no current feedback
- BIT6 (64): PWM frequency, (0) 24kHz, (1) 16kHz
- BIT7 (128): Position feedback direction phase, (0) forward, (1) reverse

!!! tip "Note"
    If multiple bits are set at the same time, the phase value is the sum of the bit values. Example: with an original phase value of 0, if the servo runs in reverse, the phase value is 128+1=129;

### 3.2 Servo Status

Servo status: 0 means normal, 1 means abnormal

**Bit (weight)**: Description

- BIT0 (1): Voltage status
- BIT1 (2): Magnetic encoder status
- BIT2 (4): Temperature status
- BIT3 (8): Current status
- BIT4 (16): -----
- BIT5 (32): Load status
- BIT6 (64): -----
- BIT7 (128): -----

!!! tip "Note"
    If multiple statuses occur at the same time, the status value is the sum of the bit values. Example: over-voltage/under-voltage together with servo overheating gives a status value of 4+1=5;

### 3.3 Unload Condition

Unload condition: 0 means off, 1 means on

**Bit (weight)**: Description

- BIT0 (1): Voltage protection
- BIT1 (2): Magnetic encoder protection
- BIT2 (4): Overheat protection
- BIT3 (8): Overcurrent protection
- BIT4 (16): -----
- BIT5 (32): Load overload
- BIT6 (64): -----
- BIT7 (128): -----

!!! tip "Note"
    If multiple bits are set at the same time, the unload condition value is the sum of the bit values. Example: with voltage protection and overheat protection enabled together, the unload condition value is 4+1=5;

### 3.4 LED Alarm Condition

LED alarm condition: 0 means off, 1 means on

**Bit (weight)**: Description

- BIT0 (1): Voltage alarm
- BIT1 (2): Magnetic encoder alarm
- BIT2 (4): Overheat alarm
- BIT3 (8): Overcurrent alarm
- BIT4 (16): -----
- BIT5 (32): Load overload alarm
- BIT6 (64): -----
- BIT7 (128): -----

!!! tip "Note"
    If multiple bits are set at the same time, the LED alarm condition value is the sum of the bit values. Example: with the voltage alarm and the overheat alarm enabled together, the alarm condition value is 4+1=5;

!!! warning "Do not reuse addresses across families"
    Other series may use the same address with a different meaning (for example, SMS address 7 is the response return delay, while in this table it is reserved). Return to [Bus Protocol](../protocol/index.md) and select the matching memory table.

## 4 Common Workflows

Once the connection is up, day-to-day work on the servo comes down to reading and writing a handful of registers. This section collects the common ones by task; how to build the instruction frames is out of scope here — see [FT-SCS Protocol](../protocol/ft-scs.md). All numbers below refer to the addresses in this table.

### 4.0 Before You Start

- EEPROM area (2.2, 5~39): write [55 Lock flag](#reg-55) = 0 before changing anything, and write 1 back afterwards, otherwise the value is not saved. Measured: **with the lock flag at 1 a write is still accepted and takes effect immediately, but it does not survive power-off**; enabling ([40 Torque switch](#reg-40)) does not affect saving.
- SRAM area (2.3, 40~55): write directly, effective immediately, lost at power-off — good for trial writes; [55 Lock flag](#reg-55) itself lives here, so writing it needs no unlock.
- 56~71 in 2.4 and 80~86 in 2.5 are read-only; reading them does not change the servo state.
- Position registers in this series count in "minimum resolution angle": by default (angle resolution in 30 = 1) a full turn is 4096 counts and each count is 0.0879° (360° ÷ 4096) — that is the "count" used below; with 30 set to K one count becomes K × 0.0879°.

### 4.1 Getting the Servo Moving

**Goal:** turn the output shaft through a small angle for the first time.

<div class="ft-t4" markdown>

| Step | Action | Register | Value / example |
|---|---|---|---|
| 1 | Read the present position, mask BIT15 (direction), call it P | [56 Present position](#reg-56) | read-only, e.g. 2048 |
| 2 | Drop the torque limit to 30% | [48 Torque limit](#reg-48) | 300 (power-on default comes from 16 = 1000) |
| 3 | Set the running speed | [46 Running speed](#reg-46) | 100 (100 × 0.732 RPM ≈ 73 RPM) |
| 4 | Turn the torque output on (can be skipped) | [40 Torque switch](#reg-40) | 1 |
| 5 | Write the goal position | [42 Goal position](#reg-42) | P + 200 (about 17.6°) |

</div>

**Turning by angle**

[42 Goal position](#reg-42) takes an absolute position, so "turn 90°" first needs a reference point. One count is 0.0879°, and 90° is exactly 1024 counts.

<div class="ft-t3a" markdown>

| Desired result | How to work it out | Worked example |
|---|---|---|
| Turn another 90° from the present position (recommended) | read [56 Present position](#reg-56) as P (mask BIT15), goal = P ± (90° ÷ 0.0879°) | P ± 1024 |
| Turn 90° from the start of the travel | goal = 90° ÷ 0.0879° | 1024 |

</div>

- Try P + 1024 first; if it turns the wrong way, use P − 1024. Which side adds depends on how the horn is mounted.
- If the result is negative, write its absolute value and set BIT15 (direction bit) in [42 Goal position](#reg-42).
- The goal must stay within [9 Minimum angle limit](#reg-9) / [11 Maximum angle limit](#reg-11) (see 4.3).

**Notes:**

- [48 Torque limit](#reg-48) is in SRAM: effective immediately, lost at power-off, and re-loaded from [16 Maximum torque](#reg-16) at power-on, so 48 is the easy one to change for a trial; use 16 only when the value must persist (EEPROM, so unlock first).
- The unit of [46 Running speed](#reg-46) is decided by BIT2 of [18 Phase](#reg-18) (0.732 RPM / 0.0146 RPM); what 0 means is decided by BIT3: with BIT3=1, 0 means the highest speed; with BIT3=0, 0 means stop (bit definitions in [3.1 Servo Phase](#31-servo-phase)).
- Besides 0/1, [40 Torque switch](#reg-40) also takes 2 (damping output) and 128 (calibrate the present position to 2048, see 4.2); for the first trial, 1 is all you need.
- Writing [42 Goal position](#reg-42) directly sets [40 Torque switch](#reg-40) to 1 automatically, so step 4 can be skipped.

**Check:** while moving, [66 Moving flag](#reg-66) is 1 and returns to 0 on arrival; [56 Present position](#reg-56) (mask BIT15) should then equal the magnitude of the goal (or the boundary value if clamped by the travel range).

### 4.2 Midpoint Calibration

**Goal:** make goal position 2048 correspond to the pose in which the horn is mounted square. The magnetic encoder zero is fixed to the factory shaft angle, and mounting a horn usually introduces an offset — this step puts the zero back where it belongs.

**Register:** [40 Torque switch](#reg-40) (write 128).

**Steps:**

- First turn the output shaft by hand to the pose in which the horn should sit square (you can take the horn off, calibrate, then refit it).
- Write [40 Torque switch](#reg-40) = 128: the servo then treats **the present position** as 2048 (half a turn).

**Notes:**

- The shaft should be unloaded and still during calibration; every execution turns "wherever it is now" into 2048, so repeating it accumulates offset.
- After calibration the whole 0~4095 zero point shifts: position readings taken before are invalid. If the travel limits are still the factory 0 / 4095 (full turn), they still cover a full turn after the shift and need no change; if you set a custom range, the values in [9 Minimum angle limit](#reg-9) / [11 Maximum angle limit](#reg-11) now refer to the shifted physical angle and must be set again against the new zero (see 4.3).
- After a power cycle, read [56 Present position](#reg-56) again to confirm the calibration held; [31 Position offset](#reg-31) is computed by the firmware during calibration and written for you (EEPROM, so it persists) — no need to edit it by hand.

**Check:** after writing, [56 Present position](#reg-56) should read 2048; then write [42 Goal position](#reg-42) = 2048 and the shaft should not move at all.

### 4.3 Setting the Travel Range

**Goal:** limit how far the output shaft may turn.

**Registers:** [9 Minimum angle limit](#reg-9) and [11 Maximum angle limit](#reg-11), in minimum resolution angle (0.0879°); both are in the EEPROM area, so unlock [55 Lock flag](#reg-55) before writing.

**Steps:**

- Convert the boundaries you want into counts (angle ÷ 0.0879°, see "Turning by angle" in 4.1).
- Write [9](#reg-9) and [11](#reg-11); 9 must be smaller than 11; write [55 Lock flag](#reg-55) = 0 first and set it back to 1 afterwards.

**Notes:**

- Factory defaults are 9 = 0 and 11 = 4095, i.e. a full turn.
- Setting both 9 and 11 to 0 enters multi-turn absolute position control, which is no longer bound to a single turn — an advanced use; to return to normal mode, write both limits again.
- The limits only constrain goal position commands; an external force can still push the shaft past them.

**Check:** first set [11 Maximum angle limit](#reg-11) to 2048 (EEPROM, so unlock first), then write [42 Goal position](#reg-42) = 3000; the shaft should stop at 2048 counts and go no further.

### 4.4 Changing the ID

**Goal:** change the bus address of the servo; do this before building a multi-servo bus.

**Registers:** [5 Servo ID](#reg-5), [55 Lock flag](#reg-55).

**Pitfalls:**

- Only write it when this is the only servo on the bus; with several servos, power them one at a time.
- Writing through the broadcast address changes every servo on the bus at once.
- ID range 0~253; never duplicate an ID on the same bus.
- After a successful write the servo answers on the new ID at once and no longer answers on the old one; the re-lock and any PING afterwards must use the new ID in the ID byte (no need to reopen the port) — sending with the old ID only times out.

**Steps:** confirm this is the only servo on the bus → write [55](#reg-55) = 0 → write [5](#reg-5) = new ID → **using the new ID** write [55](#reg-55) = 1.

**Check:** a PING with the new ID (or a read of [5](#reg-5)) should answer, and the old ID should not. For the broadcast method see [FT-SCS Protocol](../protocol/ft-scs.md).

### 4.5 Changing the Baud Rate

**Goal:** change the communication baud rate to match the host. Factory default is 1 Mbps (gear 0).

**Registers:** [6 Baud rate](#reg-6), [55 Lock flag](#reg-55).

**Pitfalls:**

- What you write is the **gear number 0~7, not the baud rate itself** (0=1000000, 1=500000, 2=250000 … full mapping in row 6 of the 2.2 table).
- **The current connection drops the moment the write succeeds**: the servo switches to the new gear at once and stops answering on the old rate, so reconnect at the new rate (change the rate and reopen the port in your serial tool, or reinitialise the UART on the host) before sending anything else — the re-lock included; with a wrong gear you can only retry gear by gear or restore the factory default.
- The host and every servo on the bus must use the same gear; a mixed bus does not communicate.

**Steps:** unlock as described in [4.0 Before You Start](#40-before-you-start) → write [6](#reg-6) = new gear (**the connection drops here**) → reconnect at the new rate → write [55](#reg-55) = 1.

**Check:** reconnect at the new rate and send a PING; an answer is all you need. For lost connections see [Troubleshooting](../../troubleshooting.md).

### 4.6 Configuring Protection

**Goal:** set temperature, voltage, current and overload protection so a stall does not damage the servo.

**Registers:**

<div class="ft-t3b" markdown>

| Role | Register | Notes |
|---|---|---|
| Temperature threshold | [13 Maximum temperature limit](#reg-13) | °C, default 70; crossing it sets BIT2 of [65](#reg-65) |
| Voltage threshold | [14 Maximum input voltage](#reg-14) / [15 Minimum input voltage](#reg-15) | 0.1 V; 15 defaults to 40, i.e. 4.0 V (set for low-voltage models; raise it for higher-voltage models) |
| Current threshold | [28 Protection current](#reg-28) / [38 Overcurrent protection time](#reg-38) | 28 is 6.5 mA per count (default 511 ≈ 3.3 A), 38 is 10 ms per count (default 200 = 2 s) |
| Whether to unload the output | [19 Unload condition](#reg-19) | bit-wise enable of voltage, magnetic encoder, overheat, overcurrent and load, with bit definitions in [3.3 Unload Condition](#33-unload-condition) |
| Whether to flash the LED | [20 LED alarm condition](#reg-20) | a separate channel; it flashes only on models with an LED, bit definitions in [3.4 LED Alarm Condition](#34-led-alarm-condition) |
| Overload torque drop | [36 Overload torque](#reg-36) / [35 Protection time](#reg-35) / [34 Holding torque](#reg-34) | once the load exceeds the torque in 36 (80 = 80%) for the time in 35 (200 = 2 s), the output torque drops to the holding torque in 34 (20 = 20%) |

</div>

All of the above are in the EEPROM area (2.2), so unlock [55 Lock flag](#reg-55) before writing.

**Key points:**

- Temperature, voltage and current protection need both conditions to fire: the matching bit of [19 Unload condition](#reg-19) set to 1 (that item enabled) **plus** the measured value crossing the threshold in [13](#reg-13)~[15](#reg-15) or [28](#reg-28) → that protection acts (unloads the output); with the bit at 0 it never acts.
- Overload is a separate chain: once the load exceeds [36 Overload torque](#reg-36) for [35 Protection time](#reg-35), the output torque drops to [34 Holding torque](#reg-34); whether the output is also unloaded is decided by BIT5 (load overload) of [19](#reg-19).
- [65 Servo status](#reg-65) reflects the detection layer: the bit is set as soon as the threshold is crossed, whether or not [19](#reg-19) is enabled; [20 LED alarm condition](#reg-20) is another independent channel — an alarm on 20 never substitutes for unloading on 19.
- BIT1 (magnetic encoder protection) of [19](#reg-19) has no threshold register: it fires when the sensor itself reports an error and does not depend on the values in 13~15 or 28.
- [13](#reg-13)~[15](#reg-15) and [28](#reg-28) are **thresholds, not switches**: writing 0 to one of them does not disable that protection.

**Check:** confirm the matching bit of [19](#reg-19) is set to 1, then cross a threshold on purpose (or stall the servo) and read [65](#reg-65): the matching bit turns 1 and the output follows the protection; both return to 0 once the condition clears.

### 4.7 Reading Status to Troubleshoot

**Goal:** locate a problem with read-only registers when the servo misbehaves.

**Registers:** [56 Present position](#reg-56), [58 Present speed](#reg-58), [60 Present load](#reg-60), [62 Present voltage](#reg-62), [63 Present temperature](#reg-63), [65 Servo status](#reg-65), [66 Moving flag](#reg-66), [67 Goal position](#reg-67), [69 Present current](#reg-69); all read-only, reading them at will does not affect the servo.

**Order to check:**

- Read [65 Servo status](#reg-65): if it is not 0, decode it bit by bit with [3.2 Servo Status](#32-servo-status) (1 = voltage, 2 = magnetic encoder, 4 = temperature, 8 = current, 32 = load).
- Read [62 Present voltage](#reg-62) / [63 Present temperature](#reg-63): compare them with the thresholds set in [13](#reg-13)~[15](#reg-15).
- Read [69 Present current](#reg-69): mask BIT15 (current direction bit), multiply by 6.5 mA and see how close it is to the threshold in [28 Protection current](#reg-28).
- Read [60 Present load](#reg-60): mask BIT10 (direction bit) first, then scale 0~1000; a value close to 100% for a long time means a stall or too little torque.
- Read [56](#reg-56) / [67](#reg-67) / [66](#reg-66): compare the present position with the goal position and check the moving flag to see whether the command was executed; mask BIT15 (direction) in both [56](#reg-56) and [58](#reg-58).

## Source Definition

- [Official `SMS_STS` header](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.h)
