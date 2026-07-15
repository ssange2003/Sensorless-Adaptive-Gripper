# Sensorless Adaptive Gripper — Load-Feedback Grasping on a Currentless Motor

**Grasping a 1 kg cylinder and a deformable sponge with the same gripper, using only the motor's load feedback — no force sensors, no tactile sensors, and (as hardware verification revealed) no current sensor either.**

Arduino Uno + DYNAMIXEL Shield controller for a rack-and-pinion gripper (DYNAMIXEL XC430, Protocol 2.0). Contact detection and grip holding are built on **Present Load (Addr 126)** feedback with a detect-then-freeze strategy, and grip-force ceiling via **Goal PWM (Addr 100)**.

> Built for the **2026 Physical AI Gripper Challenge** — **Silver Award (2nd Place)**, Team Union (5 members: Robotics + Computer Science).

---

## Hardware Verification — the Correction That Reshaped This README

The control logic was originally written as "current-based position control (Mode 5) with a goal/holding current." **Verifying against the ROBOTIS e-manual showed the XC430-W150 supports neither:** its control table has no Current-based Position Control mode, no Goal Current (102), no Current Limit (38) — those belong to current-sensored models (XM/XH series). Addr 126 is **Present Load (0.1 % units)**, not Present Current.

So why did the gripper work? Because the parts of the logic that mattered never depended on a current loop:

- The "800" threshold was a **peak load of 80.0 %** — a perfectly valid contact signal
- The hold was done by **freezing Goal Position at the contact point**, not by the (silently ignored) holding-current write
- The observed "290" during hold was the **load naturally settling at ~29 %** with the position frozen on the object

The corrected firmware (this repo) renames every register to what it actually is, removes the dead writes, and adds the XC430-native force ceiling: per the e-manual, lowering PWM output lowers torque — so **Goal PWM(100) is capped after contact**, the hardware-accurate equivalent of a holding current.

| | Assumed (before) | Verified (now) |
|---|---|---|
| Operating mode | 5 (Current-based Position) | **3 (Position)** — Mode 5 doesn't exist on XC430 |
| Contact signal | Present Current ≥ 800 | **Present Load ≥ 80.0 %** (Addr 126, 0.1 % units) |
| Hold mechanism | Goal Current → 290 | **Goal Position frozen at contact** + **Goal PWM cap** |
| Force ceiling | Current Limit (38) | **Goal PWM (100)** / PWM Limit (36) |

---

## Why This Gripper Design

**Design target:** grasp a 1 kg, ⌀100 mm cylinder without damaging soft objects — with motor feedback as the only signal. That constraint — *the feedback must be trustworthy* — drove the mechanical design.

### 1. Rack-and-Pinion, Not Links or Extra Gearing

A linkage or multi-stage gear train adds backlash and internal friction, both of which corrupt the load signal: friction consumes output unrelated to grip force, and backlash creates a dead zone between "motor moving" and "jaws contacting." A single-stage rack-and-pinion keeps the load-to-force relationship clean enough to detect contact from load feedback alone.

### 2. Mechanical Iteration for a Larger, Heavier Target

- Jaw opening widened **8 cm → 10 cm** for the ⌀100 mm spec; a top guide added to keep the rack from derailing at the wider stroke
- **Form closure over friction:** box grasps moved from 2-point to **4-point contact**; cylinder grasps use contact pads + silicone tape laid along the acceleration direction so added friction directly resists inertial slip

### 3. FEA-Driven Wall Reinforcement

Design load: 9.81 N static **plus 1.2 G motion → 12 N case**. SOLIDWORKS FEA located stress concentration at the rack section: wall thickness **4 mm → 7 mm**, peak Von Mises stress **−67 %**, verified against physical drive tests.

---

## Firmware: a Clean Load Signal on a Noisy Mechanism

### Direct Register Reads — Working Around Dropped Data

The library's batched/averaged read returned position and velocity but failed to return the load value. Fix: **read the register directly and assemble the raw bytes sequentially**, giving lossless real-time feedback. (Root cause not conclusively established — documented as observed behavior + workaround.)

### Detection: Peak Load, Not Steady-State Target

```cpp
if (abs(load) >= STALL_LOAD || (abs(vel) < 3 && abs(load) >= CONTACT_LOAD)) {
    if (++stallCount > 1) {                     // 2-read debounce vs gear-mesh noise
        dxl.write(MOTOR_ID, ADDR_GOAL_POSITION, (uint8_t*)&pos, 4);  // freeze at contact
        setPwm(PWM_HOLD);                       // cap output torque (e-manual: lower PWM = lower torque)
        return;
    }
}
```

An early version waited for velocity ≈ 0 at a target level — but in position control the drive toward the goal position dominates, so output spikes to the ceiling at stall regardless of contact. Watching for the **80 % peak load transient** catches contact before that, and the immediate freeze + PWM cap minimizes the crush window — the difference between holding a sponge and flattening it.

### Bluetooth Override

`SoftwareSerial` (pins 4/5) accepts `'1'` mid-close to abort and reopen — manual kill switch for testing and demos.

---

## Hardware Integration Lesson: Power Path

Uno and the DYNAMIXEL Shield share one UART; power turned out to be the real issue. External supply goes to the **shield**, sharing Vin — datasheet-confirmed as the direction that minimizes back-current and voltage-drop stress on the Uno's regulator. Learned destructively: before pulling the Vin jumper cap to separate supply paths, the Uno's regulator overheated and **took out both the Arduino and a Dynamixel**.

---

## Verified Numbers

| Item | Value | Source |
|---|---|---|
| Motor | DYNAMIXEL XC430, Protocol 2.0 | — |
| Operating mode | **3** (Position Control) | e-manual: XC430 mode list |
| Contact threshold | Present Load ≥ **800** (= 80.0 %) | Addr 126, 0.1 % units |
| Debounce | 2 consecutive reads | `stallCount` |
| Hold mechanism | Goal Position freeze + **Goal PWM cap** (start 186 / 885) | Addr 116 + Addr 100 |
| Hold-state load (observed) | ~**290** (= 29.0 %) | telemetry during hold |
| Design load case | 9.81 N + 1.2 G → **12 N** | FEA input |
| Rack wall | 4 mm → **7 mm**, Von Mises **−67 %** | SOLIDWORKS FEA |
| Jaw opening | 8 cm → **10 cm** | ⌀100 mm target |
| Payload | **1 kg** rigid → soft sponge, damage-free | competition runs |

---

## Author

**Kim Minsang (김민상)** — Robotics Engineering, Yeungnam University · Team Lead, Union
Embedded systems · Sensorless control · Mechanical design
Build log & post-mortems: https://blog.naver.com/kms031103 · LinkedIn: https://www.linkedin.com/in/kms2003