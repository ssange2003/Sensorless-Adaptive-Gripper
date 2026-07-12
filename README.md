# Sensorless Adaptive Gripper — Current-based Position Control

**Grasping a 1 kg cylinder and a deformable sponge with the same gripper, using only motor current feedback — no force or tactile sensors.**

Arduino Uno + Dynamixel Shield controller for a rack-and-pinion gripper (Dynamixel XC430, Protocol 2.0). Object contact, grip force, and object stiffness are all inferred from raw current-register reads, with a direct-register-read pipeline built to avoid the data loss of the library's default burst-read.

> Built for the **2026 Physical AI Gripper Challenge** — **Silver Award (2nd Place)**, Team Union (5 members, cross-department: Robotics + Computer Science).

---

## Why This Gripper Design

**Design target:** grasp a 1 kg payload, 100 mm-diameter cylinder, without damaging soft/deformable objects — using motor current as the only feedback signal.

That single constraint — *current must be trustworthy* — drove every mechanical and firmware decision below.

### 1. Rack-and-Pinion, Not Links or Extra Gearing

A linkage or multi-stage gear train adds backlash and internal friction. Both corrupt the current signal: friction burns current that has nothing to do with grip force, and backlash creates a dead zone between "motor moving" and "jaws actually contacting the object." A direct rack-and-pinion converts motor torque to linear grip force in a single stage — keeping the current-to-force relationship clean enough to detect contact from current alone.

### 2. Mechanical Iteration for a Larger, Heavier Target

The original design (from an earlier gripper built for a Changwon University competition) opened to 8 cm. Redesigned for the 100 mm-cylinder spec:

- Jaw opening widened **8 cm → 10 cm**
- A guide structure added at the top of the rack to prevent it from **derailing** at the wider stroke
- **Contact geometry redesigned for form closure, not just friction:**
  - Box objects: 2-point → **4-point contact**, distributing load and constraining the object geometrically
  - Cylindrical objects: **contact pads + silicone tape**, oriented along the direction of acceleration so the added friction directly resists gravity/inertial slip during motion

### 3. FEA-Driven Wall Reinforcement

Design load: static weight (9.81 N) **and** motion at 1.2 G, combined to a 12 N design case — not just the static 1 kg. SOLIDWORKS FEA under this load identified stress concentration at the rack section.

| | Before | After |
|---|---|---|
| Rack wall thickness | 4 mm | **7 mm** |
| Peak Von Mises stress | baseline | **-67%** |

---

## Firmware: Getting a Clean Current Signal

### The Problem: Dynamixel2Arduino's Default Read Drops Current Data

The library's standard bulk/averaged read returned position and velocity reliably, but **silently failed to return current** — the rack-and-pinion's current signal is noisy enough (constant small torque fluctuations from the gear mesh) that averaged/batched reads lost the data.

**Fix:** read the current register directly, unpacking the raw 4-byte value bit-by-bit in sequence instead of relying on the library's batched read. This gave lossless, real-time current feedback.

### Object Detection Logic — Peak Current, Not Target Current

```cpp
if (abs(cur) >= 800 || (abs(vel) < 3 && abs(cur) >= (targetCur - 10))) {
    if (++stallCount > 1) {
        dxl.write(MOTOR_ID, ADDR_GOAL_CURRENT, (uint8_t*)&holdCur, 2);
        dxl.write(MOTOR_ID, ADDR_GOAL_POSITION, (uint8_t*)&pos, 4);
        return;
    }
}
```

An early version watched for velocity ≈ 0 at the *target* current — but in Current-Position mode, the motor's drive toward the goal position takes priority even at zero velocity, so current still spikes to the ceiling regardless of contact. Reading a **peak current threshold (800)** instead of the steady-state target current catches the contact event before that override happens.

`stallCount` requires the condition to hold for 2 consecutive reads — a simple debounce against single-sample current noise from gear mesh vibration.

### Detect → Hold — Minimizing Crush Time on Soft Objects

Once contact is confirmed, current is immediately reduced from the closing current to a lower **holding current**, and the goal position is frozen at the current position (not the original closed target). This shortens the window where inertia keeps compressing the object after contact — the difference between crushing a sponge and just holding it.

### Bluetooth Override (Safety / Demo Interrupt)

`SoftwareSerial` on pins 4/5 listens for a `'1'` byte during `safeClose()`; receiving it aborts the grip cycle and calls `safeOpen()` immediately — a manual kill switch during testing/demo.

---

## Hardware Integration Lesson: Power, Not Just Firmware

Arduino Uno and the Dynamixel Shield share a single UART. The first assumption was that power also had to be fully isolated between them — but a shared ground with a single external supply proved stable.

**What did matter:** feed order. Powering shield→Arduino (rather than Arduino→shield) reduces back-current stress and voltage-drop on the Arduino's regulator, confirmed against the datasheet, since the Arduino side runs at lower voltage/current than the shield.

**What was learned the hard way:** without adequate isolation from the Vin jumper, the Arduino's onboard voltage regulator couldn't dissipate the heat from the shield's current draw — it overheated and **took out both the Arduino and a Dynamixel** before this was diagnosed. Fix: pull the Vin jumper cap to separate the supply paths.

---

## Design Evolution — Operating Mode

| Stage | Mode | Result |
|---|---|---|
| 1 | Position (jaws open/close to fixed positions) | worked, but no force awareness |
| 2 | Naive Current control, safety-factor 2.2 on a static force model | current too unstable for reliable grasp |
| 3 | **Current-based Position Control (Mode 5)**, target/hold current tuned from a 1st-order force model, safety factor 2 (final friction ceiling set by silicone-tape testing) | stable, adaptive grasp — this repo |

---

## Verified Numbers

| Item | Value | Source |
|---|---|---|
| Operating Mode | 5 — Current-based Position Control | `ADDR_OPERATING_MODE = 11` |
| Stall/contact detection threshold | **800** (current register) | `turtlebot3`-style peak-current logic, `safeClose()` |
| Debounce | 2 consecutive reads (`stallCount`) | noise filtering |
| Design load | 9.81 N (static) + 1.2 G motion → **12 N** | FEA load case |
| Rack wall thickness | 4 mm → **7 mm** | FEA iteration |
| Peak Von Mises stress reduction | **67%** | SOLIDWORKS FEA, before/after |
| Jaw opening | 8 cm → **10 cm** | mechanical redesign for 100 mm cylinder |
| Target payload | 1 kg, ⌀100 mm cylinder | competition spec |
| Max motor current avoided | 1.4 A (motor rated max) | current-limit design margin |

> **Note on current values in code:** `targetCur`/`holdCur` are set to `111` in this snippet; the design narrative above describes a tuned hold current of 290. If this file is a pre-tuning test version, update the table/code before publishing final numbers — the stall-detection threshold (800) is confirmed consistent between code and design notes.

---

## Author

**Kim Minsang (김민상)** — Robotics Engineering, Yeungnam University · Team Lead, Union
Embedded systems · Sensorless control · Mechanical design
Build log: https://blog.naver.com/kms031103 · LinkedIn: https://www.linkedin.com/in/kms2003
