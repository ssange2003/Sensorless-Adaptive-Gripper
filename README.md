# Sensorless Adaptive Gripper — 1 kg Rigid to Soft Sponge, Motor Feedback Only

**A rack-and-pinion gripper that grasps a 1 kg cylinder and a deformable sponge with the same hardware — no force or tactile sensors. Contact is detected from motor load feedback; grip force is capped in firmware.**

Arduino Uno + DYNAMIXEL Shield, DYNAMIXEL XC430 (Protocol 2.0).

> **2026 Physical AI Gripper Challenge — Silver Award (2nd Place)**, Team Union (5 members: Robotics + Computer Science).

---

## How It Works

```
[1] CLOSE — full effort
    Position PID drives toward posClose
    at PWM_FULL
              │
              ▼
[2] CONTACT — load signature
    jaws blocked
    → Present Load ≥ 80 % (raw 800)
              │
              ▼
[3] DEBOUNCE
    condition holds for 2 reads
    (~40–60 ms)
              │
              ▼
[4] HOLD
    ① Goal Position := current position
       (freeze at contact)
    ② Goal PWM := PWM_HOLD
       (torque ceiling)
```

1. **Full-effort close.** In Position Control Mode (3), the internal PID drives the jaws toward `posClose` at full output (`PWM_FULL`).
2. **Contact = load signature.** Detection uses two paths, covering both object classes:
   - **Rigid objects** block the jaws instantly — **Present Load (Addr 126, 0.1 % units)** spikes past **800 (= 80.0 %)**.
   - **Soft objects** never spike; load rises gradually while velocity collapses — caught by `|vel| < 3 AND load ≥ CONTACT_LOAD`.
3. **Debounce.** The condition must hold for 2 consecutive reads (`stallCount`), rejecting single-sample spikes from gear-mesh vibration (~40–60 ms confirmation).
4. **Freeze + cap.** Goal Position is overwritten with the *current* position — the PID's position error collapses to ~0 and effort settles (hold-state load ≈ 290 = 29 %). **Goal PWM (Addr 100)** is then lowered to `PWM_HOLD`, putting a hard ceiling on torque. The freeze lands within ~40–60 ms of contact — the short crush window is what keeps a sponge un-flattened.
5. **Empty-grip recovery.** Reaching `posClose` without a detection means nothing was grasped → automatic reopen. A Bluetooth `'1'` mid-close aborts and reopens (manual override), and a 60 s outer timeout backstops everything.

**Registers used:**

| Register | Role |
|---|---|
| Addr 11 — Operating Mode = **3** | Position Control (set with torque off — EEPROM area) |
| Addr 116 — Goal Position | close command, then frozen at contact |
| Addr 126 — Present Load | contact detection signal (0.1 % units, signed) |
| Addr 100 — Goal PWM | grip-force ceiling (FULL 885 → HOLD 186, tuned on hardware) |

---

## Why This Design

**Target:** grasp a 1 kg, ⌀100 mm cylinder without damaging soft objects, with motor feedback as the only signal. That constraint — *the feedback must be trustworthy* — drove the mechanics:

- **Rack-and-pinion, not links/gearing:** extra stages add backlash and friction, both of which corrupt the load signal (friction consumes output unrelated to grip; backlash separates "motor moving" from "jaws contacting"). Single-stage conversion keeps the load-to-force relationship clean enough to detect contact from feedback alone.
- **Form closure over friction:** box grasps use **4-point contact** to constrain the object geometrically; cylinder grasps use contact pads + silicone tape laid along the acceleration direction, so added friction directly resists inertial slip.
- **FEA-driven reinforcement:** design case 9.81 N static **+ 1.2 G motion → 12 N**. SOLIDWORKS FEA located stress concentration at the rack; wall **4 mm → 7 mm**, peak Von Mises stress **−67 %**, validated against physical drive tests.
- **Jaw opening 8 cm → 10 cm** for the ⌀100 mm spec, with a top guide keeping the rack from derailing at the wider stroke.

---

## Firmware Notes

- **Direct register reads:** position, velocity, and load are read straight from their registers (132 / 128 / 126) and assembled from raw bytes each 20 ms cycle — lossless real-time feedback streamed over Bluetooth (`P:|L:|V:`) for live tuning.
- **Write verification:** every register write is return-checked, and the operating mode is read back at boot (`MODE SET TO: 3`) — configuration is confirmed, not assumed.
- **Power path:** the external supply feeds the **DYNAMIXEL Shield** (datasheet-confirmed to minimize back-current and voltage-drop stress on the Uno's regulator), with the Vin jumper cap pulled to separate the supply paths.

---

## Verified Numbers

| Item | Value | Source |
|---|---|---|
| Motor | DYNAMIXEL XC430, Protocol 2.0, Mode **3** | e-manual |
| Contact threshold | Present Load ≥ **800** (= 80.0 %) | Addr 126, 0.1 % units |
| Debounce | 2 consecutive reads (~40–60 ms) | `stallCount` |
| Hold | position freeze + Goal PWM cap (**885 → 186**, tune on hardware) | Addr 116 + 100 |
| Hold-state load (observed) | ~**290** (= 29.0 %) | telemetry |
| Design load case | 9.81 N + 1.2 G → **12 N** | FEA input |
| Rack wall | 4 mm → **7 mm**, Von Mises **−67 %** | SOLIDWORKS FEA |
| Jaw opening | 8 cm → **10 cm** | ⌀100 mm target |
| Payload | **1 kg** rigid → soft sponge, damage-free | competition runs |

---

## Author

**Kim Minsang (김민상)** — Robotics Engineering, Yeungnam University · Team Lead, Union
Embedded systems · Sensorless control · Mechanical design
Build log: https://blog.naver.com/kms031103 · LinkedIn: https://www.linkedin.com/in/kms2003
