# Sensorless Adaptive Gripper — 1 kg Rigid to Soft Sponge, Motor Feedback Only

**A rack-and-pinion gripper that grasps a 1 kg cylinder and a deformable sponge with the same hardware — no force or tactile sensors. Contact is detected from motor load feedback; grip force is capped in firmware.**

Arduino Uno + DYNAMIXEL Shield, DYNAMIXEL XC430 (Protocol 2.0).

> **2026 Physical AI Gripper Challenge — Silver Award (2nd Place)**, Team Union (5 members: Robotics + Computer Science).

---

## The Problem — Sensing Force Without a Force Sensor

The design target: grasp a 1 kg, ⌀100 mm cylinder *and* a deformable sponge, damage-free, with **motor feedback as the only signal**. No load cells, no tactile pads — and the XC430 itself has **no current sensor**, so even "read the motor current" is off the table. The only force-correlated signal available is **Present Load (Addr 126)**, the motor's own output-effort estimate in 0.1 % units.

That single constraint cascades through the whole design:

1. If load feedback is the only force signal, **the signal path must not lie** → drives the mechanical design (rack-and-pinion, below)
2. If the motor can't limit its own force, **the firmware must decide when to stop pushing** → drives the detection logic (detect → freeze → cap, below)

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

**Why detect the peak, not the steady state?** In position control, a blocked motor doesn't settle at a polite plateau — the PID keeps pushing toward the unreached goal, so output saturates at its ceiling regardless of what was touched. Waiting for a steady-state load level therefore can't distinguish "gripping an object" from "stalled at full effort." The **transient spike at the moment of blocking (≥ 80 %)** is the only signature that marks contact itself — so the logic watches for the peak and acts inside it.

**Two detection paths, two object classes.** A rigid object blocks the jaws instantly → clean load spike (path A: `load ≥ 800`). A sponge never spikes — it compresses, so load climbs gradually while velocity collapses (path B: `|vel| < 3 AND load ≥ CONTACT_LOAD`). One condition per stiffness class is what makes "1 kg rigid to soft sponge" a single firmware.

**Why freeze the position?** Once contact is confirmed, the goal position is overwritten with the *current* position. The PID's position error collapses to ~0, so effort settles naturally (hold-state load ≈ 290 = 29 %) instead of grinding toward the original closed target. The freeze lands within ~40–60 ms of contact — that short crush window is what keeps a sponge un-flattened.

**Why cap the PWM?** Freezing bounds the *goal*, not the *force* — if the object shifts, the PID can still push hard. **Goal PWM (Addr 100)** is the XC430's one real output ceiling (lower PWM → lower torque), so after the freeze it is dropped to `PWM_HOLD`, putting a hard upper bound on grip force.

**Safety around the loop:** the condition must hold 2 consecutive reads (`stallCount`) to reject single-sample spikes from gear-mesh vibration; reaching `posClose` with no detection means an empty grip → automatic reopen; a Bluetooth `'1'` mid-close aborts immediately; a 60 s outer timeout backstops everything.

**Registers used:**

| Register | Role |
|---|---|
| Addr 11 — Operating Mode = **3** | Position Control (set with torque off — EEPROM area) |
| Addr 116 — Goal Position | close command, then frozen at contact |
| Addr 126 — Present Load | contact detection signal (0.1 % units, signed) |
| Addr 100 — Goal PWM | grip-force ceiling (FULL 885 → HOLD 186, tuned on hardware) |

---

## Why This Mechanism — Protecting the Signal

If contact must be read from load feedback, every source of mechanical slop between motor and jaws degrades the one signal the firmware depends on:

- **Backlash** opens a gap between "motor moving" and "jaws actually touching" — the contact spike arrives late and soft
- **Multi-stage friction** consumes output that has nothing to do with grip force — the load reading inflates with noise

So the transmission is a **single-stage rack-and-pinion**: motor torque converts to linear grip force in one mesh, keeping the load-to-force relationship clean enough to detect contact from feedback alone. The rest of the mechanics follows the same logic of earning trust in the grasp:

- **Form closure over friction:** box grasps use **4-point contact** to constrain the object geometrically; cylinder grasps use contact pads + silicone tape laid along the acceleration direction, so added friction directly resists inertial slip
- **FEA-driven reinforcement:** design case 9.81 N static **+ 1.2 G motion → 12 N** (a gripper on a moving arm carries more than the object's weight). FEA located stress concentration at the rack; wall **4 mm → 7 mm**, peak Von Mises stress **−67 %**, validated against physical drive tests
- **Jaw opening 8 cm → 10 cm** for the ⌀100 mm spec, with a top guide keeping the rack from derailing at the wider stroke

---

## Firmware Notes

- **Direct register reads:** position, velocity, and load are read straight from their registers (132 / 128 / 126) each 20 ms cycle and streamed over Bluetooth (`P:|L:|V:`) — the live telemetry the thresholds were tuned against.
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
