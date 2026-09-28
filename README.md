# Quadruped Robot Dog

🌐 **Language:** **English** | [한국어](README_KR.md)

> **Project status: Paused / Redesign planned**

This repository documents my personal quadruped robot project.

The first prototype was designed as a relatively feature-rich robot dog with four articulated legs, a moving head, ears, tail, displays, camera, speaker, and several sensors. Most of the mechanical structure was built, and individual actuators and peripherals were tested.

However, during integration I found that I had added too many secondary features before achieving reliable locomotion. The robot became heavier than expected, the center of mass became less favorable, and the overall control system became unnecessarily complex.

The project is therefore currently paused while I redesign it around one priority:

> **Reliable quadruped locomotion first. Everything else comes later.**

---

## Project Goal

The long-term goal is to build a small quadruped platform that can be used to study:

- Leg kinematics
- Servo control
- Gait generation
- IMU-based stabilization
- ROS 2 integration
- Simulation and Sim2Real workflows

The next version will intentionally be simpler than the first prototype.

Instead of trying to make a fully featured robot dog from the beginning, the redesign will focus on creating a stable 12-DOF walking platform first.

---

## Version 1 Prototype

The first version was physically assembled with a large number of additional features.

### Actuation

- 12 servos for the four legs
- 2 servos for the head
- 2 servos for the ears
- 1 servo for the tail
- **17 servos in total**

### Electronics and peripherals

- Raspberry Pi 5
- PCA9685 servo controller
- IMU
- Battery monitoring
- OLED displays
- Camera
- Speaker / amplifier
- DC-DC buck converters
- Battery and power distribution system

The leg mechanism used a linkage structure that required careful servo calibration and neutral-position alignment.

The mechanical design, wiring, and circuit layout were created and iterated during the first prototype stage.

---

## Why Version 1 Was Paused

The first prototype taught me an important lesson: a robot can have many working subsystems while the overall system is still not ready to perform its main task.

The largest problems were:

### 1. Too much weight

The head mechanism, ear servos, tail, speaker, displays, camera, structural parts, wiring, and additional electronics increased the total mass significantly.

This made the leg servos work harder and reduced the margin available for stable walking.

### 2. Unfavorable weight distribution

The upper structure added mass above the leg frame, making the center of mass higher and increasing the difficulty of keeping the robot stable during motion.

### 3. Excessive system complexity

The project included locomotion, head motion, ear motion, tail motion, displays, audio, camera input, sensing, and power management at the same time.

For a first quadruped project, this created too many variables to debug simultaneously.

### 4. Locomotion was not mature enough

Individual motors and modules could be tested, but reliable standing and walking had not yet been achieved.

Because locomotion is the core function of a quadruped, continuing to add secondary features would have made the project harder rather than better.

---

## Redesign Direction

The next revision will use a **locomotion-first architecture**.

Planned changes include:

- Remove the head assembly
- Remove the ear mechanism
- Remove the tail mechanism
- Remove the speaker
- Remove non-essential displays and decorative electronics
- Reduce wiring and structural mass
- Lower the center of mass
- Keep only the components required for locomotion and state estimation
- Focus on 12-DOF leg control
- Validate standing before walking
- Validate slow gait before faster gait

Optional features can be added again later, but only after the base platform can walk reliably.

---

## Planned Core Hardware

The next version is expected to keep the following core hardware:

| Category | Planned Component |
| --- | --- |
| Main Computer | Raspberry Pi 5 (16 GB) |
| Operating System | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| Leg Actuation | 12 servo motors |
| Servo Control | PCA9685-based servo control |
| State Estimation | IMU |
| Power | Battery + DC-DC regulation |
| Mechanical Structure | 12-DOF quadruped frame |

The exact configuration may change during the redesign.

---

## Software Architecture

The current plan is to reorganize the system into ROS 2 nodes instead of keeping all hardware logic in one program.

A possible structure is:

```text
ROS 2
│
├── servo_driver_node
│     └── Sends commands to leg servos
│
├── imu_node
│     └── Publishes orientation / acceleration data
│
├── gait_controller_node
│     └── Generates leg trajectories
│
├── state_estimator_node
│     └── Estimates robot body state
│
└── high_level_controller
      └── Standing / walking commands
```

This architecture is still provisional and may change as the project develops.

---

## Development Strategy

The redesign will be developed in stages.

### Stage 1 — Mechanical simplification

- Remove non-essential mechanisms
- Reduce mass
- Check joint range and interference
- Recalibrate all leg servos

### Stage 2 — Basic leg control

- Command each joint independently
- Verify joint directions
- Define neutral pose
- Set safe joint limits

### Stage 3 — Standing

- Implement forward and inverse kinematics
- Move all four feet to defined target positions
- Achieve stable static standing

### Stage 4 — Walking

- Implement a slow crawl gait
- Tune step height, stride length, and timing
- Add trot gait after basic walking becomes stable

### Stage 5 — Feedback control

- Integrate IMU feedback
- Compensate for body roll and pitch
- Improve stability during walking

### Stage 6 — Simulation / Sim2Real

After the physical model and joint definitions become stable, I plan to build a simulation model and explore reinforcement learning and Sim2Real workflows.

---

## Current Status

| Item | Status |
| --- | --- |
| Mechanical V1 prototype | ✅ Built |
| Leg mechanisms | ✅ Built |
| Head / ears / tail | ✅ Built in V1 |
| Individual servo tests | ✅ Tested |
| Sensor / peripheral tests | ✅ Partially tested |
| Full ROS 2 integration | ⏸️ Paused |
| Stable standing | ⏳ Planned |
| Reliable walking | ⏳ Planned |
| Lightweight V2 redesign | ⏳ Planned |
| Simulation model | ⏳ Planned |
| Sim2Real | ⏳ Future goal |

---

## Archived Version 1 Materials

I still have several design files from the first prototype, including:

- Fusion 360 assembly
- Fritzing circuit design
- Wiring diagram
- Servo calibration reference
- Prototype photographs

These files represent the **Version 1 architecture** and may not match the future lightweight redesign.

They will be cleaned up and organized in this repository when the project resumes.

---

## Notes

This repository currently documents an unfinished project.

I am intentionally keeping the first prototype and its problems documented because the redesign decisions came directly from those failures.

The main lesson from Version 1 was simple:

> **A quadruped should learn to stand and walk before it learns to look like a dog.**

The next revision will be smaller in scope, lighter, easier to debug, and much more focused on locomotion.

---

## Roadmap

- [x] Build first mechanical prototype
- [x] Assemble 12-DOF leg system
- [x] Build head, ear, and tail mechanisms
- [x] Test individual actuators and peripherals
- [ ] Remove non-essential V1 hardware
- [ ] Reduce robot weight
- [ ] Recalibrate all joints
- [ ] Create clean ROS 2 hardware interface
- [ ] Implement standing controller
- [ ] Implement crawl gait
- [ ] Implement trot gait
- [ ] Add IMU-based stabilization
- [ ] Create simulation model
- [ ] Explore Sim2Real

---

## License / Credits

This repository is currently a personal development log.

Licensing and third-party design credits will be organized before the project is published as a complete reproducible build.
