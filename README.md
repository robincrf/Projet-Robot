# Line-Following Robot (STM32) — Path Tracking with LEDs

C project for a wheeled robot based on STM32 capable of **following a black line** and indicating its state and trajectory using **LEDs**.  
The project focuses on **real-time execution**, motor control, and sensor reliability in an embedded environment.

---

##  Features
- **Automatic black line tracking**
- **LED system**
  - Direction indication (left / right / straight)
  - Robot states (calibration, line lost, error)
- **Real-time execution** using STM32 timers and interrupts
- Modular architecture: sensors, control, motors, LEDs

---

##  Operating Principle
1. **Infrared sensors** detect line / floor contrast.
2. A **control algorithm** computes trajectory correction.
3. **Motor PWM** is adjusted to recenter the robot.
4. **LEDs display the robot state** in real time.

---

##  Hardware Used
- **STM32** board (F1 / F4 or equivalent)
- DC motors + **H-bridge**
- Line sensors (IR reflectance)
- LEDs (GPIO outputs)
- Battery / onboard power supply

---

##  Firmware Stack
- Language: **C**
- Drivers: **STM32 HAL**
- Tools: **STM32CubeIDE** (or PlatformIO)

---
