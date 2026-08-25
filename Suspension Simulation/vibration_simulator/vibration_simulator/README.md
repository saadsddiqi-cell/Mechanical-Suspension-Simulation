# Interactive Vehicle Suspension Simulator & Co-Simulator Guide

Welcome to the **DYNA-RIDE Co-Simulator** project files! This package contains the complete hardware-software co-design files needed to construct and simulate a real-time, second-order mechanical suspension test rig. 

---

## 1. System Architecture Overview

This project consists of three main components:
1. **Interactive Web GUI (`index.html`, `index.css`, `main.js`):** A browser-based dashboard. It solves the differential equations of a mass-spring-damper suspension in real time, animates the motion, displays scrolling plots, and streams calculated positions to the hardware.
2. **Arduino Firmware (`arduino_firmware.ino`):** High-speed, non-blocking serial code that receives target angles and controls two servos to physically bounce a toy car platform.
3. **Physical Rig (Redesigned):** A mechanical testing frame consisting of high-torque servos, rigid pushrods, and linear guide rails supporting the toy car.

---

## 2. Mathematical Foundation (2nd-Order ODE)

The suspension is modeled as a classic **mass-spring-damper system** undergoing base-excitation.

### The Equation of Motion
The governing second-order linear ordinary differential equation (ODE) is:

$$M \ddot{x} + B (\dot{x} - \dot{y}) + K (x - y) = 0$$

Which simplifies to:

$$\ddot{x} + \frac{B}{M}\dot{x} + \frac{K}{M}x = \frac{K}{M}y(t) + \frac{B}{M}\dot{y}(t)$$

Where:
*   $M$ = Vehicle Body Mass (kg)
*   $K$ = Spring Stiffness (N/m)
*   $B$ = Damping Coefficient (N·s/m)
*   $y(t)$ = Road excitation height (m) — the input signal (Step, Impulse, or Sine)
*   $x(t)$ = Vehicle body displacement (m) — the output response

### Damping Ratio ($\zeta$)
The **damping ratio** determines how the system behaves when disturbed:

$$\zeta = \frac{B}{2\sqrt{KM}}$$

There are three distinct behavior states:
1.  **Underdamped ($\zeta < 1$):** The system oscillates back and forth about its equilibrium position. The oscillations decay exponentially over time. A pothole hit causes the car to bounce repeatedly.
2.  **Critically Damped ($\zeta = 1$):** The system returns to equilibrium as quickly as possible without any oscillation. This is the **ideal sweet spot** for vehicle suspensions, neutralizing shocks instantly.
3.  **Overdamped ($\zeta > 1$):** The system returns to equilibrium without oscillating, but does so very slowly and sluggishly, making the ride feel stiff and unresponsive.

---

## 3. Real-Time Numerical Simulation (RK4 Solver)

To ensure the simulation is highly stable even when stiffness $K$ is set very high, the software uses **4th-Order Runge-Kutta (RK4)** numerical integration running at a fixed frequency of **200 Hz** ($dt = 5\text{ ms}$).

Compared to simple Euler integration ($x_{n+1} = x_n + v_n dt$), which accumulates numerical drift and easily diverges (blows up), RK4 samples the derivatives at four points within each time step:
*   $k_1$: Slope at the beginning of the step.
*   $k_2$: Slope at the midpoint using $k_1$.
*   $k_3$: Slope at the midpoint using $k_2$.
*   $k_4$: Slope at the end of the step using $k_3$.

This provides a local error of $O(dt^5)$, keeping the simulation stable and mathematically accurate.

---

## 4. Hardware Redesign Guide

The initial prototype (using SG90 micro servos and copper wire cords) suffers from **weak torque** and **lag**. Here is how to redesign it into a robust, professional testing rig.

### Mechanical Upgrade
1.  **Replace SG90 Servos with MG996R Servos:**
    *   **SG90:** Plastic gears, 1.8 kg·cm torque. Too weak to support a toy car rigidly.
    *   **MG996R:** Metal gears, 10–12 kg·cm torque. Extremely rigid, fast, and powerful.
2.  **Linear Guide Rails:**
    *   To prevent the platform from twisting, tilting laterally, or shifting sideways, you must constrain it.
    *   Install two **linear shafts** (e.g., 6mm or 8mm smooth steel rods, or cheap carbon fiber shafts) vertically behind the platform.
    *   Attach **linear bearings** (or plastic straws/tubes sliding smoothly over the shafts) to the platform. This restricts movement strictly to the vertical axis.
3.  **Rigid Linkages (Pushrods):**
    *   Replace the floppy copper wires. 
    *   Use rigid pushrods (e.g., steel rods, RC car turnbuckle links with plastic ball joints, or stiff paperclips inside plastic tubes) to connect the servo horns to the platform. This creates a solid mechanical linkage, translating servo rotation directly into vertical lift without any backlash.

### Electrical Upgrade & Wiring Schematic
High-torque servos draw substantial peak currents (up to 1.5A each under load). They **must** be powered by an external 5V source, not the Arduino board.

```
       +---------------------------------------+
       |       External DC Power Supply        |
       |               (5V 3A)                 |
       +---------+-------------------+---------+
                 | (+)               | (-)
                 |                   |
                 |      +------------+----------+
                 |      |                       |
                 |      | (Common Ground)       |
                 |      |                       |
                 v      v                       v
               +----------+                  +----------+
               |  Servo   |                  |  Servo   |
               |  (Left)  |                  | (Right)  |
               +----+-----+                  +----+-----+
         Signal |   |                      Signal |
         (Pin 9)|   |                             | (Pin 10)
                |   |                             |
                v   |                             v
       +--------+---+-----------------------------+-----+
       |            | GND                               |
       |                Arduino Uno Board               |
       +------------------------------------------------+
```

> [!WARNING]
> **Common Ground Rule:** You MUST connect the GND (-) wire of the external power supply directly to the Arduino GND pin. Without this common ground reference, the servo control signal will float, causing the motors to jitter violently or fail to move.

---

## 5. Software-Hardware Serial Protocol

To eliminate latency/delay, the co-simulator streams lightweight coordinate packets over USB at 115200 baud.

*   **Packet Format:** `[LeftAngle],[RightAngle]\n` (e.g., `90,95\n`).
*   **Angle Range:** Mapped from $-10\text{ cm} \le x \le 10\text{ cm}$ to $40^\circ \le \text{Angle} \le 140^\circ$ (centered at $90^\circ$). Clamped to protect servo gears.
*   **Non-blocking Parser:** The Arduino firmware reads incoming bytes one-by-one into a buffer. It parses the command instantly using string pointers (`strchr` and `atoi`) *only* when the newline character (`\n`) is received.
*   **Why this avoids delay:** Standard functions like `Serial.parseInt()` or `Serial.readStringUntil()` block CPU execution waiting for timeout limits (default 1000ms), resulting in severe lags. Our parser executes in micro-seconds.
*   **Safety Watchdog:** If no command is received for 1000ms, the Arduino automatically centers the servos to $90^\circ$ and enters a failsafe idle state, preventing overheating if the USB cable is unplugged.

---

## 6. Project Assembly & Running Guide

1.  **Arduino Setup:**
    *   Open `arduino_firmware.ino` in the Arduino IDE.
    *   Connect your Arduino Uno/Nano via USB.
    *   Upload the sketch. Note the COM port number (e.g., `COM3` on Windows).
2.  **Web GUI Launch:**
    *   Open `index.html` in Google Chrome or Microsoft Edge.
    *   Click **CONNECT HARDWARE** on the top right.
    *   Select the matching Arduino COM port in the browser popup and click **Connect**.
    *   Watch the status dot turn green and the serial console start outputting live data.
3.  **Co-Simulation:**
    *   Select a road profile (e.g., Sine Wave) or click **TRIGGER ROAD BUMP**.
    *   Adjust the $M$, $K$, and $B$ sliders. Observe the change in the damping ratio $\zeta$.
    *   Find the exact values where $\zeta = 1.0$ (Critical Damping) to see the physical toy car instantly stabilize after a pothole hit without any residual bouncing!
