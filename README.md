Welcome! The purpose of this project is to build a flight controller from scratch using an STM32F446RE MCU, a number of commercial off-the-shelf sensors, and hobbyist drone components &mdash; a fixed-wing carbon body, motors, propellers, and so forth.

The flight controller is implemented using FreeRTOS. If using STM32CubeIDE, the source code for FreeRTOS can be imported into your project easily. The state machine for the flight controller is implemented using a multivariate Kalman filter &mdash; I used <a href="https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python">this resource</a> as a tool to help with understanding how to create and tune the filter.

For development and debugging, I use STM32CubeIDE and a NUCLEO-F446RE development board. For sensor measurements, I use:
<ul>
  <li>BMP388 pressure sensor</li>
  <li>LIS2MDL magnetomer</li>
  <li>LSM6DSO32 gyroscope and accelerometer</li>
  <li>PA1616S GPS sensor</li>
</ul>

The eventual goal is to design a custom PCB using KiCad to pivot from the prototype/breadboard setup to a smaller form factor.

For a functional overview of the code, see below:

![Drone](https://github.com/user-attachments/assets/d961efa0-c689-4938-8f23-5662fdb0ce34)
