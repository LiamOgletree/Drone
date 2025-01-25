Welcome! I'm designing a custom flight controller for drones. This Github page hosts all relevant files and technical details related to this effort, to include requisite project code and KiCAD schematic and PCB files.

The hardware design integrates STM32F446RE and ESP32-S3 microcontrollers, wireless and GPS (PA1616S) capabilities, and a custom IMU composed of an accelerometer and gyroscope (LSM6DSO32), barometer and temperature sensor (BMP388), and magnetometer (LIS2MDL). On the software side, I'm using STM32CubeIDE for my development environment; FreeRTOS for the OS; SPI for the LSM6DSO32, LIS2MDL, and BMP388 sensors; and DMA/UART for the PA1616S GPS receiver. Additionally, I use a Kalman Filter for state estimation &mdash; I would recommend <a href="https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python">this resource</a> for anyone interested in understanding Kalman Filters.

For the custom board design, I used <a href="https://www.kicad.org/">KiCAD</a> for schematic creation and the PCB layout and used <a href="https://jlcpcb.com/">JLCPCB</a> to manufacture physical boards. I'm sourcing drone components primarily from <a href="https://www.getfpv.com/">getfpv</a> (e.g., <a href="https://www.team-blacksheep.com/products/prod:sourceone_v5">this drone frame</a>) but there are many other FPV sites. Finally, for debugging purposes, I have a <a href="https://www.saleae.com/products/saleae-logic-pro-8">Saleae Logic Analyzer</a>.

See below for a functional overview of the code as it exists today:

![Drone](https://github.com/user-attachments/assets/0dd65c78-63c5-44f8-91c0-8b0bb26e35e5)

Additionally, see below for layered 2D and rendered 3D images of the PCB design:

<img width="446" alt="STM32-DRONE-FLIGHT-CONTROLLER-pcb" src="https://github.com/user-attachments/assets/6f38ba9a-b635-41bc-aad2-b4e7e889a0f3" />

![STM32-DRONE-FLIGHT-CONTROLLER-top](https://github.com/user-attachments/assets/b56469df-c838-4ffe-8abe-ee8adbbc841a)

![STM32-DRONE-FLIGHT-CONTROLLER-bottom](https://github.com/user-attachments/assets/b4607d30-73e7-4357-8c15-0bde579dc879)

Please feel free to reach me at ogletreeliam@gmail.com with any questions or comments. Thank you!
