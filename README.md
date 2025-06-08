# UWB-Indoor-Tracker

**Grade: 1,0**

This project focuses on precisely tracking the movement of an autonomous vehicle using Ultra-Wideband (UWB) technology. The vehicle is remotely monitored while moving on a predefined track. Additionally, it determines its current position and performs corresponding actions based on its location.

**Hardware Used:** DWM1000, ESP32, Arduino Uno

## Features

- 📐 Analyzed and specified the project architecture using SysML and UML diagrams.
- 📡 Implemented the Two-Way Ranging (TWR) algorithm combined with a geometric triangulation method to calculate the vehicle's 2D coordinates.
- 🧠 Integrated a Kalman Filter to reduce noise and improve position accuracy.
- 📊 Developed a Python script to receive real-time coordinates via Wi-Fi from the moving tag and visualize the trajectory using `matplotlib`.
- 🔁 Enabled I/O communication between the ESP32 and Arduino to allow the Tag (ESP32) to notify the Vehicle (Arduino) when it enters a specific zone.
- 📚 Reviewed related literature on UWB-based positioning and Kalman Filters to guide implementation and optimization.

---

Feel free to explore the code and documentation for more details!
