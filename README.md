# UWB-Indoor-Tracker
Grade: 1,0. 

The goal of the project is to precisely monitor movement of autonomous vehicle using Ultrawide-band. An autonomous vehicle will be remotely monitored while it is running on a track. Additionally, the vehicle will know it's current location and perform actions accordingly.

Hardware Used: DWM1000, ESP32, ArduinoUno.

•Analysed and specified the Project using SysML and UML diagrams.
• Implemented positioning algorithm Two-Way Ranging (TWR) along with geometric algorithm Triangulation to calculate coordinate of the vehicle. 
•Implemented a python script to receive coordinates in computer terminal via Wifi from the Moving Tag and visualised it in a graph using matplotlib.
• Integrated Kalman Filter to account for measurement noise from sensors. 
• Added I/O communication between ESP32 and Arduino so that the Tag (ESP32) alerts the Vehicle (Arduino) about the zone.
• Reviewed previously published literatures about UWB and Kalman Filter.
