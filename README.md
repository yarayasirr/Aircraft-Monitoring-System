# Aircraft Monitoring System 🛩️

An IoT-based wireless system for monitoring aircraft motion, orientation, and environmental conditions in real time using ESP32 and various sensors. This project was developed as part of the **Advanced Networks, Sensors and Components** course.

##  Overview

The system uses multiple onboard sensors connected to an ESP32 microcontroller to collect and transmit data over Wi-Fi using the **MQTT protocol**. The data is visualized through the **Blynk app**, allowing real-time monitoring of various flight parameters.

##  Hardware Components

- **ESP32** – microcontroller with built-in Wi-Fi
- **MPU6500** – 3-axis accelerometer and 3-axis gyroscope for motion and orientation tracking
- **BMP280** – pressure and temperature sensor for altitude estimation
- **IR Sensor** – for object detection
- **Blynk App** – used to display sensor readings via MQTT

##  Features

- Measures:
  - 3-axis acceleration
  - 3-axis orientation
  - Atmospheric pressure
  - Temperature
  - Altitude
- IR-based object detection
- Warning messages when critical thresholds are exceeded
- Wireless communication using **MQTT over Wi-Fi**
- Data visualization via the **Blynk mobile app**

##  Communication Protocol

The system uses the **MQTT protocol** to publish sensor readings to a broker, with the Blynk platform acting as the front-end visualization tool. The setup is fully wireless and can connect to any Wi-Fi hotspot.

##  Files Included

- `aircraft_monitoring.ino` – Arduino code for ESP32
- `Report.pdf` – Full project report
- `Poster.pdf` – Academic poster for presentation


##  Future Improvements

- Add motor or actuator control from the app
- Implement cloud storage for long-term data logging
- Use secure MQTT (MQTTS) for encrypted communication

##  Course Info

This project was submitted for the course **Advanced Networks, Sensors and Components**.

---
