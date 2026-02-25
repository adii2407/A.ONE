# A.ONE

IoT & aerospace learning project for atmospheric data collection with sensors and GPS tracking.

## Features
- Atmospheric data logging (temperature, pressure)
- GPS tracking 
- Data storage in **flash memory (SPIFFS/Flash)**
- Recovery alert system with **buzzer and LED**
- LoRa-based data transmission 
- Sensor-based data collection
- Modular IoT architecture

## Components Used
- ESP32
- BMP280
- MPU6050
- Flash memory (SPIFFS)
- LoRa module (for long-range communication)
- Buzzer & LED (recovery alert)
- Power system (Li-ion)

## Project Goal
A.ONE collects atmospheric data, transmits data via LoRa (if enabled), and stores it in flash memory for analysis. Recovery system alerts with buzzer and LED when required.

## How It Works
1. Device powers on
2. Sensors collect data
3. Data stored in flash memory
4. LoRa transmits data 
5. Recovery alert activates 
6. Data can be retrieved for analysis

## Future Scope
- Live telemetry via LoRa/cloud
- Camera integration for aerial imaging
- Advanced recovery tracking
- Multi-flight data comparison
- Machine learning-based analysis

## License
MIT License

## Author
Aditya Rajput
