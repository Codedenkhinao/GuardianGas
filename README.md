#GuardianGas
Overview
GuardianGas is an embedded system project developed for an Embedded Systems Design course. The system monitors gas concentration levels to ensure safety by detecting potential gas leaks. It integrates an STM32F429ZIT6 microcontroller, an ESP8266 Wi-Fi module, and an MQ-2 gas sensor, providing real-time monitoring and alerts via the Blynk IoT platform.
Features

Gas Detection: Utilizes the MQ-2 sensor to measure gas concentration (in ppm).
Real-Time Monitoring: The STM32F429ZIT6 reads sensor data via ADC and checks if the gas level exceeds the safety threshold.
Audible Alerts: Activates a buzzer when gas concentration surpasses the safe limit.
IoT Integration: Transmits gas concentration data and alerts to the ESP8266 via UART, which uploads the data to the Blynk IoT platform.
User Notifications: Sends email alerts to users through the Blynk app when gas levels are unsafe.
Remote Access: Allows users to monitor gas levels in real-time using the Blynk IoT mobile app.

Hardware Components

STM32F429ZIT6: Main microcontroller for processing sensor data and controlling the system.
ESP8266: Wi-Fi module for IoT connectivity and data transmission to Blynk.
MQ-2 Gas Sensor: Detects gas concentration levels.
Buzzer: Provides audible alerts for unsafe gas levels.

System Workflow

The MQ-2 sensor measures the gas concentration.
The STM32F429ZIT6 reads the sensor's analog output via ADC.
The microcontroller compares the gas level against a predefined safety threshold.
If the threshold is exceeded:
The buzzer is activated to sound an alarm.
The gas concentration data and alert status are sent to the ESP8266 via UART.


The ESP8266 pushes the data to the Blynk IoT platform.
Users can monitor gas levels in real-time via the Blynk app and receive email notifications if unsafe levels are detected.

Setup and Installation

Hardware Setup:
Connect the MQ-2 sensor to the ADC input of the STM32F429ZIT6.
Interface the buzzer with the STM32's GPIO pins.
Connect the STM32 to the ESP8266 via UART.
Ensure the ESP8266 is configured for Wi-Fi and Blynk connectivity.


Software Setup:
Clone this repository: git clone https://github.com/Codedenkhinao/GuardianGas.git
Program the STM32F429ZIT6 with the provided firmware (see /firmware directory).
Configure the ESP8266 with the Blynk IoT credentials (refer to /esp8266 directory for configuration details).
Install the Blynk IoT app and set up a project to visualize gas concentration and receive alerts.


Dependencies:
STM32CubeIDE for STM32 firmware development.
Blynk IoT library for ESP8266.
Appropriate drivers for the MQ-2 sensor and buzzer.



Usage

Power on the system and ensure all components are properly connected.
Open the Blynk IoT app to monitor real-time gas concentration.
If gas levels exceed the safety threshold, the buzzer will sound, and an email alert will be sent via Blynk.

Project Structure

/firmware: STM32F429ZIT6 firmware source code.
/esp8266: ESP8266 configuration and Blynk integration code.

Contributing.
This project was developed as part of an academic assignment. Contributions are welcome for improvements or bug fixes. Please submit a pull request or open an issue for discussion.
License
This project is licensed under the MIT License. See the LICENSE file for details.
Acknowledgments

Developed for the Embedded Systems Design course.
Thanks to the Blynk IoT platform for enabling seamless IoT integration.

