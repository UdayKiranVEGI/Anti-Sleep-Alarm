# Anti Sleep Alarm

![Anti Sleep Alarm] 
![Circuit](https://github.com/UdayKiranVEGI/Anti-Sleep-Alarm/assets/84384630/e363a8eb-1e9d-4b9c-9cad-dae0088ad626)


## Objective
###
Feeling sleepy while doing work or reading can be detected by this Anti-Sleep Alarm.
###
Feeling sleepy while driving could cause hazardous traffic accidents. However, when driving alone on the highway or over a long period of time, drivers are inclined to feel bored and sleepy, or even fall asleep. To address this issue, we have developed a system for detecting drowsy driving and alerting the drivers to prevent accidents and create a safer driving environment.

## Introduction

### Project Motivation and Purpose

The goal of this project is to develop a system that can accurately detect sleepy driving and raise alarms accordingly. The system aims to prevent drivers from drowsy driving and promote safer driving practices. The project incorporates a Webcam for image capture, a Beagle board for image processing algorithms, a feedback circuit for generating alarms, and a power supply system.

### Functions and Features

- Eye extraction using open and close to determine sleepiness.
- Daytime and night detection for varied lighting conditions.
- Real-time image processing and detection for prompt response.
- Sound and flashing LED warning system to redirect the driver's attention.
- Little interference and potential hazards to the driver's normal driving.
- Portable size with a car cigarette charger socket for power supply.

## Hardware System Overview / Block Diagram

![Block Diagram] ("C:\Users\udayk\OneDrive\Desktop\Anti-Sleep-Alarm\Block_diagram.jpg.crdownload")

### Components Required

1. Relay
2. Piezo Buzzer
3. Gear Motor
4. Wheel
5. Arduino Nano
6. Jumper wires
7. SPST Switch
8. Eye Blink Sensor
9. Battery

### Components Specifications

#### Arduino NANO:
- Provides various communication options such as UART, I2C, and SPI.
- Powered by a 5V DC voltage and offers a software serial library for communication.
- Simplifies the use of the I2C bus through the Wire library.

#### Piezo Buzzer:
- Generates sounds when powered by DC voltage.
- Comes in different designs and uses, producing different sounds.

#### Eye Blink Sensor:
- Based on IR and consists of an IR transmitter and IR receiver.
- Illuminates the eye with infrared light and detects changes in reflected light to sense eye blinks.

## Circuit Diagram

![Circuit Diagram] ("C:\Users\udayk\OneDrive\Desktop\Anti-Sleep-Alarm\circuit_diagram.webp")

### Procedure

1. Connect the "VCC" pin of the sensor to the 5V pin of the Arduino.
2. Connect the "GND" pin to the GND pin of the Arduino.
3. Connect the "TRIGPIN" pin to the 3 pin and "ECHOPIN" to the 2 pin of the Arduino.
4. Fix the DC motor into the handwash.
5. Connect pins to the ground of an Arduino pin GND.
6. Connect one pin to the 4 pin of the Arduino.
7. Write the code in Arduino NANO and compile.

## Conclusion
Our Driver Sleep Detection and Alarming System help warn drivers when their physical condition is not suitable for driving, preventing dangerous behaviors and ensuring the safety and welfare of the public. By using openCV and related libraries, we have improved the algorithm for eye closeness detection, contributing to a better and safer driving experience.

We value feedback and honest criticism of our technical work and acknowledge the contributions of others in this project.
## Contributors
### B. Harsha Vardhan (20A91A0405)
### P. Nanda Kishore (20A91A0442)
### T. Deva Sai Kishore (20A91A0452)
### V. Mallesh Uday Kiran (20A91A0458)

https://github.com/UdayKiranVEGI/Anti-Sleep-Alarm/assets/84384630/0fb4bef7-48cb-400b-944d-060669755f83


https://github.com/UdayKiranVEGI/Anti-Sleep-Alarm/assets/84384630/45d237ce-8ac4-4009-8cb3-b2bb326825fb

![Testing_by_DSK](https://github.com/UdayKiranVEGI/Anti-Sleep-Alarm/assets/84384630/c2ec2755-8d4a-4903-ad5e-e9c940355232)


### CODE

```c
// Paste your Arduino code here

#define Relay 13
#define buzzer A0
static const int sensorPin = 10; // sensor input pin
int SensorStatePrevious = LOW; // previousstate of the sensor
unsigned long minSensorDuration = 3000; // Time we wait before the sensor active as long
unsigned long minSensorDuration2 = 6000;
unsigned long SensorLongMillis; // Time in ms when the sensor was active
bool SensorStateLongTime = false; // True if it is a long active
const int intervalSensor = 50; // Time between two readings sensor state
unsigned long previousSensorMillis; // Timestamp of the latest reading
unsigned long SensorOutDuration; // Time the sensor is active in ms
//// GENERAL ////
unsigned long currentMillis; // Variabele to store the number of milleseconds since the 
Arduino has started
void setup() {
Serial.begin(9600); // Initialise the serial monitor
pinMode(sensorPin, INPUT); // set sensorPin as input
Serial.println("Press button");
pinMode(Relay,OUTPUT);
pinMode(buzzer,OUTPUT);
}
// Function for reading the sensor state
void readSensorState() {
// If the difference in time between the previous reading is larger than intervalsensor
if(currentMillis - previousSensorMillis > intervalSensor) {
// Read the digital value of the sensor (LOW/HIGH)
![Circuit](https://github.com/UdayKiranVEGI/Anti-Sleep-Alarm/assets/84384630/4372db4d-07f6-4737-a9a8-f83507e036d9)
int SensorState = digitalRead(sensorPin);
// If the button has been active AND
// If the sensor wasn't activated before AND
// IF there was not already a measurement running to determine how long the sensor has been 
activated
if (SensorState == LOW && SensorStatePrevious == HIGH && !SensorStateLongTime) {
SensorLongMillis = currentMillis;
SensorStatePrevious = LOW;
Serial.println("Button pressed");
}
// Calculate how long the sensor has been activated
SensorOutDuration = currentMillis - SensorLongMillis;
// If the button is active AND
// If there is no measurement running to determine how long the sensor is active AND
// If the time the sensor has been activated is larger or equal to the time needed for a long active
if (SensorState == LOW && !SensorStateLongTime && SensorOutDuration >= minSensorDuration) {
SensorStateLongTime = true;
digitalWrite(Relay,HIGH);
Serial.println("Button long pressed");
}
if (SensorState == LOW && SensorStateLongTime && SensorOutDuration >= minSensorDuration2) {
SensorStateLongTime = true;
digitalWrite(buzzer,HIGH);
delay(1000);
Serial.println("Button long pressed");
}
// If the sensor is released AND
// If the sensor was activated before
if (SensorState == HIGH && SensorStatePrevious == LOW) {
SensorStatePrevious = HIGH;
SensorStateLongTime = false;
digitalWrite(Relay,LOW);
digitalWrite(buzzer,LOW);
Serial.println("Button released");
}
// store the current timestamp in previousSensorMillis
previousSensorMillis = currentMillis;
}
}
void loop() {
currentMillis = millis(); // store the current time
readSensorState(); // read the sensor state
}
