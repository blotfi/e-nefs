# e-nefs
Emergency Ventilator control using ESP32

on April 13, 2020; **This code is not complete**

One can use it or adapt it to its own needs / hardware configuration.
I am using ESP32 Lolin32
## Implemented features
- Wifi connection, SSID/PW read from ESP32 Flash memory using SPIFFS
- Web server as HMI (Human Machine Interface)
- ESPAsyncWebServer using html files in data SPIFFS
- Using ArduinoJson to send/get data to client navigator
- html/javascript wep pages to dynamically: display data, change parameters, ~~save recorded data~~
- Save configuration ESP32 Flash memory
- DC Motor control using **16 kHz 12 bits PWM**, Duty cycle range 0-4095
- Timer based Interrupt Service Routine (**ISR**) at sampling period of Ts = 20 ms
- State machine routine to go through the respiration cycle (inspiratory phase, plateau phase, expiratory phase, PEEP phase)
- Record data over 1000*20 ms period
- Reading **sensors** data through I2C: BMP280 for pressure, Si7021 for temperature and humiditidy
- Display relevant data on SSD1302 **OLED screen** through I2C - U8g2 library

#### Todo
- Save recorded data
- Implement sensors and routine to limit the moving path of the arm (Reed switch)
- Implement sensors and routine to count the incremental motion of the motor (IR based self made sensor)
- Replce both sensors with a VL53L0X **Time-of-Flight** (ToF) Laser Ranging Sensor (to be acquired)
- Reposition the mechanical balloon to get more compression
- Estimate or measure the bidirectional air flow 
- Test on **artificial lung**
- Validate by physicians
- Test on real cases
- Obtain approval and contact local industries for large scale production

## Prototype photos
These are the first stage photos of the mechanical prototype

![proto1](/img/proto1s.png)  ![proto2](/img/proto2s.png)

![proto1](/img/enefs_2.png) 


Also the HMI on the web browser, using the ESP32 as the web server
![ihm3](/img/ihm3s.png)

Videos of working prototype:<br/>
https://www.youtube.com/watch?v=TYXenkf9Xuk&list=PLXYd8IyLhtrE_5peLIUVFXS2KW_LJ0eVU&index=1

Last updated May 29, 2020