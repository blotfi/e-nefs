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

## proto images
These are first images of mechanical prototype
![proto1](/img/proto1.jpeg)
![proto1](/img/proto2.jpeg)


Also the HMI on the web browser, using the ESP32 as the web server
![ihm3](/img/ihm3.png)

