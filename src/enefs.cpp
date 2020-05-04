// enefs
// e-ventilator control code
// LOLIN32, Web Server, BTS7960, No MPX5050DP, GY21 BMP280/si7021
// L. BAGHLI 02/04/2020 - 11/04/2020

#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <Time.h>
#include <U8g2lib.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_Si7021.h"
#include <Wire.h>
#include <NeoPixelBus.h>

// fonction definitions
String getFormattedDateTime();
void SetDefaultParam();
void RefreshiTxx();

// PWM
const uint8_t EN_PWM = 4; // L_EN et R_EN
const uint8_t L_PWM = 18;
const uint8_t R_PWM = 5;
const unsigned int Channel_L_PWM = 1;
const unsigned int Channel_R_PWM = 2;
const unsigned int f_PWM = 16000;  // 16 kHz -> not audible
const unsigned int res_PWM = 12;   // PDCx 0-4095
const unsigned int MinPWM = 0;    // min duty cycle (PDCx) -> negative motor voltage
const unsigned int FullPWM = 4095; // max duty cycle (PDCx) -> positive motor voltage
const unsigned int HalfPWM = 2048; // half duty cycle -> zero motor voltage
//const double Umax = 9;           // 9 V DC bus
const double Half_U = 227.5556;    // HalfPWM / Umax   170.6667 pour 12V
unsigned int PDC;

//int brightness = 0;    // how bright the LED is
//int fadeAmount = 1;    // how many points to fade the LED by

// setting LED PWM properties
const int freq = 16000;
const int ledChannel = 0;
const int resolution = 12; //Resolution 8, 10, 12, 15


const uint32_t SERIAL_SPEED = 115200; ///< Set the baud rate for Serial I/O
#define r2 1.414213562373095
int NSensorcount = 20, Sensorcount = 0; // 10 *20ms = 200 ms
int DispNcount = 50, Dispcount = 0; // 50 *20ms = 1s
//int DispNcount = 100, Dispcount=0; // 100 *20ms = 2 s
//int  betatLed = 0;
String etatLed = "OFF";
// LED_BUILTIN redefinir au lieu d' IO 5 c'est IO 22
#define LED_BUILTIN 22

#define OLED_SDA 23
#define OLED_SCL 19
#define GY21_SDA 13  // GY21 BMP280 / Si7021
#define GY21_SCL 15  // GY21 BMP280 / Si7021
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);   // (rotation, [reset [, clock, data]])
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/
                                         OLED_SDA);   // (rotation, clock, data [, reset])
Adafruit_Si7021 Si7021 = Adafruit_Si7021(&Wire1);
Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1);;
// attention pour la carte GY-21P utiliser BMP280_ADDRESS_ALT (0x76) au lieu de 0x77 
double Si7021_humid, Si7021_temp, /*BMP280_temp,*/ BMP280_Pressure;

const uint16_t PixelCount = 4; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 25;  // ESP32
#define colorSaturation 16  // 32 : plus lumineux
NeoPixelBus <NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
//NeoPixelBus<NeoRgbFeature, Neo400KbpsMethod> strip(PixelCount, PixelPin);
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);
RgbColor ltblue(0, colorSaturation*0.8, colorSaturation); // 255,105,180
RgbColor yellow(colorSaturation, colorSaturation, 0);
RgbColor orange(colorSaturation, colorSaturation * 0.6, 0);
RgbColor pink(colorSaturation, 0, colorSaturation); // 255,105,180
RgbColor violet(colorSaturation * 0.5, 0, colorSaturation * 0.8); // minutes
RgbColor violetred(colorSaturation * 0.8, 0, colorSaturation * 0.8);
RgbColor blueviolet(colorSaturation * 0.3, 0, colorSaturation);

// MXP5050dp Differential Pressure Sensor
// If max v of 4.5v = 50kPa, then sensorValue of 921 (920.7) = 50kPa
const int MXPsensorPin = 34;   // GPIO 34 (Analog ADC1_CH6)
int MXPsensorValue = 0, MXPsensorMax = 1023, MXPsensorOffset = 0;    // 52
//todo revoir les tensions car alim en 3.3V
//float MXPvoltage=0, MXPkpa=0, MXPvoltageMax=5.0, MXPkpaRangeTopVoltage=4.5;
float MXPvoltage = 0, MXPkpa = 0, MXPvoltageMax = 3.3, MXPkpaRangeTopVoltage = 2.97;

const int TopReedSwitchPin = 32;    // input and pull-up
const int BottomReedSwitchPin = 33; // input and pull-up
const int IRdeflectiveSensorPin = 26; // TCRT5000 IR deflective sensor Input

volatile int IRdeflState = 0, IRdeflOldState = 0;
int TopReedState = 1, BottomReedState = 1;
volatile int Position = 0, PositionMax = 0;
const int mmUnit = 8;   // unit of IR position sensor between lines


int tcntMLI = 0, countReg = 0;
const int tcntMLIPRD = 100;
hw_timer_t *timer = NULL;
hw_timer_t *timerIR = NULL;
const double Tsamp = 0.02;  // 20 ms
const double Tsamp_us = 20000;  // 20 ms = 20000 us
struct {
    unsigned Running: 1;
    unsigned Display: 1;
    unsigned Sensor: 1;
    unsigned unused: 29;
} Flags;

// profile
double Uin, Upl, Uex, Upe;
double Tin, Tpl, Tex, Tpe, Tcycle;
unsigned int iTin, iTpl, iTex, iTpe, iTcycle,
        icnt;
int phase = 0;
double kPindt = 0.02;
double kPexdt = 0.01;

// curves
unsigned int it = 0;
const int itmax = 1000;
volatile double Press, Vol, Umot; //todo debg increm
double t_Press[itmax], t_Vol[itmax], t_Umot[itmax];

bool SOFTAP = false;
const char *ssid_AP = "esp32_";
const char *password_AP = "stpassword";
String ssid, password;

#define iotnamedatakey        "name"
#define datetimedatakey        "datetime"
#define tempdatakey                "temp"
#define humiddatakey            "humid"
#define accelXdatakey            "accelX"
#define accelYdatakey            "accelY"
#define accelZdatakey            "accelZ"

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
IPAddress ip(192, 168, 1, 30); // where xx is the desired IP Address
IPAddress gateway(192, 168, 1, 1); // set gateway to match your network
AsyncWebServer server(80);


//---------------------------------------------------
String getFormattedDateTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return "NA";
    }
    char S[50]; //50 chars should be enough
    strftime(S, 50, "%d/%m/%Y %H:%M:%S", &timeinfo);
    String SS = String(S);
    return (SS);
}

//-----------------------------------------------------
void WriteParam() {
    Serial.println("-- Writing Param to SPIFFS on : ");
//	String datetime_str = getFormattedDateTime();
//  Serial.println(datetime_str);
    StaticJsonDocument<256> doc;
    doc["Tin"] = Tin;
    doc["Tpl"] = Tpl;
    doc["Tex"] = Tex;
    doc["Tpe"] = Tpe;
    serializeJsonPretty(doc, Serial);
    File configFile = SPIFFS.open("/f_param.json", "w");
    if (!configFile) {
        Serial.println("Failed to open parameters file");
        return;
    }
    // Serialize JSON to SPIFFS file
    if (serializeJson(doc, configFile) == 0) {
        Serial.println(F("Failed to write to SPIFFS file"));
    }
    configFile.close();
}

//-----------------------------------------------------
void ReadParam() {
    Serial.println("-- Reading Param from SPIFFS ");
    StaticJsonDocument<256> doc;
    File configFile = SPIFFS.open("/f_param.json", "r");
    if (!configFile) {
        Serial.println("Failed to open parameters file");
        Serial.println("Using default parameters");
        SetDefaultParam();
        return;
    }
    DeserializationError err = deserializeJson(doc, configFile);
    configFile.close();
    if (err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.c_str());
        Serial.println("Using default parameters");
        SetDefaultParam();
        return;
    }
    serializeJsonPretty(doc, Serial);
    //const char* name = doc["name"];
    Tin = doc["Tin"].as<double>();
    Tpl = doc["Tpl"].as<double>();
    Tex = doc["Tex"].as<double>();
    Tpe = doc["Tpe"].as<double>();

    if (Tin == 0) SetDefaultParam();
}

//-----------------------------------------------------
void APWifiParam() {
    Serial.println("Empty wifi Param from SPIFFS, use AP :");
    ssid = ssid_AP;
    password = password_AP;
    SOFTAP = true;
    Serial.print("AP Wifi Server: ");
    Serial.println(ssid);
    Serial.print("PW: ");
    Serial.println(password);
}

//-----------------------------------------------------
void ReadWifiParam() {
    Serial.println("-- Reading Wifi Param from SPIFFS");
    StaticJsonDocument<256> doc;
    File configFile = SPIFFS.open("/wifi.json", "r");
    if (!configFile) {
        Serial.println("Failed to open parameters file, using AP");
        APWifiParam();
        return;
    }
    DeserializationError err = deserializeJson(doc, configFile);
    configFile.close();
    if (err) {
        Serial.print(F("deserializeJson() failed with code, using AP "));
        Serial.println(err.c_str());
        APWifiParam();
    }
//	serializeJsonPretty(doc, Serial);
    ssid = doc["ssid"].as<String>();
    Serial.print("Wifi SSID lu : ");
    Serial.println(ssid);
    password = doc["pw"].as<String>();

    if (ssid.isEmpty()) {
        Serial.println("Wifi parameters file empty, using AP");
        APWifiParam();
    };
}

//-----------------------------------------------------
void ReadSensor() {
//  BMP280_temp = bmp.readTemperature(); inutile
    BMP280_Pressure = bmp.readPressure();
    Si7021_humid = Si7021.readHumidity();
    Si7021_temp = Si7021.readTemperature();
    // Read MXP5050dp sensor & adjust with offset
    MXPsensorValue = analogRead(MXPsensorPin) - MXPsensorOffset;
    if (MXPsensorValue == 0) {
        MXPkpa = 0;
        MXPvoltage = 0;
    } else {
        MXPvoltage = MXPsensorValue * (MXPvoltageMax / MXPsensorMax);
        MXPkpa = ((MXPvoltage / MXPkpaRangeTopVoltage) - 0.04) / 0.018;    //Voltage to KPA calculation
    }
}

void IRAM_ATTR IRdeflISR() {    // ISR à 200 us pour compter le déplacement
    IRdeflState = digitalRead(IRdeflectiveSensorPin);
    if (IRdeflState != IRdeflOldState) {
        if (!IRdeflState && IRdeflOldState) { // passage blanc vers noir
            if (Umot > 0) Position++;
            if (Umot < 0) Position--;
        }
        IRdeflOldState = IRdeflState;
    }
}

void CheckLimits() {
    TopReedState = digitalRead(TopReedSwitchPin);
    BottomReedState = digitalRead(BottomReedSwitchPin);
    if (!TopReedState) { // si Top
        strip.SetPixelColor(1, ltblue);
        Position = 0;
        if (phase==3) { // si monte
            Umot = 0;
            phase = 4;  // -> PEEP : arrête de monter
            icnt = iTpe;
            strip.SetPixelColor(3, blue);
        }
    }
    if (!BottomReedState) { // si Bottom
        strip.SetPixelColor(1, pink);
        PositionMax = Position;
        if (phase==1) { // si descend
            Umot = 0;
            phase = 2;  // -> Plateau : arrête de descendre
            icnt = iTpl;
            strip.SetPixelColor(3, violet);
        }
    }
    if (TopReedState && BottomReedState) strip.SetPixelColor(3, yellow);
}
//---------------------------------------------------
void ReadPress() {
    if (phase == 0) {
        if (Flags.Running)   {
            phase = 1; // nouvelle respiration
            strip.SetPixelColor(0, green);
        }
    }
    else if (--icnt == 0) {
        phase++;
        if (phase > 4) {
            if (!Flags.Running) {
                phase = 0;  // arrêt en haut
                strip.SetPixelColor(0, red);
            } else {
                phase = 1; // nouvelle respiration
                strip.SetPixelColor(0, green);
            }
        }
        switch (phase) {
            case 1 : // Inspiration
                icnt = iTin;
                strip.SetPixelColor(1, red);
                break;
            case 2 : // Plateau
                icnt = iTpl;
                strip.SetPixelColor(1, pink);
                break;
            case 3 : // Exspiration
                icnt = iTex;
                strip.SetPixelColor(1, blue);
                break;
            case 4 : // PEEP
                icnt = iTpe;
                strip.SetPixelColor(1, ltblue);
                break;
        }
    }
    CheckLimits();
// à changer pour le calcul de la pression ou de la montée progressive de Umot
    Press = MXPkpa; // debug
    Vol = MXPvoltage;
    switch (phase) {
        case 0 : // arrêt
            Umot = 0;
            strip.SetPixelColor(2, black);
            break;
        case 1 : // Inspiration
//            Press = 1.1; // debug
//            Vol += Press * kPindt;
            Umot = Uin;
            strip.SetPixelColor(2, red);
            break;
        case 2 : // Plateau
            Umot = Upl;
            strip.SetPixelColor(2, pink);
            break;
        case 3 : // Exspiration
//            Press = MXPkpa; // debug
//            Vol -= Press * kPexdt;  // mesure du volume réelle ?
            Umot = Uex;
            strip.SetPixelColor(2, blue);
            break;
        case 4 : // PEEP
            Umot = Upe;
            strip.SetPixelColor(2, violetred);
            break;
    }
    // duty cycle
    PDC = Half_U * Umot + HalfPWM;
    strip.Show();   // rafraichi    todo check ISR
}

//---------------------------------------------------
String sendDataArray() {
    char data[100];
    int i;
    const int ds = 5; // 1 pt sur 5 donc 100 ms
    String resp = "{\"Press\":[";
    sprintf(data, "{\"Press\":[");
    for (i = 0; i < itmax; i += ds) {
        sprintf(data, "%.2f,", t_Press[i]);
        resp += data;
    }
    resp.setCharAt(resp.length() - 1, ']');
    resp += ",\"Umot\":[";
    for (i = 0; i < itmax; i += ds) {
        sprintf(data, "%.2f,", t_Umot[i]);
        resp += data;
    }
    resp.setCharAt(resp.length() - 1, ']');
    resp += ",\"Vol\":[";
    for (i = 0; i < itmax; i += ds) {
        sprintf(data, "%.2f,", t_Vol[i]);
        resp += data;
    }
    resp.setCharAt(resp.length() - 1, ']');
    resp += "}";
//  Serial.println(resp);
    return resp;
}

//---------------------------------------------------
String sendData() {
    char data[200];
    sprintf(data, "{\"Press\":%.2f,\"Umot\":%.2f,\"Vol\":%.2f,\"bPress\":%.2f,\
            \"Temp\":%.2f,\"Humid\":%.2f,\
            \"Pos\":%d,\"PosMax\":%d,\"Run\":%d}",
            Press, Umot, Vol, BMP280_Pressure,
            Si7021_temp, Si7021_humid,
            Position*mmUnit, PositionMax*mmUnit, Flags.Running);
//    sprintf(data, "{\"Press\":%.2f,\"Umot\":%.2f,\"Vol\":%.2f,\"bPress\":%.2f,\"Temp\":%.2f,\"Humid\":%.2f,\"Run\":%d}",
//    Press, Umot, Vol, BMP280_Pressure, Si7021_temp, Si7021_humid, Flags.Running);
    Serial.println(data);
    return data;
}

//---------------------------------------------------
String sendParam() {
    char data[200];
    sprintf(data, "{\"Tin\":%.2f,\"Tpl\":%.2f,\"Tex\":%.2f,\"Tpe\":%.2f}",
            Tin, Tpl, Tex, Tpe);
    Serial.println(data);
    return data;
}

//---------------------------------------------------
String getFile() {
    char data[200];
    int i, il;
    const int ds = 5; // 1 pt sur 5 donc 100 ms
    String page = "e-nefs data\n";
    page += "DTH: ";
    if (!SOFTAP) page += getFormattedDateTime();
    page += "\n\nParameteres\n";
    page += "Tin,Tpl,Tex,Tpe\n";
    sprintf(data, "%.2f,%.2f,%.2f,%.2f\n",
            Tin, Tpl, Tex, Tpe);
    page += data;
    page += "\n\nData:\n";
//    sprintf(data, "it,%d,its,%.2f\n", it, it*Tsamp); //  debug
//    page += data;
    page += "t,Press,Vol,Umot\n";
    int itp1 = it+1;
    for (i = it+1; i < it+1+itmax; i += ds) {
        //il = i % itmax;
        il = i;
        if (il>itmax)   il -= itmax;
        sprintf(data, "%.2f,%.2f,%.2f,%.2f\n",
                Tsamp*(i-itp1), t_Press[il], t_Vol[il], t_Umot[il]);
        page += data;
    }
//  Serial.println(page);
    return page;
}

//---------------------------------------------------
void IRAM_ATTR onTimer() { // ici on est toutes les Tsamp = 20 ms
    ReadPress();
    if (++Sensorcount >= NSensorcount) {
        Sensorcount = 0;   // 200 ms
        Flags.Sensor = 1;
    }
    t_Press[it] = Press;
    t_Vol[it] = Vol;
    t_Umot[it] = Umot;
    if (++it >= itmax) it = 0;

    if (++Dispcount >= DispNcount) {
        Dispcount = 0;
        Flags.Display = 1;
    }
    // essai PWM sur LED to fade
    ledcWrite(Channel_L_PWM, PDC);
    ledcWrite(Channel_R_PWM, FullPWM - PDC);
    ledcWrite(ledChannel, PDC);     // LED blue affiche le niveau de tension.
}

//---------------------------------------------------
void SetDefaultParam() {
    Tin = 1.5;
    Tpl = 0.2;
    Tex = 4.0;
    Tpe = 0.3;
}

//---------------------------------------------------
// init Variables
void initVar() {
    Uin = 6;
    Upl = 0;
    Uex = -4;
    Upe = 0;
    RefreshiTxx();
    Vol = 0;
    Press = 0;
    Umot = 0;
    PDC = HalfPWM;
    phase = 1;  // démarre inspiration
    icnt = iTin;   // démarre cycle Inspiration
    it = 0;   // démarre stockage des mesures
}

//---------------------------------------------------
// Refresh parameter
void RefreshiTxx() {
    Tcycle = Tin + Tpl + Tex + Tpe;
    iTin = Tin * 50;
    iTpl = Tpl * 50;
    iTex = Tex * 50;
    iTpe = Tpe * 50;
    iTcycle = Tcycle * 50;
//  icnt = ???; // debug, on démarre à chaque fois ?
}

//---------------------------------------------------
// Refresh OLED display
void refreshDisplay() {
    Serial.print("Phase is ");
    Serial.print(phase);
    Serial.print(" :  icnt = ");
    Serial.print(icnt);
    Serial.print(" :  Umot = ");
    Serial.print(Umot, 2);
    Serial.print(" V   Press = ");
    Serial.print(Press, 2);
    Serial.println(" Pa");

    Serial.print("BMP280 Press: ");
    Serial.print(BMP280_Pressure);
    Serial.print(" Pa    Si7021 Humid: ");
    Serial.print(Si7021_humid, 2);
    Serial.print("  Temp: ");
    Serial.println(Si7021_temp, 2);

    // todo debug
    Serial.print("MXPsensorValue(offset");
    Serial.print(MXPsensorOffset);
    Serial.print("): ");
    Serial.print(MXPsensorValue, DEC);
    Serial.print("   Voltage: ");
    Serial.print(MXPvoltage, 3);
    Serial.print("   MXPkpa: ");
    Serial.println(MXPkpa, 3);
    // todo debug
    Serial.print("** Top: ");
    Serial.print(TopReedState);
    Serial.print("  Bot: ");
    Serial.print(BottomReedState);
    Serial.print("  IR: ");
    Serial.print(IRdeflState);
    Serial.print("  Pos: ");
    Serial.println(Position);

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setCursor(0, 16);
    u8g2.print("ph ");
    u8g2.print(phase);
    u8g2.print(" T=");
    u8g2.print(Si7021_temp, 1);
    u8g2.print("°C");
    u8g2.setCursor(0, 32);
    u8g2.print("U ");
    u8g2.print(Umot, 1);
    u8g2.print(" V H ");
    u8g2.print(Si7021_humid, 1);
    u8g2.print("%");
    u8g2.setCursor(0, 48);
    u8g2.print("P=");
    u8g2.print(Press, 2);
    u8g2.print(" bar");
    u8g2.setCursor(0, 64);
    u8g2.print("P=");
    u8g2.print(BMP280_Pressure, 1);
    u8g2.print(" Pa");

//    u8g2.setCursor(0, 64);
//    u8g2.print("E=");   
//    u8g2.print(Emes,3);   
//    u8g2.print(" kWh");  
    u8g2.sendBuffer();
}

//---------------------------------------------------
void setup() {
    Serial.begin(SERIAL_SPEED);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);  // OFF
    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);   //PWM Value varries from 0 to 1023
    // attach the channel to the GPIO2 to be controlled
    ledcAttachPin(LED_BUILTIN, ledChannel);
    pinMode(TopReedSwitchPin, INPUT_PULLUP); // input and pull-up
    pinMode(BottomReedSwitchPin, INPUT_PULLUP); // input and pull-up
    pinMode(IRdeflectiveSensorPin, INPUT); // TCRT5000 IR deflective sensor
    strip.Begin();
    strip.Show();
// Configure Motor Control PWM
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(EN_PWM, OUTPUT);
    digitalWrite(EN_PWM, 1);        // Enable PWM
    ledcSetup(Channel_L_PWM, f_PWM, res_PWM);   // PDCx 0-4095
    ledcSetup(Channel_R_PWM, f_PWM, res_PWM);   // PDCx 0-4095
    ledcAttachPin(L_PWM, Channel_L_PWM);
    ledcAttachPin(R_PWM, Channel_R_PWM);
    ledcWrite(Channel_L_PWM, 0);    // L_PDC = 0
    ledcWrite(Channel_R_PWM, 0);    // R_PDC = 0

    // Initialize SPIFFS
    if (!SPIFFS.begin()) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    Serial.println(F("Starting e-nefs Program"));
    Serial.println("Connexion BMP280 Si7021");
    Wire1.begin(GY21_SDA, GY21_SCL);
    Serial.println(F("BMP280 test"));
    if (!bmp.begin(BMP280_ADDRESS_ALT)) {  // 0x76
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
//    while (1);
    }
    Serial.println("Si7021 test");
    if (!Si7021.begin()) {
        Serial.println("Did not find Si7021 sensor!");
        //   while (true);
    }
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); // No Standby time !

    Serial.print(F("Initializing OLED and Wifi\n"));
    u8g2.begin();
    u8g2.setFont(u8g2_font_helvB12_tr);  // set the target font to calculate the pixel width
    u8g2.setFontMode(1);    // enable transparent mode, which is faster
    ReadParam();  // Read config parameters from Flash
    initVar();
    ReadWifiParam();  // Read Wifi config from Flash

    if (SOFTAP) {
        WiFi.softAP(ssid.c_str(), password.c_str());
        // Connexion WiFi établie / WiFi connexion is OK
        Serial.println("");
        Serial.print("Wifi Server: ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.softAPIP());
    } else {
        IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your network
        IPAddress dns1(8, 8, 8, 8);
        IPAddress dns2(8, 8, 4, 4);
        WiFi.begin(ssid.c_str(), password.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.config(ip, gateway, subnet, dns1, dns2);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        // Connexion WiFi établie / WiFi connexion is OK
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        //init and get the time
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        Serial.println(getFormattedDateTime());
    }

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setCursor(0, 16);
    u8g2.print("Connection Wifi");
    u8g2.setCursor(0, 32);
    u8g2.print(ssid);
    u8g2.setCursor(0, 48);
    u8g2.print(WiFi.localIP());
    u8g2.sendBuffer();

    // route principale
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        timerAlarmDisable(timer);
        request->send(SPIFFS, "/index.html");
        timerAlarmEnable(timer);
    });

    // route pour la page de config des paramètres
    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        timerAlarmDisable(timer);
        int paramsNr = request->params();
        Serial.println("-------- config --------");
        for (int i = 0; i < paramsNr; i++) {
            AsyncWebParameter *p = request->getParam(i);
            Serial.print("Param name: ");
            Serial.println(p->name());
            Serial.print("Param value: ");
            Serial.println(p->value());
            if (p->name() == "Tin") {
                Tin = p->value().toDouble();
//          Serial.print("change Kp : ");
//          Serial.println(Kp);
            }
            if (p->name() == "Tpl") {
                Tpl = p->value().toDouble();
            }
            if (p->name() == "Tex") {
                Tex = p->value().toDouble();
            }
            if (p->name() == "Tpe") {
                Tpe = p->value().toDouble();
            }
        }
        if (paramsNr > 0) {
            WriteParam(); // Write Config Parameters to Flash
            RefreshiTxx();
        }
        request->send(SPIFFS, "/config.html");
        timerAlarmEnable(timer);
    });

    // route de download  text/csv
    server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
        timerAlarmDisable(timer);
        request->send(200, "text/csv", getFile());  // envoie le fichier
        timerAlarmEnable(timer);
    });
    // route to send ARRAY of Press, Umot, Vol data
    server.on("/dataarray.json", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", sendDataArray());  // envoie les Datas 1000 pts
    });
    // route to send Press, Umot, Vol data
    server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", sendData());  // envoie les Datas 1 pt
    });
    server.on("/run", HTTP_GET, [](AsyncWebServerRequest *request) {
        Flags.Running = !Flags.Running;
        String data;
        if (Flags.Running) data = "{\"Run\":1}";
        else data = "{\"Run\":0}";
        request->send(200, "application/json", data);
    });
    // route to send pressure data
    server.on("/param.json", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", sendParam());  // envoie les paramètres
    });
    /* trop lourd, reset l'ESP32
     // route for script files
    server.on("/highcharts.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/highcharts.js", "text/javascript");
    });
    server.on("/jquery-1.11.2.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery-1.11.2.min.js", "text/javascript");
    });
  */
    // route de external
//  server.on("/embesystems.com", HTTP_GET, [](AsyncWebServerRequest *request){
//      request->redirect("http://embesystems.com/produits.php");  
//  });

    server.begin();
    Serial.println("HTTP server started");
    Serial.println();
//    attachInterrupt(IRdeflectiveSensorPin, IRdeflISR, FALLING);
    timerIR = timerBegin(1, 80, true);      // config timer1
    timerAttachInterrupt(timerIR, &IRdeflISR, true);
    timerAlarmWrite(timerIR, 200, true);  // config timer1 200 us
    timerAlarmEnable(timerIR);

    timer = timerBegin(0, 80, true);        // config timer0
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, Tsamp_us, true);  // config timer0  20 ms
    timerAlarmEnable(timer);

    Flags.Running = 1;    // starts On : debug then swith off
    strip.SetPixelColor(0, green);
    Flags.Display = 0;
} // of method setup()
//---------------------------------------------------
void loop() {
    if (Flags.Sensor == 1) {
        ReadSensor();
        Flags.Sensor = 0;
    }
    if (Flags.Display == 1) {
        refreshDisplay();
        Flags.Display = 0;
    }
}
