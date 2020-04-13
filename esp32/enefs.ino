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

// PWM
const uint8_t EN_PWM = 4; // L_EN et R_EN
const uint8_t L_PWM = 18;
const uint8_t R_PWM = 5;
const unsigned int Channel_L_PWM = 1;
const unsigned int Channel_R_PWM = 2;
const unsigned int f_PWM = 16000;  // 16 kHz -> not audible
const unsigned int res_PWM = 12;   // PDCx 0-4095
const unsigned int MinPWM  = 0;    // min duty cycle (PDCx) -> negative motor voltage
const unsigned int FullPWM = 4095; // max duty cycle (PDCx) -> positive motor voltage
const unsigned int HalfPWM = 2048; // half duty cycle -> zero motor voltage
//const double Umax = 12;           // 12 V DC bus
const double Half_U = 170.6667;     // HalfPWM / Umax
unsigned int PDC;
 
//int brightness = 0;    // how bright the LED is
//int fadeAmount = 1;    // how many points to fade the LED by
 
// setting LED PWM properties
const int freq = 16000;
const int ledChannel = 0;
const int resolution = 12; //Resolution 8, 10, 12, 15
 

const uint32_t SERIAL_SPEED     = 115200; ///< Set the baud rate for Serial I/O
#define r2 1.414213562373095
int NSensorcount = 20, Sensorcount=0; // 10 *20ms = 200 ms
int DispNcount = 50, Dispcount=0; // 50 *20ms = 1s
//int DispNcount = 100, Dispcount=0; // 100 *20ms = 2 s
//int  betatLed = 0;
String  etatLed = "OFF";
// BUILTIN_LED redefinir au lieu d' IO 5 c'est IO 22
#define BUILTIN_LED 22

#define OLED_SDA 23
#define OLED_SCL 19
#define GY21_SDA 13  // GY21 BMP280 / Si7021
#define GY21_SCL 15  // GY21 BMP280 / Si7021
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);   // (rotation, [reset [, clock, data]])
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);   // (rotation, clock, data [, reset]) 
Adafruit_Si7021 Si7021 = Adafruit_Si7021(&Wire1); 
Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire1); ; 
// attention pour la carte GY-21P utiliser BMP280_ADDRESS_ALT (0x76) au lieu de 0x77 
double Si7021_humid, Si7021_temp, /*BMP280_temp,*/ BMP280_Pressure;

int tcntMLI=0, countReg=0;
const int tcntMLIPRD = 100;
hw_timer_t * timer = NULL;
const int DS_TempReg = 1;
struct {
			unsigned Running		: 	1;
			unsigned Display		:		1;
			unsigned Sensor		:		1;
			unsigned unused 		:		29;
		}	Flags;

// profile
double Uin, Upl, Uex, Upe;
double Tin, Tpl, Tex, Tpe, Tcycle;
unsigned int iTin, iTpl, iTex, iTpe, iTcycle,
  icnt;
int phase = 0;
double kPindt = 0.02;
double kPexdt = 0.01;

// curves
unsigned int it=0;
const unsigned int itmax=1000;
double Press, Vol, Umot; 
double t_Press[itmax], t_Vol[itmax], t_Umot[itmax];

bool SOFTAP = false;
const char *ssid_AP = "esp32_";
const char *password_AP = "stpassword";
String ssid, password;

#define iotnamedatakey		"name"
#define datetimedatakey		"datetime"
#define tempdatakey				"temp"
#define humiddatakey			"humid"
#define accelXdatakey			"accelX"
#define accelYdatakey			"accelY"
#define accelZdatakey			"accelZ"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
IPAddress ip(192, 168, 1, 30); // where xx is the desired IP Address
IPAddress gateway(192, 168, 1, 1); // set gateway to match your network
AsyncWebServer  server ( 80 );
//---------------------------------------------------
String getFile()
{
  String page = "DTH: "; 
  if(! SOFTAP)  page +=getFormattedDateTime();
  page += "\nParameteres : ";
  page += Tin;
  page += " s\n";
  page += "Imes : ";
  page += Tpl;
  page += " s\n";
  page += "Vmes : ";
  page += Tex;
  page += " s\n";
  page += Tpe;
  page += " s\n";
  return page;
}
//---------------------------------------------------
String getFormattedDateTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return "NA";
  }
  char S[50]; //50 chars should be enough
  strftime(S, 50, "%d/%m/%Y %H:%M:%S", &timeinfo);
  String SS = String(S);
  return (SS);
}
//-----------------------------------------------------
void WriteParam(){
  Serial.println("-- Writing Param to SPIFFS on : ");
//	String datetime_str = getFormattedDateTime();
//  Serial.println(datetime_str);
	StaticJsonDocument<256> doc;
  doc["Tin"]	= Tin;
  doc["Tpl"]	= Tpl;
  doc["Tex"]	= Tex;
  doc["Tpe"]	= Tpe;
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
void ReadParam(){
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

  if (Tin==0) SetDefaultParam();
}
//-----------------------------------------------------
void APWifiParam(){
  Serial.println("Empty wifi Param from SPIFFS, use AP :");
  ssid = ssid_AP;
  password = password_AP;
  SOFTAP = true;
  Serial.print ( "AP Wifi Server: " ); Serial.println ( ssid );
  Serial.print ( "PW: " ); Serial.println ( password );
}
//-----------------------------------------------------
void ReadWifiParam(){
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
  Serial.print ( "Wifi SSID lu : " ); Serial.println ( ssid );
  password = doc["pw"].as<String>();

  if (ssid.isEmpty()) {
    Serial.println("Wifi parameters file empty, using AP");
    APWifiParam();
  };
}
//-----------------------------------------------------
void ReadSensor()
{
//  BMP280_temp = bmp.readTemperature(); inutile
  BMP280_Pressure = bmp.readPressure();
  Si7021_humid = Si7021.readHumidity();
  Si7021_temp = Si7021.readTemperature();
}
//---------------------------------------------------
void ReadPress()
{
  if (phase != 0)
    if (--icnt == 0)  {
        phase++;
        if (phase>4)  phase = 1; // nouvelle respiration
        switch (phase)
        {
        case 1 : // Inspiration
          icnt = iTin;
          break; 
        case 2 : // Plateau
          icnt = iTpl;
          break; 
         case 3 : // Exspiration
          icnt = iTex;
          break; 
        case 4 : // PEEP
          icnt = iTpe;
         break; 
        }
      }
// à changer pour le calcul de la pression ou de la montée progressive de Umot
  switch (phase)
  {
  case 0 : // arrêt
    Umot = 0;
    break; 
  case 1 : // Inspiration
    Press = 1.1; // debug
    Vol += Press*kPindt;
    Umot = Uin;
    break; 
  case 2 : // Plateau
    Umot = Upl;
    Press = 1.0; // debug
    break; 
   case 3 : // Exspiration
    Press = 0.95; // debug
    Vol -= Press*kPexdt;  // mesure du volume réelle ?
    Umot = Uex;
    break; 
  case 4 : // PEEP
     Umot = Upe;
   break; 
 }
 // duty cycle
 PDC = Half_U*Umot + HalfPWM;
}
//---------------------------------------------------
String sendDataArray()
{
char data[100];
int i;
const int ds=5; // 1 pt sur 5 donc 100 ms
String resp="{\"Press\":[";
  sprintf(data, "{\"Press\":[");
  for ( i=0; i<itmax; i+=ds) {  
    sprintf(data, "%.2f,", t_Press[i]);
    resp += data;
    }
  resp.setCharAt(resp.length()-1, ']');
  resp += ",\"Umot\":[";
  for ( i=0; i<itmax; i+=ds) { 
    sprintf(data, "%.2f,", t_Umot[i]);
    resp += data;
    }
  resp.setCharAt(resp.length()-1, ']');
  resp += ",\"Vol\":[";
  for ( i=0; i<itmax; i+=ds) {
    sprintf(data, "%.2f,", t_Vol[i]);
    resp += data;
    }
  resp.setCharAt(resp.length()-1, ']');
  resp += "}";
//  Serial.println(resp);
  return resp;
}
//---------------------------------------------------
String sendData() {
char data[100];
  sprintf(data, "{\"Press\":%.2f,\"Umot\":%.2f,\"Vol\":%.2f,\"bPress\":%.2f,\"Temp\":%.2f,\"Humid\":%.2f,\"Run\":%d}",
              Press, Umot, Vol, BMP280_Pressure, Si7021_temp, Si7021_humid, Flags.Running);
  Serial.println(data);
  return data;
}
//---------------------------------------------------
String sendParam() {
char data[100];
 sprintf(data, "{\"Tin\":%.2f,\"Tpl\":%.2f,\"Tex\":%.2f,\"Tpe\":%.2f}",
              Tin, Tpl, Tex, Tpe);
  Serial.println(data);
  return data;
}
//---------------------------------------------------
void IRAM_ATTR onTimer()
{ // ici on est toutes les 20 ms
    ReadPress();
    if (++Sensorcount>=NSensorcount)
      {
      Sensorcount = 0;   // 200 ms
      Flags.Sensor = 1;
      }

    t_Press[it] = Press;
    t_Vol[it] = Vol;
    t_Umot[it] = Umot;
    if (++it >= itmax)  it = 0;
    
 //   digitalWrite(BUILTIN_LED, betatLed);
    if (++Dispcount>=DispNcount)
      {
      Dispcount = 0;  
      Flags.Display = 1;
      }
  // essai PWM sur LED to fade
  ledcWrite(Channel_L_PWM, PDC);
  ledcWrite(Channel_R_PWM, FullPWM - PDC);
  ledcWrite(ledChannel, PDC);     // LED blue affiche le niveau de tension.
//  brightness = brightness + fadeAmount;
//  if (brightness <= 0 || brightness >= 4095)
//      fadeAmount = -fadeAmount;
}
//---------------------------------------------------
void SetDefaultParam()
{
  Tin=2.5; Tpl=0.1; Tex=5; Tpe=0.3;
}
//---------------------------------------------------
// init Variables
void initVar()
{
  Uin = 8; Upl=2; Uex=-5; Upe=0;
  RefreshiTxx();
  Vol = 0; Press = 0; Umot = 0;
  PDC = HalfPWM;
  phase = 1;  // démarre inspiration
  icnt = iTin;   // démarre cycle Inspiration
  it = 0;   // démarre stockage des mesures
}
//---------------------------------------------------
// Refresh parameter
void RefreshiTxx()
{
  Tcycle = Tin+Tpl+Tex+Tpe;
  iTin = Tin*50;
  iTpl = Tpl*50;
  iTex = Tex*50;
  iTpe = Tpe*50;
  iTcycle = Tcycle*50;
//  icnt = ???; // debug, on démarre à chaque fois ?
}
//---------------------------------------------------
// Refresh OLED display
void refreshDisplay()
{
  Serial.print("Phase is ");
  Serial.print(phase);
  Serial.print(" :  icnt = ");
  Serial.print(icnt);
  Serial.print(" :  Umot = ");
  Serial.print(Umot,2);
  Serial.print(" V   Press = ");
  Serial.print(Press,2);
  Serial.println(" Pa");

  Serial.print("BMP280 Press: ");
  Serial.print(BMP280_Pressure);
  Serial.print(" Pa    Si7021 Humid: ");
  Serial.print(Si7021_humid, 2);
  Serial.print("  Temp: ");
  Serial.println(Si7021_temp, 2); 

  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setCursor(0, 16);
  u8g2.print("ph ");   
  u8g2.print(phase);   
  u8g2.print(" T=");   
  u8g2.print(Si7021_temp,1);   
  u8g2.print("°C");   
  u8g2.setCursor(0, 32);
  u8g2.print("U ");   
  u8g2.print(Umot,1);   
  u8g2.print(" V H ");   
  u8g2.print(Si7021_humid,1);   
  u8g2.print("%");   
  u8g2.setCursor(0, 48);
  u8g2.print("P=");   
  u8g2.print(Press,2);   
  u8g2.print(" bar");   
  u8g2.setCursor(0, 64);
  u8g2.print("P=");   
  u8g2.print(BMP280_Pressure,1);   
  u8g2.print(" Pa");   

//    u8g2.setCursor(0, 64);
//    u8g2.print("E=");   
//    u8g2.print(Emes,3);   
//    u8g2.print(" kWh");  
    u8g2.sendBuffer();
}
//---------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_SPEED);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 1);  // OFF
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);   //PWM Value varries from 0 to 1023  
  // attach the channel to the GPIO2 to be controlled
  ledcAttachPin(BUILTIN_LED, ledChannel);
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
  if(!SPIFFS.begin()){
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

  if(SOFTAP)
    {
      WiFi.softAP ( ssid.c_str(), password.c_str() );
      // Connexion WiFi établie / WiFi connexion is OK
      Serial.println ( "" ); 
      Serial.print ( "Wifi Server: " ); Serial.println ( ssid );
      Serial.print ( "IP address: " ); Serial.println ( WiFi.softAPIP() );
    }
  else
    {
    IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your network
    IPAddress dns1(8,8,8,8);
    IPAddress dns2(8,8,4,4);
    WiFi.begin ( ssid.c_str(), password.c_str() );
    WiFi.mode(WIFI_STA);
    WiFi.config(ip, gateway, subnet, dns1, dns2);
    while ( WiFi.status() != WL_CONNECTED ) {
      delay ( 500 ); Serial.print ( "." );
    }
    // Connexion WiFi établie / WiFi connexion is OK
    Serial.println ( "" ); 
    Serial.print ( "Connected to " ); Serial.println ( ssid );
    Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );
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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      timerAlarmDisable(timer);
      request->send(SPIFFS, "/index.html");
      timerAlarmEnable(timer);
  });

  // route pour la page de config des paramètres
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
     timerAlarmDisable(timer);
     int paramsNr = request->params();
      Serial.println("-------- config --------");
      for(int i=0;i<paramsNr;i++) {
        AsyncWebParameter* p = request->getParam(i);
        Serial.print("Param name: ");
        Serial.println(p->name());
        Serial.print("Param value: ");
        Serial.println(p->value());
        if (p->name()== "Tin") {
          Tin = p->value().toDouble();
//          Serial.print("change Kp : ");
//          Serial.println(Kp);
          }
         if (p->name()== "Tpl") {
          Tpl = p->value().toDouble();
          }
        if (p->name()== "Tex") {
          Tex = p->value().toDouble();
          }
        if (p->name()== "Tpe") {
          Tpe = p->value().toDouble();
          }
        } 
    if (paramsNr>0) {
      WriteParam(); // Write Config Parameters to Flash
      RefreshiTxx();
      }
    request->send(SPIFFS, "/config.html");
    timerAlarmEnable(timer);
  });

  // route de download  text/csv
  server.on("/data.csv", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send ( 200, "text/csv", getFile() );  // envoie le fichier
  });
  // route to send ARRAY of Press, Umot, Vol data
  server.on("/dataarray.json", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send ( 200, "application/json", sendDataArray() );  // envoie les Datas 1000 pts
  });
  // route to send Press, Umot, Vol data
  server.on("/data.json", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send ( 200, "application/json", sendData() );  // envoie les Datas 1 pt
  });
  // route to send pressure data
  server.on("/param.json", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send ( 200, "application/json", sendParam() );  // envoie les paramètres
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
  Serial.println ( "HTTP server started" );
  Serial.println();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  //timerAlarmWrite(timer, 1000, true);  // config timer 1 ms
  timerAlarmWrite(timer, 20000, true);  // config timer 20 ms
  timerAlarmEnable(timer);

  Flags.Running = 1;    // starts On : debug then swith off
  Flags.Display = 0;
} // of method setup()
//---------------------------------------------------
void loop()
{
  if (Flags.Sensor==1)  {
    ReadSensor();
    Flags.Sensor = 0;
    }  
  if (Flags.Display==1)  {
    refreshDisplay();
    Flags.Display = 0;
    }  
}
