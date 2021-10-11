#include "esp_camera.h"
#include <WiFi.h>
#include "DHT.h"

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#include "camera_pins.h"

const char* ssid = "hamzam0n";
const char* password = "@pass$word@";

extern int gpLb = 14; // Left 1
extern int gpLf = 15; // Left 2
extern int gpRb = 13; // Right 1
extern int gpRf = 12; // Right 2
extern int gpLed =  4; // Light
//servo = 2 ; gas=16 ; dht=0
extern int servoPin = 3;  // set digital pin GPIO2 as servo pin (use SG90)

//const int gasPin = 2;
// values

int tempSensor = 0 ;
int humSensor = 0 ;
int gasSensor = 0 ;
int flag = 1001 ;

extern int usingUltraSonic  ;


extern int triggerPin = 0;
extern int echoPin = 16;


#define DHTPIN 2
#define DHTTYPE DHT11

// object initialization
DHT dht(DHTPIN, DHTTYPE);

long duration, distance;

void startCameraServer();
void display_distance();
void display_temperature();


void initUltraSonic(){
  pinMode(triggerPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);
}

/*
void InitGasSensor(){
  pinMode(gasPin,INPUT_PULLUP);
}*/
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
void initMotors()
{
  pinMode(gpLb, OUTPUT); //Left Backward
  pinMode(gpLf, OUTPUT); //Left Forward
  pinMode(gpRb, OUTPUT); //Right Forward
  pinMode(gpRf, OUTPUT); //Right Backward

  //initialize
  digitalWrite(gpLb, LOW);
  digitalWrite(gpLf, LOW);
  digitalWrite(gpRb, LOW);
  digitalWrite(gpRf, LOW);
}

void initServo() {
  ledcSetup(8, 50, 16); /*50 hz PWM, 16-bit resolution and range from 3250 to 6500 */
  ledcAttachPin(servoPin, 8);
}

void initFlash() {
  ledcSetup(7, 5000, 8); /* 5000 hz PWM, 8-bit resolution and range from 0 to 255 */
  ledcAttachPin(gpLed, 7);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("starting....");
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  flag = 1001;
  usingUltraSonic = 0;
  initMotors();
  initServo();
  initFlash();
  dht.begin();
  //initUltraSonic();
  //InitGasSensor();
  //dht.begin();

  //flash
  
  for (int i = 0; i < 3; i++) {
    ledcWrite(7, 10);
    delay(300);
    ledcWrite(7, 0);
    delay(100);
  }

  
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {

  if(flag > 1000 ){
    humSensor = dht.readHumidity();
    // Read temperature in Celsius, for Fahrenheit use .readTempF()
    tempSensor = dht.readTempC();
    //gasSensor = analogRead(gasPin);
    Serial.print(F("Humidity: ")); Serial.print(humSensor); Serial.print(F(" [%]\t"));
    Serial.print(F("Temp: ")); Serial.print(tempSensor); Serial.println(F(" [C]\t"));
  //  Serial.print(F("GAS: ")); Serial.println(analogRead(gasPin));
    delay(10);
    flag = 0 ;
  }else{
    initUltraSonic();
    display_distance();
    
      //display_temperature();
      if(distance < 30 ) {
        WheelMove(LOW, LOW, LOW, LOW);
        delay(600);
      }
      if(usingUltraSonic != 0 ){
        do{
          //turn right
          WheelMove(LOW, HIGH, HIGH, LOW);
          delay(30);
          display_distance();
        }while(distance < 30 );
        
         WheelMove(HIGH, LOW, HIGH, LOW); // move forword
      }
    }
    
  }

 Serial.print("flag = ");  Serial.println(flag);
 Serial.print("usingUltraSonic = ");  Serial.println(usingUltraSonic);
  
 
  flag++; 
  delay(100);
}


void display_distance(){
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = (long) pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
}
/*
void display_temperature(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F(" Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("C "));
  Serial.print(f);
  Serial.print(F("F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("C "));
  Serial.print(hif);
  Serial.println(F("F"));
}
*/

void WheelMove(int nLf, int nLb, int nRf, int nRb)
{
 digitalWrite(gpLf, nLf);
 digitalWrite(gpLb, nLb);
 digitalWrite(gpRf, nRf);
 digitalWrite(gpRb, nRb);
}
