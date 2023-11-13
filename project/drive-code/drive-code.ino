//
// MME 4487 Project
//
// Language (Arduino C++)
// Target ESP32
// Authors: Eric De Faro, Nidhish Sochit, Laura K
//

// #define SERIAL_STUDIO
// #define PRINT_SEND_STATUS
// #define PRINT_INCOMING

#include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>
// #include <Adafruit_TCS34725.h>


// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);


// Control data packet structure
struct ControlDataPacket  
{
  int dir;                                                     // drive direction: 1 = forward, -1 = reverse, 0 = stop
  unsigned long time;                                          // time packet sent
  int steerDir;                                                // steering direction: 1 = left, -1 - right, 0 = straight
  int servoPos;                                                // Servo motor arm position to collect item
};


// Drive data packet structure
struct DriveDataPacket  
{
  unsigned long time;
  int ledState;
  uint16_t r, g, b, c;
};


// Encoder structure
struct Encoder
{
  const int chanA;                                             // GPIO pin for encoder channel A
  const int chanB;                                             // GPIO pin for encoder channel B
  long pos;                                                    // current encoder position
};


// Constants
const int cHeartBeatLED = 2;                                   // GPIO pin of built-in LED for heartbeat
const int cStatusLED = 00;                                     // GPIO pin of commmunication status LED
const int cTCSLED = 00;                                        // GPIO pin of TCS led
const int cHeartbeatInterval = 500;                            // heartbeat blink interval in ms
const int cNumMotors = 4;                                      // Number of DC motors
const int cIN1Pin[] = {};
const int cIN1Chan[] = {};
const int cIN2Pin[] = {};
const int cIN2Chan[] = {};
const int cPWMRes = 8;
const int cMinPWM = 0;
const int cMaxPWM = pow(2, cPWMres) - 1;
const int cPWMFreq = 20000;
const int cCountsRev = 1096;
const int cMaxSpeedInCounts = 1600;
const int cMaxChange = 14;
const int cMaxDroppedPackets = 20;
const float kp = 1.5;
const float ki = 0.2;
const float kd = 0.8;


// Variables
unsigned long lastHeartbeat = 0;                                // time of last heartbeat state change
unsigned long lastTime = 0;                                     // last time of motor control  update
unsigned int commsLossCount = 0;                                // number of sequential sent packets that have dropped
Encoder encoder1[] = {{00, 00, 0},                              // encoder 0 on GPIO 00 and 00, 0 position
                      {00, 00, 0}};                             // encoder 1 on GPIO 00 and 00, 0 position
// Encoder encoder2[] = {{00, 00, 0},                           // not sure if need to have more than one structure
//                       {00, 00, 0}};
long target[] = {0, 0};                                         // target count for motor
long lastEncoder[] = {0, 0};                                    // encoder count at last control cycle
float targetF[] = {0.0, 0.0};                                   // target for motor as float
ControlDataPacket inData;
DriveDataPacket driveData;


// Connection between the two ESP32 controllers
uint8_t receoverMacAddress[] = {0xB0, 0xA7, 0x32, 0x28, 0x8B, 0xB4};
esp_now_peer_info_t peerInfo = {};


// TCS colour sensor configuration
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;




void setup() {
  Serial.begin(115200);
  Wifi.mode(WIFI_STA);
  Serial.print("MAC Address ");
  Serial.println(WiFi.macAddress());
  Wifi.disconnect();


// Connect to TCS34725 colour sensor
  if (tcs.begin())  
  {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);
  } 
  else  
  {
    Serial.printf("No TCS34725 found ... check your conections\n");
    tcsFlag = false;
  }

  pinMode(cHeartBeatLED, OUTPUT);
  pinMode(cStatusLED, OUTPUT);
  pinMode(cTCSLED, OUTPUT);

// Setup motors with encoders
  for (int k = 0; k < cNumMotors; k++)  
  {
    ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);
    ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);
    ledcAttachPin(cIN2Pin[k], cIN2Chan[k]);
    ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);
    pinMode(encoder[k].chanA, INPUT);
    pinMode(encoder[k].chanB, INPUT);
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }


// Initialize ESP-NOW

// Set controller info

// Add controller as ESP-NOW peer

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Heartbeat LED blink function
void doHeartbeat()  {
}

// Motor control signals based on direction and PWM
void setMotor(int dir, int pwm, int in1, int in2) {
}

// Encoder ISR, argument is pointer to encoder structure, which is statically cast to an Encoder structure (more than 1 encoder ISR able to be created)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
}

// Callback function for when data is recieved
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)  {
}
