////
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
const int cStatusLED = 27;                                     // GPIO pin of commmunication status LED
const int cTCSLED = 23;                                        // GPIO pin of TCS led
const int cHeartbeatInterval = 500;                            // heartbeat blink interval in ms
const int cNumMotors = 2;                                      // Number of DC motors
const int cIN1Pin[] = {17, 19};
const int cIN1Chan[] = {0, 1};
const int cIN2Pin[] = {16, 18};
const int cIN2Chan[] = {2, 3};
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
                      {00, 00, 0}};
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
  if (esp_now_init() != ESP_OK
  {
    Serial.printf("Error initializing ESP-NOW\n");
    return;
  }
  else
  {
    Serial.printf("Sucessfully initialized ESP-NOW\n");
  }
  esp_now_register_recv_cb(onDataRecv);               // register callback function for received data
  esp_now_register_send_cb(onDataSent);               // register callback function for data transmission

// Set controller info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel
  peerInfo.encrypt = false;                           // no encryption of data

// Add controller as ESP-NOW peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.printf("Failed to add peer\n");
    return;
  }
  else
  {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n to list", receiverMacAddress[0], receiverMacAddress[1], 
                                                                         receiverMacAddress[2], receiverMacAddress[3], 
                                                                         receiverMacAddress[4], receiverMacAddress[5]);
  }
}

void loop() {
  float deltaT = 0;                                   // time interval
  long pos[] = {0, 0};                                // current motor positions
  float velEncoder[] = {0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0};                         // change in position for set speed
  long e[] = {0, 0};                                  // position error
  float ePrev[] = {0, 0};                             // previous position error
  float dedt[] = {0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0};                         // integral of error 
  float u[] = {0, 0};                                 // PID control signal
  int pwm[] = {0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1};                                 // direction that motor should turn
  uint16_t r, g, b, c;
  int ledState;

  if (tcsFlag)  {
    tcs.getRawData(&r, &g, &b, &c);
#ifdef PRINT_COLOUR
          Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
    if (r > 35 && g > 10 && b > 60)  {
      driveData.ledState = 1;
    }
  }

  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
   if (commsLossCount > cMaxDroppedPackets) {
    delay(1000);                                      // okay to block here as nothing else should be happening
    ESP.restart();                                    // restart ESP32
  }

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                     // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          // read and store current motor position
  }
  interrupts();                                       // turn interrupts back on

  unsigned long curTime = micros();                   // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle
    driveData.time = curTime;                         // update transmission time
    for (int k = 0; k < cNumMotors; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      // update target for set direction
      posChange[k] = (float) (inData.dir * map(inData.potPos, 0, 4095, 0, cMaxChange)); // update with maximum speed
      targetF[k] = targetF[k] + posChange[k];         // set new target position
      if (k == 0) {                                   // assume differential drive
        target[k] = (long) targetF[k];                // motor 1 spins one way
      }
      else {
        target[k] = (long) -targetF[k];               // motor 2 spins in opposite direction
      }

      // use PID to calculate control signal to motor
      e[k] = target[k] - pos[k];                      // position error
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    // derivative of error
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    // integral of error (finite difference)
      u[k] = kp * e[k] + kd * dedt[k] + ki * eIntegral[k]; // compute PID-based control signal
      ePrev[k] = e[k];                                // store error for next control cycle
  
      // set direction based on computed control signal
      dir[k] = 1;                                     // default to forward directon
      if (u[k] < 0) {                                 // if control signal is negative
        dir[k] = -1;                                  // set direction to reverse
      }

      // set speed based on computed control signal
      u[k] = fabs(u[k]);                              // get magnitude of control signal
      if (u[k] > cMaxSpeedInCounts) {                 // if control signal will saturate motor
        u[k] = cMaxSpeedInCounts;                     // impose upper limit
      }
      pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM); // convert control signal to pwm
      if (commsLossCount < cMaxDroppedPackets / 4) {
        
        if (k == 0 && inData.steer == 1)  {
        setMotor(dir[k] = -1, pwm[k], cIN1Chan[k], cIN2Chan[k]); // update motor speed and direction
        }
        else if (k == 1 && inData.steer == -1)  {
          setMotor(dir[k] = -1, pwm[k], cIN1Chan[k], cIN2Chan[k]);    // only change made is instead of turning off motor, reverse direction to turn in place
        }
        else  {
          setMotor(dir[k], pwm[k], cIN1Chan[k], cIN2Chan[k]);
        }
      }
      else {
        setMotor(0, 0, cIN1Chan[k], cIN2Chan[k]);     // stop motor
      }

#ifdef SERIAL_STUDIO
      if (k == 0) {
        printf("/*");                                 // start of sequence for Serial Studio parsing
      }
      printf("%d,%d,%d,%0.4f", target[k], pos[k], e[k], velMotor[k]);  // target, actual, error, velocity
      if (k < cNumMotors - 1) {
        printf(",");                                  // data separator for Serial Studio parsing
      }
      if (k == cNumMotors -1) {
        printf("*/\r\n");                             // end of sequence for Serial Studio parsing
      }
#endif
    }
 
    // send data from drive to controller
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &driveData, sizeof(driveData)); 
    if (result == ESP_OK) {                           // if sent successfully
      digitalWrite(cStatusLED, 0);                    // turn off communucation status LED
    }
    else {                                            // otherwise
      digitalWrite(cStatusLED, 1);                    // turn on communication status LED
    }
  }
  doHeartbeat();                                      // update heartbeat LED
}

// Heartbeat LED blink function
void doHeartbeat() {
  unsigned long curMillis = millis();                 // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}

// Motor control signals based on direction and PWM
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                     // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                              // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// Encoder ISR, argument is pointer to encoder structure, which is statically cast to an Encoder structure (more than 1 encoder ISR able to be created)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);            // cast pointer to static structure
  
  int b = digitalRead(s->chanB);                      // read state of channel B
  if (b > 0) {                                        // high, leading channel A
    s->pos++;                                         // increase position
  }
  else {                                              // low, lagging channel A
    s->pos--;                                         // decrease position
  }
}

// Callback function for when data is recieved
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)                                       // if empty packet
  {
    return;                                           // return
  }
  memcpy(&inData, incomingData, sizeof(inData));      // store drive data from controller
#ifdef PRINT_INCOMING
  Serial.printf("%d, %d\n", inData.dir, inData.time);
#endif
}

// callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef PRINT_SEND_STATUS
  Serial.printf("Last packet send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(cStatusLED, 1);                      // turn on communication status LED
    commsLossCount++;                                 // increase lost packet count
  }
  else {
    commsLossCount = 0;                               // reset communication loss counter
  }
}
