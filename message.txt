/*
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes an auto-calibration and offsets generator tasks. Different 
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin.

  The teapot processing example may be broken due FIFO structure change if using DMP
  6.12 firmware version. 
    
  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

#include <WiFi.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "MPU6050_6Axis_MotionApps612.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define redButton 16
#define blueButton 18
#define greenButton 23

const String serverUrl = "http://192.168.2.29:5501/update";

#define WIFI_NETWORK  "BellFIQ"
#define WIFI_PASSWORD "Nabeeha17"
#define WIFI_TIMEOUT_MS 20000 // 20 seconds timeout

#define buzzer 19

#define knob 15

#define accelThreshold 800 // 250
#define veloThreshold 4000
#define mpuRange 2

#define XOffset 20
#define YOffset 3
#define ZOffset -30

#define cycleDuration 200


/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming 
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if theer is no magnetometer present.

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/ 
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

int const INTERRUPT_PIN = 17;  // Define the interruption #0 pin
bool blinkState;

// matrixes that store velocity and acceleration data
// first box: 
// 0 => past value
// 1 => current value

// second box:
// 0 => X
// 1 => y
// 2 => z (up/down) (ignore)
int accelDataRaw[2][3];
float veloDataNorm[2][3];

float totalHoriDistTravelled = 0;

/*---State/Software Variables---*/ 
String programState = "idle";
//String[] vehicleCategories = {"Sedan", "SUV", "Truck", "Bus", "Motorcycle"};
std::string vehicleCategories[] = {"Sedan", "SUV", "Truck", "Bus", "Motorcycle"};
float gasBurnedPerKm[] = {0.09, 0.11, 0.16, 0.22, 0.04};
float co2BurnedNow = 0;
int activeVehicleIndex = 0; 


// matrix of distance

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void resetI2C() {
  Wire.end();        // Close the current I2C bus
  delay(100);        // Wait briefly before re-initializing
  Wire.begin();      // Reinitialize the I2C bus
  mpu.initialize();  // Reinitialize the MPU6050
}

void lcdWrite(String strLine1, String strLine2){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(strLine1);
    lcd.setCursor(0,1);
    lcd.print(strLine2);
}

void lcdWrite(String strLine1){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(strLine1);
}

int setupMPU(){
  // code courtesy of Electronic Cats
  // https://github.com/ElectronicCats/mpu6050/tree/master/examples/MPU6050_DMP6

  //-16k to 16k mapped to -2 g's to 2 g's
  mpu.setFullScaleAccelRange(mpuRange);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  while(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    lcdWrite("MPU6050 connection failed");
    resetI2C();
    delay(1000);
  }

  Serial.println("MPU6050 connection successful");

  /*Wait for Serial input
    Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());                 // Wait for data
  while (Serial.available() && Serial.read()); // Empty buffer again
  */

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  lcdWrite("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(89);
  mpu.setZGyroOffset(373);
  mpu.setXAccelOffset(-3073);
  mpu.setYAccelOffset(332);
  mpu.setZAccelOffset(16383);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    lcdWrite("MPU initializing...", "please lay flat"); 
    delay(2000);

    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    lcdWrite("DMP Error!", "retrying...."); 
    
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  return devStatus;
};

void updateAccelData(){
  // code courtesy of Electronic Cats
  // https://github.com/ElectronicCats/mpu6050/tree/master/examples/MPU6050_DMP6

  // get latest packet
  mpu.dmpGetCurrentFIFOPacket(FIFOBuffer);

  /* Display initial world-frame acceleration, adjusted to remove gravity
  and rotated based on known orientation from Quaternion */
  mpu.dmpGetQuaternion(&q, FIFOBuffer);
  mpu.dmpGetAccel(&aa, FIFOBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);

  // move current data to previous
  accelDataRaw[0][0] = accelDataRaw[1][0];
  accelDataRaw[0][1] = accelDataRaw[1][1];
  accelDataRaw[0][2] = accelDataRaw[1][2];

  // add data to accelData matrix
  accelDataRaw[1][0] = aaWorld.x + XOffset;
  accelDataRaw[1][1] = aaWorld.y + YOffset;
  accelDataRaw[1][2] = aaWorld.z + ZOffset;
}

void updateVeloData(){
    // MPU6050 scale factor: 1 LSB = [mpuRange] /16384 g = 2/32768.0 * 9.81 m/s² * deltaT
  float scaleFactor = mpuRange * 9.81 / 32768.0 * (cycleDuration / 1000);

  // move current data to previous
  veloDataNorm[0][0] = veloDataNorm[1][0];
  veloDataNorm[0][1] = veloDataNorm[1][1];
  veloDataNorm[0][2] = veloDataNorm[1][2];

  float accelX = accelDataRaw[1][0] * scaleFactor;
  float accelY = accelDataRaw[1][1];
  float accelZ = accelDataRaw[1][2];

  // Apply a threshold to ignore noise/drift around zero
  if (abs(accelDataRaw[0][0]) < accelThreshold) accelX = 0;
  if (abs(accelDataRaw[0][1]) < accelThreshold) accelY = 0;
  if (abs(accelDataRaw[0][2]) < accelThreshold) accelZ = 0;

  // Integrate acceleration to compute velocity
  // v = v_previous + a * delta_t
  veloDataNorm[1][0] = veloDataNorm[0][0] + accelX;
  veloDataNorm[1][1] = veloDataNorm[0][1] + accelY;
  veloDataNorm[1][2] = veloDataNorm[0][2] + accelZ;


  // reset velodata to 0 if below threshold
  // only reset if velocity is decreasing
    if (abs(veloDataNorm[1][0]) < veloThreshold &&
    abs(veloDataNorm[0][0]) >= abs(veloDataNorm[1][0])) veloDataNorm[1][0] = 0;
    if (abs(veloDataNorm[1][1]) < veloThreshold &&
    abs(veloDataNorm[0][1]) >= abs(veloDataNorm[1][1])) veloDataNorm[1][1] = 0;
    if (abs(veloDataNorm[1][2]) < veloThreshold &&
    abs(veloDataNorm[0][2]) >= abs(veloDataNorm[1][2])) veloDataNorm[1][2] = 0;

  // Debug print velocity data
  Serial.print("Velocity (m/s)\t");
  Serial.print(veloDataNorm[1][0]);
  Serial.print("\t");
  Serial.print(veloDataNorm[1][1]);
  Serial.print("\t");
  Serial.println(veloDataNorm[1][2]);
}

void updateDistance() {
  // Calculate the time elapsed between cycles (in seconds)
  float deltaTime = cycleDuration / 1000.0;  // 50ms in seconds

  // Integrate velocity to compute distance: distance = velocity * time
  // Use Pythagorean theorem for horizontal distance (x and y directions)
  totalHoriDistTravelled += sqrt(pow(veloDataNorm[1][0] * deltaTime, 2) + pow(veloDataNorm[1][1] * deltaTime, 2)) / 10000;

  // Debugging output
  Serial.print("Horizontal Distance Travelled (m): ");
  Serial.println(totalHoriDistTravelled, 4);  // Print the distance with 4 decimal places

  // Optional: Display the distance on the LCD
  // lcdWrite("Dist: " + String(totalHoriDistTravelled, 2) + "m");
}

// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  // Attempt to connect to Wi-Fi until timeout
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
    Serial.print(".");
    delay(100); // Wait 100ms between attempts
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi Connection Failed!");
  } else {
    Serial.println("Connected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

// Function to send data to the backend server
void sendDataToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("http://192.168.2.29:5501/update"); // Use local IP and port
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"distance\":10,\"co2\":5}";
    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
    } else {
        Serial.print("Error: ");
        Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected");
  }
}

void displayIdle() {
  lcdWrite("CO2 Emissions Tracker", "'Red' to Start"); 
}

void displayVehicleMenu() {
  lcdWrite("Choose Vehicle:", String(vehicleCategories[activeVehicleIndex].c_str()));
}

// My edit's are here.... im makign the dropdown menu 
void displayTracking() {
  float gasBurnedRightNow = totalHoriDistTravelled * (gasBurnedPerKm[activeVehicleIndex]/1000); // in literes 
  co2BurnedNow = gasBurnedRightNow * 2.3; // fixed constnat 2.3 kg of carbon per l litre of gas burned 
  co2BurnedNow = co2BurnedNow * 1000;

  lcdWrite("CO2: " + String(co2BurnedNow, 2) + " g", "Dist: " + String(totalHoriDistTravelled, 2) + " m");  
}

void checkButtons(){

  if (digitalRead(redButton) == HIGH){
    // what happens if red button is pressed
    digitalWrite(buzzer, HIGH);
    if (programState == "idle") {
      programState = "menu"; 
      activeVehicleIndex=0;
      co2BurnedNow=0;
      totalHoriDistTravelled=0; // reset the distance 
    }
    else if (programState == "tracking") {
      programState = "idle";
    }
  }

  else if (digitalRead(blueButton)  == HIGH){
    // what happens if blue button is pressed
    digitalWrite(buzzer, HIGH);
    // only for switching through the vehicles (dropdown menu)
    if (programState == "menu") {
      activeVehicleIndex++;
      if (activeVehicleIndex >= 5) {
        activeVehicleIndex=0; 
      } displayVehicleMenu();
    }
  }

  else if (digitalRead(greenButton)  == HIGH){
    // what happens if green button is pressed
    // the green button is like the button that takes you to different states!
    digitalWrite(buzzer, HIGH);
    if (programState == "menu") {
      programState = "tracking"; 
    }
   
    // lcdWrite("green");
  }

  else{
    // no buttons are pressed
    digitalWrite(buzzer, LOW);
  }

  if (programState == "idle") {
    displayIdle();
  }
  else if (programState == "menu") {
    displayVehicleMenu();
  }
  else if (programState == "tracking") {
    displayTracking();
  }
  
}


void setup() {
    Serial.begin(115200);

    // initialize the lcd using default i2c pins:
    lcd.init();
	  lcd.backlight();

    checkButtons();

    // wait until user is ready
    lcdWrite("press green btn", "make sure flat");
    while (digitalRead(greenButton) == false);

    // setup MPU
    while (setupMPU() != 0){
      resetI2C();
      delay(1000);
    }

    // output success
    lcdWrite("success!", "grn btn end trip");
    delay(1000);

    // button pins
    pinMode(redButton, INPUT);  
    pinMode(blueButton, INPUT);
    pinMode(greenButton, INPUT); 

    // potentiometer (knob) pin
    pinMode(knob, INPUT);

    // buzzer pin
    pinMode(buzzer, OUTPUT);
}

void verifyAccelData(){
    // check if mpu encountered any error:
  if (
    accelDataRaw[1][0] == accelDataRaw[0][0] &&
    accelDataRaw[1][1] == accelDataRaw[0][1] &&
    accelDataRaw[1][2] == accelDataRaw[0][2] 
  ){
    // reset the mpu
    Serial.println("MPU stopped working. Re-initializing...");
    lcdWrite("MPU stopped working. Re-initializing...");
    resetI2C();
    setupMPU();

    // random value for accelDataRaw[0] otherwise it will loop
    accelDataRaw[0][0] = -1;
    accelDataRaw[0][1] = -1;
    accelDataRaw[0][2] = -1;

  }

  // values too high, possibly due to a drop or shock, ignore values
  if (abs(accelDataRaw[1][0]) > 75000 || abs(accelDataRaw[1][1]) > 75000 || abs(accelDataRaw[1][2]) > 75000) {
    accelDataRaw[1][0] = 0;
    accelDataRaw[1][1] = 0;
    accelDataRaw[1][2] = 0;
  }
}

void loop() {
  unsigned long startTime = millis(); // get current time

  updateAccelData();
  verifyAccelData();
  updateVeloData();
  updateDistance();

  checkButtons();

  // DEBUG:
  Serial.print(" | Accel: ");
  Serial.print(accelDataRaw[1][0]);
  Serial.print(", ");
  Serial.print(accelDataRaw[1][1]);
  Serial.print(", ");
  Serial.println(accelDataRaw[1][2]);

  Serial.print("Previous tick: ");
  Serial.print(0);
  Serial.print(" | Accel: ");
  Serial.print(accelDataRaw[0][0]);
  Serial.print(",");
  Serial.print(accelDataRaw[0][1]);
  Serial.print(",");
  Serial.println(accelDataRaw[0][2]);

  //lcdWrite((String)accelDataRaw[1][0] + ", " + (String)accelDataRaw[1][1], (String)veloDataNorm[1][0] + ", " + (String)veloDataNorm[1][1] + ", " + 1);

  while (millis() - startTime < cycleDuration) {
        // wait until 100ms has elapsed for next cycle
  }
}