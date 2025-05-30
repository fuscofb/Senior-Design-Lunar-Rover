//Arduino Mega
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h> 

//Libraries for LIDAR
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// RF
RF24 radio(7, 8); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};  //Setting the two addresses. One for transmitting and one for receiving

//IMU (also includes Wire.h)
#include <I2Cdev.h>
#include <MPU6050.h>

//LIDAR
#define LOX1_ADDRESS 0x30 //The first intended 12C address
#define LOX2_ADDRESS 0x31 //The second intended 12C address
#define SHT_LOX1 33 //Arduino pin for the X wire of the first VL53L0X
#define SHT_LOX2 32 //Arduino pin for the X wire of the second VL53L0X
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X(); //Setting up the sensors using the adafruit library
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
const int FwdPin = 34;  //Forward Motor Pin
const int BwdPin = 35; //Backward Motor Pin
int MaxSpd = 60;
boolean DirFlag = true; //Flag for direction
int sensor1, sensor2; //Creates int type variable for sensors
int sum = 0;
int counter;
float xangle = 0;
int xinterval = 0;
int heading = 0;
int marker = 0;


// IMU
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;


struct AllDataToSend {
  byte aX;
  byte aY;
  byte aZ;
  byte gX;
  byte gY;
  byte gZ;
  float heading;
  float sensor1;
  float sensor2;
};

AllDataToSend alldatatosend;

// Stick input parameters from Arduino Uno
float responseData[6];

//Control input initilization
float stick_1_y = 0;
float stick_1_x = 0;
float stick_2_y = 0;
float stick_2_x = 0;

// DC Motor setups
int enA_Driver1 = 47;
int in1_M1_D1 = 45; // Top right***
int in2_M1_D1 = 43;  
int enB_Driver1 = 42;
int in3_M2_D1 = 46; // Bottom right*** 
int in4_M2_D1 = 44;

int enA_Driver2 = 22;
int in1_M1_D2 = 24; // Top left***
int in2_M1_D2 = 26;
int enB_Driver2 = 23;
int in3_M2_D2 = 25; // Bottom left***
int in4_M2_D2 = 27;

// Stepper motor pins
const int in1 = 13;
const int in2 = 12;
const int in3 = 11;
const int in4 = 10;
const int stepPin = 5;
const int dirPin = 6; 
// Stepper Motor Setup
int buttonA = 0;
int buttonB = 0;
bool automaticSpooling = false;
bool autoSpoolingActive = false;
bool spoolingComplete = false;
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// VREF --> V = IR, VREF = Imax/2 --> Imax = 2 --> VREF = 1

void setup() {
  Serial.begin(9600);
  radio.begin();                           //Starting the radio communication
  radio.openWritingPipe(addresses[1]);     //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[0]);
  //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);
  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver. 
  Serial.println("Radio initialized");
  //IMU
  Wire.begin();
  mpu.initialize();

  // DC Motor
  // Set used pins as output
  pinMode(enA_Driver1, OUTPUT);
  pinMode(in1_M1_D1, OUTPUT);
  pinMode(in2_M1_D1, OUTPUT);

  pinMode(enA_Driver2, OUTPUT);
  pinMode(in1_M1_D2, OUTPUT);
  pinMode(in2_M1_D2, OUTPUT);

  pinMode(enB_Driver1, OUTPUT);
  pinMode(in3_M2_D1, OUTPUT);
  pinMode(in4_M2_D1, OUTPUT);

  pinMode(enB_Driver2, OUTPUT);
  pinMode(in3_M2_D2, OUTPUT);
  pinMode(in4_M2_D2, OUTPUT);
    
  // DC motor off
  digitalWrite(in1_M1_D1, LOW);
  digitalWrite(in2_M1_D1, LOW);
  digitalWrite(in1_M1_D2, LOW);
  digitalWrite(in2_M1_D2, LOW);
  digitalWrite(in3_M2_D2, LOW);
  digitalWrite(in4_M2_D2, LOW);

  // Stepper motor 
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Stepper motor
  stepper.setMaxSpeed(500);

  // LIDAR
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  //Serial.println("Shutdown pins initiated...");
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  //Serial.println("Both in reset mode...(pins are low)");
  //Serial.println("Starting...");
  setID(); //Runs code to assign unique 12C addresses to each of the two sensors.
  pinMode(FwdPin, OUTPUT);
  pinMode(BwdPin, OUTPUT);
  if (DirFlag)
  {
    analogWrite(FwdPin, MaxSpd); //Send instructions to Forward motor pin. Block this line out to stop motor from running
  }
  lox1.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);
  calibration(); //Runs calibration code (see function below)
  Serial.println("set up complete");
}

void loop() 
{  
  delay(5); 
  radio.startListening(); //This sets the module as receiver
  while(radio.available()){                         //Looking for incoming data
    radio.read(&responseData, sizeof(responseData)); //Reading the data
    Serial.println(formatResponse(responseData, 6));

    stick_1_y = responseData[0];
    stick_1_x = responseData[1]; // not needed
    stick_2_y = responseData[2];
    stick_2_x = responseData[3]; // not needed
    buttonA = responseData[4];
    buttonB = responseData[5];
  }

  delay (5);
  radio.stopListening(); //This sets the module as transmitter
  // Tank drive
  DriveMotors(stick_1_y, stick_2_y);

  SpoolMotors(stick_1_y, stick_2_y);

  // Get IMU data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply Kalman filter to each axis
  double filtered_ax = KALMAN(ax, kalmanEstimate_ax, kalmanError_ax);
  double filtered_ay = KALMAN(ay, kalmanEstimate_ay, kalmanError_ay);
  double filtered_az = KALMAN(az, kalmanEstimate_az, kalmanError_az);
  double filtered_gx = KALMAN(gx, kalmanEstimate_gx, kalmanError_gx);
  double filtered_gy = KALMAN(gy, kalmanEstimate_gy, kalmanError_gy);
  double filtered_gz = KALMAN(gz, kalmanEstimate_gz, kalmanError_gz);

  // Map the filtered data into 0–255 and assign to struct
  alldatatosend.aX = map((int)filtered_ax, -17000, 17000, 0, 255);
  alldatatosend.aY = map((int)filtered_ay, -17000, 17000, 0, 255);
  alldatatosend.aZ = map((int)filtered_az, -17000, 17000, 0, 255);
  alldatatosend.gX = map((int)filtered_gx, -17000, 17000, 0, 255);
  alldatatosend.gY = map((int)filtered_gy, -17000, 17000, 0, 255);
  alldatatosend.gZ = map((int)filtered_gz, -17000, 17000, 0, 255);

  // LIDAR
  while (millis() < 30000) { //This while loop cuts the sensor after 30 seconds of function, for a short test
    readsensors(); //Runs the function that records the distance measured from each sensor
    // Fill Structure
    alldatatosend.heading = heading;
    alldatatosend.sensor1 = sensor1;
    alldatatosend.sensor2 = sensor2;
    // Serial.print(heading); //Prints the data to Serial in the manner expected by the Processing code, split by commas
    // Serial.print(",");
    // Serial.print(sensor1);
    // Serial.print(",");
    // Serial.println(sensor2);
  }
  

  if (radio.available()) {  
    radio.write(&alldatatosend, sizeof(AllDataToSend));
  }
}

// **Function to format response as CSV string**
String formatResponse(float* arr, int size) {
    String result = "";
    for (int i = 0; i < size; i++) {
        result += String(arr[i], 2); // Two decimal places
        if (i < size - 1) result += ",";
    }
    return result;
}

// Determines wheel motor speed in tank drive where stick 1 and 2 are floats between -1 and 1
void DriveMotors(float stick1, float stick2)
{
  // Tank drive
  float speed1 = abs(stick1*255/2);//left 2 wheels
  float speed2 = abs(stick2*255/2);//right 2 wheels

  analogWrite(enA_Driver1, speed1);//top left
  digitalWrite(in1_M1_D1, (stick1>0)?HIGH:LOW);
  digitalWrite(in2_M1_D1, (stick1>0)?LOW:HIGH);
    
  analogWrite(enB_Driver1, speed1);//bottom left
  digitalWrite(in3_M2_D1, (stick1>0)?HIGH:LOW);
  digitalWrite(in4_M2_D1, (stick1>0)?LOW:HIGH);

  analogWrite(enA_Driver2, speed2);//top right
  digitalWrite(in1_M1_D2, (stick2>0)?HIGH:LOW);
  digitalWrite(in2_M1_D2, (stick2>0)?LOW:HIGH);
    
  analogWrite(enB_Driver2, speed2);//bottom right
  digitalWrite(in3_M2_D2, (stick2>0)?HIGH:LOW);
  digitalWrite(in4_M2_D2, (stick2>0)?LOW:HIGH);
}

void SpoolMotors(float stick1, float stick2)
{

  if (buttonA == 1) {
    automaticSpooling = true;
    autoSpoolingActive = true;
    spoolingComplete = false;
  }
  if (buttonB == 1) {
    automaticSpooling = false;
    autoSpoolingActive = false;
    spoolingComplete = false;
  }

  if (automaticSpooling) {
    stepper.setAcceleration(20);
    stepper.setSpeed(50);
    stepper.run();

    if (autoSpoolingActive && !spoolingComplete) {
      readsensors();  // update sensor1 and sensor2

      if (abs(sensor1 - 50) <= 5 || abs(sensor2 - 50) <= 5) {
        // Stop the spool
        analogWrite(FwdPin, 0);
        analogWrite(BwdPin, 0);
        spoolingComplete = true;
        autoSpoolingActive = false;
      } else {
        // Continue spooling
        analogWrite(FwdPin, MaxSpd);
        analogWrite(BwdPin, 0);
      }
    }
  } else {
    analogWrite(FwdPin, MaxSpd);
    analogWrite(BwdPin, 0);
  }
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  // initiating LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
  }
  delay(10);
  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  //initiating LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
  }
}

void calibration() {
  //CALIBRATION SEQUENCE
  //This conducts a series of test rotations, and from that determines the rotational speed of the sensor.
  //This was found to be consistent, so you can then measure how many readings you can complete per turn.
  //This value is then used to calculate this interval that is the angular increment of each reading.
  for (int i = 0; i <= 4; i++) {
    counter = 0;
    sensor1 = 0;
    while (sensor1 < 45 || sensor1 > 55) {
      lox1.rangingTest(&measure1, false);
      lox2.rangingTest(&measure2, false);
      sensor1 = measure1.RangeMilliMeter;
      delay(2);
    }
    counter ++;
    while (sensor1 < 45 || sensor1 > 55 || counter <= 6) {
      lox1.rangingTest(&measure1, false);
      lox2.rangingTest(&measure2, false);
      sensor1 = measure1.RangeMilliMeter;
      delay(2);
      counter ++;
    }
    sum = sum + counter;
  }
  xangle = sum / 5;
  xangle = 360 / xangle;
  xangle = round(xangle);
  xinterval = int(xangle);
  return (xinterval);
}

void readsensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  sensor1 = measure1.RangeMilliMeter;
  sensor2 = measure2.RangeMilliMeter;
  //The line below identified when the sensor is seeing the pillar of the LiDAR tower
  //When obscured the sensor was found to read between these values.
  if (sensor1 > 45 && sensor1 < 55) {
    if (marker == 0) {
      heading = 0; //Resets the heading (of that particular distance measurement) to 0
      marker ++;
    }
    else {
      heading = heading + xinterval;
      marker ++;
    }
  }
  else {
    heading = heading + xinterval; //Adds the angular interval to each successive reading
    marker = 0;
  }
  //The line below identified when the received range is out of range
  //If the sensor did not receive a good reading (due to out of range) it would produce random value below 45
  if (sensor1 <= 45 || sensor1 >= 500) {
    sensor1 = 500;
  }
  heading = heading % 360; // ensure heading stays in 0-359 range
  if (heading >= 360) { //Filters out when heading (due to an error) has accidentally exceeded a full rotation
    heading = 0;
  }
}

double KALMAN(double U) {

  // Update Kalman gain
  kalmanGain = kalmanError*hValue/(hValue*kalmanError*hValue+noiseCovariance);
  // Update estimate
  filteredEstimate = filteredEstimate + kalmanGain*(U-hValue*filteredEstimate);

  // Update Error Covariance
  kalmanError = (1-kalmanGain*hValue)*kalmanError+rValue;

  return filteredEstimate;

}