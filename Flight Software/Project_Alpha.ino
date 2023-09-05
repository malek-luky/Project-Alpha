#include <SD.h>
#include <Servo.h> 
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#define LOG_FILE "data.csv"
#define LED_PIN 13

//BMP
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

////////////////////////////////////
// SETUP ELECTRONICS COMPONENTS
////////////////////////////////////

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp; // Initialize BMP388 sensor
const int chipSelect = BUILTIN_SDCARD;
File logFile;
Servo ServoX;
Servo ServoY;

////////////////////////////////////
// CHANGE VARIABLES HERE
////////////////////////////////////
double xCenter = 73;
double yCenter = 91;
double maxAngle = 8;
double weird_offsetX = 1.5;
double weird_offsetY = 1.4;
double x_origin = 0;//calibrated values
double y_origin = 0;//calibrated values
double pX[] = {0.0002, 0.0108, 1.6939, -0.0027};
double pY[] = { 0.0012, 0.0189, 2.3005, -0.0136};

////////////////////////////////////
// CHANGE PID HERE
////////////////////////////////////
double Kp = 0.20825;
double Ki = 0.32;
double Kd = 0.06;

////////////////////////////////////
// DECLARE ALL VARIABLES
////////////////////////////////////
double errorX, errorY;
double proportionalX = 0, proportionalY = 0;
double integralX = 0, integralY = 0;
double derivativeX = 0, derivativeY = 0;
double prev_errorX = 0, prev_errorY = 0;
double OutputX = 0, OutputY = 0;
double MotorAngleX = 0, MotorAngleY = 0;
double x = 0, y=0;
double liftoff,streak,prev_liftoff = 0;
double xAngle=90, yAngle=0;
bool start = false;
bool launch = false;
bool skip_calib = false;
double currentTime = micros();
double cycleTime = micros();
double flightTime = 0;
double dt = 0;
double launchTime = 0;

////////////////////////////////////
// SETUP
////////////////////////////////////

void setup()
{
  // Welcome message
  Serial.begin(115200);
  Serial.println("TVC Alpha Flight Computer"); Serial.println("");

  // Turn on the LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Attach Teensy pins
  ServoX.attach(15);
  ServoY.attach(14);


  // Initialize SD card
  if (!SD.begin(chipSelect))
  {
    Serial.println("Failed to initialize SD card!");
    while (1);
  }

  // Open the log file
  SD.remove(LOG_FILE);
  logFile = SD.open(LOG_FILE, FILE_WRITE);
  if (!logFile)
  {
    Serial.println("Failed to open log file!");
    while (1);
  }

  //Print PIDF values
  Serial.println(Kp,4);
  Serial.println(Ki,4);
  Serial.println(Kd,4);
  logFile.println(Kp,4);
  logFile.println(Ki,4);
  logFile.println(Kd,4);

  // Write the header line to the log file 
  logFile.println("state,currentTime,x,y,liftoff,acc,errorX,errorY,proportionalX,proportionalY,integralX,integralY,derivativeX,derivativeY,MotorAngleX,MotorAngleY,OutputX,OutputY,prev_errorX,prev_errorY,flightTime,dt,x_origin,y_origin, pressure");

  // Initialise the sensor
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your &Wire or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true); //enables external crystal for clock

  // Check BMP
  if (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // Set temperature oversampling
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X); // Set pressure oversampling
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3); // Set IIR filter coefficient
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Set output data rate

  // Go to center and wait
  ServoX.write(xCenter);
  ServoY.write(yCenter);
  delay(2000);
  currentTime = micros();
}

void loop()
{  
  // Get the time difference
  dt = (micros()-cycleTime)/1000000;

  // Update current time
  currentTime = micros();
  
  // Get a new sensor event
  sensors_event_t orientationData , accelerometerData;
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  x = -orientationData.orientation.y - x_origin; //x = y
  y = orientationData.orientation.z - y_origin; //y = -z
  prev_liftoff = liftoff;
  liftoff = accelerometerData.acceleration.z;
  // Check calibration
//  while (system!=3 or accel!=3)
//  {
//    if (system == 3 and accel ==3)
//    {
//      digitalWrite(LED_PIN, LOW);
//      break;
//    }
//    delay(1000);
//    Serial.println("Calibrate sensor!");
//  }
  
  // BMP
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }

  
  ////////////////////////////////////////
  // CHECK BNO INIT
  ////////////////////////////////////////
  if (x==0 and y==0 and liftoff==0)
  {
    while(1);
  }

  ////////////////////////////////////////
  // CALIBRATION
  ////////////////////////////////////////
  skip_calib=false;
  if (skip_calib==true)
  {
    start = true;
  }
  if (start==false and skip_calib==false)
  {
    // Write the new values
    start = true;
    delay(3000);
    
    for (int i = 0; i < 4000; i++) {
      // Calculate new X and Y angles
      xAngle += 0.01;
      
      // Calculate new X and Y positions
      double MotorAngleX = maxAngle*sin(xAngle);

      // Transfer angle to readl position
      OutputX = weird_offsetX * (pX[0] * pow(MotorAngleX, 3) + pX[1] * pow(MotorAngleX, 2) + pX[2] * MotorAngleX + pX[3]);
      
      // Add the centre values
      OutputX = OutputX+xCenter;
      
      // Move servos to new X and Y positions
      Serial.print("OutputX: ");
      Serial.println(OutputX-xCenter,4);
      Serial.print("MotorAngleX: ");
      Serial.println(MotorAngleX,4);
      ServoX.write(OutputX);
      
      // Wait for servos to reach new positions
      delay(2);
    }

    // Center postiion
    ServoX.write(xCenter);
    ServoY.write(yCenter);
    delay(2000);
    
    for (int i = 0; i < 4000; i++) {
      // Calculate new X and Y angles
      yAngle += 0.01;
      
      // Calculate new X and Y positions
      double MotorAngleY = maxAngle*sin(yAngle);

      // Transfer angle to readl position
      OutputY = weird_offsetY * (pY[0] * pow(MotorAngleY, 3) + pY[1] * pow(MotorAngleY, 2) + pY[2] * MotorAngleY + pY[3]);
      
      // Add the centre values
      OutputY = OutputY+yCenter;
      
      // Move servos to new X and Y positions
      ServoY.write(OutputY);
      
      // Wait for servos to reach new positions
      delay(2);
    }

    // Update new x and y values
    sensors_event_t event;
    bno.getEvent(&event);    
    x = event.orientation.y - x_origin; //x = y
    y = -event.orientation.z - y_origin; //y = -z
  
    // Center postiion
    ServoX.write(xCenter);
    ServoY.write(yCenter);
    delay(3000);
  }

  ////////////////////////////////////////
  // LIFTOFF DETECTION
  ////////////////////////////////////////
  if (abs(liftoff-prev_liftoff)>1.5 and start==true and launch ==false)
  {
    streak=streak+1;
    if (streak>1)
    {
    launchTime = micros();
    launch = true;
    Serial.print("Liftoff: ");
    Serial.println(launchTime);
    logFile.print("Liftoff: ");
    logFile.println(launchTime);
    //0.615s liftoff
    //1.2s Back to 3N
    //=0.585s after liftoff
    }
  }
  else
  {
    streak = 0;
  }
  if (launch==true)
  {
    flightTime = (currentTime-launchTime)/1000000;
  }

  ////////////////////////////////////////
  // STATE LOG
  ////////////////////////////////////////
  // Frequency 50Hz - dt>0.019
  // Start after the first thrust peak: launch==true and (currentTime-laumchTime)/1000000>0.585
  // In first 3s the reading is corrupted influencing Integrative term
  
  if (dt>0.019)
  {
  if (launch==true and currentTime>3000000)
  {    
    Serial.print("State: ");
    Serial.print("Liftoff, ");
    logFile.print("Liftoff, ");

    // Update time
    cycleTime = micros();

    ////////////////////////////////////////
    // PID
    ////////////////////////////////////////
    // Get the errors
    errorX = x;
    errorY = y;

    // Compute Proportional
    proportionalX = errorX*Kp;
    proportionalY = errorY*Kp;

    // Compute integral
    integralX += (errorX * dt)*Ki;
    integralY += (errorY * dt)*Ki;

    // Prevent Wind-up
    integralX = constrain(integralX, -maxAngle, +maxAngle);
    integralY = constrain(integralY, -maxAngle, +maxAngle);

    // Compute derivative
    derivativeX = Kd * (errorX - prev_errorX) / dt;
    derivativeY = Kd * (errorY - prev_errorY) / dt;

    // Compute output using PID formula
    MotorAngleX = proportionalX + integralX + derivativeX;
    MotorAngleY = proportionalY + integralY + derivativeY;

    // Limit motor angle range
    MotorAngleX = constrain(MotorAngleX, -maxAngle, maxAngle);
    MotorAngleY = -constrain(MotorAngleY, -maxAngle, maxAngle);
    
    // Transfer motor angle to servo values
    OutputX = weird_offsetX * (pX[0] * pow(MotorAngleX, 3) + pX[1] * pow(MotorAngleX, 2) + pX[2] * MotorAngleX + pX[3]);
    OutputY = weird_offsetY * (pY[0] * pow(MotorAngleY, 3) + pY[1] * pow(MotorAngleY, 2) + pY[2] * MotorAngleY + pY[3]);

    // Add the centre values
    OutputX = OutputX+xCenter;
    OutputY = OutputY+yCenter;

    // Update servo positions
    ServoX.write(OutputX);
    ServoY.write(OutputY);
    
    // Update previous error
    prev_errorX = errorX;
    prev_errorY = errorY;
  }

  else
  {
    Serial.print("State: ");
    Serial.print("Ground, ");
    logFile.print("Ground, ");

    // Update time
    cycleTime = micros();
  }

  ////////////////////////////////////////
  // DATA LOG
  ////////////////////////////////////////
  // Display the doubleing point data
  Serial.print("currentTime: ");
  Serial.print(currentTime);
  Serial.print(", x: ");
  Serial.print(x, 4);
  Serial.print(", y: ");
  Serial.print(y, 4);
  Serial.print(", liftoff: ");
  Serial.print(liftoff-prev_liftoff, 4);
  Serial.print(", acc: ");
  Serial.print(liftoff, 4);
  Serial.print(", ErrorX: ");
  Serial.print(errorX, 4);
  Serial.print(", ErrorY: ");
  Serial.print(errorY, 4);
  Serial.print(", proportionalX: ");
  Serial.print(proportionalX, 4);
  Serial.print(", proportionalY: ");
  Serial.print(proportionalY, 4);
  Serial.print(", integralX: ");
  Serial.print(integralX, 4);
  Serial.print(", integralY: ");
  Serial.print(integralY, 4);
  Serial.print(", derivativeX: ");
  Serial.print(derivativeX, 4);
  Serial.print(", derivativeY: ");
  Serial.print(derivativeY, 4);
  Serial.print(", MotorAngleX: ");
  Serial.print(MotorAngleX, 4);
  Serial.print(", MotorAngleY: ");
  Serial.print(MotorAngleY, 4);
  Serial.print(", OutputX: ");
  Serial.print(OutputX, 4);
  Serial.print(", OutputY: ");
  Serial.print(OutputY, 4);
  Serial.print(", flightTime: ");
  Serial.print(flightTime,4);
  Serial.print(", dt: ");
  Serial.print(dt,4);
  Serial.print(", Pressure: ");
  Serial.print(bmp.pressure,4);
  Serial.println();

  // Write the data to the log file
  logFile.print(currentTime);
  logFile.print(",");
  logFile.print(x, 4);
  logFile.print(",");
  logFile.print(y, 4);
  logFile.print(",");
  logFile.print(liftoff-prev_liftoff, 4);
  logFile.print(",");
  logFile.print(liftoff, 4);
  logFile.print(",");
  logFile.print(errorX, 4);
  logFile.print(",");
  logFile.print(errorY, 4);
  logFile.print(",");
  logFile.print(proportionalX, 4);
  logFile.print(",");
  logFile.print(proportionalY, 4);
  logFile.print(",");
  logFile.print(integralX, 4);
  logFile.print(",");
  logFile.print(integralY, 4);
  logFile.print(",");
  logFile.print(derivativeX, 4);
  logFile.print(",");
  logFile.print(derivativeY, 4);
  logFile.print(",");
  logFile.print(MotorAngleX, 4);
  logFile.print(",");
  logFile.print(MotorAngleY, 4);
  logFile.print(",");
  logFile.print(OutputX, 4);
  logFile.print(",");
  logFile.print(OutputY, 4);
  logFile.print(",");
  logFile.print(prev_errorX, 4);
  logFile.print(",");
  logFile.print(prev_errorY, 4);
  logFile.print(",");
  logFile.print(flightTime, 4);
  logFile.print(",");
  logFile.print(dt, 4);
  logFile.print(",");
  logFile.print(x_origin, 4);
  logFile.print(",");
  logFile.print(y_origin, 4);
  logFile.print(",");
  logFile.print(bmp.pressure,4);
  logFile.println();

  // Flush SD CArd
  logFile.flush();
  }
}
