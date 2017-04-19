#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_VS1053.h>

// Audio Player pin definitions
#define CLK 2       // SPI Clock, shared with SD card
#define MISO 4      // Input data, from VS1053/SD card
#define MOSI 3      // Output data, to VS1053/SD card
#define BREAKOUT_RESET  1      // VS1053 reset pin (output)
#define BREAKOUT_CS     0     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    6      // VS1053 Data/command select pin (output)
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    5     // VS1053 Data/command select pin (output)
#define CARDCS 4     // Card chip select pin
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

// The Music Player Object
//Adafruit_VS1053_FilePlayer musicPlayer = 
//  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
   
// Assign a unique base ID for this sensor  
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

// Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


//---END OF CONFIGURATION CODE---//

// Calibration Variables
int xycalib = 0;
int xzcalib = 0;
int yzcalib = 0;

// Heading Variables
int headingxy;
int headingxz;
int headingyz;

// Impingement Variables - Found through data analysis
int xyImpThreshHigh;
int xyImpThreshLow;
int xzImpThreshHigh;
int xzImpThreshLow;
int yzImpThreshHigh;
int yzImpThreshLow;

// Pushup up/down states - Found through data analysis
int xyDownAngle;
int xyUpAngle;
int xzDownAngle;
int xzUpAngle;
int yzDownAngle;
int yzUpAngle;

// Mode booleans
boolean latraisemode = false;

boolean pushupmode = false;
boolean pushupupmode = false;
boolean pushupdownmode = false;

boolean normThreshold; // Found through data analysis

// Counters
int pushupcounter = 0;

// Repetition Variables (will be used for both modes)
boolean justFinishedRep = false;
boolean impingement = false;

// Data Collection Variables
int record = 0;

void setup(void) 
{

  // Setting up baud rate
  while (!Serial);
  Serial.begin(9600);

//  //*****Setting up the Music Player*****//
//  if (!musicPlayer.begin()) { // initialise the music player
//     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
//     while (1);
//  }
//  Serial.println(F("VS1053 found"));
//  
//   if (!SD.begin(CARDCS)) {
//    Serial.println(F("SD failed, or not present"));
//    while (1);  // don't do anything more
//  }
  
  pinMode(11, INPUT); // Pushup Mode Button
  pinMode(12, INPUT); // Lat Raise Mode Button

  pushupcounter = 0;

  // list files
//  printDirectory(SD.open("/"), 0);
  
  // Set volume for left, right channels. lower numbers == louder volume!
//  musicPlayer.setVolume(20,20);

//  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ interrupt
  //*****End of Music Player Setup*****//
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");

  //creating outputfile
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
int datacount = 0;

void loop(void) {

  // DATA COLLECTION BLOCK
  if(Serial.available() > 0){
    Serial.read();
    if (record == 0){
      Serial.println("Data Now Recording...");
      datacount = 0;
    } else {
      Serial.print("Recording Stopped. Total Data Points: "); Serial.println(datacount);
    }
    record = (record + 1) % 2;
  }
  datacount ++;
  // END OF DATA COLLECTION BLOCK
  
  // Get a new sensor event
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // *****Retrieving angles from the magnetometer readings***** //
  
  // Block 1: XY
  if(mag.magnetic.y > 0) {
    headingxy = 90 - atan(mag.magnetic.x/mag.magnetic.y) * (180 / 3.1415926);
  } else if(mag.magnetic.y < 0) {
    headingxy = 270 - atan(mag.magnetic.x/mag.magnetic.y) * (180 / 3.1415926);
  } else if(mag.magnetic.y == 0 && mag.magnetic.x < 0) {
    headingxy = 180;
  } else if(mag.magnetic.y == 0 && mag.magnetic.x > 0) {
    headingxy = 0;
  }
  
  // Block 2: XZ
  if(mag.magnetic.x > 0) {
    headingxz = 90 - atan(mag.magnetic.z/mag.magnetic.x) * (180 / 3.1415926);
  } else if(mag.magnetic.x < 0) {
    headingxz = 270 - atan(mag.magnetic.z/mag.magnetic.x) * (180 / 3.1415926);
  } else if(mag.magnetic.x == 0 && mag.magnetic.z < 0) {
    headingxz = 180;
  } else if(mag.magnetic.x == 0 && mag.magnetic.z > 0) {
    headingxz = 0;
  }

  // Block 3: YZ
  if(mag.magnetic.z > 0) {
    headingyz = 90 - atan(mag.magnetic.y/mag.magnetic.z) * (180 / 3.1415926);
  } else if(mag.magnetic.z < 0) {
    headingyz = 270 - atan(mag.magnetic.y/mag.magnetic.z) * (180 / 3.1415926);
  } else if(mag.magnetic.z == 0 && mag.magnetic.y < 0) {
    headingyz = 180;
  } else if(mag.magnetic.z == 0 && mag.magnetic.y > 0) {
    headingyz = 0;
  }

  // DATA COLLECTION RECORDING
  if(record) {
    headingxy = (headingxy + (360 - xycalib)) % 360 - 180;
    headingxz = (headingxz + (360 - xzcalib)) % 360 - 180;
    headingyz = (headingyz + (360 - yzcalib)) % 360 - 180;
    Serial.print(headingxy); Serial.print(",");
    Serial.print(headingxz); Serial.print(",");
    Serial.print(headingyz); Serial.print("\n");
  }

  analyzeData(); // Calls the helper function

  // Audio Cases
//  if(justFinishedRep) {
//    justFinishedRep = false;
//
//    if(pushupmode) {
//      switch(pushupcounter) {
//        case 1:
//          musicPlayer.playFullFile("1.mp3");
//          break;
//        
//        case 2:
//          musicPlayer.playFullFile("2.mp3");
//          break;
//        
//        case 3:
//          musicPlayer.playFullFile("3.mp3");
//          break;
//        
//        case 4:
//          musicPlayer.playFullFile("4.mp3");
//          break;
//        
//        case 5:
//          musicPlayer.playFullFile("5.mp3");
//          musicPlayer.playFullFile("good_job.mp3");
//          break;
//        
//        case 6:
//          musicPlayer.playFullFile("6.mp3");
//          break;
//        
//        case 7:
//          musicPlayer.playFullFile("7.mp3");
//          break;
//        
//        case 8:
//          musicPlayer.playFullFile("8.mp3");
//          break;
//        
//        case 9:
//          musicPlayer.playFullFile("9.mp3");
//          break;
//        
//        case 10:
//          musicPlayer.playFullFile("10.mp3");
//          musicPlayer.playFullFile("congratulations.mp3");
//          break;
//        
//      }
//    } else if(latraisemode) {
//      if(impingement) {
//        impingement = false;
//        musicPlayer.playFullFile("impingement.mp3");
//      }
//    }
//  }

  // Calibration Block
  if(digitalRead(11)) {
    xycalib = headingxy;
    xzcalib = headingxz;
    yzcalib = headingyz;

    pushupmode = !pushupmode;
    pushupupmode = true;
    pushupupmode = false;
    
    latraisemode = !latraisemode;
    
  }
  
  delay(250); // Controls sampling frequency - delay in ms
}

// Helper to analyze data and modify the justFinishedRep boolean value
// Also impingement
void analyzeData() {

  // justFinishedRep Block
  if(pushupmode) {

    if(pushupupmode) {
      int norm = sqrt(((headingxy - xyDownAngle)*(headingxy - xyDownAngle)) + ((headingxz - xzDownAngle)*(headingxz - xzDownAngle)) + ((headingyz - yzDownAngle)*(headingyz - yzDownAngle)));
      
      if(norm < normThreshold) {
        pushupupmode = false;
        pushupdownmode = true;
      }
    } else if(pushupdownmode) {
      int norm = sqrt(((headingxy - xyUpAngle)*(headingxy - xyUpAngle)) + ((headingxz - xzUpAngle)*(headingxz - xzUpAngle)) + ((headingyz - yzUpAngle)*(headingyz - yzUpAngle)));
    
      if(norm < normThreshold) {
        pushupupmode = true;
        pushupdownmode = false;
        pushupcounter = pushupcounter + 1;
      }
    }
    
    if(headingxy) {
      
    }
    
  } else if(latraisemode) {

    
  }
  
  // Impingement Block
  if(headingxy > xyImpThreshHigh || headingxy < xyImpThreshLow) {
    impingement = true;
    
  } else if(headingxz > xzImpThreshHigh || headingxz < xzImpThreshLow) {
    impingement = true;
    
  } else if(headingyz > yzImpThreshHigh || headingyz < yzImpThreshLow) {
    impingement = true;
  }
  
}

// File listing helper for the music player
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

