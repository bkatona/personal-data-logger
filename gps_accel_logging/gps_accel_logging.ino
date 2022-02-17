//Create a datalogger using Adafruit Ultimate GPS and Adafruit BNO055 IMU

//GPS Setup

//Define GPS hardware serial
#define GPSSerial Serial1

//Accelerometer Libraries and Setup

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//Set delay between samples
#define BNO055_SAMPLERATE_DELAY_MS (10)

//SD Libraries and Setup

#include <SPI.h>
#include <SD.h>
File logfile; //declare logging file name
unsigned long current_time = millis(); // maintain time for timer
const int BUFF_MAX = 85; //maximum length of GPS serial buffer
char inBuffer[BUFF_MAX];    // buffer used to read NMEA lines from GPS
int sizeBuffer = 0;        // counter of how many chars per line

//LED Setup

#define LED 13 //Red LED #13
#define SD_LED 8 //Green LED #8

//Button Setup

#define BUTTON 5
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time
int pause_flag = true; //keep track if pause is triggered
int isPaused = true; //keep track if currently puased
int pause_msg = true; //keep track if puase message has been printed (also keeps track if in first iteration of pause loop)
int redLedState = LOW;
unsigned long ledPreviousMillis = 0; //Keep track of led timer

void setup()
{
  //LED initialization
  
  pinMode(LED, OUTPUT); // Red LED #13
  pinMode(SD_LED, OUTPUT); //Green LED #8

  //Button Setup
  
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  //Serial Initialization
  Serial.begin(115200);

  //GPS Initiallizatoin
  initialize_GPS();

  //Accelerometer Initialization
  initialize_accel();

  // SD Card Initialization
  initialize_SD();
}

void loop() {
  //Check if pause button has been pushed
  
  isPaused = check_button();

  //If button has been pushed, enter pause loop
  
  if (isPaused == true) {
    pause();
  }

  // If not paused, record GPS and accelerometer readings
  
  else {

    //If first time after being puased (pause_msg flag not yet reset), then clear GPS buffer of old data
    
    if (pause_msg == true) {

      //Clear pause LED if still on
      digitalWrite(LED, LOW);

      //Read old GPS data
      while (Serial1.available() > 0) {
        sizeBuffer = Serial1.readBytesUntil('\n', inBuffer, BUFF_MAX);
      }
      
      memset(inBuffer, 0, sizeof(inBuffer)); //Clear old GPS data
      logfile.println(F("Start")); //Write Start message to file
      pause_msg = false; //clear pause_msg flag
    }

    //Handle GPS
    record_GPS();

    //Handle Accelerometer
    record_accel();

    //Every 30 seconds, flush SD buffer
    
    if (millis() - current_time > 30000) {
      digitalWrite(SD_LED, HIGH); //Flash SD LED
      logfile.flush();
      Serial.println(F("Flushing"));
      current_time = millis();
    }
    
    //Otherwise delay before querying accelerometer again
    
    else {
      digitalWrite(SD_LED, LOW);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
}








int check_button() {
  // read the state of the switch into a local variable:

  int reading = digitalRead(BUTTON);

  // If the switch changed, due to noise or pressing:

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // State has changed long enough to not be noise

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the flag if the new button state is LOW
      if (buttonState == LOW) {
        pause_flag = !pause_flag;

      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  return (pause_flag);
}

void initialize_GPS() {
  //GPS Initialization
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSSerial.begin(9600);
  delay(100);

  // Set which NMEA sentences to send
  //GPSSerial.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28")); // Uncomment to turn on GPRMC and GPGGA
  GPSSerial.println(F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29")); // Uncomment to turn on only the GPRMC sentence

  //Set GPS update rate in milliseconds
  GPSSerial.println(F("$PMTK220,1000*2F")); //(Change the value between comma ',' and asterix '*' -> (,100*))

  // Prevent antenna messages from sending
  GPSSerial.println("$PGCMD,33,0*6D");

}

void initialize_accel() {
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.println(F("BNO055 not detected"));
    while (1) {
      digitalWrite(LED, LOW);
      delay(100);
      digitalWrite(LED, HIGH);
      delay(100);
    }
  }
  bno.setExtCrystalUse(true);
}

void initialize_SD() {
  digitalWrite(SD_LED, HIGH);
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  const int chipSelect = 4;
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    while (1) {
      digitalWrite(SD_LED, HIGH);
      delay(100);
      digitalWrite(SD_LED, LOW);
      delay(100);
    }
  }

  Serial.println(F("card initialized."));
  logfile = SD.open("logfile.txt", FILE_WRITE);
  delay(1000);
  digitalWrite(SD_LED, LOW);

}

void flash_led(int led_pin, const long led_interval) {
  int ledState = digitalRead(led_pin);
  unsigned long ledCurrentMillis = millis();
  if (ledCurrentMillis - ledPreviousMillis >= led_interval) {
    ledPreviousMillis = ledCurrentMillis;
    ledState = !ledState;
    //Serial.println(ledState);
    digitalWrite(led_pin, ledState);
  }
}

void record_GPS() {
  //If serial data is available, read and record GPS
  while (Serial1.available() > 0) {// if serial data available from GPS
    sizeBuffer = Serial1.readBytesUntil('\n', inBuffer, BUFF_MAX);  // read one NMEA line from GPS until end of line

    if (logfile) {
      logfile.write(inBuffer, sizeBuffer);  // write GPS NMEA output to Serial
      logfile.print(F("\r\n"));

    }
  }
}

void record_accel() {
  //Read acclerometer data
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  //Record acclerometer data
  if (logfile) {

    logfile.print(acc.x());    // write Accelerometer (X) to Serial
    logfile.print(F(" , "));
    logfile.print(acc.y());    // write Accelerometer (Y) to Serial
    logfile.print(F(" , "));
    logfile.print(acc.z());    // write Accelerometer (Z) to Serial
    //logfile.print(F(" , "));
    logfile.print(F("\r\n"));

  }
}

void pause() {
  //If first time through iteration (pause message has not yet been printed), flush SD and write message
  if (pause_msg == false) {

    logfile.println(F("*Paused*")); //Print pause message to file
    digitalWrite(SD_LED, HIGH); //Flash SD LED
    logfile.flush(); // Flush SD buffer
    pause_msg = true; //Indicate msg has been printed, finished first pause loop iteration
    Serial.println(F("Flushing"));
    digitalWrite(SD_LED, LOW);

  }

  //If not on first pause loop iteration, flash red LED
  else {
    flash_led(LED, 500);
  }
}
