///Debugging script for using Adafruit Ultimate GPS and Adafruit BNO055

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

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (10)

//LED Setup
#define LED 13

//Button Setup
#define BUTTON 5
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time
int pause_flag = false;
int pause = false;

void setup()
{
  //Serial Initialization
  while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  //GPS Initialization
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPSSerial.begin(9600);

  // Set which NMEA sentences to send
  //GPSSerial.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28")); // Uncomment to turn on GPRMC and GPGGA
  GPSSerial.println(F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29")); // Uncomment to turn on only the GPRMC sentence

  //Set GPS update rate in milliseconds
  GPSSerial.println(F("$PMTK220,1000*2F")); //(Change the value between comma ',' and asterix '*' -> (,100*))

  // Prevent antenna messages from sending
  GPSSerial.println("$PGCMD,33,0*6D");

  //Accelerometer Initialization
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.println(F("BNO055 not detected"));
    while (1)
    { digitalWrite(LED, LOW);
      delay(100);
      digitalWrite(LED, HIGH);
      delay(100);
    }
  }
  bno.setExtCrystalUse(true);

  //Button Setup
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  delay(1000);

}

void loop()
{

  //Setup Buffer
  int BUFF_MAX = 100;
  char inBuffer[BUFF_MAX];    // buffer used to read NMEA lines from GPS
  byte outBuffer[BUFF_MAX];   // buffer used to write NMEA lines to SD card
  int sizeBuffer = 0;        // counter of how many chars per line
  pause = check_button();

  if (pause == true) {
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(500);

  }
  else {
    while (Serial1.available() > 0) // if serial data available from GPS
    {

      //Handle GPS
      sizeBuffer = Serial1.readBytesUntil('\n', inBuffer, BUFF_MAX);  // read one NMEA line from GPS until end of line

      Serial.write(inBuffer, sizeBuffer);  // write GPS NMEA output to Serial
      Serial.print(F("\r\n"));

    }

    //Read accelerometer
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    //Write accelerometer
    Serial.print(acc.x());    // write ANALOG0 (X) to Serial
    Serial.print(F(", "));
    Serial.print(acc.y());    // write ANALOG1 (Y) to Serial
    Serial.print(F(", "));
    Serial.print(acc.z());    // write ANALOG2 (Z) to Serial
    Serial.print(F(", "));
    Serial.print(F("\r\n"));
    delay(BNO055_SAMPLERATE_DELAY_MS);

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
