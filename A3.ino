/*
 * CSV file format: heartrate, cadence, distance, speed, timstamp
 * 
 * This project reads a csv file generated from a Garmin watch and
 * displays the data via a servo and aa dc gearbox DC motor. 
 * The servo operates the legs of a moving model representing cadence.
 * The DC motor pumps a heart representing heartrate. 
 */

#include <SevSeg.h>
#include <AFMotor.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

float hr = 0;
float cadence = 0;
float distance = 0;
float pace = 0;
float timestamp = 0;
float garbage = 0;
const int chipSelect = 4;
unsigned long lastUpdate;
int updateInterval = 0;
File dataFile;
const int numReadings = 5;
int index = 0;
float hrAvg[numReadings];
int hrAverage = 0;
int hrTotal = 0;
float cadenceAvg[numReadings];
int cadenceAverage = 0;
int cadenceTotal = 0;
unsigned long startTime = 0;


int cadenceArr[160] =
{ 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, //9
  72, 71, 70, 69, 68, 67, 66, 65, 64, 63, //19
  62, 61, 60, 59, 58, 57, 56, 55, 54, 53, //29
  52, 51, 50, 49, 48, 47, 46, 45, 44, 43, //39
  42, 41, 40, 38, 37, 36, 35, 35, 35, 34, //49
  33, 32, 32, 31, 31, 30, 30, 29, 29, 28, //59
  28, 27, 27, 26, 26, 25, 25, 25, 25, 24, //69
  24, 23, 23, 22, 22, 22, 22, 21, 21, 21, //79
  21, 20, 20, 20, 20, 19, 19, 18, 18, 18, //89
  18, 18, 18, 17, 17, 17, 17, 16, 16, 16, //99
  16, 16, 16, 15, 15, 15, 15, 15, 15, 15, //109
  15, 14, 14, 14, 14, 13, 13, 13, 13, 13, //119
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, //129
  12, 12, 12, 11, 11, 11, 11, 11, 11, 11, //139
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10, //149
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10  //159
};

class Sweeper
{
    Servo servo;              // the servo
    int pos;              // current servo position
    int increment;        // increment to move for each interval
    int  updateInterval;      // interval between updates
    unsigned long lastUpdate; // last update of position

  public:
    Sweeper(int interval)
    {
      updateInterval = interval;
      increment = 1;
    }

    void setUpdateInterval(int interval)
    {
      updateInterval = interval;
    }

    void sendTo(int gotoPosition) {
      servo.write(gotoPosition);
    }

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void Detach()
    {
      servo.detach();
    }

    void Update(unsigned long currentMillis)
    {
      if ((currentMillis - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        pos += increment;
        servo.write(pos);
        if ((pos >= 35) || (pos <= 0)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      }
    }
};


Sweeper sweeper1(200);
AF_DCMotor motor(1, MOTOR12_64KHZ);
SevSeg heartrate;

void setup() {
  //Declare what pins are connected to the segments
  int digit1 = 22; //Pin 12 on 5461AS 
  int digit2 = 23; //Pin 9 on 5461AS
  int digit3 = 24; //Pin 8 on 5461AS
  int digit4 = 25; //Pin 6 on 5461AS  
  int segA = 26; //Pin 11 on 5461AS
  int segB = 27; //Pin 7 on 5461AS
  int segC = 28; //Pin 4 on 5461AS
  int segD = 29; //Pin 2 on 5461AS
  int segE = 30; //Pin 1 on 5461AS
  int segF = 31; //Pin 10 on 5461AS
  int segG = 32; //Pin 5 on 5461AS
  int segDP = 33; //Pin 3 on 5461AS
  int numberOfDigits = 4; 
  int displayType = COMMON_CATHODE;
  
  heartrate.Begin(displayType, numberOfDigits, digit1, digit2, digit3, digit4, segA, segB, segC, segD, segE, segF, segG, segDP);

  heartrate.SetBrightness(100); //Set the display to 100% brightness level
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  // open the file.
  dataFile = SD.open("run.csv");
  while (!dataFile) {};
  
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  motor.setSpeed(150);
  motor.run(FORWARD);
  sweeper1.sendTo(10);
  sweeper1.Attach(9);
}

SIGNAL(TIMER0_COMPA_vect)
{
  unsigned long currentMillis = millis();
  sweeper1.Update(currentMillis);
}

void loop() {

  if (dataFile) {
    unsigned long currentTime = millis();

    //unsigned long currentLegTime = millis();
    if (currentTime - startTime >= 1000) {
      //Serial.println("Time to go!");
      getData();
      hrAvg[index % 5] = hr;
      hrTotal = 0;
      for (int i = 0; i < numReadings; i++) {
        hrTotal += hrAvg[i];
      }
      hrAverage = hrTotal / numReadings;

      cadenceAvg[index % 5] = cadence;
      cadenceTotal = 0;
      for (int i = 0; i < numReadings; i++) {
        cadenceTotal += cadenceAvg[i];
      }
      cadenceAverage = cadenceTotal / numReadings;
      index++;
      //Serial.println(cadenceAverage);
      sweeper1.setUpdateInterval(cadenceArr[cadenceAverage]);

      //Serial.println(hrAverage);
      motor.setSpeed(map(hrAverage, 0, 200, 50, 255));
      
      startTime = currentTime;
      //printRawData();
      //printAverageData();
    }
    char tempString[10];
    sprintf(tempString, "%4d", hrAverage);
    heartrate.DisplayString(tempString, 3);
  }
}

void printAverageData() {
  Serial.print("hr array: {");
  for (int i = 0; i < numReadings; i++) {
    Serial.print(hrAvg[i]);
    Serial.print(" ");
  }
  Serial.println(']');
  Serial.print("hr average: ");
  Serial.println(hrAverage);

  Serial.print("cadence array: [");
  for (int i = 0; i < numReadings; i++) {
    Serial.print(cadenceAvg[i]);
    Serial.print(" ");
  }
  Serial.println(']');
  Serial.print("cadence average: ");
  Serial.println(cadenceAverage);

}
void printRawData() {
  Serial.print("heartrate: ");
  Serial.print(hr);
  Serial.print(", ");

  Serial.print("cadence: ");
  Serial.print(cadence);
  Serial.print(", ");

  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print(", ");

  Serial.print("pace: ");
  Serial.print(pace);
  Serial.print(", ");

  Serial.print("timestamp: ");
  Serial.print(timestamp);
  Serial.print(", ");

  Serial.print("garbage: ");
  Serial.println(garbage);
}

void getData() {
  hr = readFloat(dataFile);
  cadence = readFloat(dataFile);
  distance = readFloat(dataFile);
  pace = readFloat(dataFile);
  timestamp = readFloat(dataFile);
  garbage = readFloat(dataFile);
}

float readFloat(File dataFile) {
  char cArray[6];
  int pos = 0;
  char c;

  c = dataFile.read();
  while (isDigit(c) || c == '.') {
    if (pos <= 5) {
      cArray[pos] = c;
      pos++;
    }
    c = dataFile.read();
  }

  float f = atof(cArray);
  return f;
}
