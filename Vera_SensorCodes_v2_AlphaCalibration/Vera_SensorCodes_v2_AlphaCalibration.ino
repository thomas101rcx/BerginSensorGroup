/* Bergin Lab - Air Sensor Package
 * Logs box number, date, time, rh, temp, CO2, PM1/2.5/10
 * by Alan Guo & Vera Xu
 *
 *Format Logging in SD Card:
 *yyyy/mm/dd hh:mm:ss tsi_pm1 tsi_pm2.5 tsi_pm10 pm1 pm2.5 pm10 CO2 temp(C) humidity(%)
 *
 */

const char* buffer = "photon.txt";
String boxLabel = "Sensors Box 2";
String calibrationTag = "Box01";

//Libraries
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <SD.h>
#include <SD_t3.h>
#include <SHT1X.h>

//Definitions
#define LOOP_DELAY 200
RTC_DS3231 RTC;
int sampleSize;
int redPin = 8;
unsigned long timeElapsed;
unsigned long firstTime;
unsigned long logTime;
unsigned long prevTimeElapsed;

//PM Sensors:
uint16_t pm10 = 0;
uint16_t pm25 = 0;
uint16_t pm100 = 0;
uint16_t tpm10 = 0;
uint16_t tpm25 = 0;
uint16_t tpm100 = 0;

uint16_t tpm10Sum;
uint16_t tpm25Sum;
uint16_t tpm100Sum;
uint16_t pm10Sum;
uint16_t pm25Sum;
uint16_t pm100Sum;
int PMerrors = 0;
uint8_t buf[24];

uint16_t TPM01ValueAvg;
uint16_t TPM2_5ValueAvg;
uint16_t TPM10ValueAvg;
uint16_t PM01ValueAvg;
uint16_t PM2_5ValueAvg;
uint16_t PM10ValueAvg;

//COZIRf: CO2 Sensors
double val;
double CO2conc = 0.0;
double FS = 2000;
double vsupply = 1023;
double CO2concSum;
double CO2concAvg;

//Temp & Hum Sensor:
int tempCerrors = 0;
int humErrors = 0;
float tempC = 0;
float tempCsum;
float tempCavg;

float humidity = 0;
float humiditySum;
float humidityAvg;

const int dataPin = 6;
const int clockPin = 5;  //used in SHT1x sht1x(dataPin, clockPin);

//Alphasensor:
float workNO, worksumNO, auxNO, auxsumNO, workCO, worksumCO, auxCO, auxsumCO, NO, CO, sumNO, sumCO, avgNO, avgCO = 0;
float workNO2, worksumNO2, auxNO2, auxsumNO2, workO3, worksumO3, auxO3, auxsumO3, NO2, O3, avgNO2, avgO3, sumNO2, sumO3 = 0;
float NOsum, COsum, NO2sum, O3sum, NOavg, COavg, NO2avg, O3avg;
int workNO2pin = A0;
int auxNO2pin = A1;
int workO3pin = A2;
int auxO3pin = A3;
int workNOpin = A6;
int auxNOpin = A7;
int workCOpin = A8;
int auxCOpin = A9;


//Calibration:
float zeroNO, aeNO, sensitivityNO, zeroCO, aeCO, sensitivityCO, zeroNO2, aeNO2, sensitivityNO2, zeroO3, aeO3, sensO3, auxNO2O3;

//SD Logging:
const int chipSelect = 10;
File myFile;
String dataString = ""; // make a string for assembling the data to log:
String firstDataString = "";

//Photon Control:
int Pin1 = 8;
int Pin2 = 9;
bool SDoff = 0;
int SDmarker;

//***SET UP***//
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  pinMode(redPin, OUTPUT);
  SPI.begin();

  //Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");  // keep flashing LED
      digitalWrite(redPin, HIGH);
      delay(300);
      digitalWrite(redPin, LOW);
      delay(300);
    return;
  }
  Serial.println("done.");

  //Initialize COZIR
  Serial.print("Initializing COZIR...");
  pinMode(A14, INPUT);
  Serial.println("done");

  //Initialize RTC
  Serial.print("Initializing RTC Chronodot...");
  digitalWrite(redPin, HIGH);   //LED lights on before RTC is set up

  Wire.begin();
  RTC.begin();

  Serial.print("Doing RTC checks...");
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
  }
  RTC.adjust(DateTime(__DATE__, __TIME__));
  Serial.print("Setting up RTC now...");
  char datastr[100];
  RTC.getControlRegisterData( datastr[0]  );
  delay(1000);
  Serial.println("done");

  // SD logging string
  firstDataString = TimeString();
  //myFile = SD.open(buffer, FILE_WRITE);
  String firstStr = boxLabel + "...New Logging Session..." + firstDataString;
  sdLog(buffer, firstStr);
  String infoStr = "yyyy/mm/dd, hh:mm:ss, TPM1, TPM2.5, TPM10, PM1, PM2.5, PM10, CO2, TempC, Humidity, NO, CO, NO2, O3";
  sdLog(buffer, infoStr);

  //Timing
  firstTime = millis();
  prevTimeElapsed = 0;

  //Calibration:
  String sdCali = sdRead("Cali.txt");
  //Serial.print(sdCali);
  Calibration(sdCali);

  //mark the end of setup
  digitalWrite(redPin, LOW);
  delay(1000);
  digitalWrite(redPin, HIGH);
  int SDmarker=1;

}

void Calibration(String in) {
  String curr;
  String myData[14];
  int index = 0;
  int start = 0;;
  for (int i = 0; i < 69; i++) {
    curr = in.substring(i, i + 1);
    if (curr == " ") {
      myData[index] = in.substring(start, i);
      index++;
      start = i + 1;
    }
  }
  myData[index] = in.substring(start);

  //  for (int i = 1; i < 14; i++) {
  //    Serial.print(myData[i].toFloat(), 3);
  //    Serial.print("\n");
  //  }

  zeroNO = myData[1].toFloat();
  aeNO = myData[2].toFloat();
  sensitivityNO = myData[3].toFloat();
  zeroCO = myData[4].toFloat();
  aeCO = myData[5].toFloat();
  sensitivityCO = myData[6].toFloat();
  zeroNO2 = myData[7].toFloat();
  aeNO2 = myData[8].toFloat();
  sensitivityNO2 = myData[9].toFloat();
  zeroO3 = myData[10].toFloat();
  aeO3 = myData[11].toFloat();
  auxNO2O3 = myData[12].toFloat();
  sensO3 = myData[13].toFloat();
}

void loop() {
  bool getFlag = getPMValues();
  if (getFlag) {
    sampleSize++;
    TimePrint();
    PMPrint();
    getCO2Values();
    getSHT1X();
    getAlpha();
    TimeElapseCalculation();
    Serial.print("\n");

    if (timeElapsed > 6000) {   //Take Avg Every Minute
      AverageCalculation();
      dataString = TimeString();

      dataString += String(TPM01ValueAvg) + ", " + String(TPM2_5ValueAvg) + ", " + String(TPM10ValueAvg);
      dataString += ", " + String(PM01ValueAvg) + ", " + String(PM2_5ValueAvg) + ", " + String(PM10ValueAvg);
      dataString += ", " + String(CO2concAvg, 2) + ", " + String(tempCavg, 2) + ", " + String(humidityAvg, 2);
      dataString += ", " + String(NOavg) + ", " + String(COavg) + ", " + String(NO2avg) + ", " + String(O3avg);

      Serial.println("--------MINUTE AVERAGE below------------");
      Serial.println(dataString);
      Serial.println("----------------------------------------");

      //check if log is successful:
      SDmarker = sdLog(buffer, dataString);
      while (!SDmarker) {
          Serial.println("Card failed, or not present");  // keep flashing LED         
            digitalWrite(redPin, HIGH);
            delay(300);
            digitalWrite(redPin, LOW);
            delay(300);
      }
      //Reset firstTime to the last logTime value
      firstTime = logTime;
      ReSet(true);
      //photon();
    }
  }
  delay(100);
}

//***To Calculate Time Elapsed***//

void TimeElapseCalculation() {
  logTime = millis();
  timeElapsed = logTime - firstTime;
  Serial.print("Time elapsed: ");
  Serial.println(timeElapsed);

  long intervalTime = timeElapsed - prevTimeElapsed;
  Serial.print("Time from last measurement: ");
  Serial.println(intervalTime);
  prevTimeElapsed = timeElapsed;
}

//***To Print Date and Time***//

void TimePrint() {
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" ");
}

//***To Get Value from CO2 Sensor***//
void getCO2Values() {
  val = analogRead(A14);
  CO2conc = FS * (val / vsupply);
  CO2concSum += CO2conc;
  Serial.print("CO2=");
  Serial.print(CO2conc);
}

//***To Get Value from Temperature and Humidity Sensor***//
void getSHT1X() {
  SHT1x sht1x(dataPin, clockPin);
  tempC = sht1x.readTemperatureC();
  if (tempC < 0 || tempC > 50) {
    tempCerrors++;
  }
  else {
    tempCsum += tempC;
  }
  humidity = sht1x.readHumidity();
  if (humidity < 0 || humidity > 100) {
    humErrors++;
  }
  else {
    humiditySum += humidity;
  }
  Serial.print(" Temp = ");
  Serial.print(tempC);
  Serial.print(" ");
  Serial.print("C");
  Serial.print(" Humidity = ");
  Serial.print(humidity);
  Serial.print("%");
  Serial.print("\n");
}

//***To Get Value from Alphasensor***//
void getAlpha() {
  workNO = analogRead(workNOpin) * 4.9;
  auxNO = analogRead(auxNOpin) * 4.9;
  workCO = analogRead(workCOpin) * 4.9;
  auxCO = analogRead(auxCOpin) * 4.9;

  workNO2 = analogRead(workNO2pin) * 4.9;
  auxNO2 = analogRead(auxNO2pin) * 4.9;
  workO3 = analogRead(workO3pin) * 4.9;
  auxO3 = analogRead(auxO3pin) * 4.9;

  NO = ((workNO - zeroNO) - (auxNO - aeNO)) / sensitivityNO;
  CO = ((workCO - zeroCO) - (auxCO - aeCO)) / sensitivityCO;
  NO2 = ((workNO2 - zeroNO2) - (auxNO2 - aeNO2)) / sensitivityNO2;
  O3 = (((workO3 - zeroO3) - (auxO3 - aeO3)) - NO2 * auxNO2O3) / sensO3;

  //    NO = ((workNO - 271) - (auxNO - 270)) / 0.352;
  //    CO = ((workCO - 280) - (auxCO - 279)) / 0.243;
  //    NO2 = ((workNO2 - 298) - (auxNO2 - 294)) / 0.456;
  //    O3 = (((workO3 - 412) - (auxO3 - 407)) - NO2 * 0.333) / 0.222;

  Serial.print("NO = ");
  Serial.print(NO);
  Serial.print(" CO = ");
  Serial.print(CO);
  Serial.print(" NO2 = ");
  Serial.print(NO2);
  Serial.print(" O3 = ");
  Serial.println(O3);

  NOsum += NO;
  COsum += CO;
  NO2sum += NO2;
  O3sum += O3;
}

//***To Get Value from PM Sensor***//
bool getPMValues() {
  int idx;
  bool hasPm25Value = false;
  int timeout = 200;
  String printout = "";
  bool flag = false;
  while (!hasPm25Value) {
    idx = 0;
    memset(buf, 0, 24);
    while (Serial1.available()) {
      buf[idx++] = Serial1.read();
    }

    if (buf[0] == 0x42 && buf[1] == 0x4d) {
      pm25 = ( buf[12] << 8 ) | buf[13];
      pm10 = ( buf[10] << 8 ) | buf[11];
      pm100 = ( buf[14] << 8 ) | buf[15];
      tpm10 = ( buf[4] << 8 ) | buf[5];
      tpm25 = ( buf[6] << 8 ) | buf[7];
      tpm100 = ( buf[8] << 8 ) | buf[9];

      if (checkValue(buf, 24)) {
        flag = true;
        tpm10Sum += tpm10;  tpm25Sum += tpm25;  tpm100Sum += tpm100;
        pm10Sum += pm10; pm10Sum += pm25; pm100Sum += pm100;
        //Serial.println(PMerrors);
      } else {
        PMerrors++;
      }
      hasPm25Value = true;

      // Debbugging Tool: Print out 24 bytes PM data (See communication protocol for details)
      //      for (int j = 0; j < 24; j++) {
      //        printout = printout + " " + buf[j];
      //      }
      //      Serial.print(printout);
      //      Serial.print("\n");
      //      Serial.print("\n");
    }
    timeout--;
    if (timeout < 0) {
      break;
    }
  }
  return flag;
}

void PMPrint() {
  Serial.print("pm1.0= ");
  Serial.print(pm10);
  Serial.print(" ug/m3 ");
  Serial.print("pm2.5= ");
  Serial.print(pm25);
  Serial.print(" ug/m3 ");
  Serial.print("pm100= ");
  Serial.print(pm100);
  Serial.print(" ug/m3 ");
  Serial.print("tpm1.0= ");
  Serial.print(tpm10);
  Serial.print(" ug/m3 ");
  Serial.print("tpm2.5= ");
  Serial.print(tpm25);
  Serial.print(" ug/m3 ");
  Serial.print("tpm100= ");
  Serial.print(tpm100);
  Serial.print(" ug/m3 \n");
}

// ***Checksum for PM values ***//
int checkValue(uint8_t thebuf[24], int leng)
{
  char receiveflag = 0;
  int receiveSum = 0;
  int i = 0;

  for (i = 0; i < leng; i++) {
    receiveSum = receiveSum + thebuf[i];
  }

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1] + thebuf[leng - 2] + thebuf[leng - 1])) //checksum the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

// ***Log values into SD Card ***//
int sdLog(const char* fileName, String stringToWrite) {
  int marker;
  File myFile = SD.open(fileName, FILE_WRITE);
  for (int x = 0; x < 5; x++) {
    digitalWrite(redPin, LOW);
    delay(50);
    digitalWrite(redPin, HIGH);
    delay(50);
  }

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print(" to ");
    Serial.print(fileName);
    Serial.print("...");
    myFile.println(stringToWrite);
    myFile.close();
    Serial.println("done.");
    digitalWrite(redPin, HIGH);
    marker = 1;
  } else {
    Serial.print("error opening ");
    Serial.println(fileName);
    digitalWrite(redPin, LOW);
    marker = 0;
  }
  return marker;
}

//**Module to form datastring which will be logged into SD card**//
String TimeString() {
  DateTime now = RTC.now();
  dataString = String(now.year(), DEC);
  dataString += "/";
  dataString += String(now.month(), DEC);
  dataString += "/";
  dataString += String(now.day(), DEC);
  dataString += ", ";
  dataString += String(now.hour(), DEC);
  dataString += ":";
  dataString += String(now.minute(), DEC);
  dataString += ":";
  dataString += String(now.second(), DEC);
  dataString += ", ";

  return dataString;
}

void ReSet(boolean PMReSet) {
  CO2concSum = 0;
  tempCsum = 0;
  humiditySum = 0;
  NOsum = 0;
  NO2sum = 0;
  O3sum = 0;
  COsum = 0;

  CO2concAvg = 0;
  tempCavg = 0;
  humidityAvg = 0;

  sampleSize = 0;
  humErrors = 0;
  tempCerrors = 0;

  if (PMReSet) {
    tpm10Sum = 0;
    tpm25Sum = 0;
    tpm100Sum = 0;
    pm10Sum = 0;
    pm25Sum = 0;
    pm100Sum = 0;

    TPM01ValueAvg = 0;
    TPM2_5ValueAvg = 0;
    TPM10ValueAvg = 0;
    PM01ValueAvg = 0;
    PM2_5ValueAvg = 0;
    PM10ValueAvg = 0;

    PMerrors = 0;
  }
}

void AverageCalculation() {
  Serial.print("A minute has elapsed...");

  TPM01ValueAvg = tpm10Sum / sampleSize;
  TPM2_5ValueAvg = tpm25Sum / sampleSize;
  TPM10ValueAvg = tpm100Sum / sampleSize;
  PM01ValueAvg = pm10Sum / sampleSize;
  PM2_5ValueAvg = pm25Sum / sampleSize;
  PM10ValueAvg = pm100Sum / sampleSize;

  CO2concAvg = CO2concSum / sampleSize;
  tempCavg = tempCsum / sampleSize;
  humidityAvg = humiditySum / sampleSize;

  NOavg = NOsum / sampleSize;
  COavg = COsum / sampleSize;
  NO2avg = NO2sum / sampleSize;
  O3avg = O3sum / sampleSize;

  Serial.println("Calculated avgs...");
}

String sdRead(const char* fileName) {
  File myFile = SD.open(fileName, FILE_READ);
  String data, data2, sdOut;

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print(" from ");
    Serial.print(fileName);
    Serial.print("...");

    while ((data = myFile.read()) != "z") {
      data2 += data;
    }
    myFile.close();
    digitalWrite(redPin, HIGH);
  } else {
    Serial.print("error opening ");
    Serial.println(fileName);
    for (int x = 0; x < 3; x++) {
      digitalWrite(redPin, HIGH);
      delay(50);
      digitalWrite(redPin, LOW);
      delay(50);
    }
    digitalWrite(redPin, LOW);
  }

  //Truncate String Read to Matching Tag:
  bool out = true;
  while (out) {
    if (data2.substring(0, 5) == calibrationTag) {
      sdOut = data2.substring(0, 69);
      out = false;
    } else {
      data2 = data2.substring(69);
    }
  }
  Serial.print("done\n");
  return sdOut;
}

void photon() {
  SPI.end();
  Serial.print("Closed SPI...\n");

  //Turn on Photon:
  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);
  Serial.print("Turned on Photon\n");

  delay(1000); //10s

  //Turn off Photon:
  digitalWrite(Pin2, LOW);
  digitalWrite(Pin1, HIGH);
  Serial.print("Turned off Photon\n");
  SDoff = 1;
}

void restartSD() {
  //Initialize SD Card
  //  SPI.begin();
  //  Serial.print("Restarting SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");  // keep flashing LED
    while (true) {
      digitalWrite(redPin, HIGH);
      delay(300);
      digitalWrite(redPin, LOW);
      delay(300);
    }
    return;
  }
  Serial.println("done.");
  SDoff = 0;
}



