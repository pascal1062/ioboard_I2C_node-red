
/*****************************************************************************
  2017-04-09 -> testing new AD filters instead of average.... 
  2017-04-05 -> Add INPUTS average calculations 
  2016-12-06 -> Add crc to bytes sended on I2C. now message is 20 bytes long.
******************************************************************************/

// inslude the SPI library:
#include <SPI.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x12

SPISettings MCP3008(500000, MSBFIRST, SPI_MODE0);
SPISettings TLC5620(500000, MSBFIRST, SPI_MODE1);

// set slave select pins
const int adcChipSelect = 8;
const int dacLoadPin[] = {10, 9}; //LOAD pin. 2 DACs TLC5620

long previousMillisA = 0;
long previousMillisB = 0;

int analogInput[8] = {0,23,189,302,443,604,903,1023};  //un test pour les averages.....
byte analogInputBytes[16]; //analog input 0@7 separate in two bytes MSB,LSB {MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB,MSB,LSB}
int idx = 0;
int aiIdx = 0;

byte analogOutput[8];
const byte dacAddress[] = {0x03, 0x01, 0x05, 0x07}; //DAC addr and range bit. first bytes to send to TLC5620

int analogInputAvg[8]; 
int readingsAI1[10];
int readingsAI2[10];
int readingsAI3[10];
int readingsAI4[10];
int readingsAI5[10];
int readingsAI6[10];
int readingsAI7[10];
int readingsAI8[10];
byte count = 0;


void setup() {

  // set the ChipSelect as an output:
  pinMode(adcChipSelect, OUTPUT);
  pinMode(dacLoadPin[0], OUTPUT);
  pinMode(dacLoadPin[1], OUTPUT);

  // initialize SPI:
  SPI.begin();
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.begin(9600);

  digitalWrite(adcChipSelect, LOW);
  digitalWrite(adcChipSelect, HIGH);

  digitalWrite(dacLoadPin[0], HIGH);
  digitalWrite(dacLoadPin[1], HIGH);
}

void loop() {

  for (int i = 0; i <= 7; i++) {
    //analogInput[i] = adcChannelRead(i);
    analogInTwoBytes(i);
  }

  readAvgAnalogIn();
  analogOutputWrite();
  serialOut();
 
}

int adcChannelRead(byte readAddress) {

  byte dataMSB = 0;
  byte dataLSB = 0;
  byte JUNK = 0x00;

  SPI.beginTransaction(MCP3008);
  digitalWrite(adcChipSelect, LOW);
  SPI.transfer(0x01); // Start Bit
  dataMSB = SPI.transfer(((0x01 <<  3) | readAddress) << 4) & 0x03; // Send readAddress and receive MSB data, masked to two bits
  dataLSB = SPI.transfer(JUNK); // Push junk data and get LSB byte return
  digitalWrite(adcChipSelect, HIGH);
  SPI.endTransaction();

  return dataMSB << 8 | dataLSB;
}

void readAvgAnalogIn() {  
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillisA  > 5) {
    previousMillisA = currentMillis;

    if (count > 9) count = 0;
    readingsAI1[count] = analogInput[0];
    readingsAI2[count] = analogInput[1];
    readingsAI3[count] = analogInput[2];
    readingsAI4[count] = analogInput[3];
    readingsAI5[count] = analogInput[4];
    readingsAI6[count] = analogInput[5];
    readingsAI7[count] = analogInput[6];
    readingsAI8[count] = analogInput[7]; 
    count++;   

    analogInputAvg[0] = getAgerage(readingsAI1);
    analogInputAvg[1] = getAgerage(readingsAI2);
    analogInputAvg[2] = getAgerage(readingsAI3);
    analogInputAvg[3] = getAgerage(readingsAI4);
    analogInputAvg[4] = getAgerage(readingsAI5);
    analogInputAvg[5] = getAgerage(readingsAI6);
    analogInputAvg[6] = getAgerage(readingsAI7);
    analogInputAvg[7] = getAgerage(readingsAI8);
    
  }
}

int getAgerage(int readings[10]) {
    int sum = 0;
    int avg = 0;
    
    for (int i = 0; i < 10; i++) {
       sum += readings[i]; 
    }
    avg = sum / 10; 
   
    return avg;
}

void analogInTwoBytes(int id) { //Put 10 bits analog input in two separate bytes. For sending through I2C.
  //extract high and low from analog input readings. Stores in array after....
  int AnlgIn = analogInputAvg[id];
  byte high = (AnlgIn >> 8) & 0xFF;
  byte low = AnlgIn & 0xFF;

  if (id == 0) {
    aiIdx = 0;
  }
  analogInputBytes[aiIdx] = high;
  analogInputBytes[aiIdx + 1] = low;
  aiIdx += 2;
}

void dacChannelWrite(byte high, byte low, int loadPin) {

  SPI.beginTransaction(TLC5620);
  digitalWrite(loadPin, HIGH);
  SPI.transfer(high);
  SPI.transfer(low);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, LOW);
  digitalWrite(loadPin, HIGH);
  SPI.endTransaction();
}

void analogOutputWrite() {

  for (int a = 0; a <= 3; a++) {
    dacChannelWrite(dacAddress[a], analogOutput[a], dacLoadPin[0]); //write addr and value to DAC #1 TCL5620
  }
  for (int a = 0; a <= 3; a++) {
    dacChannelWrite(dacAddress[a], analogOutput[a + 4], dacLoadPin[1]); //write addr and value to DAC #2 TCL5620
  }
}

void receiveData(int bytes) {

  int operation = Wire.read();
  byte value;

  // Update analog Output value
  if (operation == 85 && bytes > 1) {
    value = Wire.read();
    analogOutput[0] = value;
  } else if (operation == 86 && bytes > 1) {
    value = Wire.read();
    analogOutput[1] = value;
  } else if (operation == 87 && bytes > 1) {
    value = Wire.read();
    analogOutput[2] = value;
  } else if (operation == 88 && bytes > 1) {
    value = Wire.read();
    analogOutput[3] = value;
  } else if (operation == 89 && bytes > 1) {
    value = Wire.read();
    analogOutput[4] = value;
  } else if (operation == 90 && bytes > 1) {
    value = Wire.read();
    analogOutput[5] = value;
  } else if (operation == 91 && bytes > 1) {
    value = Wire.read();
    analogOutput[6] = value;
  } else if (operation == 92 && bytes > 1) {
    value = Wire.read();
    analogOutput[7] = value;
  } else if (operation == 0xAA && bytes > 1) {
    // Reset analog readings...
    idx = 0;
  }

  // Consume any remainder bytes...
  while (Wire.available()) {
    Wire.read();
  }
}

void sendData() {
  byte data[20];
  unsigned int crc16;
  data[0] = 0; //start byte
  data[1] = 16; //lenght (exclude start and crc calc)
    for (int i=2; i<=17; i++) {
      data[i] = analogInputBytes[i-2];
  }
  crc16 = generateCRC(analogInputBytes, 16);
  data[18] = crc16 >> 8;
  data[19] = crc16 & 0xFF;
  Wire.write((byte*) &data, 20);   
  
  //Wire.write((byte*) &analogInputBytes, sizeof(analogInputBytes));
}

unsigned int generateCRC(byte *buf, byte messageLength) {
  unsigned int crc = 0xFFFF;
  unsigned int crcHigh = 0;
  unsigned int crcLow = 0;
  int i, j = 0;

  for (i = 0;i < messageLength;i++)
  {
    crc ^= buf[i];
    for (j = 8; j != 0; j--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  //bytes are wrong way round so doing a swap here..
  crcHigh = (crc & 0x00FF) << 8;
  crcLow = (crc & 0xFF00) >> 8;
  crcHigh |= crcLow;
  crc = crcHigh;
  return crc;
}

void serialOut() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisB > 1000) {
    // save the last time
    previousMillisB = currentMillis;

    Serial.print(analogInputAvg[0]);
    Serial.print(";");
    Serial.print(analogInputAvg[1]);
    Serial.print(";");
    Serial.print(analogInputAvg[2]);
    Serial.print(";");
    Serial.print(analogInputAvg[3]);
    Serial.print(";");
    Serial.print(analogInputAvg[4]);
    Serial.print(";");
    Serial.print(analogInputAvg[5]);
    Serial.print(";");
    Serial.print(analogInputAvg[6]);
    Serial.print(";");
    Serial.print(analogInputAvg[7]);
    Serial.println();

    for (int i = 0; i<16; i++) {
      Serial.print(analogInputBytes[i]);
      Serial.print(";");      
    }
    Serial.println();
    
    //Serial.print(analogOutput[0]);
    //Serial.print(";");
    //Serial.print(analogOutput[1]);
    //Serial.print(";");
    //Serial.print(analogOutput[2]);
    //Serial.print(";");
    //Serial.print(analogOutput[3]);
    //Serial.print(";");
    //Serial.print(analogOutput[4]);
    //Serial.print(";");
    //Serial.print(analogOutput[5]);
    //Serial.print(";");
    //Serial.print(analogOutput[6]);
    //Serial.print(";");
    //Serial.print(analogOutput[7]);
    //Serial.println();;
  }
}

//Fin


