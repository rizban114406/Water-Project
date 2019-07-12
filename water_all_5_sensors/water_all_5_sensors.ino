#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 33, en = 35, d4 = 37, d5 = 39, d6 = 41, d7 = 43;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



#define Vref 4.95
#define VREF 5000 


#define turibidity_sensorPin A0
#define DoSensorPin  A1
#define PH_sensorPin A2
#define ONE_WIRE_BUS 2

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float temperature;
float temperature2 = 25.00;
float doValue;
unsigned long int avgValue;     //Store the average value of the sensor feedback
int i=0;
float ph;
float sensorValue;
int m;
long sensorSum;
int buf[10]; 
int device_id=10003;
const int csPin = 53; 
float ntu;

int dataSendingInterval=1;
long previousMillis = 0;
boolean initialValue = false;
#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];    // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 12          //the address of the Saturation Oxygen voltage stored in the EEPROM
#define SaturationDoTemperatureAddress 16      //the address of the Saturation Oxygen temperature stored in the EEPROM
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};


void setup() {
  Serial.begin(9600);
  sensors.begin();
  lcd.begin(20, 4);
  pinMode(DoSensorPin,INPUT);
  readDoCharacteristicValues();
  Serial.println("LoRa Sender");
  LoRa.setPins(csPin);
  if (!LoRa.begin(433E6)) { //868200000
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSyncWord(0x34); 
  
}

void loop() {
  
  //Serial.println(doorStatus);
  
  readTempSensor();
  getNTUValue();
  PH();
  Dissolved_Oxygen();
  Serial.println("");
 
  float difference = float(float((millis() - previousMillis)/1000)/60); // Calculate Time Difference
 if(difference > dataSendingInterval)
  {
   sendLastValue();
  }

  lcd.setCursor(0, 0);
  lcd.print("Temp: "); lcd.print(temperature);
  lcd.setCursor(0, 1);
  lcd.print("Turb: "); lcd.print(ntu);
  lcd.setCursor(0, 2);
  lcd.print("PH  : "); lcd.print(ph);
  lcd.setCursor(0, 3);
  lcd.print("DO  : "); lcd.print(doValue);

  
  Serial.print("Sec:");Serial.println(difference);
  delay(2000);

}



void sendLastValue() // This Function is Used to Send Device Data
{
  Serial.print("Sending packet: ");
 // compose and send packet
  LoRa.beginPacket();
  LoRa.print("<");
  LoRa.print(device_id);
  LoRa.print(">field1=");
  LoRa.print(temperature);
  LoRa.print("&field2=");
  LoRa.print(ntu); 
  LoRa.print("&field3=");
  LoRa.print(0);
  LoRa.print("&field4=");
  LoRa.print(ph);
  LoRa.print("&field5=");
  LoRa.print(doValue);
  LoRa.endPacket();
  previousMillis = millis();
 
}



void readTempSensor() // Function to Check Temperature + Humidity Value
{
  //memset(temperatureValue, '\0', sizeof(temperatureValue));
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0);
  if (isnan(temperature))
  {
    temperature = 0;
  }
  Serial.print("Temp:");
  Serial.println(temperature);
 // dtostrf(temperature,4,2,temperatureValue);
}

void getNTUValue()
{
  float volt = 0;
  ntu = 0;
  volt = 0;
  for(int i=0; i<800; i++)
  {
      volt += ((float)analogRead(turibidity_sensorPin)/1023)*5;
  }
  volt = volt/800;
  volt = round_to_dp(volt,1);
  if(volt < 1.5){
    ntu = 3000;
  }else{
    ntu = -1120.4*(volt*volt)+5742.3*volt-4353.8; 
  }
  Serial.print("NTU:");
  Serial.println(ntu);
 // dtostrf(ntu,4,2,ntuValue);
}

float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}


void PH()
{
                   //buffer for read analog
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PH_sensorPin);//Connect the PH Sensor to A0 port
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        int temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
       avgValue=0;
 
      for(int i=2;i<8;i++)                      //take the average value of 6 center sample
      avgValue+=buf[i];
    
     sensorValue =   avgValue/6;

     ph = 7-1000*(sensorValue-365)*Vref/59.16/1023,2;

    Serial.print("PH value: ");
    Serial.println(ph);
    
   


}





void Dissolved_Oxygen()
{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }



   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
      
      doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
      Serial.print(F("DO Value:"));
      Serial.print(doValue,2);
      Serial.println(F("mg/L"));
   }

   if(serialDataAvailable() > 0)
   {
      byte modeIndex = uartParse();  //parse the uart command received
      doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
   }

}

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 )
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
    receivedBufferIndex = 0;
    strupr(receivedBuffer);
    return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL)
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL)
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;

      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;

     case 2:
      if(enterCalibrationFlag)
      {
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperature2);
         SaturationDoVoltage = averageVoltage;
         SaturationDoTemperature = temperature2;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)
               Serial.print(F(">>>Calibration Successful"));
            else
              Serial.print(F(">>>Calibration Failed"));
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }
}
