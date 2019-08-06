#include <avr/pgmspace.h>
#include <EEPROM.h>
#define DoSensorPin  A1    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5000    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
float doValue;
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define ReceivedBufferLength 100
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

#include <SoftwareSerial.h> // This Library is Used to Communicate with GSM
#define gsmPin 5 // GSM Power is Connected to Pin 5
SoftwareSerial mySerial(8,9);
byte gsmONOFF = 0; // Check the Power Status of GSM
byte gsmStatus = 0; // Check the Network Status of GSM
uint8_t answer = 0;
String smsSIMNumber; 
int smsNumber;
char devicePassword[7];
#define PASSWORDLOCATION 100
#define SMSBUFFERSIZE 100
#define MAXRESPONSEBUFFER 50
char aux_str[MAXRESPONSEBUFFER];
byte iCounter = 0;
void setup() {
  Serial.begin(9600);
  pinMode(DoSensorPin,INPUT);
  pinMode(gsmPin, OUTPUT);
  readDoCharacteristicValues();

}

void loop() {
  // put your main code here, to run repeatedly:

}

void restartGSM() // Function to Shutdown the GSM
{
  digitalWrite(gsmPin, LOW);
  gsmONOFF = 0;
  gsmStatus = 0;
  delay(1000);
}

void turnOnGSM() // Function to Shutdown the GSM
{
  digitalWrite(gsmPin, HIGH);
  delay(15000);
  gsmONOFF = 1;
}
void checkSerialPortSMS()
{
  char receivedSMS[SMSBUFFERSIZE];
  char message[100];
  char receivedChar;
  if (gsmONOFF == 0)
  {
    turnOnGSM();
  }
  answer = sendATcommand("AT+CMGF=1", "OK", 1000);
  if (answer == 1)
  {
    answer = sendATcommand("AT+CMGL=\"ALL\"", "AT+CMGL=\"ALL\"", 15000);
    unsigned long previous = millis();
    if (answer == 1)
    {
      memset(receivedSMS,'\0',sizeof(receivedSMS));
      answer = 0;
      byte smsCharCount = 0;
      while(mySerial.available() == 0);
      do
      {
        if(mySerial.available() > 0)
        {
          receivedSMS[smsCharCount] = char(mySerial.read());
          if(receivedSMS[smsCharCount] == '\n')
          {
            char *smsRestPart;
            if (strstr(receivedSMS,"+CMGL:") != NULL)
            {
              findNecessaryCommads(receivedSMS, 0);          
            }
            else if(strstr(receivedSMS,"WHM") != NULL)
            {
              char *command;
              char *password;
              strtok_r(receivedSMS,",",&smsRestPart);
              command = strtok_r(smsRestPart,",",&smsRestPart);
              password = strtok_r(smsRestPart,",",&smsRestPart);
              strupr(command);
              if (strcmp(password,devicePassword) == 0)
              {
                if (strcmp(command,"STATUS") == 0)
                {
                  findNecessaryCommads(smsRestPart, 1);
                }
                else if (strcmp(command,"CALIBRATION") == 0)
                {
                  findNecessaryCommads(smsRestPart, 2);
                }
                else if (strcmp(command,"EXIT") == 0)
                {
                  findNecessaryCommads(smsRestPart, 3);
                }
                else if (strcmp(command,"SATCAL") == 0)
                {
                  findNecessaryCommads(smsRestPart, 4);
                } 
              }
            }
          }
          else if (smsCharCount > SMSBUFFERSIZE)
          {
            smsCharCount = -1;
            memset(receivedSMS,'\0',sizeof(receivedSMS));
          }
          else if (strcmp(receivedSMS, "OK") == 0 )
          {
            answer = 1;
            deleteSMS(smsNumber);
          }
          else if (strcmp(receivedSMS, "ERROR") == 0 )
          {
            answer = 1;
          }
          smsCharCount++;
        }      
      }while((answer == 0) && ((millis() - previous) < 10000));
    }
  }
}

void deleteSMS(byte smsNo) // Function to Delete Read SMSs
{
  if (smsNo > 0)
  {
    snprintf(aux_str,sizeof(aux_str),"AT+CMGD=%d",smsNo);
    sendATcommand(aux_str,"OK", 2000);
  }
}

byte findNecessaryCommads(char* SMS, byte intCommand)
{
  char *smsRestPart;
  static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
  float voltageValueStore;
  switch(intCommand)
  {
    case 0:
    strtok_r(SMS, ", ",&smsRestPart);
    smsNumber = atoi(strtok_r(smsRestPart, ", ",&smsRestPart));
    strtok_r(smsRestPart, ",",&smsRestPart);
    smsSIMNumber = strtok_r(smsRestPart, ", ",&smsRestPart);
    break;

    case 1:

    case 2:
    enterCalibrationFlag = 1;
    doCalibrationFinishFlag = 0;
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
        
    case 4:
    if(enterCalibrationFlag)
    {
      EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
      SaturationDoVoltage = averageVoltage;
      doCalibrationFinishFlag = 1;
    }
    break; 
  }
}

void getDevicePassword() // Function to Get Stored Password
{
  char getPassword[6];
  char ch;
  memset(getPassword, '\0', sizeof(getPassword));
  memset(devicePassword, '\0', sizeof(devicePassword));
  for (iCounter = PASSWORDLOCATION; iCounter < EEPROM.length(); iCounter++)
  {
    //Serial.println(i);
    if(EEPROM.read(iCounter) == 0xFF || EEPROM.read(iCounter) == 0)
    {
      break;
    }
    ch = EEPROM.read(iCounter);
    getPassword[iCounter-PASSWORDLOCATION] = ch; 
  }
  if(strlen(getPassword) > 0)
  {
    strcpy(devicePassword,getPassword);
  }
  else
  {
    strcpy(devicePassword,"123456");
  }
}
void readDOValue()
{
  analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);
  analogBufferIndex++;
  if(analogBufferIndex == SCOUNT)
  {
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    {
      analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    }
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
    doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
    analogBufferIndex = 0;   
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
  if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
  else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
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

int8_t sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout) // Function to Communicate with GSM
{
  uint8_t x=0;
  char response[MAXRESPONSEBUFFER];
  unsigned long previous;
  answer = 0;
  memset(response,'\0',sizeof(response));// Initialize the string
  delay(100);
//  Serial.println(ATcommand);
  while( mySerial.available() > 0) mySerial.read();
  mySerial.println(ATcommand);    // Send the AT command 
  x = 0;
  previous = millis();
  do{
    if(mySerial.available() != 0)
    {    
      response[x] = mySerial.read();
      x++;
      if (strstr(response, expected_answer1) != NULL)    
      {
        answer = 1;
      }
      else if(x < MAXRESPONSEBUFFER)
      {
        x = 0;
        memset(response,'\0',sizeof(response));
      }
    }
  }
  while((answer == 0) && ((millis() - previous) < timeout));
  Serial.println(response);
  return answer;
}
