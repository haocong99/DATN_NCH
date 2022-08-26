#include <ArduinoJson.h>
#include <DueFlashStorage.h>
#define butUp 8
#define butDown 9
#define butSelect 10
DueFlashStorage dueFlashStorage;
float tc;
float td;
float Tn = 0;
float T1 = 0, T5 = 0;
float irms1, irms2, irms3, irms4, irms5;
//int Idm1, Idm2, Idm3, Idm4, Idm5;
uint8_t Idm_array[5];
uint8_t Idm_array_last[5] = {0, 0, 0, 0, 0};
unsigned long updateTime = 0;
int count = 0;
double C1, C2, C3, C4, C5 = 0;
uint32_t t = 0;
float M;
float tI = 0;
float dem ;
float dem1 ;
int last_count = 0;
float adcValue1, adcValue2, adcValue3, adcValue4, adcValue5;
float acValueSum1 , acValueSum2 , acValueSum3, acValueSum4 , acValueSum5 ;
unsigned long time1;
struct IdmConfig
{
  uint8_t Idm1;
  uint8_t Idm2;
  uint8_t Idm3;
  uint8_t Idm4;
  uint8_t Idm5;
};
IdmConfig idmconfig;
int readEEPROM()
{
  byte* idm_read = dueFlashStorage.readAddress(4);
  IdmConfig Idmconfig_read;
  memcpy(&Idmconfig_read, idm_read, sizeof(IdmConfig));
  idmconfig.Idm1 = Idmconfig_read.Idm1;
  idmconfig.Idm2 = Idmconfig_read.Idm2;
  idmconfig.Idm3 = Idmconfig_read.Idm3;
  idmconfig.Idm4 = Idmconfig_read.Idm4;
  idmconfig.Idm5 = Idmconfig_read.Idm5;
}
void updateEEPROM()
{
  Idm_array[0] = idmconfig.Idm1;
  Idm_array[1] = idmconfig.Idm2;
  Idm_array[2] = idmconfig.Idm3;
  Idm_array[3] = idmconfig.Idm4;
  Idm_array[4] = idmconfig.Idm5;
  for (int i = 0; i < 5; i++)
  {
    if (Idm_array[i] == Idm_array_last[i])
    {
    }
    else
    {
      if (millis() - updateTime >= 30000)
      {
        updateTime = millis();
        byte idm_update[sizeof(IdmConfig)];
        memcpy(idm_update, &idmconfig, sizeof(IdmConfig));
        dueFlashStorage.write(4, idm_update, sizeof(IdmConfig));
        Idm_array_last[0] = idmconfig.Idm1;
        Idm_array_last[1] = idmconfig.Idm2;
        Idm_array_last[2] = idmconfig.Idm3;
        Idm_array_last[3] = idmconfig.Idm4;
        Idm_array_last[4] = idmconfig.Idm5;
      }
    }
  }
}
void interrupt() {
  dem = millis();
  Serial.println(dem);
}
void sendJSON()
{
  StaticJsonDocument<500> doc;
  doc["count"] = count;
  JsonArray data = doc.createNestedArray("data");
  data.add(idmconfig.Idm1);
  data.add(idmconfig.Idm2);
  data.add(idmconfig.Idm3);
  data.add(idmconfig.Idm4);
  data.add(idmconfig.Idm5);
  serializeJson(doc, Serial3);
  Serial3.println();
  Serial.println("Send JSON Completed!");
}
void FlashCheck()
{
  uint8_t codeRunningForTheFirstTime = dueFlashStorage.read(0);
  Serial.print("codeRunningForTheFirstTime: ");
  if (codeRunningForTheFirstTime == 255) {
    Serial.println("yes");
    idmconfig.Idm1 = 10;
    idmconfig.Idm2 = 10;
    idmconfig.Idm3 = 10;
    idmconfig.Idm4 = 10;
    idmconfig.Idm5 = 10;
    byte idm_stat[sizeof(IdmConfig)];
    memcpy(idm_stat, &idmconfig, sizeof(IdmConfig));
    dueFlashStorage.write(4, idm_stat, sizeof(IdmConfig));
    dueFlashStorage.write(0, 0);
  }
  else
  {
    Serial.println("no");
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  FlashCheck();
  readEEPROM();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(12, OUTPUT);
  dem = 0;
  dem1 = 0;
  td = 0;
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(12, LOW);
  tc = 0;
  pinMode(butUp, INPUT);
  pinMode(butDown, INPUT);
  pinMode(butSelect, INPUT);
  attachInterrupt(butSelect, Select, RISING);
  attachInterrupt(butUp, IdmUp, RISING);
  attachInterrupt(butDown, IdmDown, RISING);
}
float getIrms()
{
  acValueSum1 = 0; acValueSum2 = 0; acValueSum3 = 0; acValueSum4 = 0; acValueSum5 = 0;
  for (int i = 0 ; i < 700; i++)
  {
    float f =  analogRead(A5) ;
    adcValue1 =   float( analogRead(A2) - f) * 0.32258064516;//(((float( a - f) / 1023) * 3.3) / 10) * 1000;
    acValueSum1 =  acValueSum1 + adcValue1 * adcValue1;

    adcValue2 =  float( analogRead(A3) - f) * 0.32258064516;//(((float(b - f) / 1023) * 3.3) / 10) * 1000;
    acValueSum2 =  acValueSum2 + adcValue2 * adcValue2;

    adcValue3 = float( analogRead(A0) - f) * 0.32258064516;//(((float(c - f) / 1023) * 3.3) / 10) * 1000;
    acValueSum3 =  acValueSum3 + adcValue3 * adcValue3;

    adcValue4 = float( analogRead(A1) - f) * 0.32258064516;//(((float(d - f) / 1023) * 3.3) / 10) * 1000;
    acValueSum4 =  acValueSum4 + adcValue4 * adcValue4;

    adcValue5 =  float( analogRead(A4) - f) * 0.32258064516;//(((float(e - f) / 1023) * 3.3) / 10) * 1000;
    acValueSum5 =  acValueSum5 + adcValue5 * adcValue5;
  }
  irms1 = sqrt(acValueSum1 / 700);
  irms2 = sqrt(acValueSum2 / 700);
  irms3 = sqrt(acValueSum3 / 700);
  irms4 = sqrt(acValueSum4 / 700);
  irms5 = sqrt(acValueSum5 / 700);

  if (irms1 < 0.5) {
    irms1 = 0;
  }
  if (irms2 < 0.5) {
    irms2 = 0;
  }
  if (irms3 < 0.5) {
    irms3 = 0;
  }
  if (irms4 < 0.5) {
    irms4 = 0;
  }
  if (irms5 < 0.5) {
    irms5 = 0;
  }
//  Serial.println(String("dòng điện định mức 1 irms1 = ") + irms1);
//  Serial.println(String("dòng điện định mức 2 irms1 = ") + irms2);
//  Serial.println(String("dòng điện định mức 3 irms1 = ") + irms3);
//  Serial.println(String("dòng điện định mức 4 irms1 = ") + irms4);
//  Serial.println(String("dòng điện định mức 5 irms1 = ") + irms5);

  //td = millis();
  //Serial.println(td);
  return irms1, irms2, irms3, irms4, irms5;
}
float gettI(float I, float Ip)
{
  M = I / Ip;
  if (M > 1 )
  {
    tI = (28.2 / (M * M - 1)) + 0.1217;
  }
  if (M < 1 )
  {
    tI = 29.1 / (M * M - 1);
  }
  return tI;
}
void Select()
{
  if (digitalRead(butSelect) == 1)
  {
    count = count + 1;
    if (count > 5)
    {
      count = 1;
    }
    else
    {
      count = count;
    }
  }
  while (!digitalRead(butSelect));
  switch (count)
  {
    case 1:  Serial.println("Chọn dòng điện định mức Relay 1"); break;
    case 2:  Serial.println("Chọn dòng điện định mức Relay 2"); break;
    case 3:  Serial.println("Chọn dòng điện định mức Relay 3"); break;
    case 4:  Serial.println("Chọn dòng điện định mức Relay 4"); break;
    case 5:  Serial.println("Chọn dòng điện định mức Relay 5"); break;
  }
}
void IdmUp()
{
  if (digitalRead(butUp) == 1)
  {
    switch (count)
    {
      case 1: idmconfig.Idm1 = idmconfig.Idm1 + 1; break;
      case 2: idmconfig.Idm2 = idmconfig.Idm2 + 1; break;
      case 3: idmconfig.Idm3 = idmconfig.Idm3 + 1; break;
      case 4: idmconfig.Idm4 = idmconfig.Idm4 + 1; break;
      case 5: idmconfig.Idm5 = idmconfig.Idm5 + 1; break;
    }
  }
  while (digitalRead(butUp) == 1);
}
void IdmDown()
{
  if (digitalRead(butDown) == 1)
  {
    switch (count)
    {
      case 1: idmconfig.Idm1 = idmconfig.Idm1 - 1; break;
      case 2: idmconfig.Idm2 = idmconfig.Idm2 - 1; break;
      case 3: idmconfig.Idm3 = idmconfig.Idm3 - 1; break;
      case 4: idmconfig.Idm4 = idmconfig.Idm4 - 1; break;
      case 5: idmconfig.Idm5 = idmconfig.Idm5 - 1; break;
    }
  }
  while (digitalRead(butDown) == 1);
}
void loop() {
  updateEEPROM();
  t = millis();
  getIrms();
  
  if (( irms1 > idmconfig.Idm1))
  {
  digitalWrite(12,HIGH);
  }
  C1 = C1 + (tc / gettI(irms1, idmconfig.Idm1));
  if (C1 >= 1) {
    digitalWrite(6, HIGH);
    Tn=0;
  }
  if (C1 < 0) {
      C1 = 0;
      digitalWrite(6, LOW);
      //T1=millis()-Tn;
      //Serial.println(String("T1:")+T1);
      Tn=0;
    }

  if ((irms2 > idmconfig.Idm2))
  { 
    //digitalWrite(12,HIGH);
    C2 = C2 + (tc / gettI(irms2, idmconfig.Idm2));
    if (C2 >= 1)
    {
      digitalWrite(5, HIGH);
    }
    else {
      if (C2 < 0) {
        C2 = 0;
        digitalWrite(5, LOW);
      }
    }
  }
  if (irms3 > idmconfig.Idm3)
  {
    C3 = C3 + (tc / gettI(irms3, idmconfig.Idm3));
    if (C3 >= 1) {
      digitalWrite(4, HIGH);
    }
    else {
      if (C3 < 0) {
        C3 = 0;
        digitalWrite(4, LOW);
      }
    }
  }
  if (irms4 > idmconfig.Idm4)
  {
    C4 = C4 + (tc / gettI(irms4, idmconfig.Idm4));
    if (C4 >= 1) {
      digitalWrite(3, HIGH);
     // dem1 = millis();
    }
    else {
      if (C4 < 0) {
        C4 = 0;
        digitalWrite(3, LOW);
      }
    }
  }
  if (irms5 > idmconfig.Idm5)
  {
    C5 = C5 + (tc / gettI(irms5, idmconfig.Idm5));
    if (C5 >= 1) {
      digitalWrite(2, HIGH);
    }
    else {
      if (C5 < 0) {
        C5 = 0;
        digitalWrite(2, LOW);
      }
    }
  }
//  Serial.println(String("C1:")+C1);
//  Serial.println(String("C2:")+C2);
//  Serial.println(String("C3:")+C3);
//  Serial.println(String("C4:")+C4);
//  Serial.println(String("C5:")+C5);
   tc = (millis() - t) / 1000.0;
  if (millis() - time1 >= 300)
  {
    sendJSON();
    time1 = millis();
   // Serial.println(String(idmconfig.Idm1) + "--" + String(idmconfig.Idm2) + "--" + String(idmconfig.Idm3) + "--" + String(idmconfig.Idm4) + "--" + String(idmconfig.Idm5));
  }
}
