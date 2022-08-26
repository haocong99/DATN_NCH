#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
//#include <DS18B20.h>
#include <ModbusMaster.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#define TFT_GREY 0x5AEB
#define MSG_BUFFER_SIZE  (50)
TaskHandle_t Task1;
TaskHandle_t Task2;
#define MSG_BUFFER_SIZE  (50)
#define MAX485_DE      3
#define MAX485_RE_NEG  2
static char recv_buff[50];
int Idm[10];
int count;
const int RX_3_Due = 5;
unsigned long timeCount;
//const byte ds18b20Pin = 33;
const byte firealarmPin = 15;
int OCstat_last[5] = {1, 1, 1, 1, 1};
int SCstat_last[5] = {1, 1, 1, 1, 1};
int RLstat_last[5] = {1, 1, 1, 1, 1};
int arrayIrmsLast[5] = {0, 0, 0, 0, 0};
int OCstat_current[5];
int SCstat_short[5];
int fireLast = 1;
int tempLast = 0;
int x=0;
const int ledWifi = 26;
const char* mqtt_server = "ems.ioteamvn.com";
String preFixTopic = "NCKH/NumbericRelay/wifi/";
unsigned long lastMsg = 0;
unsigned long lastSendTemp = 0;
unsigned long timeFire = 0;
bool interruptFire = false;
float temperature;
char msg[MSG_BUFFER_SIZE];
String msg_overCurrentString;
String msg_shortCircuitString;
int booloverCurrent[5];
int boolshortCircuit[5];
int fire = 0;
uint8_t relayPin[6] = {0x0000,0x0001,0x0002,0x0003,0x0004};
const byte overCurrentPin[5] = {18, 19, 21, 22, 23};
const byte shortCircuitPin[5] = {32, 35, 34, 39, 36};
bool shortCircuitStatePin[5];
unsigned long dem;
bool state[6];
int this_relay;
int sc = 0;
int oc = 0;
int Idm1, Idm2, Idm3, Idm4, Idm5;
//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int a;
ModbusMaster node;
SoftwareSerial irmsSerial(RX_3_Due, -1);
WiFiClient espClient;
PubSubClient wifiClient(espClient);
//DS18B20 ds(ds18b20Pin);
TFT_eSPI tft = TFT_eSPI();
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
} 
void(* resetFunc) (void) = 0;
void setupWifi() {
  WiFiManager wm;
  WiFi.mode(WIFI_STA);
  bool res = wm.autoConnect("Numberic Relay Conf", "66668888");
  if (!res) {
    Serial.print("Connecting Failed");
  }
  else {
    Serial.println("Connecting Completed");
    digitalWrite(ledWifi, LOW);
  }
}
void reconnect() {
  while (!wifiClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    Serial.println(clientId);
    if (wifiClient.connect(clientId.c_str())) {
      Serial.println("MQTT Connected");
      char subTopic[100];

      String subTopicString = preFixTopic + "cmnd/+";
      subTopicString.toCharArray(subTopic, subTopicString.length() + 1);
      wifiClient.subscribe(subTopic);
      Serial.println(subTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(wifiClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String payloadString = "";
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + (char)payload[i];
    Serial.print((char)payload[i]);
    if (payload[0] == 'r')
    {
      resetFunc();
    }
  }
  String topicBuffer = topic;
  int splashIndex[10];
  int j = 0;
  for (int i = 0; i < topicBuffer.length(); i++) {
    if (topicBuffer.substring(i, i + 1) == "/") {
      splashIndex[j] = i;
      j++;
    }
  }
  int rl = topicBuffer.substring(splashIndex[3] + 1).toInt();
  Serial.println("   relay: " + String(rl));
  if (rl > 0 && rl < 6)
  {
    if ((char)payload[0] == '1') {
      //digitalWrite(relayPin[rl - 1], HIGH);
      state[rl - 1]=true;
      node.writeSingleCoil(relayPin[rl - 1], state[rl - 1]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave     
      //Serial.println(String("state[")+(rl-1)+"]:"+state[rl-1]);
      
    } else {
      //digitalWrite(relayPin[rl - 1], LOW);
        state[rl - 1]=false;
        node.writeSingleCoil(relayPin[rl - 1], state[rl - 1]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave
        //Serial.println(String("state[")+(rl-1)+"]:"+state[rl-1]);
    }
    
  }
}
//void sendTemp()
//{
//  char pubTopicTemp[100];
//  char msg_temp[50];
//
//  if (millis() - lastSendTemp > 10000)
//  {
//    lastSendTemp = millis();
//    temperature = ds.getTempC();
//  }
//  if (temperature != tempLast)
//  {
//    String msgTempString = String(temperature);
//    Serial.print("Nhiet do: ");
//    Serial.println(temperature);
//    String pubTopicTempString = preFixTopic + "stat/temperature";
//    pubTopicTempString.toCharArray(pubTopicTemp, pubTopicTempString.length() + 1);
//    msgTempString.toCharArray(msg_temp, msgTempString.length() + 1);
//    wifiClient.publish(pubTopicTemp, msg_temp);
//    tempLast = temperature;
//    displayTemp();
//  }
//}
void IRAM_ATTR OCinterrupt()
{
  //portENTER_CRITICAL_ISR(&mux);
  for (int i = 0; i < 5; i++)
  {
    if ((digitalRead(overCurrentPin[i]) == 1))
    {
     // digitalWrite(relayPin[i], HIGH);
       state[i]=true;
       node.writeSingleCoil(relayPin[i], state[i]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave        
    }
  }
  //portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR SCinterrupt()
{
//  portENTER_CRITICAL_ISR(&mux);
  for (int i = 0; i < 5; i++)
  {
    if (digitalRead(shortCircuitPin[i]) == 1)
    {
//      digitalWrite(relayPin[i], HIGH);
  state[i]=true;
  node.writeSingleCoil(relayPin[i], state[i]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave
  shortCircuitStatePin[i]=true;
    }   
  }
 // portEXIT_CRITICAL_ISR(&mux);
}
void sendWarning()
{
  char pubTopicoverCurrent[100];
  char pubTopicshortCircuit[100];
  char msgoverCurrent[50];
  char msgshortCircuit[50];
  String msgshortCircuitString;
  String msgoverCurrentString;
  for (int i = 0; i < 5; i++)
  {
    int stat = digitalRead(overCurrentPin[i]);
    
    OCstat_current[i] = ( stat == 1 && state[i]/*digitalRead(relayPin[i])*/ == 1) ? 1 : 0  ; // Thoa man thi 1 ko thoa man thi 0
    //        OCstat_current[i] = ( digitalRead(relayPin[i]) == 1) ? 1 : 0  ;
    //    //    Serial.print(OCstat[i]) ;
//  Serial.println(String("stat:")+stat);
//  Serial.println(String("state:")+state[i]);
//  Serial.println(String("OCstat_current:")+OCstat_current[i]);
  }

  for (int x = 0; x < 5; x++)
  {
    if (OCstat_current[x] == OCstat_last[x])
    {
    }
    else
    {
      int int_OCstat = OCstat_current[x];
      msgoverCurrentString = String(int_OCstat);
      String pubTopicoverCurrentString = preFixTopic + "stat/" + "OC" + String(x + 1);
      pubTopicoverCurrentString.toCharArray(pubTopicoverCurrent, pubTopicoverCurrentString.length() + 1);
      msgoverCurrentString.toCharArray(msgoverCurrent, msgoverCurrentString.length() + 1);
      wifiClient.publish(pubTopicoverCurrent, msgoverCurrent );
      OCstat_last[x] = int_OCstat;
    }
  }
  for (int i = 0; i < 5; i++)
  {
//    int stat_short = digitalRead(shortCircuitPin[i]);
//    SCstat_short[i] = (stat_short == 1 /*&& digitalRead(relayPin[i])*/state[i] == 1) ? 1 : 0;
//    Serial.println(String("stat_short:")+stat_short);
//   Serial.println(String("SCstat_short:")+SCstat_short[i]);
//   Serial.println(String("state:")+state[i]);
 int stat_short= (shortCircuitStatePin[i]==1) ? 1 : 0;
 SCstat_short[i]= stat_short;
 if(state[i]==0) shortCircuitStatePin[i]=0;
  }
  for (int x = 0; x < 5; x++)
  {
    //    Serial.println(String(SCstat_short[x])+"    "+String(SCstat_last[x]));
    if (SCstat_short[x] == SCstat_last[x])
    {
    }
    else
    {
      int int_SCstat = SCstat_short[x];
      msgshortCircuitString = String(int_SCstat);
      String pubTopicshortCircuitString = preFixTopic + "stat/" + "SC" + String(x + 1);
      pubTopicshortCircuitString.toCharArray(pubTopicshortCircuit, pubTopicshortCircuitString.length() + 1);
      msgshortCircuitString.toCharArray(msgshortCircuit, msgshortCircuitString.length() + 1);
      wifiClient.publish(pubTopicshortCircuit, msgshortCircuit);
      SCstat_last[x] = int_SCstat;
    }
  }
}
//void fireInterrupt()
//{
//  timeFire = millis();
//  interruptFire = true;
//}
void fireProcess()
{
  char pubTopicAlarm[100];
  char msg_alarm[50];
  String msgFireString;
 
  if (digitalRead(firealarmPin) == 0)
  {
    //timeFire = millis();
    x++;
  }
  else{}
  if (x==10)
    {
      fire = 1;
      for (int i = 0; i < 5; i++)
      {
//        digitalWrite(relayPin[i], HIGH);
  state[i]=false;
  node.writeSingleCoil(relayPin[i], state[i]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave
  //Serial.println("chasy");
      }
      x=0;
    }
  else
  {
    fire = 0;
  }
  if (fire == fireLast)
  {
  }
  else
  {
    msgFireString = String(fire);
    String pubTopicAlarmString = preFixTopic + "stat/FA";
    pubTopicAlarmString.toCharArray(pubTopicAlarm, pubTopicAlarmString.length() + 1);
    msgFireString.toCharArray(msg_alarm, msgFireString.length() + 1);
    wifiClient.publish(pubTopicAlarm, msg_alarm);
    fireLast = fire;
  }
}
void MQTTPublishRelay()
{
  char pubTopic[100];
  char msgRelay[50];
  int thisRelay;
  String msgrelayString;
  int RL_stat[5];
  for (int i = 0; i < 5; i++)
  {
    bool stat = /*digitalRead(relayPin[i])*/ state[i];
    RL_stat[i] = (stat == 1) ? 1 : 0;
    
  }
  for (int x = 0; x < 5; x++)
  {
    //    Serial.println(String(RL_stat[x]) + "    " + String(RLstat_last[x]));
    if (RL_stat[x] == RLstat_last[x])
    {
    }
    else
    {
      displayRelay();
      int int_RLStat = /*digitalRead(relayPin[x])*/ (state[x]== 1) ? 1 : 0;
      msgrelayString = String(int_RLStat);
      String pubTopicString = preFixTopic + "stat/" + String(x + 1);
      pubTopicString.toCharArray(pubTopic, pubTopicString.length() + 1);
      msgrelayString.toCharArray(msgRelay, msgrelayString.length() + 1);
      wifiClient.publish(pubTopic, msgRelay);
      RLstat_last[x] = int_RLStat;
    }
  }
}

//************************TFT CODE*****************************//
void drawBorder()
{
  tft.drawLine(0, 23, 319, 23, TFT_WHITE);
  tft.drawLine(0, 60, 319, 60, TFT_WHITE);
  tft.drawLine(0, 23, 0, 239, TFT_WHITE);
  tft.drawLine(0, 239, 319, 239, TFT_WHITE);
  tft.drawLine(319, 23, 319, 239, TFT_WHITE);
  tft.drawLine(0, 200, 320, 200, TFT_WHITE);
  //Duong doc
  tft.drawLine(80, 23, 80, 200, TFT_WHITE);
  tft.drawLine(140, 23, 140, 200, TFT_WHITE);
  tft.drawLine(200, 23, 200, 200, TFT_WHITE);
  tft.drawLine(260, 23, 260, 200, TFT_WHITE);
  tft.drawLine(160, 200, 160, 240, TFT_WHITE);
}
void displayRelay()
{
  int x = 72;
  tft.setCursor(27, 35);
  tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
  tft.println("Relay");
  tft.setCursor(92, 35);
  tft.println("Status");
  for (int i = 0; i < 5; i++)
  {
    int stat =(state[i] == 1) ? 1 : 0 /*digitalRead(relayPin[i])*/;
    if (stat == 0)
    {
      tft.setCursor(20, x);
      tft.setTextColor(TFT_GREEN); tft.setTextSize(1);
      tft.println("Relay " + String(i + 1) + " ");
      tft.fillRect(100, x, 35, 15, TFT_BLACK);
      tft.setCursor(105, x);
      tft.setTextColor(TFT_GREEN); tft.setTextSize(1);
      tft.println("ON");
      x = x + 25;
    }
    else
    {
      tft.setCursor(20, x);
      tft.setTextColor(TFT_RED); tft.setTextSize(1);
      tft.println("Relay " + String(i + 1) + " ");
      tft.fillRect(100, x, 35, 15, TFT_BLACK);
      tft.setCursor(105, x);
      tft.setTextColor(TFT_RED); tft.setTextSize(1);
      tft.println("OFF");
      x = x + 25;
    }
  }
}
void displayFireAlarm()
{
  tft.setCursor(175, 212);
  tft.setTextColor(TFT_GREEN);
  tft.print("Fire Warning");
  if (fire == 1)
  {
    tft.fillRect(260, 210, 45, 20, TFT_RED);
    tft.setCursor(175, 212);
    tft.setTextColor(TFT_RED);
    tft.print("Fire Warning");
  }
  else
  {
    tft.fillRect(260, 210, 45, 20, TFT_GREEN);
  }
}
void displayTemp()
{
  tft.setCursor(10, 212);
  tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
  tft.println("Temperature");
  tft.fillRect(93, 212, 42, 15, TFT_BLACK);
  tft.setCursor(97, 212);
  tft.setTextColor(TFT_YELLOW); tft.setTextFont(2);
  tft.setTextSize(1);
  tft.println(temperature);
  tft.fillCircle(137, 215, 2, TFT_YELLOW);
  tft.setCursor(143, 212);
  tft.println("C");
}
void displayIrms()
{
  int arrayIrms[5];
  arrayIrms[0] = Idm1;
  arrayIrms[1] = Idm2;
  arrayIrms[2] = Idm3;
  arrayIrms[3] = Idm4;
  arrayIrms[4] = Idm5;
  for (int i = 0; i < 5; i++)
  {
    if (arrayIrms[i] == arrayIrmsLast[i])
    {
    }
    else
    {
      tft.setCursor(150, 35);
      tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
      tft.println("Current");

      tft.fillRect(150, 72, 44, 15, TFT_BLACK);
      tft.setCursor(155, 72);
      tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.print(Idm1);
      tft.println(" A");

      tft.fillRect(150, 97, 44, 15, TFT_BLACK);
      tft.setCursor(155, 97);
      tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.print(Idm2);
      tft.println(" A");

      tft.fillRect(150, 122, 44, 15, TFT_BLACK);
      tft.setCursor(155, 122);
      tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.print(Idm3);
      tft.println(" A");

      tft.fillRect(150, 147, 44, 15, TFT_BLACK);
      tft.setCursor(155, 147);
      tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.print(Idm4);
      tft.println(" A");

      tft.fillRect(150, 172, 44, 15, TFT_BLACK);
      tft.setCursor(155, 172);
      tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
      tft.print(Idm5);
      tft.println(" A");
      char pubtopicIrms[100];
      char msgIrms[50];
      int thisIrms;
      String msgIrmsString;
      int intIrms = arrayIrms[i];
      msgIrmsString = String(intIrms);
      String pubTopicIrmsString = preFixTopic + "stat/" + "Irms" + String(i + 1);
      pubTopicIrmsString.toCharArray(pubtopicIrms, pubTopicIrmsString.length() + 1);
      msgIrmsString.toCharArray(msgIrms, msgIrmsString.length() + 1);
      wifiClient.publish(pubtopicIrms, msgIrms);
      arrayIrmsLast[i] = intIrms;
    }
  }
}
void displayoverCurrent()
{
  int x = 72;
  tft.setCursor(218, 26);
  tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
  tft.print("Over");
  tft.setCursor(210, 42);
  tft.println("Current");

  for (int i = 0; i < 5; i++)
  {
    if (OCstat_current[i] == 1)
    {
      tft.fillRect(210, x, 40, 15, TFT_RED);
      x = x + 25;
    }
    else
    {
      tft.fillRect(210, x, 40, 15, TFT_GREEN);
      x = x + 25;
    }
  }
}
void displayshortCircuit()
{
  int x = 72;
  tft.setCursor(275, 26);
  tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
  tft.print("Short");
  tft.setCursor(271, 42);
  tft.print("Circuit");
  for (int i = 0; i < 5; i++)
  {
    if (SCstat_short[i] == 1)
    {
      tft.fillRect(272, x, 40, 15, TFT_RED);
      x = x + 25;
    }
    else
    {
      tft.fillRect(272, x, 40, 15, TFT_GREEN);
      x = x + 25;
    }
  }
}
void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.begin(9600);
  irmsSerial.begin(9600);
  Serial2.begin(19200);
  // Modbus slave ID 1
  node.begin(1, Serial2);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
    for (int i = 0 ; i < 5; i++)
  {
//    pinMode(relayPin[i], OUTPUT);
//    digitalWrite(relayPin[i], LOW);
  state[i]=false;
  node.writeSingleCoil(relayPin[i], state[i]);// ghi giá trị bit state vào thanh ghi có địa chỉ 0x0001 trên slave
  shortCircuitStatePin[i]=false;
  }
  setupWifi();
  wifiClient.setServer(mqtt_server, 1883);
  wifiClient.setCallback(callback);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(30, 0, 2);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(1);
  tft.fillRect(0, 0, 320, 23, TFT_BLUE);
  tft.setCursor(130, 3);
  tft.println("**IoTeamVN**");
  drawBorder();
  displayTemp();
  displayRelay();
  displayIrms();
  for (int i = 0 ; i < 5; i++)
  {
//    pinMode(relayPin[i], OUTPUT);
//    digitalWrite(relayPin[i], LOW);
    pinMode(overCurrentPin[i], INPUT);
    pinMode(shortCircuitPin[i], INPUT);
    digitalWrite(shortCircuitPin[i], LOW);
    digitalWrite(overCurrentPin[i], LOW);
    attachInterrupt(shortCircuitPin[i], SCinterrupt, RISING);
    attachInterrupt(overCurrentPin[i], OCinterrupt, RISING);
  }
  pinMode(ledWifi, OUTPUT);
  pinMode(firealarmPin, INPUT_PULLUP);
}
void loop()
{
 
  if (irmsSerial.available())
  {
    String data = "";
    while (irmsSerial.available()) {

      data = irmsSerial.readStringUntil('\r');
    }
    DynamicJsonDocument  doc(4096);
    DeserializationError error = deserializeJson(doc, data);
    if (!error) {
      serializeJson(doc, Serial);
      //Read data
      count = doc["count"];
      Idm1 = doc["data"][0];
      Idm2 = doc["data"][1];
      Idm3 = doc["data"][2];
      Idm4 = doc["data"][3];
      Idm5 = doc["data"][4];
    }
  }
  displayIrms();
  displayFireAlarm();
  displayoverCurrent();
  displayshortCircuit();
  if (!wifiClient.connected())
  {
    reconnect();
  }
  wifiClient.loop();
  
  unsigned long now = millis();
  if (now - lastMsg >= 200) {
    lastMsg = now;
    MQTTPublishRelay(); 
    fireProcess();
    sendWarning(); 
  }
  
  //  sendTemp();
}
