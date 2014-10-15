
//#define USE_SERIAL 1
#include <mqttsn-messages.h>
#include <Mote.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "JeeLib.h" 

//ISR(WDT_vect) { Sleepy::watchdogEvent();}

MQTTSN mqttsn;
Mote mote;
MilliTimer sendTimer;
int slow;  
uint8_t connectFlagz;
uint8_t publishFlagz;
uint16_t keepAliveTimer;
char* clientid;
boolean mqttInitDoneSent;
//LED
int ledPin;
SoftwareSerial DbgSer(5, 6); // RX, TX
//one wire temp
#define ONE_WIRE_BUS 4                                                  // Data wire is plugged into port 2 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);                                          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);                                    // Pass our oneWire reference to Dallas Temperature.
//DeviceAddress address_T1 = { 0x28, 0xFC, 0x72, 0xD9, 0x04, 0x00, 0x00, 0xF3 };    //blue
DeviceAddress address_T1 = { 0x28, 0xC8, 0x8B, 0xD9, 0x04, 0x00, 0x00, 0x97 };  //red
uint8_t u8Counter;
uint16_t u16TopicPubID;
uint16_t u16TopicSubID;
bool subscribed;
bool registered;
//use the id as the postfix
//input values-> ai and bi
char* topicPub1;
char* topicPub2;
char* topicPub51;
char* topicPub52;
//output values -> bo
char* topicSub101;
char* topicSub151;
char* topicSub102;
char* topicSub152;

void setup() {  
  // put your setup code here, to run once:
  //serial
  DbgSer.begin(115200);
  Serial.begin(115200);
  mqttInitDoneSent = false;
  //LED Output config
  ledPin = 9;//13;      // default arduino uno LED
  pinMode(ledPin, OUTPUT); 
  digitalWrite(ledPin, HIGH); delay(500); digitalWrite(ledPin, LOW); 
  delay(200);
  digitalWrite(ledPin, HIGH); delay(500); digitalWrite(ledPin, LOW); 
  delay(500); digitalWrite(ledPin, HIGH); 
  //rf12 setup
  DbgSer.println(F("Initialising RF12...")); 
  rf12_initialize(11, RF12_433MHZ, 210);
  //rf12_sleep(RF12_SLEEP);
  DbgSer.println(F("RF12 init done")); 
  //Misc config
  slow=500;
  //Mqttsn Config
  setupMqttsnConfigs();
  //Mote Cofig
  setupMoteConfigs();
  digitalWrite(ledPin, HIGH); 
  DbgSer.println(F("setup methods done")); 
  mote.init();
  DbgSer.println(F("Mote init done")); 
  //init callbacks
  mqttsn.setCallback(mqttsnPubHandlerCallback);
  mote.setCallback(moteIoChangeCallback);
  mote.setCallbackAnalog(moteAnalogCallback);
  DbgSer.println(F("callbacks done")); 
  //watchdog
  wdt_enable(WDTO_8S);  //Enable watchdog: max 8 sec
  DbgSer.println(F("watchdog enable done")); 
  //connect to gateway
  mqttsn.connect_(connectFlagz,keepAliveTimer, clientid); 
  //delay(slow);//Sleepy::loseSomeTime(slow);
  mqttsn.poll();
  DbgSer.println(F("first mqttsn connect done")); 
 //registerTopics
 registerTopics(); 
 //subscribe
 subscribe(); 
  DbgSer.println(F("Setup done")); 
  //digitalWrite(ledPin, HIGH); Sleepy::loseSomeTime(20); digitalWrite(ledPin, LOW); 
  //Sleepy::loseSomeTime(50);
  //digitalWrite(ledPin, HIGH); Sleepy::loseSomeTime(20); digitalWrite(ledPin, LOW); 
  mqttsn.poll();
  if (registered & subscribed)  digitalWrite(ledPin, LOW); 
}

void loop() {
  //reset watchdog
  wdt_reset();
  //connect
  if(!mqttsn.connected() && sendTimer.poll(1000)){
    DbgSer.println(F("Setup didnt connect. Try now...")); 
    mqttsn.connect_(connectFlagz,keepAliveTimer, clientid); 
    //delay(10000);//Sleepy::loseSomeTime(10000);
    //delay(10000);
  }
 
 //registerTopics
 registerTopics(); 
 //subscribe
 subscribe(); 

  //Sleepy::loseSomeTime(slow);
  //poll mqttsn for new data
  mqttsn.poll();
  //scan io for changes
  mote.scanIo();
  
  if (registered && subscribed && !mqttInitDoneSent){
     digitalWrite(ledPin, LOW);
     mqttInitDoneSent = true;
  } 
}

void subscribe(){
  if( !subscribed && mqttsn.connected() && sendTimer.poll(500)){
    mqttsn.subscribe_(topicSub101,0); 
    mqttsn.subscribe_(topicSub151,0); 
    mqttsn.subscribe_(topicSub102,0); 
    mqttsn.subscribe_(topicSub152,0); 
    subscribed = mqttsn.subscribed();
    //if(subscribed){
    //  mqttsn.subscribe_(topicSub151,0); 
    //  subscribed = mqttsn.subscribed();
    //}
  }
  
 //Sleepy::loseSomeTime(1000); 
}

void registerTopics(){
  if(!registered && mqttsn.connected() && sendTimer.poll(700)){
    mqttsn.registertopic_(topicPub1); 
    mqttsn.registertopic_(topicPub2);
    mqttsn.registertopic_(topicPub51);
    mqttsn.registertopic_(topicPub52);
    registered = mqttsn.registered();
  }
}

void setupMqttsnConfigs(){
  connectFlagz = 0x24;  //24 for clean session   FLAG_QOS_0;
  keepAliveTimer = 5;  //keep alive timer is 5 sec
  clientid= "mote11";
  publishFlagz = 0x00;
  subscribed=0;
  //topic names to match io
  //input values-> ai and bi
  topicPub1= "11/1/@I/1";
  topicPub2= "11/1/@I/2";
  topicPub51= "11/1/@A/51";
  topicPub52= "11/1/@A/52";
  //output values -> bo
  topicSub101 ="1/11/$L/101";
  topicSub151="1/11/$P/151";
  topicSub102 ="1/11/$L/102";
  topicSub152="1/11/$P/152";
}


void setupMoteConfigs(){  
  //AL
  mote.al_srcNode = 11;
  mote.al_destNode = 1;
  mote.al_retries=3;
  mote.al_confirmTimeout=5000;
  //Scanner
  mote.scanner_period=0.5;
  
  //IO
  //Binary Input at pin11
  //args: id, pinid, objType, debounce
  ////mote.addBi(1,7,"@I",100);
  //Binary Input @ pin12
  //args: id, pinid, objType, debounce
  ////mote.addBi(2,8,"@I",100);
  //Analog Input
  //Args: id, pinid, delta
  ////mote.addAi(51,5,100);
  //negative pinid are software analogs like onewire temperature in this case 
  mote.addAi(52,-1,2);
  //binary Output Pulse
  //args: id, pinid
  mote.addBoL(101,9);
  mote.addBoL(102,7);
  //Binary Output Pulse
  //args: id, pinid
  mote.addBoP(151,9);
  mote.addBoP(152,7);
}


int moteAnalogCallback(uint8_t id){
   sensors.requestTemperatures();                                        // Send the command to get temperatures
   int val =  sensors.getTempC(address_T1)*10;
   //DbgSer.print(F("Sketch:moteAnalogCallback: temperature = "));
   //DbgSer.println(val);
   return val;
}

void moteIoChangeCallback(char* topic, char* payload, int payload_length){
   DbgSer.print(F("Sketch:ioChangeCallback: topic="));
   DbgSer.println(topic);
   DbgSer.print(F("Sketch:ioChangeCallback: payload="));
   DbgSer.println(payload);
   mqttsn.publish_(topic,payload, payload_length,publishFlagz);
}
void mqttsnPubHandlerCallback(const char* topic,const char* payload){
    DbgSer.print(F("Sketch:mqttsnPubHandlerCallback:Topic= "));
    DbgSer.println(topic);
    DbgSer.print(F("Sketch:mqttsnPubHandlerCallback:Payload= "));
    DbgSer.println((String(payload)).toInt());
    mote.execute(topic, payload);
}

