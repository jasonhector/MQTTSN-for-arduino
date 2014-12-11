/*
mqttsn-messages.h

The MIT License (MIT)

Copyright (C) 2014 John Donovan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#define USE_LPL_RF69 1

#ifndef __MQTTSN_MESSAGES_H__
#define __MQTTSN_MESSAGES_H__
#include "mqttsn.h"
//#include "Arduino.h"
//#include "WString.h"
//#include "SoftwareSerial.h"
#ifdef USE_LPL_RF69
    #include "SPI.h"
    //#include "SPIFlash.h"
    #include "RFM69.h"
#endif

//#define USE_DBG_NONE
//#define USE_DBG_SOFT
#define USE_DBG_UART

#define SERIAL_BAUD   115200

#ifdef USE_DBG_SOFT
  #include <SoftwareSerial.h>
  #define DBG_RX 5
  #define DBG_TX 6
  #define dbg(input)   {DbgSer.print(input); delay(1);}
  #define dbgln(input) {DbgSer.println(input); delay(1);}
  #define dbgH(input, hex){DbgSer.print(input, hex); delay(1);}
  #define dbglnH(input, hex){DbgSer.println(input, hex); delay(1);}
  #define dbgF(input)   {DbgSer.print(F(input)); delay(1);}
  #define dbglnF(input) {DbgSer.println(F(input)); delay(1);}
#endif
#ifdef USE_DBG_UART
  #define dbg(input)   {Serial.print(input); delay(1);}
  #define dbgln(input) {Serial.println(input); delay(1);}
  #define dbgH(input, hex){Serial.print(input, hex); delay(1);}
  #define dbglnH(input, hex){Serial.println(input, hex); delay(1);}
  #define dbgF(input)   {Serial.print(F(input)); delay(1);}
  #define dbglnF(input) {Serial.println(F(input)); delay(1);}
#endif
#ifdef USE_DBG_NONE
  #define dbg(input);
  #define dbgln(input);
  #define dbgF(input);
  #define dbglnF(input);
  #define dbgH(input,hex);
  #define dbglnH(input,hex);
#endif


//typedef void (*mqttsnMessagesCallbackT) (uint8_t* response);
typedef void (*mqttsnPubHandlerCallbackT) (char* topic,char* payload);

#define MAX_TOPICS 15		//20
#define MAX_BUFFER_SIZE 66
#define TOPIC_LENGTH  15		//31
#define PAYLOAD_LENGTH  31		//31

class MQTTSN {
public:
    MQTTSN();
    virtual ~MQTTSN();

    uint16_t find_topic_id(const char* name, uint8_t* index);
    bool wait_for_response();
	bool connected();
	bool subscribed();
	bool registered();
	void PrePubCallback (const uint16_t topicId,const char* payload);
	void setCallback(mqttsnPubHandlerCallbackT callback);
	void poll();
	void delay_(uint16_t ms);
	
#ifdef USE_SERIAL
    void parse_stream();
#endif
#ifdef USE_RF12
    void parse_rf12();
#endif
#ifdef USE_LPL_RF69
    void parse_lpl_rf69();
    void setRadioArgs(byte freqBand, byte ID, byte networkID, char* encryptKey);
#endif
    void searchgw(const uint8_t radius);
    void connect(const uint8_t flags, const uint16_t duration, const char* client_id);
    void willtopic(const uint8_t flags, const char* will_topic, const bool update = false);
    void willmsg(const void* will_msg, const uint8_t will_msg_len, const bool update = false);
    bool register_topic(const char* name);
    void publish(const uint8_t flags, const uint16_t topic_id, const void* data, const uint8_t data_len);
#ifdef USE_QOS2
    void pubrec();
    void pubrel();
    void pubcomp();
#endif
    void subscribe_by_name(const uint8_t flags, const char* topic_name);
    void subscribe_by_id(const uint8_t flags, const uint16_t topic_id);
    void unsubscribe_by_name(const uint8_t flags, const char* topic_name);
    void unsubscribe_by_id(const uint8_t flags, const uint16_t topic_id);
    void pingreq(const char* client_id);
    void pingresp();
    void disconnect(const uint16_t duration);
    void printByteA(volatile uint8_t* data);
    void flashLed(int pinid, int ms);
    void doubleFlashLed(int pinid, int ms);
        
    //mqttsnMessagesCallbackT callback;
    mqttsnPubHandlerCallbackT callback;
    //int my_putc( char c, FILE *t);
    
    //API methods exposed for sketch users
    void connect_(uint8_t flagz,uint16_t kat, char* clientid);
    void registertopic_(char* topPub);
    void subscribe_(char* topic, uint8_t flags);
	void publish_(char* topic,char* payload, uint8_t flags);	
protected:

	bool waiting_for_response;
    #ifdef USE_DBG_SOFT
        SoftwareSerial DbgSer;
    #endif
    uint16_t slow;
    #ifdef USE_LPL_RF69
        RFM69 radio;
    #endif
    
    virtual void advertise_handler(const msg_advertise* msg);
    virtual void gwinfo_handler(const msg_gwinfo* msg);
    virtual void connack_handler(const msg_connack* msg);
    virtual void willtopicreq_handler(const message_header* msg);
    virtual void willmsgreq_handler(const message_header* msg);
    virtual void regack_handler(const msg_regack* msg);
    virtual void publish_handler(const msg_publish* msg);
    virtual void register_handler(const msg_register* msg);
    virtual void puback_handler(const msg_puback* msg);
#ifdef USE_QOS2
    virtual void pubrec_handler(const msg_pubqos2* msg);
    virtual void pubrel_handler(const msg_pubqos2* msg);
    virtual void pubcomp_handler(const msg_pubqos2* msg);
#endif
    virtual void suback_handler(const msg_suback* msg);
    virtual void unsuback_handler(const msg_unsuback* msg);
    virtual void pingreq_handler(const msg_pingreq* msg);
    virtual void pingresp_handler();
    virtual void disconnect_handler(const msg_disconnect* msg);
    virtual void willtopicresp_handler(const msg_willtopicresp* msg);
    virtual void willmsgresp_handler(const msg_willmsgresp* msg);

    void regack(const uint16_t topic_id, const uint16_t message_id, const return_code_t return_code);
    void puback(const uint16_t topic_id, const uint16_t message_id, const return_code_t return_code);
	//int sizeofCharA(char *ptr);
    
    
private:
    struct topic {
        char name[TOPIC_LENGTH];		//-4];
        uint16_t id;
    };

    void dispatch();
    uint16_t bswap(const uint16_t val);
    void send_message();

    // Set to true when we're waiting for some sort of acknowledgement from the
    //server that will transition our state.
    //bool waiting_for_response;
	bool _connected;
	bool _subscribed;
	bool _registered;
    uint16_t _message_id;
    uint8_t topic_count;
    
    uint8_t message_buffer[MAX_BUFFER_SIZE];
    uint8_t response_buffer[MAX_BUFFER_SIZE];
    topic topic_table[MAX_TOPICS];

    uint8_t _gateway_id;
    uint32_t _response_timer;
    uint8_t _response_retries;

};


/// The millisecond timer can be used for timeouts up to 60000 milliseconds.
/// Setting the timeout to zero disables the timer.
///
/// * for periodic use, poll the timer object with "if (timer.poll(123)) ..."
/// * for one-shot use, call "timer.set(123)" and poll as "if (timer.poll())"

class MilliTimer {
    word next;
    byte armed;
public:
    MilliTimer () : armed (0) {}
    
    /// poll until the timer fires
    /// @param ms Periodic repeat rate of the time, omit for a one-shot timer.
    byte poll(word ms =0);
    /// Return the number of milliseconds before the timer will fire
    word remaining() const;
    /// Returns true if the timer is not armed
    byte idle() const { return !armed; }
    /// set the one-shot timeout value
    /// @param ms Timeout value. Timer stops once the timer has fired.
    void set(word ms);
};

#endif