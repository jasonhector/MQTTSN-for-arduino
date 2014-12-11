/*
mqttsn-messages.cpp

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
//serial interface for mqttsn
//#define USE_SERIAL 1
//for use with Jeelib's RF12 radios
//#define USE_RF12 1
//for use with LowPowerLab's RF69 radios
//#define USE_LPL_RF69 1
//#include <Arduino.h>

#include "mqttsn-messages.h"
//#include "mqttsn.h"

#ifdef USE_RF12
//#include "JeeLib.h" 
#endif

#ifdef USE_LPL_RF69
//#include "SPI.h"
//#include "SPIFlash.h"
//#include "RFM69.h"
//RFM69 radio;
#endif

#if !(USE_RF12 || USE_SERIAL || USE_LPL_RF69)
#error "You really should define one of the USE_X ie USE_RF12, USE_SERIAL, USE_LPL_RF69"
#endif

MQTTSN::MQTTSN() :
waiting_for_response(false),
_connected(false),
_message_id(0),
topic_count(0),
_gateway_id(1),
_response_timer(30),
_response_retries(3),
slow(20),       //assume the ack time for messages is 10-15 ms, so we wait 20ms before polling for rx messages
#ifdef USE_DBG_SOFT
    DbgSer(DBG_RX, DBG_TX),
#endif
radio()
{
    memset(topic_table, 0, sizeof(topic) * MAX_TOPICS);
    memset(message_buffer, 0, MAX_BUFFER_SIZE);
    memset(response_buffer, 0, MAX_BUFFER_SIZE);
    #ifdef USE_DBG_SOFT
        DbgSer.begin(SERIAL_BAUD);
    #endif
    #ifdef USE_DBG_UART
        Serial.begin(SERIAL_BAUD);
    #endif
    #ifdef USE_LPL_RF69
        //RFM69 radio;
        //radio.setAddress(11);
        //radio((byte)10,(byte)2,(bool)false,(byte)0);
    #endif
    
}

MQTTSN::~MQTTSN() {
}

//TODO Implement the poll with callback service
void MQTTSN::poll(){
	#ifdef USE_SERIAL
	parse_stream();
	#endif
	#ifdef USE_RF12
	parse_rf12();
	#endif
    #ifdef USE_LPL_RF69
    parse_lpl_rf69();
    #endif
}

bool MQTTSN::wait_for_response() {
    if (waiting_for_response) {
        // TODO: Watch out for overflow.
        if ((millis() - _response_timer) > (T_RETRY * 1000L)) {
            _response_timer = millis();

            if (_response_retries == 0) {
                waiting_for_response = false;
                disconnect_handler(NULL);
            } else {
                send_message();
            }

            --_response_retries;
        }
    }

    return waiting_for_response;
}

bool MQTTSN::connected() {
	return _connected;
}
bool MQTTSN::subscribed() {
	return _subscribed;
}
bool MQTTSN::registered() {
	return _registered;
}

uint16_t MQTTSN::bswap(const uint16_t val) {
    return (val << 8) | (val >> 8);
}

uint16_t MQTTSN::find_topic_id(const char* name, uint8_t* index) {
    for (uint8_t i = 0; i < topic_count; ++i) {
        if (strcmp(topic_table[i].name, name) == 0 && topic_table[i].id != 0xffff) {
            *index = i;
            return topic_table[i].id;
        }
    }

    return 0xffff;
}

#ifdef USE_SERIAL
void MQTTSN::parse_stream() {
	//dbgln("mqttsn-messages:parse_stream:");
	//Serial.println("mqttsn-messages:parse_stream: [HERE]");
    //doubleFlashLed(13,50);
    if (Serial.available() > 0) {
		//doubleFlashLed(13,10);
        uint8_t* response = response_buffer;
        uint8_t packet_length = (uint8_t)Serial.read();
        *response++ = packet_length--;

        while (packet_length > 0) {
            while (Serial.available() > 0) {
                *response++ = (uint8_t)Serial.read();
                --packet_length;
            }
        }

        dispatch();
    }
}
#endif

#ifdef USE_RF12
void MQTTSN::parse_rf12() {
	//dbgln(F(" mqttsn-messages::parse_rf12: >>> "));
	if (rf12_recvDone()){//&& (rf12_hdr & RF12_HDR_CTL) == 0) {
		dbgF(" mqttsn-messages::parse_rf12: recvDone()=1     rf12_crc=");
		//dbg(F(" mqttsn-messages::parse_rf12: rf12_crc = "));
		dbgln(rf12_crc);
		//dbg(F(" mqttsn-messages::parse_rf12: rf12 data = "));
		//for (int i=0;i<rf12_len;i++){
		//	dbg(rf12_data[i],HEX);
		//	dbg(F(" "));
		//}
		//dbgln(F(" "));
		if(rf12_crc == 0 ){
			//dbgln(F(" mqttsn-messages::parse_rf12: crc free RF data received! "));
			dbgF(" mqttsn-messages::parse_rf12: rf12 data = ");
			for (int i=0;i<rf12_len;i++){
				dbg(rf12_data[i],HEX);
				dbgF(" ");
			}
			dbglnF(" ");
			dbgF(" mqttsn-messages::parse_rf12: rf12 hdr = ");
			dbgln(rf12_hdr);
			memcpy(response_buffer, (const void*)rf12_data, RF12_MAXDATA < MAX_BUFFER_SIZE ? RF12_MAXDATA : MAX_BUFFER_SIZE);
				if (RF12_WANTS_ACK) {
					dbglnF(" Sending Rf12 ACK");
					rf12_sendStart(RF12_ACK_REPLY, 0,0,1);
				}
			//rf12_recvDone();
			dispatch();
		}
	}
}
#endif

#ifdef USE_LPL_RF69
void MQTTSN::parse_lpl_rf69() {

    //flashLed(9,50);
    //dbgln(F(" mqttsn-messages::parse_lpl_rf69: >>> "));
    //dbg(F(" mqttsn-messages::parse_lpl_rf69: radio.receiveDone()= "));
    //dbgln(radio.receiveDone());
    //dbg(F(" mqttsn-messages::parse_lpl_rf69: RSSI= "));
    //dbgln(radio.readRSSI());
    if(radio.receiveDone()){
        //dbg(F(" mqttsn-messages::parse_lpl_rf69: recvDone()=1"));
        //dbg(F(" mqttsn-messages::parse_lpl_rf69: rf69 data = "));
        //for (int i=0;i<rf12_len;i++){
        //  dbg(rf12_data[i],HEX);
        //  dbg(F(" "));
        //}
        //dbgln(F(" "));

    if (radio.ACKRequested()){
        byte theNodeID = radio.SENDERID;
        radio.sendACK();
        dbglnF(" mqttsn-messages::parse_lpl_rf69: Sending Rf69 ACK");
        }
        
    dbglnF("<<<-------------------->>>");
    for (int i=0;i<=topic_count;i++){
        dbgF("topicTable[");
        dbg(i);
        dbgF("].id.name=");
        dbg(topic_table[i].id);
        dbgF(" ");
        dbgln(topic_table[i].name);
    }
    dbglnF("<<<-------------------->>>");

        
            
            dbgF(" mqttsn-messages::parse_rf69: rf69 data = ");
            for (int i=0;i<radio.DATALEN;i++){
                dbgH(radio.DATA[i],HEX);
                dbgF(" ");
            }
            dbglnF(" ");
            
          //memcpy(response_buffer, (const void*)radio.DATA, radio.RF69_MAX_DATA_LEN < MAX_BUFFER_SIZE ? radio.RF69_MAX_DATA_LEN : MAX_BUFFER_SIZE);
            memcpy(response_buffer, (const void*)radio.DATA, 61);//< MAX_BUFFER_SIZE ? radio.RF69_MAX_DATA_LEN : MAX_BUFFER_SIZE);

            doubleFlashLed(9, 10);
            dispatch();
    }
}

void MQTTSN::setRadioArgs(byte freqBand, byte ID, byte networkID, char* encryptKey){
    dbglnF("mqttsn-messages:Initialising RF69..."); 
    radio.initialize(freqBand, ID, networkID);
    radio.encrypt(encryptKey);
    dbglnF("mqttsn-messages:RF69 init done"); 
}
#endif

void MQTTSN::dispatch() {
    message_header* response_message = (message_header*)response_buffer;
	//dbgF("mqttsn-messages:dispatch: response_msg len=");
	//dbgln(response_message->length);
	//dbgF("mqttsn-messages:dispatch: response_msg type= ");
	//dbglnH(response_message->type,HEX);
	//dbg(F("mqttsn-messages:dispatch: response_buffer= "));
	//for (int i=0;i<response_message->length;i++){
	//	dbg(response_buffer[i],HEX);
	//	dbg(F(" "));
	//}
	//dbgln("");
	//dbg("mqttsn-messages:dispatch: response_buffer=");
	//printByteA(response_buffer);
	//doubleFlashLed(13,500);
    
    switch (response_message->type) {
    case ADVERTISE:
		dbglnF("mqttsn-messages:dispatch: advertiseHandler");
        advertise_handler((msg_advertise*)response_buffer);
        break;

    case GWINFO:
		dbglnF("mqttsn-messages:dispatch: gwinfoHandler");
        gwinfo_handler((msg_gwinfo*)response_buffer);
        break;

    case CONNACK:
		dbglnF("mqttsn-messages:dispatch: connackHandler");
        connack_handler((msg_connack*)response_buffer);
        break;

    case WILLTOPICREQ:
		dbglnF("mqttsn-messages:dispatch: willtopicHandler");
        willtopicreq_handler(response_message);
        break;

    case WILLMSGREQ:
		dbglnF("mqttsn-messages:dispatch: willMsgHandler");
        willmsgreq_handler(response_message);
        break;

    case REGISTER:
		dbglnF("mqttsn-messages:dispatch: registerHandler");
        register_handler((msg_register*)response_buffer);
        break;

    case REGACK:
		dbglnF("mqttsn-messages:dispatch: regackHandler");
        regack_handler((msg_regack*)response_buffer);
        break;

    case PUBLISH:
		dbglnF("mqttsn-messages:dispatch: publishHandler");
        publish_handler((msg_publish*)response_buffer);
        break;

    case PUBACK:
		dbglnF("mqttsn-messages:dispatch: pubackHandler");
        puback_handler((msg_puback*)response_buffer);
        break;

    case SUBACK:
		dbglnF("mqttsn-messages:dispatch: subackHandler");
        suback_handler((msg_suback*)response_buffer);
        break;

    case UNSUBACK:
		dbglnF("mqttsn-messages:dispatch: unsubackHandler");
        unsuback_handler((msg_unsuback*)response_buffer);
        break;

    case PINGREQ:
		dbglnF("mqttsn-messages:dispatch: pinreqHandler");
        pingreq_handler((msg_pingreq*)response_buffer);
        break;

    case PINGRESP:
		dbglnF("mqttsn-messages:dispatch: pingrespHandler");
        pingresp_handler();
        break;

    case DISCONNECT:
		dbglnF("mqttsn-messages:dispatch: disconnectHandler");
        disconnect_handler((msg_disconnect*)response_buffer);
        break;

    case WILLTOPICRESP:
		dbglnF("mqttsn-messages:dispatch: willtopicrespHandler");
        willtopicresp_handler((msg_willtopicresp*)response_buffer);
        break;

    case WILLMSGRESP:
		dbglnF("mqttsn-messages:dispatch: willmsgrespHandler");
        willmsgresp_handler((msg_willmsgresp*)response_buffer);
        break;

    default:
        return;
    }
    waiting_for_response = false;
}

void MQTTSN::send_message() {
    message_header* hdr = reinterpret_cast<message_header*>(message_buffer);
#ifdef USE_LPL_RF69
   //int i=0;
   // while (!radio.canSend()){//} && i<10) {
        //dbg(F("mqttsn-messages:send_message:radio.canSend() = "));
        //dbg(radio.canSend());
        //dbg(F("."));
        //rf12_recvDone();
        //i++;
        //delay(32);
        //doubleFlashLed(9,50);
    //}

    dbglnF("<<<<<<<-------------------->>>>>>>");
    for (int i=0;i<=topic_count;i++){
        dbgF("topicTable[");
        dbg(i);
        dbgF("].id.name=");
        dbg(topic_table[i].id);
        dbgF(" ");
        dbgln(topic_table[i].name);
    }
    dbglnF("<<<<<<<-------------------->>>>>>>");

    dbgF("mqttsn-messages:send_message: Sending to [");
    dbg(_gateway_id);
    dbgF("]");
    radio.sendWithRetry(_gateway_id, message_buffer, hdr->length);
    dbglnF("..sent");
    //rf12_sendWait(2);
    //rf12_sleep(RF12_SLEEP);
    //doubleFlashLed(9, 30);
#endif

#ifdef USE_RF12
	//rf12_sleep(RF12_WAKEUP);
   int i=0;
    while (!rf12_canSend() && i<10) {
        //dbg(F("mqttsn-messages:advertise_handler: rf12_canSend() = "));
		//dbg(rf12_canSend());
		//dbg(F(" "));
        rf12_recvDone();
        i++;
        delay(slow);
        //Sleepy::loseSomeTime(32);
    }
    //dbgln(F(" "));
    rf12_sendStart(_gateway_id, message_buffer, hdr->length);
    rf12_sendWait(2);
    //rf12_sleep(RF12_SLEEP);
    //doubleFlashLed(9, 20);
#endif
#ifdef USE_SERIAL
    //Serial.print("mqttsn-messages:send_message: message_buffer=");
    //printByteA(message_buffer);
    Serial.write(message_buffer, hdr->length);
	//Serial.write('\r');
    Serial.flush();
#endif

    if (!waiting_for_response) {
        _response_timer = millis();
        _response_retries = N_RETRY;

        // Cheesy hack to ensure two messages don't run-on into one send.
        delay_(20);
    }else{
        waiting_for_response=wait_for_response();
    }
}

void MQTTSN::advertise_handler(const msg_advertise* msg) {
    _gateway_id = msg->gw_id;
    dbgF("mqttsn-messages:advertise_handler: _gateway_id=");
    dbgln(_gateway_id);
}

//extern void MQTTSN_gwinfo_handler(const msg_gwinfo* msg);
void MQTTSN::gwinfo_handler(const msg_gwinfo* msg) {
//	MQTTSN_gwinfo_handler(msg);
}

void MQTTSN::connack_handler(const msg_connack* msg) {
	_connected = 1;
	dbglnF("mqttsn-messages:connack_handler: _connected=1");
}

void MQTTSN::willtopicreq_handler(const message_header* msg) {
}

void MQTTSN::willmsgreq_handler(const message_header* msg) {
}

void MQTTSN::regack_handler(const msg_regack* msg) {
    if (msg->return_code == 0 && topic_count < MAX_TOPICS && bswap(msg->message_id) == _message_id) {
        topic_table[topic_count].id = bswap(msg->topic_id);
        ++topic_count;
        _registered=1;
    }
}

void MQTTSN::puback_handler(const msg_puback* msg) {
	waiting_for_response = false;
}

#ifdef USE_QOS2
void MQTTSN::pubrec_handler(const msg_pubqos2* msg) {
}

void MQTTSN::pubrel_handler(const msg_pubqos2* msg) {
}

void MQTTSN::pubcomp_handler(const msg_pubqos2* msg) {
}
#endif

void MQTTSN::pingreq_handler(const msg_pingreq* msg) {
    pingresp();
}

void MQTTSN::suback_handler(const msg_suback* msg) {
	//dbgln(F("MQTTSN:subackhandler: ...")); 
	//Need to have returned topic id stored in the topic_table
	//dbg("mqttsn-messages:suback_handler: msg->message_id=");
    //dbgln(bswap(msg->message_id));
    //dbg("mqttsn-messages:suback_handler: _message_id=");
    //dbgln(_message_id);
	if (msg->return_code == 0 && topic_count < MAX_TOPICS && bswap(msg->message_id) == _message_id) {
	topic_table[topic_count].id = bswap(msg->topic_id);
	dbgF("mqttsn-messages:suback_handler: topicTable[");
    dbg(topic_count);
    dbgF("].name=");
	dbgln(topic_table[topic_count].name);
	dbgF("mqttsn-messages:suback_handler: topicTable[");
    dbg(topic_count);
    dbgF("].id=");
	dbgln(topic_table[topic_count].id);
	++topic_count;
	_subscribed=1;
	}
}

void MQTTSN::unsuback_handler(const msg_unsuback* msg) {
}

void MQTTSN::disconnect_handler(const msg_disconnect* msg) {
	_connected = false;
    //lock up here and let the watchdog reset mcu to start sync over
    dbgF("mqttsn-messages:disconnect_handler: LOCKING UP for watchdog reset!!!");
    while(true){}
}

void MQTTSN::pingresp_handler(){
}

//extern void MQTTSN_publish_handler(const msg_publish* msg);
void MQTTSN::publish_handler(const msg_publish* msg) {
	//dbgln(F("Mmqttsn-messages::publishhandler: ..."));  
    if (msg->flags & FLAG_QOS_1) {
        return_code_t ret = REJECTED_INVALID_TOPIC_ID;
        const uint16_t topic_id = bswap(msg->topic_id);

        for (uint8_t i = 0; i < topic_count; ++i) {
            if (topic_table[i].id == topic_id) {
                ret = ACCEPTED;
				//MQTTSN_publish_handler(msg);
				//do something with the rx data - callback to sketch
				///The payload is variable length!! We have to use the 
				///msg->len to determine how much of the msg->data to copy
				///and send to PrePubCallback
                //also to append null terminator
				uint8_t payloLength = msg->length - sizeof(msg_publish);
				char paylo[payloLength+1];
				//memcpy(paylo, msg->data, payloLength);
				strncpy(paylo,msg->data,payloLength);
                paylo[payloLength]='\0';
				//dbg(F("mqttsn-messages:publish_handler: msg->length="));
				//dbgln(msg->length);
				//dbg(F("mqttsn-messages:publish_handler: sizeof(msg_publish)="));
				//dbgln(sizeof(msg_publish));
				//dbg(F("mqttsn-messages:publish_handler: sizeofCharA(paylo)="));
				//dbgln(sizeofCharA(paylo));
			
				//dbg(F("mqttsn-messages:publish_handler: payload="));
				//dbgln(paylo);//msg->data);
				//dbg(F("mqttsn-messages:publish_handler: payloadLength="));
				//dbgln(payloLength);//sizeofCharA(const_cast<char*>(msg->data)));
				dbglnF("mqttsn-messages:publish_handler: <<<ACTION Callback>>>");
				PrePubCallback(i, paylo);//msg->data);
                break;
            }
        }

        puback(msg->topic_id, msg->message_id, ret);
    }
    
	const uint16_t topic_id = bswap(msg->topic_id);
	//dbg(F("mqttsn-messages:publish_handler: msg->topic_count="));
	//dbgln(topic_count);
	for (uint8_t i = 0; i < topic_count; ++i) {
		//dbg(F("mqttsn-messages:publish_handler: topicId="));
		//dbgln(topic_table[i].id);
		//dbg(F("mqttsn-messages:publish_handler: topicName="));
		//dbgln(topic_table[i].name);
		if (topic_table[i].id == topic_id) {
			//MQTTSN_publish_handler(msg);
			///The payload is variable length!! We have to use the 
			///msg->len to determine how much of the msg->data to copy
			///and send to PrePubCallback
			uint8_t paylLength = msg->length - sizeof(msg_publish);
			char payl[paylLength+1];
			//memcpy(payl, msg->data,paylLength);
            strncpy(payl,msg->data,paylLength);
            payl[paylLength]='\0';

			//dbgF("mqttsn-messages:publish_handler: msg->length=");
			//dbgln(msg->length);
			//dbgF("mqttsn-messages:publish_handler: sizeof(msg_publish)=");
			//dbgln(sizeof(msg_publish));
			//dbgF("mqttsn-messages:publish_handler: paylLength=");
			//dbgln(paylLength);
			///sizeofCharA is looking for null termination - which isnt in the msg->data
			//dbg(F("mqttsn-messages:publish_handler: printf(payl)="));
			//dbgf("%.*s", paylLength,payl);
			
            dbgF("mqttsn-messages:publish_handler: topic=");
            dbgln(topic_table[i].name);
			dbgF("mqttsn-messages:publish_handler: payload=");
			dbgln(payl);
			//dbg(F("mqttsn-messages:publish_handler: payloadLength="));
			//dbgln(paylLength);//sizeofCharA(const_cast<char*>(msg->data)));
			dbglnF("mqttsn-messages:publish_handler: <<<ACTION Callback>>>");
			PrePubCallback(i, payl);//msg->data);
			break;
		}
	}
}

void MQTTSN::register_handler(const msg_register* msg) {
    return_code_t ret = REJECTED_INVALID_TOPIC_ID;
    uint8_t index;
    uint16_t topic_id = find_topic_id(msg->topic_name, &index);

    if (topic_id != 0xffff) {
        topic_table[index].id = bswap(msg->topic_id);
        ret = ACCEPTED;
    }

    regack(msg->topic_id, msg->message_id, ret);
}

void MQTTSN::willtopicresp_handler(const msg_willtopicresp* msg) {
}

void MQTTSN::willmsgresp_handler(const msg_willmsgresp* msg) {
}

void MQTTSN::searchgw(const uint8_t radius) {
    msg_searchgw* msg = reinterpret_cast<msg_searchgw*>(message_buffer);

    msg->length = sizeof(msg_searchgw);
    msg->type = SEARCHGW;
    msg->radius = radius;

    send_message();
    waiting_for_response = true;
}

void MQTTSN::connect(const uint8_t flags, const uint16_t duration, const char* client_id) {
    msg_connect* msg = reinterpret_cast<msg_connect*>(message_buffer);

    msg->length = sizeof(msg_connect) + strlen(client_id);
    msg->type = CONNECT;
    msg->flags = flags;
    msg->protocol_id = PROTOCOL_ID;
    msg->duration = bswap(duration);
    strcpy(msg->client_id, client_id);
    send_message();
	_connected = false;
    waiting_for_response = true;
}

void MQTTSN::willtopic(const uint8_t flags, const char* will_topic, const bool update) {
    if (will_topic == NULL) {
        message_header* msg = reinterpret_cast<message_header*>(message_buffer);

        msg->type = update ? WILLTOPICUPD : WILLTOPIC;
        msg->length = sizeof(message_header);
    } else {
        msg_willtopic* msg = reinterpret_cast<msg_willtopic*>(message_buffer);

        msg->type = update ? WILLTOPICUPD : WILLTOPIC;
        msg->flags = flags;
        strcpy(msg->will_topic, will_topic);
    }

    send_message();

    if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
        waiting_for_response = true;
    }
}

void MQTTSN::willmsg(const void* will_msg, const uint8_t will_msg_len, const bool update) {
    msg_willmsg* msg = reinterpret_cast<msg_willmsg*>(message_buffer);

    msg->length = sizeof(msg_willmsg) + will_msg_len;
    msg->type = update ? WILLMSGUPD : WILLMSG;
    memcpy(msg->willmsg, will_msg, will_msg_len);

    send_message();
}

void MQTTSN::disconnect(const uint16_t duration) {
    msg_disconnect* msg = reinterpret_cast<msg_disconnect*>(message_buffer);

    msg->length = sizeof(message_header);
    msg->type = DISCONNECT;

    if (duration > 0) {
        msg->length += sizeof(msg_disconnect);
        msg->duration = bswap(duration);
    }

    send_message();
    waiting_for_response = true;
}

bool MQTTSN::register_topic(const char* name) {
    if (!waiting_for_response && topic_count < (MAX_TOPICS - 1)) {
        ++_message_id;

        // Fill in the next table entry, but we only increment the counter to
        // the next topic when we get a REGACK from the broker. So don't issue
        // another REGISTER until we have resolved this one.
        
        //topic_table[topic_count].name = name;
        strcpy(topic_table[topic_count].name,name);
        topic_table[topic_count].id = 0xffff;

        msg_register* msg = reinterpret_cast<msg_register*>(message_buffer);

        msg->length = sizeof(msg_register) + strlen(name);
        msg->type = REGISTER;
        msg->topic_id = 0;
        msg->message_id = bswap(_message_id);
        strcpy(msg->topic_name, name);

        send_message();
        waiting_for_response = true;
        return true;
    }

    return false;
}

void MQTTSN::regack(const uint16_t topic_id, const uint16_t message_id, const return_code_t return_code) {
    msg_regack* msg = reinterpret_cast<msg_regack*>(message_buffer);

    msg->length = sizeof(msg_regack);
    msg->type = REGACK;
    msg->topic_id = bswap(topic_id);
    msg->message_id = bswap(message_id);
    msg->return_code = return_code;

    send_message();
}

void MQTTSN::publish(const uint8_t flags, const uint16_t topic_id, const void* data, const uint8_t data_len) {
    ++_message_id;

    msg_publish* msg = reinterpret_cast<msg_publish*>(message_buffer);
    
    msg->length = sizeof(msg_publish) + data_len;
    msg->type = PUBLISH;
    msg->flags = flags;
    msg->topic_id = bswap(topic_id);
    msg->message_id = bswap(_message_id);
    memcpy(msg->data, data, data_len);
	
    send_message();
    
    if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
        waiting_for_response = true;
    }
}

#ifdef USE_QOS2
void MQTTSN::pubrec() {
    msg_pubqos2* msg = reinterpret_cast<msg_pubqos2*>(message_buffer);
    msg->length = sizeof(msg_pubqos2);
    msg->type = PUBREC;
    msg->message_id = bswap(_message_id);
    send_message();
}

void MQTTSN::pubrel() {
    msg_pubqos2* msg = reinterpret_cast<msg_pubqos2*>(message_buffer);
    msg->length = sizeof(msg_pubqos2);
    msg->type = PUBREL;
    msg->message_id = bswap(_message_id);

    send_message();
}

void MQTTSN::pubcomp() {
    msg_pubqos2* msg = reinterpret_cast<msg_pubqos2*>(message_buffer);
    msg->length = sizeof(msg_pubqos2);
    msg->type = PUBCOMP;
    msg->message_id = bswap(_message_id);

    send_message();
}
#endif

void MQTTSN::puback(const uint16_t topic_id, const uint16_t message_id, const return_code_t return_code) {
    msg_puback* msg = reinterpret_cast<msg_puback*>(message_buffer);

    msg->length = sizeof(msg_puback);
    msg->type = PUBACK;
    msg->topic_id = bswap(topic_id);
    msg->message_id = bswap(message_id);
    msg->return_code = return_code;

    send_message();
}

void MQTTSN::subscribe_by_name(const uint8_t flags, const char* topic_name) {
    ++_message_id;

    msg_subscribe* msg = reinterpret_cast<msg_subscribe*>(message_buffer);

    // The -2 here is because we're unioning a 0-length member (topic_name)
    // with a uint16_t in the msg_subscribe struct.
    msg->length = sizeof(msg_subscribe) + strlen(topic_name) - 2;
    msg->type = SUBSCRIBE;
    msg->flags = (flags & QOS_MASK) | FLAG_TOPIC_NAME;
    msg->message_id = bswap(_message_id);
    strcpy(msg->topic_name, topic_name);

    send_message();
    
    // Fill in the next table entry, but we only increment the counter to
    // the next topic when we get a SUBACK from the broker. So don't issue
    // another SUB until we have resolved this one.
    
    //topic_table[topic_count].name = topic_name;
    strcpy(topic_table[topic_count].name,topic_name);
    //strcpy(const_cast<char*>(topic_table[topic_count].name),topic_name);
    
    topic_table[topic_count].id = 0xffff;
    dbgF("mqttsn-messages:subscribe_by_name: topicTable[");
    dbg(topic_count);
    dbgF("].name=");
	dbgln(topic_table[topic_count].name);
	//dbgF("mqttsn-messages:subscribe_by_name: topicTable[");
    //dbg(topic_count);
    //dbgF("].id=");
	//dbgln(topic_table[topic_count].id);
	
    //if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
    //    waiting_for_response = true;
    //}
    waiting_for_response = true;
}

void MQTTSN::subscribe_by_id(const uint8_t flags, const uint16_t topic_id) {
    ++_message_id;

    msg_subscribe* msg = reinterpret_cast<msg_subscribe*>(message_buffer);

    msg->length = sizeof(msg_subscribe);
    msg->type = SUBSCRIBE;
    msg->flags = (flags & QOS_MASK) | FLAG_TOPIC_PREDEFINED_ID;
    msg->message_id = bswap(_message_id);
    msg->topic_id = bswap(topic_id);

    send_message();

    if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
        waiting_for_response = true;
    }
    //waiting_for_response = true;
}

void MQTTSN::unsubscribe_by_name(const uint8_t flags, const char* topic_name) {
    ++_message_id;

    msg_unsubscribe* msg = reinterpret_cast<msg_unsubscribe*>(message_buffer);

    // The -2 here is because we're unioning a 0-length member (topic_name)
    // with a uint16_t in the msg_unsubscribe struct.
    msg->length = sizeof(msg_unsubscribe) + strlen(topic_name) - 2;
    msg->type = UNSUBSCRIBE;
    msg->flags = (flags & QOS_MASK) | FLAG_TOPIC_NAME;
    msg->message_id = bswap(_message_id);
    strcpy(msg->topic_name, topic_name);

    send_message();

    if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
        waiting_for_response = true;
    }
}

void MQTTSN::unsubscribe_by_id(const uint8_t flags, const uint16_t topic_id) {
    ++_message_id;

    msg_unsubscribe* msg = reinterpret_cast<msg_unsubscribe*>(message_buffer);

    msg->length = sizeof(msg_unsubscribe);
    msg->type = UNSUBSCRIBE;
    msg->flags = (flags & QOS_MASK) | FLAG_TOPIC_PREDEFINED_ID;
    msg->message_id = bswap(_message_id);
    msg->topic_id = bswap(topic_id);

    send_message();

    if ((flags & QOS_MASK) == FLAG_QOS_1 || (flags & QOS_MASK) == FLAG_QOS_2) {
        waiting_for_response = true;
    }
}

void MQTTSN::pingreq(const char* client_id) {
    msg_pingreq* msg = reinterpret_cast<msg_pingreq*>(message_buffer);
    msg->length = sizeof(msg_pingreq) + strlen(client_id);
    msg->type = PINGREQ;
    strcpy(msg->client_id, client_id);

    send_message();

    waiting_for_response = true;
}

void MQTTSN::pingresp() {
    message_header* msg = reinterpret_cast<message_header*>(message_buffer);
    msg->length = sizeof(message_header);
    msg->type = PINGRESP;

    send_message();
}
/*
void MQTTSN::printByteA(volatile uint8_t* data){
    char* cha = (char*)data;
    String str = String(cha);
    int len = cha[0];
    
    //dbg(F("cha[0]="));
    //dbgln(cha[0],DEC);
    //dbg(F("len="));
    //dbgln(len);
    
    for(int j=0;j<len;j++){
        dbgF("[");
        dbgH(data[j],HEX);
        dbgF("]");
    }
    dbglnF("");
}
*/
void MQTTSN::flashLed(int pinid, int ms){
    digitalWrite(pinid, HIGH);
    delay(ms);
    digitalWrite(pinid, LOW);
}
void MQTTSN::doubleFlashLed(int pinid, int ms){
	flashLed(pinid,ms);
	delay(ms);
	flashLed(pinid,ms);
    delay(2*ms);
}

void MQTTSN::setCallback(mqttsnPubHandlerCallbackT callback)
{
  this->callback = callback;
}

void MQTTSN::PrePubCallback (const uint16_t topicId,const char* payload){
	//This function is used to copy the char* and send the copied char* as args into the callback
	//this helps to prevent modification of the char* pointers in this library by external code
	//copy the char arrays
	//uint8_t topicSize = sizeofCharA(const_cast<char*>(topic));
	//uint8_t payloadSize = sizeofCharA(const_cast<char*>(payload));
	//char copiedTopic[topicSize+1];
	//char copiedPayload[payloadSize+1];
	//memcpy(copiedTopic, topic, topicSize+1);
	//memcpy(copiedPayload, payload, payloadSize+1);
	//call the callback
	//callback(copiedTopic, copiedPayload);
    char localTopic[TOPIC_LENGTH];
    char localPayload[PAYLOAD_LENGTH];
    strcpy(localTopic, topic_table[topicId].name);
    strcpy(localPayload, payload);
    callback(localTopic, localPayload);
}

/*
//returns the size of a character array using a pointer to the first element of the character array
int MQTTSN::sizeofCharA(char *ptr){
    //variable used to access the subsequent array elements.
    int offset = 0;
    //variable that counts the number of elements in your array
    int count = 0;
    //While loop that tests whether the end of the array has been reached
    while (*(ptr + offset) != '\0')
    {
        //increment the count variable
        ++count;
        //advance to the next element of the array
        ++offset;
    }
    //return the size of the array
    return count;
}
*/
//int MQTTSN::my_putc( char c, FILE *t) {
//  DbgSer.write( c );
//}


void MQTTSN::connect_(uint8_t flagz,uint16_t kat, char* clientid){
  connect(flagz, kat, clientid);
  dbglnF("Connect msg sent ");
  delay_(slow);
  poll();
  dbgF("mqttsn.connected()=");
  dbgln(connected());
}

void MQTTSN::registertopic_(char* topPub){
  uint8_t index;
  uint16_t topicPubId;
  topicPubId = find_topic_id(topPub, &index);
  //dbg("TopicPubID: ");
  //dbgln(u16TopicPubID);
  if (topicPubId == 0xffff)
  {
     //Topic is not registered yet
     register_topic(topPub); 
     dbgF("Register topic sent for: "); 
     dbgln(topPub);  
     delay_(slow);
     poll();  //for regack
     _registered=0;
  }
  //delay(slow);
  //mqttsn.poll();
}

void MQTTSN::subscribe_(char* topic, uint8_t flags){
        // ------- MQTT-SN subscribe logic -------
    uint8_t index;
    uint16_t topicSubId;
    topicSubId = find_topic_id(topic, &index);
    //dbg(F("TopicSubID: "));
    //dbgln(u16TopicSubID);
    if (topicSubId == 0xffff && !waiting_for_response) 
    {
      // Topic is not registered yet
      subscribe_by_name(flags, topic);  // Flags=0x20   
      dbglnF("Subscribe by name sent"); 
      _subscribed=0;
      //delay_(slow);
      //poll();  
    } 
    
    
    
}

void MQTTSN::publish_(char* topic,char* payload, uint8_t flags){
  uint8_t index;
  uint16_t topicId = find_topic_id(topic, &index); 
  //CHECK IF CONNECTED and TOPIC REGISTERED - ONLY PUB THEN
  if(connected()){
	  if(topicId!=0xffff){
		//Topic Registered >> now publish  
		  //dbgln("Publishing: ");
		  //dbg("Topic name="); 
		  //dbg(topic);
		  //dbg("    Topic id=");
		  //dbg(topicId);
		  //dbg("    Payload="); 
		  //dbgln(payload);
		  publish(flags, topicId, payload, strlen(payload));  // Flags=0x20         
	  }else{
		//TOPIC NOT REGISTERED - REGISTER
		dbglnF("topic not registered...registering... ");
		registertopic_(topic);
		delay_(slow);
		poll();
		//should now be registered- then publish
		if(registered()){
			publish(flags, topicId, payload, strlen(payload));  // Flags=0x20  
		}
		} 
   }else{
	   dbglnF("Cant publish before connecting...."); 
	  //skech main loop should retry the connection, so no need to do it here   
    }
    delay_(slow);
    poll();
}

void MQTTSN::delay_(uint16_t ms){
	delay(ms);
}

byte MilliTimer::poll(word ms) {
    byte ready = 0;
    if (armed) {
        word remain = next - millis();
        // since remain is unsigned, it will overflow to large values when
        // the timeout is reached, so this test works as long as poll() is
        // called no later than 5535 millisecs after the timer has expired
        if (remain <= 60000)
            return 0;
        // return a value between 1 and 255, being msecs+1 past expiration
        // note: the actual return value is only reliable if poll() is
        // called no later than 255 millisecs after the timer has expired
        ready = -remain;
    }
    set(ms);
    return ready;
}

word MilliTimer::remaining() const {
    word remain = armed ? next - millis() : 0;
    return remain <= 60000 ? remain : 0;
}

void MilliTimer::set(word ms) {
    armed = ms != 0;
    if (armed)
        next = millis() + ms - 1;
}