Mqttsn Library

MQTT-SN library for Arduino that has been "forked" from http://bitbucket.org/MerseyViking/mqtt-sn and https://github.com/boriz/MQTT-SN-Arduino. Sample sketches for publishing and subscribing are provided

You will need the Mote library for Arduino to get the example sketches going- this provides Remote Terminal Unit [RTU] functionality to arduino with features like analog deadbands, binary debounce, pulse and latch controls, etc. The Mote protocol is an application layer protocol to the MQTTSN libraries and has been defined in the this repos wiki


The Really Small Message Broker [RSMB] is the only broker currently that supports both Mqtt and Mqttsn. The config is in the wiki.

Essentially split into 2 architectures: -> Architecture 0 - direct connectivity between router and arduino over serial -> Architecture 1 - extension to replace direct with RFM12B/RFM69W radio comms that is supported in MerseyViking's Mqttsn library


Architecture 1:

[RSMB]----(UDP)----[TwistedRouter]----(SERIAL)----[RFM12PI]----(RADIO)----[Arduino with Mqttsn and Mote and Jeelib libs]

The wiki contains:
Mqttsn reference sheet : I decided to teach myself MQTTSN and creating a reference sheet helped
Screenshots

Todo:
Code littered with debug print statements - clean this up
additional sample sketches
extend mqttsn functionality - only core features implemented currently
