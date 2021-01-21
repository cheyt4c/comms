#include <Arduino.h>
/* Hamshield
 * Example: AFSK Serial Messenger
 * Serial glue to send messages over APRS. You will need a 
 * seperate AFSK receiver to test the output of this example.
 * Connect the HamShield to your Arduino. Screw the antenna 
 * into the HamShield RF jack. After uploading this program 
 * to your Arduino, open the Serial Monitor to monitor. Type 
 * a message under 254 characters into the bar at the top of 
 * the monitor. Click the "Send" button. Check for output on 
 * AFSK receiver.
 *
 * To send a message: connect to the Arduino over a Serial link.
 * Send the following over the serial link:
 * `from,to,:message
 * example: * KG7OGM,KG7OGM,:Hi there`
 */



#include <HamShield.h>
#include <DDS.h>
#include <packet.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>  
//#include <base64.hpp> //what is this for???
SoftwareSerial Serial1(2, 3);
// #define MIC_PIN 9
// #define RESET_PIN A3
// #define SWITCH_PIN 2
// #define NCS_PIN A1
// #define DATA_PIN 20
// #define CLK_PIN 21

#define MIC_PIN 3
#define RESET_PIN A3
#define SWITCH_PIN 2
#define NCS_PIN A1

#define MASK B00111111
#define FP_DP 5

typedef struct {
    uint8_t err;
    uint8_t pad[3];
    uint32_t rtcTime;
    uint32_t gpsTime;
    uint32_t millisTime;
    float humidity;
    float temp1;
    float temp44;
    float temp45;
    float temp46;
    float pressure;
    long lat1;  // combine 2 longs on pc where 64 bit doubles are available.
    long lat2;
    long lng1;  // combine 2 longs on pc where 64 bit doubles are available.   
    long lng2; 
    float accX;
    float accY;
    float accZ;
    float rotX;
    float rotY;
    float rotZ;
    //uint32_t pad_1;
    uint32_t crc32;
    uint32_t pad2;
} sensor_packet_t;

static sensor_packet_t sensorData = {0};
sensor_packet_t sensorDataPrint = {0};


HamShield radio/*(NCS_PIN, CLK_PIN, DATA_PIN)*/;
DDS dds;
AFSK afsk;
String messagebuff = "";
String encodedMEGAMessage = "";
String origin_call = "";
String destination_call = "";
String textmessage = "";
int msgptr = 0;
byte packetEncoded[22*6] = {0};
long *packetStruct;

float testFloat1 = 63527.72192;
float testFloat2 = -1827.36283618;

long *floathack;

char encodedMEGA[100] = {0};

// Forward declarations to keep real life C++ compiler happy
void longPacketEncode(long *n);
void getStruct(void);
void packetPrePrint(Stream *s);
void prepMessage();
//Sephiroth!

void setup() {

        sensorData.err = 100;
        sensorData.temp1 = 12.34;
        sensorData.temp44 = -92.393;
        sensorData.lat1=1920922;
        sensorData.lat2=679898;
        sensorData.gpsTime = 1721;
  
  // NOTE: if not using PWM out, it should be held low to avoid tx noise
  pinMode(MIC_PIN, OUTPUT);
  //pinMode(3,INPUT);
  // pinMode(A4,INPUT);
  // pinMode(A5,INPUT);
  digitalWrite(MIC_PIN, LOW);

 
  
  // prep the switch
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  // set up the reset control pin
  pinMode(RESET_PIN, OUTPUT);
  // turn on the radio
  digitalWrite(RESET_PIN, HIGH);
  delay(5); // wait for device to come up
  
  Serial.begin(9600);
  //Serial1.begin(9600);
  //Serial.print(testFloat2,10);

  Serial.println(radio.testConnection() ? "success" : "fail");

  radio.initialize();
  radio.frequency(144390); // default aprs frequency in North America
  radio.setRfPower(0);
  radio.setVolume1(0xFF);
  radio.setVolume2(0xFF);
  radio.setSQHiThresh(-100);
  radio.setSQLoThresh(-100);
  //radio.setSQOn();
  radio.bypassPreDeEmph();
  dds.start();
  afsk.start(&dds);
  delay(100);
  radio.setModeReceive();
  //Serial.println("HELLO");
  radio.dangerMode();

}

void loop() {
    //Serial.println("looping");

    // if(Serial1.available()) {
    //    getStruct();
    //    //Serial.println("Got out of getStruct");
    //    longPacketEncode(packetStruct);
    //    //Serial.println("Got out of LPE");
    //    prepMessage(); 
    //    //Serial.println("Got out of prepMessage");
    //    msgptr = 0; 
    //    messagebuff = "";
    //    Serial.println("!!");    
    // }

    /*if(Serial.available()) { 
      char temp = (char)Serial.read();
      if(temp == '\n') { 
       //Serial.println(messagebuff);
       //longEncode(*(long *)&testFloat2);
       //packetStruct = (long *)&sensorData;
       longPacketEncode(packetStruct);
       prepMessage(); 
       msgptr = 0; 
       messagebuff = "";
       Serial.print("!!");
      } 
      else { 
        messagebuff += temp;
        msgptr++;
      }
    }*/
    if(msgptr > 254) { messagebuff = ""; Serial.print("X!"); }
    prepMessage();
   if(afsk.decoder.read() || afsk.rxPacketCount()) {
      // A true return means something was put onto the packet FIFO
      // If we actually have data packets in the buffer, process them all now
      while(afsk.rxPacketCount()) {
        AFSK::Packet *packet = afsk.getRXPacket();
        Serial.print(F("Packet: "));
        if(packet) {
          packet->printPacket(&Serial);
          AFSK::PacketBuffer::freePacket(packet);
        }
      }
    }
}
    

void longPacketEncode(long *n) {
        int i = 0;
        int j = 0;
        
        while (i < 22) {
                //Serial.println(n[i],HEX);
                j = 0;
                while (j < 6) {
                        packetEncoded[6*i + j] = ((n[i] >> 6*j) & MASK) + 35;
                        j++;
                        //Serial.println("I am doing stuff.");
                        //Serial.println(packetEncoded[6*i+j-1]);
                }
                
                i++;                
        }
        
        return;
}

// void getStruct(void) {
//         //Serial.println("in getStruct");
//         int i = 0;
//         byte structBuffer[88] = {0};
//         while(i < 88) {
//                //Serial.print("Got before the read for the ");
//                //Serial.print(i);
//                //Serial.println("th byte.");
//               while(!Serial1.available()){}
//               //packetStruct[i/4] = (long)Serial1.read() << (i%4)*8;
              
//               //Serial.print(Serial1.read(),HEX);

//               structBuffer[i] = Serial1.read();
           
              
//               //Serial.println(i);
//               i++;
              
//         }
//         //Serial.println("Finished reading");
//         packetStruct = (long *)structBuffer;
//         i = 0;

        

//         /*for(int j = 0; j < 88; j++) {
//                 Serial.print(structBuffer[j], HEX);
//                 Serial.print(" ");
//         }*/

//         Serial.println("");
        

//         packetPrePrint(&Serial);
        

// }

void packetPrePrint(Stream *s) {
        sensorDataPrint = *(sensor_packet_t *)packetStruct;
        s->print("Packet: VK2UNS-15 > VK2UNS U(3,F0) len 150: ");
        s->print(sensorDataPrint.err);
        s->print(F(","));
        s->print(sensorDataPrint.rtcTime);
        s->print(F(","));
        s->print(sensorDataPrint.gpsTime);
        s->print(F(","));
        s->print(sensorDataPrint.millisTime);
        s->print(F(","));
        s->print(sensorDataPrint.humidity,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.temp1,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.temp44,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.temp45,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.temp46,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.pressure,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.lat1);
        s->print(F(","));
        s->print(sensorDataPrint.lat2);
        s->print(F(","));
        s->print(sensorDataPrint.lng1);
        s->print(F(","));
        s->print(sensorDataPrint.lng2);
        s->print(F(","));
        s->print(sensorDataPrint.accX,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.accY,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.accZ,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.rotX,FP_DP);
        s->print(F(","));
        s->print(sensorDataPrint.rotY,FP_DP);
        s->print(F(","));
        s->println(sensorDataPrint.rotZ,FP_DP);

}

void prepMessage() { 
  Serial.println("got into prepmessage");
   
  radio.setModeTransmit();

  Serial.println("set mode to transmit");
  
  delay(1000);
  origin_call = "VK2UNS";                                          // get originating callsign
  destination_call = "VK2UNS"; // get the destination call


  
  textmessage = "";

  //Serial.println("got before textmessage");
  for(int i = 0;i<6*22;i++) {
        /*if((char)packetEncoded[i] == '"') {
                packetEncoded[i] = 'z';        
        }*/

        
        textmessage += (char)packetEncoded[i];
  }

  Serial.println("set textmessage as: ");
  //textmessage = "/59<I$DbPJ$#Z.(###A[)###/+1%($##GI'$RB?F*b#313)$#/VM*$#/VM*$*?+###0*;###`+%###0*;####32%)$#32%)$#=1S*$#32%)$#/VM*$VD'3&$*=P'##\\LS[(#";
  textmessage = "/59<I$DbPJ$#Z.(###A[)###/+1%($##GI'$RB?F*b#313)$#/VM*$#/VM*$*?+###0*;###`+%###0*;####32%)$#32%)$#=1S*$#32%)$#/VM*$VD'3&$*=P'##\\LS[(#";
  Serial.println(textmessage);
  //Serial.println(textmessage);
  
 // Serial.print("From: "); Serial.print(origin_call); Serial.print(" To: "); Serial.println(destination_call); Serial.println("Text: "); Serial.println(textmessage);

  AFSK::Packet *packet = AFSK::PacketBuffer::makePacket(origin_call.length()+destination_call.length()+textmessage.length());

  packet->start();
  packet->appendCallsign(origin_call.c_str(),0);
  packet->appendCallsign(destination_call.c_str(),15,true);   
  packet->appendFCS(0x03);
  packet->appendFCS(0xf0);
  packet->print(textmessage);
  packet->finish();

  bool ret = afsk.putTXPacket(packet);

  if(afsk.txReady()) {
    //Serial.println(F("txReady"));
    radio.setModeTransmit();
    //delay(100);
    if(afsk.txStart()) {
      Serial.println(F("txStart"));
    } else {
      radio.setModeReceive();
    }
  }
  // Wait 2 seconds before we send our beacon again.
  //Serial.println("tick");
  // Wait up to 2.5 seconds to finish sending, and stop transmitter.
  // TODO: This is hackery.
  for(int i = 0; i < 500; i++) {
    if(afsk.encoder.isDone())
       break;
    delay(50);
  }
  Serial.println("Done sending");
  radio.setModeReceive();
} 
 

ISR(TIMER2_OVF_vect) {
  TIFR2 = _BV(TOV2);
  static uint8_t tcnt = 0;
  if(++tcnt == 8) {
    dds.clockTick();
    tcnt = 0;
  }
}

ISR(ADC_vect) {
  static uint8_t tcnt = 0;
  TIFR1 = _BV(ICF1); // Clear the timer flag
  dds.clockTick();
  if(++tcnt == 1) {
    afsk.timer();
    tcnt = 0;
  }
}
