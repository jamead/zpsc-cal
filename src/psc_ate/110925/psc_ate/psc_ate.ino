/*
PSC ATE
Build 05/07/25 Initial release
Build 11/09/25 Added unipolar power converter behavior


D. Bergman

Libraries are placed in /root/Arduino/libraries when installed by the library manager or may be
placed manually in ~/Arduino install directory/libraries/

Need to install the following libraries:
1. UIPEthernet
   SdFat
2. Create directory "OneWire" in /Arduino install directory/libraries/, and put the following files in it:
      DS2482.cpp, DS2482.h, DS18B20_DS2482.cpp, DS18B20_DS2482.h
      Need to comment out blockTillConversionComplete in DS18B20_DS2482::requestTemperatures(), line 336 in DS18B20_DS2482.cpp 

3. Need modified pins_arduino.h file to use non-Arduino digital IO on Atmega chip. Put in 
C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\mega\. Restart Arduino IDE.

Loop rate:
Read 1 ADC channel per loop and very infrequent RS-232 writes: 3800 /s
Read 16 ADC channels per loop, infrequent RS-232 writes: ~2-3 ms
HK packet takes 33 ms to send out at 115 kbaud

*/

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>

//#include <avr/wdt.h>
#include <SdFat.h> // need this to use fgets() for line parsing
//#include "pins_arduino_nguyen.h"

#include <UIPEthernet.h>

// test board 192.168.0.6
// SN002 192.168.0.7
// SN004 192.168.0.8

IPAddress ip(10,69,26,3);
byte mac[] = {0x02,0x00,0x00,0x01,0x01,0x03};

unsigned int port = 5000;
char packetBuffer[100]; // message from client to initiate UDP sending of housekeeping data
//char datagram[2000];// housekeeping data packet UDP datagram
char scratch1[10]; // scratchpad for building datagram


// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


//uint8_t ipOctet[4];
//byte mac[6];
float hall_cal[] = {1.0, 1.0, 1.0, 1.0}; // one correction for each PWM power daughterboard
float mod_serial = 0;
float buildnumber = 251109; // 



/*
int readSD(void) {
const int sdCS = 16; // chip select pin for SD module
char *p1;
char *pEnd; // dummy output of strtol()
char s1[30];
char hdr1[30];
char* delim1 = ":";
char* delim2 = ".";
char* delim3 = ",";
char buf[128]; // buffer for each line of file
int line=1;
size_t n;
int sdTry;

SdFat sd;
SdFile file;
//File file;

sdTry=0;
Serial.print("Initializing SD card...");
while(!sd.begin(sdCS) && sdTry<10){
  sdTry++;
  delay(100);
}
if(sdTry==10){
  Serial.println("SD initialization failed. Halting...");
  //while(true); // halt bootup
  return(-1);
}
Serial.println("SD initialization done.");

Serial.println("Reading SD Card...");
if(!file.open("config.txt", O_READ)) {
   Serial.print("Can't open SD card config.txt file! Halting...");
   //while(true);
   return(-1);
   }

while( (n = file.fgets(buf, 100)) > 0) { // one line from file each loop
  
  p1 = strtok(buf, delim1); // first call to strtok returns pointer to buf with length 
                            // up to delim1
  strcpy(hdr1, p1);
  Serial.print(hdr1);
  Serial.print(": "); 
  
  switch(line){
    case 1:
    //while(p1 != NULL){
    for(int k=0; k<4; k++){
    p1 = strtok(NULL, delim2); // returns pointer starting after delim1 with length
                   // up to first delim2
    strcpy(s1, p1);
    ipOctet[k] = atoi(s1);
    Serial.print(ipOctet[k]);
    Serial.print('.');
    }
    break;

    case 2:
    //while(p1 != NULL){
    for(int k=0; k<6; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    //mac[k] = atoi(s1);
    mac[k] = byte(strtol(s1, &pEnd, 16));
    Serial.print(mac[k],HEX);
    Serial.print(' ');
    }
    break;


 
  } // end switch
    
Serial.println("");
line++;
}

file.close();
Serial.println("...Done Reading SD Card");
Serial.print("\n");

return(0);
}
*/


//unsigned long td1 = 30; // minimum ms delay needed after changing Delta PS page before query

float HKdata[60] = {};

char cmd[20];
unsigned long start;
unsigned long i = 0; // loop index
unsigned long i1 = 0; // loop index used for measuring loop timing
unsigned long i2 = 0; // loop index used for measuring loop timing
unsigned uptime_H; // uptime count high word
unsigned long uptime_L0; //uptime count low word previous loop 
unsigned long uptime_L1; // uptime count low word current loop
unsigned long uptime_s; // uptime in seconds
int j = 0;// i % 16 for ADCs
int k = 1; // 1 to 4 counter for Delta PS page changes
int n=0;
int resetFlag;
int flipclk=0;


//status 
int ovc_CH1 = 0; // overcurrent
int ovc_CH2 = 0;
int ovc_CH3 = 0;
int ovc_CH4 = 0;
//int hall_mismatch_CH1 = 0;
//int hall_mismatch_CH2 = 0;
int ovt1 = 0; // over temperature
int ovt2 = 0;
int sumfault_CH1 = 0; // summary fault
int sumfault_CH2 = 0;
int sumfault_CH3 = 0;
int sumfault_CH4 = 0;

int sts_mod[]={LOW,LOW,LOW,LOW};
int ON1cmdCH1[]={LOW,LOW};
int ON1cmdCH2[]={LOW,LOW};
int ON1cmdCH3[]={LOW,LOW};
int ON1cmdCH4[]={LOW,LOW};
int CH1_ON[]={0,0}; // CH1 state
int CH2_ON[]={0,0}; // CH2 state
int CH3_ON[]={0,0}; // CH3 state
int CH4_ON[]={0,0}; // CH4 state
int ON2cmdCH1;
int ON2cmdCH2;
int ON2cmdCH3;
int ON2cmdCH4;
int heartbeat=1;

int pducmdCH1[]={LOW,LOW};
int pducmdCH2[]={LOW,LOW};
int pducmdCH3[]={LOW,LOW};
int pducmdCH4[]={LOW,LOW};
int PDU1_ON[]={0,0}; // PDU CH1 state
int PDU2_ON[]={0,0}; // PDU CH2 state
int PDU3_ON[]={0,0}; // PDU CH3 state
int PDU4_ON[]={0,0}; // PDU CH4 state



//Channels 0-3 and 8-11 are Hall current sensors. Channels 4 and 12 are Vout. 5 and 13 are Vcap.
//Channels 6,7, 14, and 15 are temperature sensors.
//int chan_list[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
//int chan_list[] = {A0, A1, A2, A3, A8, A9, A10, A11}; // Bridge 1, 2, 3 ,4
//int ADCs[8];
//float ADC_[8];
//float I_hall[8];
//float I_hall_avg[8];
//float I_hall_buf[8][10]; // running average buffer, 8 channels, up to 10 avgs per channel
//float I_hall_buf1[8][2]; // buffer for IIR filter
//int tmr1[4] = {0, 0, 0, 0};
//int tmr2[4] = {0, 0, 0, 0};
int tmr1; // channel 1 OVC fault delay
int tmr2; // channel 2 OVC fault delay 
int tmr3; // channel 3 OVC fault delay
int tmr4; // channel 4 OVC fault delay
int tmr_ON1fltCH1=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1fltCH2=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1fltCH3=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1fltCH4=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1CH1turnOFF=15; //countdown when ON1 pulses go away
int tmr_ON1CH2turnOFF=15; 
int tmr_ON1CH3turnOFF=15;
int tmr_ON1CH4turnOFF=15;
int tmr_ON1CH1turnON=0; //count up when ON1 pulses start
int tmr_ON1CH2turnON=0;
int tmr_ON1CH3turnON=0; 
int tmr_ON1CH4turnON=0;

int tmr_pduCH1turnOFF=15; //countdown when ON1 pulses go away
int tmr_pduCH2turnOFF=15; 
int tmr_pduCH3turnOFF=15;
int tmr_pduCH4turnOFF=15;
int tmr_pduCH1turnON=0; //count up when ON1 pulses start
int tmr_pduCH2turnON=0;
int tmr_pduCH3turnON=0; 
int tmr_pduCH4turnON=0;


//int tmr_ON1 = 0; // delay timers for deglitching on/off command
//int tmr_OFF1 = 0;
//int tmr_ON2 = 0;
//int tmr_OFF2 = 0;
//int tmr_ON3 = 0; // delay timers for deglitching on/off command
//int tmr_OFF3 = 0;
//int tmr_ON4 = 0;
//int tmr_OFF4 = 0;


//unsigned long fan_ctrl; // parameter for computing fan pwm

//int N = 256;
//int ctr_buf = 0;
int pwm6;
int pwm7;
int pwm8;
uint32_t PSMODSTAT;
uint32_t PSFLTSTAT;
unsigned long tk=0; // time in milliseconds updated once each loop
//unsigned long tk1=0;
//unsigned long tk2=0;
//unsigned long tk3=0;
//unsigned long clk1 = 0; // updates every ? s for housekeeping comm
//unsigned long clk2 = 0; // updates every ? s for circular buffer
int HBrate = 1000; // HB rate = 1000 ms if SD card reads good
unsigned long tk_HBrate = 0; // millis() tick value updated at rate set by HBrate
unsigned long tk_halfsec = 0; // millis() tick value updated every 0.5 s
unsigned long tk_1s = 0; // millis() tick value updated every 1 s
unsigned long tk_2s = 0; // millis() tick value updated every 2 s
unsigned long tk_onCH1 = 0; // millis() value at Channel 1 turn-on
unsigned long tk_onCH2 = 0; // millis() value at Channel 2 turn-on
unsigned long tk_onCH3 = 0; // millis() value at Channel 3 turn-on
unsigned long tk_onCH4 = 0; // millis() value at Channel 4 turn-on
//unsigned long clk6 = 0; // updates every ? s to stop and re-begin UDP
//unsigned long clk7 = 0; // updates every ? s to get loop index
//unsigned long clk8 = 0; // updates every ? s for web server
//unsigned long clk9 = 0; // updates every ? s for Temp sensors for fan control
//unsigned long clk10 = 0; // updates every ? s for fast heartbeat
//unsigned long clk11 = 0; // updates every second for uptime computation
uint16_t ctr_packet=0;
int turning_onCH1;
int turning_onCH2;
int turning_onCH3;
int turning_onCH4;
int changingpage = LOW;
int SDfail = 0;
int ON1faultCH1 = 0;
int ON1faultCH2 = 0;
int ON1faultCH3 = 0;
int ON1faultCH4 = 0;
int OWconverting = LOW;
int flag1=0;

int mode=0; // mode=0 for BPC, mode=1 for UPC

uint8_t msb;
uint8_t lsb;
uint16_t adc_data[4];
int adc_k=0;

uint16_t ignd_data;
uint8_t ignd_msb;
uint8_t ignd_lsb;

uint8_t fault_byte=0;

uint8_t vgain;
uint8_t igain;

uint32_t caldac_data;
uint8_t caldac_byte2;
uint8_t caldac_byte1;
uint8_t caldac_byte0;


//Interlock limits
//float OVC_limits[8] = {-13.0, 13.0, -13.0, 13.0,-13.0, 13.0, -13.0, 13.0}; //CH1low,CH1high,CH2low,CH2high...
//float OVT_limits[2] = {67.0, 67.0};// Heatsink 1 and 2

// pin assignments
int pin_on1_cmd_CH1 = 12; // ON1 input from PSC
int pin_reset_cmd_CH1 = 10; // input from PSC
int pin_spare_cmd_CH1 = 30; 
int pin_on2_cmd_CH1 = 11;
int pin_on_sts_CH1 = 31; // output to PSC
int pin_flt1_sts_CH1 = 32; // output to PSC
int pin_flt2_sts_CH1 = 33; // output to PSC
int pin_spare_sts_CH1 = 35; 

int pin_on1_cmd_CH2 = 36; // ON1 input from PSC
int pin_reset_cmd_CH2 = 38; // input from PSC
int pin_spare_cmd_CH2 = 79;
int pin_on2_cmd_CH2 = 37;
int pin_on_sts_CH2 = 78; // output to PSC
int pin_flt1_sts_CH2 = 77; // output to PSC
int pin_flt2_sts_CH2 = 18; // output to PSC
int pin_spare_sts_CH2 = 19;

int pin_on1_cmd_CH3 = 72; // ON1 input from PSC
int pin_reset_cmd_CH3 = 3; // input from PSC
int pin_spare_cmd_CH3 = 2;
int pin_on2_cmd_CH3 = 71;
int pin_on_sts_CH3 = 5; // output to PSC
int pin_flt1_sts_CH3 = 70; // output to PSC
int pin_flt2_sts_CH3 = 4; // output to PSC
int pin_spare_sts_CH3 = 76; 

int pin_on1_cmd_CH4 = 75; // ON1 input from PSC
int pin_reset_cmd_CH4 = 40; // input from PSC
int pin_spare_cmd_CH4 = 41;
int pin_on2_cmd_CH4 = 39;
int pin_on_sts_CH4 = 17; // output to PSC
int pin_flt1_sts_CH4 = 6; // output to PSC
int pin_flt2_sts_CH4 = 7; // output to PSC
int pin_spare_sts_CH4 = 8;

int pin_HB = 48;

int pin_dcctflt_sel0 = 29;
int pin_dcctflt_sel1 = 28;
int pin_ignd_dac_cs = 27;
int pin_adc_cs = 26;
int pin_on_ch1 = 25;
int pin_on_ch2 = 24;
int pin_on_ch3 = 23;
int pin_on_ch4 = 22;
int pin_on_cal = 13;

int pin_cnvt = 73;
int pin_test_cal_ch1 = 9;
int pin_test_cal_ch2 = 74;
int pin_test_cal_ch3 = 81;
int pin_test_cal_ch4 = 82;

int pin_rst = 15;
int pin_ical_dac_cs = 14;
int pin_vi_cs_ch1 = 83;
int pin_vi_cs_ch2 = 84;
int pin_vi_cs_ch3 = 49;
int pin_vi_cs_ch4 = 47;

int pin_ignd_sel0 = 46;
int pin_ignd_sel1 = 45;

int pin_adc_mux_sel0 = 44;
int pin_adc_mux_sel1 = 43;

int pin_ldac = 42;

int onfault1 = 0;
int onfault2 = 0;
int onfault3 = 0;
int onfault4 = 0;

int flag=0;
int flag2 = 0; 

int byte_i = 0; 


SPISettings settingsSlow(500000, MSBFIRST, SPI_MODE0); // slow speed for Imon digipot

void setup() {
// put your setup code here, to run once:

pinMode(pin_on1_cmd_CH1, INPUT);
pinMode(pin_reset_cmd_CH1, INPUT);
pinMode(pin_spare_cmd_CH1, INPUT);
pinMode(pin_on2_cmd_CH1, INPUT);
pinMode(pin_on_sts_CH1, OUTPUT);
pinMode(pin_flt1_sts_CH1, OUTPUT);
pinMode(pin_flt2_sts_CH1, OUTPUT); 
pinMode(pin_spare_sts_CH1, OUTPUT);

pinMode(pin_on1_cmd_CH2, INPUT);
pinMode(pin_reset_cmd_CH2, INPUT);
pinMode(pin_spare_cmd_CH2, INPUT);
pinMode(pin_on2_cmd_CH2, INPUT);
pinMode(pin_on_sts_CH2, OUTPUT);
pinMode(pin_flt1_sts_CH2, OUTPUT);
pinMode(pin_flt2_sts_CH2, OUTPUT);
pinMode(pin_spare_sts_CH2, OUTPUT);

pinMode(pin_on1_cmd_CH3, INPUT);
pinMode(pin_reset_cmd_CH3, INPUT);
pinMode(pin_spare_cmd_CH3, INPUT);
pinMode(pin_on2_cmd_CH3, INPUT);
pinMode(pin_on_sts_CH3, OUTPUT);
pinMode(pin_flt1_sts_CH3, OUTPUT);
pinMode(pin_flt2_sts_CH3, OUTPUT); 
pinMode(pin_spare_sts_CH3, OUTPUT);

pinMode(pin_on1_cmd_CH4, INPUT);
pinMode(pin_reset_cmd_CH4, INPUT);
pinMode(pin_spare_cmd_CH4, INPUT);
pinMode(pin_on2_cmd_CH4, INPUT);
pinMode(pin_on_sts_CH4, OUTPUT);
pinMode(pin_flt1_sts_CH4, OUTPUT);
pinMode(pin_flt2_sts_CH4, OUTPUT);
pinMode(pin_spare_sts_CH4, OUTPUT);

pinMode(A8, OUTPUT); // dcctflt_en
pinMode(pin_dcctflt_sel0, OUTPUT);
pinMode(pin_dcctflt_sel1, OUTPUT);
pinMode(pin_ignd_dac_cs, OUTPUT);
pinMode(pin_adc_cs, OUTPUT);
pinMode(pin_on_ch1, OUTPUT); // turns on +/15V to ATEW channel
pinMode(pin_on_ch2, OUTPUT);
pinMode(pin_on_ch3, OUTPUT);
pinMode(pin_on_ch4, OUTPUT);
pinMode(pin_on_cal, OUTPUT);

pinMode(pin_cnvt, OUTPUT);
pinMode(pin_test_cal_ch1, OUTPUT);
pinMode(pin_test_cal_ch2, OUTPUT);
pinMode(pin_test_cal_ch3, OUTPUT);
pinMode(pin_test_cal_ch4, OUTPUT);

//pinMode(pin_rst, OUTPUT);
pinMode(pin_ical_dac_cs, OUTPUT);
pinMode(pin_vi_cs_ch1, OUTPUT);
pinMode(pin_vi_cs_ch2, OUTPUT);
pinMode(pin_vi_cs_ch3, OUTPUT);
pinMode(pin_vi_cs_ch4, OUTPUT);

pinMode(pin_ignd_sel0, OUTPUT);
pinMode(pin_ignd_sel1, OUTPUT);

pinMode(pin_adc_mux_sel0, OUTPUT);
pinMode(pin_adc_mux_sel1, OUTPUT);

pinMode(pin_ldac, OUTPUT);

pinMode(pin_HB, OUTPUT);


digitalWrite(pin_test_cal_ch1, HIGH);
digitalWrite(pin_ignd_dac_cs, HIGH);
digitalWrite(pin_adc_cs, HIGH);
digitalWrite(pin_ical_dac_cs, HIGH);
digitalWrite(pin_vi_cs_ch1, HIGH);
digitalWrite(pin_vi_cs_ch2, HIGH);
digitalWrite(pin_vi_cs_ch3, HIGH);
digitalWrite(pin_vi_cs_ch4, HIGH);


//Serial.begin(9600);
Serial.begin(115200);

Serial.println("");
Serial.println("");
Serial.println("PSC ATE");
Serial.println("Firmware build: ");
Serial.println(buildnumber, 0);
Serial.println("");

delay(200);

/*
if( readSD() ) // get MAC, IP address, 1-wire IDs, and Hall coefficients. If fail, sumfault and fast heartbeat
{ 
  SDfail = 1;
  HBrate = 200;
}
*/

//Start the Ethernet and UDP
//IPAddress ip(ipOctet[0],ipOctet[1],ipOctet[2],ipOctet[3]); // 
//IPAddress ip(192,168,1,3); 
Ethernet.begin(mac,ip);
//Udp.begin(port);
//server.begin(); // web server

digitalWrite(pin_HB, heartbeat);

//Serial.print("IP Address: ");
//Serial.println(Ethernet.localIP());
//Serial.println("");


resetFlag=1;
uptime_H=0;
uptime_L0=0;
uptime_L1=0;

//wdt_enable(WDTO_2S);

turning_onCH1 = LOW;
turning_onCH2 = LOW;
turning_onCH3 = LOW;
turning_onCH4 = LOW;

digitalWrite(pin_spare_sts_CH1, HIGH);
digitalWrite(pin_spare_sts_CH2, HIGH);
digitalWrite(pin_spare_sts_CH3, HIGH);
digitalWrite(pin_spare_sts_CH4, HIGH);
digitalWrite(pin_test_cal_ch1, HIGH);
digitalWrite(pin_test_cal_ch2, HIGH);
digitalWrite(pin_test_cal_ch3, HIGH);
digitalWrite(pin_test_cal_ch4, HIGH);

SPI.begin();
SPI.setDataMode(SPI_MODE0);

flag=0;

} // end setup()



void loop() {
// put your main code here, to run repeatedly:
//tk1 = tk;
tk = millis();
Udp.begin(port);
//delay(1); // make loop rate ~500 loops/s

if(millis()>4000 && !flag){
  flag=1;
  //IPAddress ip(ipOctet[0],ipOctet[1],ipOctet[2],ipOctet[3]);
  //IPAddress ip1(192,168,1,3);
  //byte mac1[] = {0x02, 0x01, 0x01, 0x01, 0x01, 0x03};
  //Ethernet.begin(mac1,ip1);
  Serial.println(Ethernet.hardwareStatus());
  Serial.println(Ethernet.linkStatus());
}


flag2 = !flag2;
//digitalWrite(pin_spare_sts_CH1, flag2);


//digitalWrite(A12, pulse); // output bit toggles every loop (TP1)
//pulse = !pulse;

if (tk>5000) resetFlag=0; // clear flag 5 s after restart

//heartbeat pulse
if ( tk - tk_HBrate > HBrate ) {
//wdt_reset();
if (heartbeat == LOW) heartbeat = HIGH;
else
heartbeat = LOW;
//if(flt2_CH1 || flt2_CH2 || flt2_CH3 || flt2_CH4) heartbeat = HIGH;
digitalWrite(pin_HB, heartbeat);

//digitalWrite(pin_L, heartbeat);
tk_HBrate = tk;
}

/*
if (tk - clk10 > 200){
  HBfast = !HBfast;
  digitalWrite(pin_flt2_sts_CH1, HBfast);
  digitalWrite(pin_flt2_sts_CH2, HBfast);
  clk10 = tk;
}
*/

// get ADC data
digitalWrite(pin_cnvt, HIGH);
delay(1);
digitalWrite(pin_adc_cs, LOW);
msb = SPI.transfer(0);
lsb = SPI.transfer(0);
adc_data[adc_k] = msb*256+lsb;
digitalWrite(pin_adc_cs, HIGH);
digitalWrite(pin_cnvt, LOW);
//Serial.println((uint16_t)adc_data[adc_k], HEX);
adc_k++;
if(adc_k>=4) adc_k=0;
if(adc_k==0){
  digitalWrite(pin_adc_mux_sel0, LOW);
  digitalWrite(pin_adc_mux_sel1, LOW);
}
if(adc_k==1){
  digitalWrite(pin_adc_mux_sel0, HIGH);
  digitalWrite(pin_adc_mux_sel1, LOW);
}
if(adc_k==2){
  digitalWrite(pin_adc_mux_sel0, LOW);
  digitalWrite(pin_adc_mux_sel1, HIGH);
}
if(adc_k==3){
  digitalWrite(pin_adc_mux_sel0, HIGH);
  digitalWrite(pin_adc_mux_sel1, HIGH);
}






//If power converter is turned off by command or by fault, disable HBridges and DC supply. 
//Since Hbridges turn off first, magnet current will freewheel through DC supply acting as 
//a sink until the DC supply turns off. 
//Then remaining energy will be either given to cap or dissipated in TVS if cap voltage
//goes too high. 
//ON fault happens if DC supplies fail to turn on when ON1 command changes from OFF to ON or
//if DC supply is on when ON1 is in the OFF state.
//If DC supply status readbacks go low while power converter is on, power converter should
//take no action because it might be able to ride through power dip. PSC should turn power 
//converter off if current loop error goes out of bounds. Power converter needs to be able 
//to tolerate discharge/dynamic currents in cap during AC voltage disturbances.

// If ON1 command detected (rising edge, pulse train, etc) and no fault...
// unpark and pwm enable are active low
//turn on DC supplies before enabling PWM so PWM electronics isn't coming up with PWM enabled.


if (mode == 0){ // ON2 enables/inhibits BPC

ON1cmdCH1[0] = ON1cmdCH1[1];
ON1cmdCH2[0] = ON1cmdCH2[1];
ON1cmdCH3[0] = ON1cmdCH3[1];
ON1cmdCH4[0] = ON1cmdCH4[1];
ON1cmdCH1[1] = digitalRead(pin_on1_cmd_CH1);
ON1cmdCH2[1] = digitalRead(pin_on1_cmd_CH2);
ON1cmdCH3[1] = digitalRead(pin_on1_cmd_CH3);
ON1cmdCH4[1] = digitalRead(pin_on1_cmd_CH4);
ON2cmdCH1 = digitalRead(pin_on2_cmd_CH1);
ON2cmdCH2 = digitalRead(pin_on2_cmd_CH2);
ON2cmdCH3 = digitalRead(pin_on2_cmd_CH3);
ON2cmdCH4 = digitalRead(pin_on2_cmd_CH4);

CH1_ON[0] = CH1_ON[1];
if(tmr_ON1CH1turnOFF != 0) tmr_ON1CH1turnOFF--; //decrement each loop until zero
if(tmr_ON1CH1turnOFF == 0) CH1_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH ||
   ON1cmdCH1[0]==HIGH && ON1cmdCH1[1]==LOW && ON2cmdCH1) tmr_ON1CH1turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH && tmr_ON1CH1turnON < 5) tmr_ON1CH1turnON++;
if(tmr_ON1CH1turnON == 5) CH1_ON[1] = 1;
if(CH1_ON[1]) tmr_ON1CH1turnON = 0;

CH2_ON[0] = CH2_ON[1];
if(tmr_ON1CH2turnOFF != 0) tmr_ON1CH2turnOFF--; //decrement each loop until zero
if(tmr_ON1CH2turnOFF == 0) CH2_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH ||
   ON1cmdCH2[0]==HIGH && ON1cmdCH2[1]==LOW && ON2cmdCH2) tmr_ON1CH2turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH && tmr_ON1CH2turnON < 5) tmr_ON1CH2turnON++;
if(tmr_ON1CH2turnON == 5) CH2_ON[1] = 1;
if(CH2_ON[1]) tmr_ON1CH2turnON = 0;

CH3_ON[0] = CH3_ON[1];
if(tmr_ON1CH3turnOFF != 0) tmr_ON1CH3turnOFF--; //decrement each loop until zero
if(tmr_ON1CH3turnOFF == 0) CH3_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH3[0]==LOW && ON1cmdCH3[1]==HIGH ||
   ON1cmdCH3[0]==HIGH && ON1cmdCH3[1]==LOW && ON2cmdCH3) tmr_ON1CH3turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH3[0]==LOW && ON1cmdCH3[1]==HIGH && tmr_ON1CH3turnON < 5) tmr_ON1CH3turnON++;
if(tmr_ON1CH3turnON == 5) CH3_ON[1] = 1;
if(CH3_ON[1]) tmr_ON1CH3turnON = 0;

CH4_ON[0] = CH4_ON[1];
if(tmr_ON1CH4turnOFF != 0) tmr_ON1CH4turnOFF--; //decrement each loop until zero
if(tmr_ON1CH4turnOFF == 0) CH4_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH4[0]==LOW && ON1cmdCH4[1]==HIGH ||
   ON1cmdCH4[0]==HIGH && ON1cmdCH4[1]==LOW && ON2cmdCH4) tmr_ON1CH4turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH4[0]==LOW && ON1cmdCH4[1]==HIGH && tmr_ON1CH4turnON < 5) tmr_ON1CH4turnON++;
if(tmr_ON1CH4turnON == 5) CH4_ON[1] = 1;
if(CH4_ON[1]) tmr_ON1CH4turnON = 0;


if(CH1_ON[1]==0 || sumfault_CH1) CH1_ON[1] = 0;
if(CH2_ON[1]==0 || sumfault_CH2) CH2_ON[1] = 0;
if(CH3_ON[1]==0 || sumfault_CH3) CH3_ON[1] = 0;
if(CH4_ON[1]==0 || sumfault_CH4) CH4_ON[1] = 0;

//write output status bits
if(CH1_ON[1] && !sumfault_CH1) digitalWrite(pin_on_sts_CH1, HIGH);
if(!CH1_ON[1] || sumfault_CH1) digitalWrite(pin_on_sts_CH1, LOW);

if(CH2_ON[1] && !sumfault_CH2) digitalWrite(pin_on_sts_CH2, HIGH);
if(!CH2_ON[1] || sumfault_CH2) digitalWrite(pin_on_sts_CH2, LOW);

if(CH3_ON[1] && !sumfault_CH3) digitalWrite(pin_on_sts_CH3, HIGH);
if(!CH3_ON[1] || sumfault_CH3) digitalWrite(pin_on_sts_CH3, LOW); 

if(CH4_ON[1] && !sumfault_CH4) digitalWrite(pin_on_sts_CH4, HIGH);
if(!CH4_ON[1] || sumfault_CH4) digitalWrite(pin_on_sts_CH4, LOW);

digitalWrite(pin_flt1_sts_CH1, sumfault_CH1);
digitalWrite(pin_flt1_sts_CH2, sumfault_CH2);
digitalWrite(pin_flt1_sts_CH3, sumfault_CH3);
digitalWrite(pin_flt1_sts_CH4, sumfault_CH4);

digitalWrite(pin_flt2_sts_CH1, heartbeat);
digitalWrite(pin_flt2_sts_CH2, heartbeat);
digitalWrite(pin_flt2_sts_CH3, heartbeat);
digitalWrite(pin_flt2_sts_CH4, heartbeat);

// only reset fault if ON command is de-asserted
if (digitalRead(pin_reset_cmd_CH1) == HIGH && tmr_ON1CH1turnOFF == 0) {
  ovc_CH1 = 0;
  ovt1 = 0;
  ON1faultCH1=0;
  tmr_ON1fltCH1=5000;
}

if (digitalRead(pin_reset_cmd_CH2) == HIGH && tmr_ON1CH2turnOFF == 0) {
  ovc_CH2 = 0;
  ovt1 = 0;
  ON1faultCH2=0;
  tmr_ON1fltCH2=5000;
 }


if (digitalRead(pin_reset_cmd_CH3) == HIGH && tmr_ON1CH3turnOFF == 0) {
  ovc_CH3 = 0;
  ovt2 = 0;
  ON1faultCH3=0;
  tmr_ON1fltCH3=5000;
}

if (digitalRead(pin_reset_cmd_CH4) == HIGH && tmr_ON1CH4turnOFF == 0) {
  ovc_CH4 = 0;
  ovt2 = 0;
  ON1faultCH4=0;
  tmr_ON1fltCH4=5000;
 }



}




if (mode == 1){ // UPC

ON1cmdCH1[0] = ON1cmdCH1[1];
ON1cmdCH2[0] = ON1cmdCH2[1];
ON1cmdCH3[0] = ON1cmdCH3[1];
ON1cmdCH4[0] = ON1cmdCH4[1];
ON1cmdCH1[1] = digitalRead(pin_on1_cmd_CH1);
ON1cmdCH2[1] = digitalRead(pin_on1_cmd_CH2);
ON1cmdCH3[1] = digitalRead(pin_on1_cmd_CH3);
ON1cmdCH4[1] = digitalRead(pin_on1_cmd_CH4);

  
CH1_ON[0] = CH1_ON[1];
if(tmr_ON1CH1turnOFF != 0) tmr_ON1CH1turnOFF--; //decrement each loop until zero
if(tmr_ON1CH1turnOFF == 0) CH1_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH ||
   ON1cmdCH1[0]==HIGH && ON1cmdCH1[1]==LOW) tmr_ON1CH1turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH && tmr_ON1CH1turnON < 5) tmr_ON1CH1turnON++;
if(tmr_ON1CH1turnON == 5) CH1_ON[1] = 1;
if(CH1_ON[1]) tmr_ON1CH1turnON = 0;

CH2_ON[0] = CH2_ON[1];
if(tmr_ON1CH2turnOFF != 0) tmr_ON1CH2turnOFF--; //decrement each loop until zero
if(tmr_ON1CH2turnOFF == 0) CH2_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH ||
   ON1cmdCH2[0]==HIGH && ON1cmdCH2[1]==LOW) tmr_ON1CH2turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH && tmr_ON1CH2turnON < 5) tmr_ON1CH2turnON++;
if(tmr_ON1CH2turnON == 5) CH2_ON[1] = 1;
if(CH2_ON[1]) tmr_ON1CH2turnON = 0;

CH3_ON[0] = CH3_ON[1];
if(tmr_ON1CH3turnOFF != 0) tmr_ON1CH3turnOFF--; //decrement each loop until zero
if(tmr_ON1CH3turnOFF == 0) CH3_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH3[0]==LOW && ON1cmdCH3[1]==HIGH ||
   ON1cmdCH3[0]==HIGH && ON1cmdCH3[1]==LOW) tmr_ON1CH3turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH3[0]==LOW && ON1cmdCH3[1]==HIGH && tmr_ON1CH3turnON < 5) tmr_ON1CH3turnON++;
if(tmr_ON1CH3turnON == 5) CH3_ON[1] = 1;
if(CH3_ON[1]) tmr_ON1CH3turnON = 0;

CH4_ON[0] = CH4_ON[1];
if(tmr_ON1CH4turnOFF != 0) tmr_ON1CH4turnOFF--; //decrement each loop until zero
if(tmr_ON1CH4turnOFF == 0) CH4_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH4[0]==LOW && ON1cmdCH4[1]==HIGH ||
   ON1cmdCH4[0]==HIGH && ON1cmdCH4[1]==LOW) tmr_ON1CH4turnOFF = 15; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH4[0]==LOW && ON1cmdCH4[1]==HIGH && tmr_ON1CH4turnON < 5) tmr_ON1CH4turnON++;
if(tmr_ON1CH4turnON == 5) CH4_ON[1] = 1;
if(CH4_ON[1]) tmr_ON1CH4turnON = 0;


pducmdCH1[0] = pducmdCH1[1];
pducmdCH2[0] = pducmdCH2[1];
pducmdCH3[0] = pducmdCH3[1];
pducmdCH4[0] = pducmdCH4[1];
pducmdCH1[1] = digitalRead(pin_on2_cmd_CH1);
pducmdCH2[1] = digitalRead(pin_on2_cmd_CH2);
pducmdCH3[1] = digitalRead(pin_on2_cmd_CH3);
pducmdCH4[1] = digitalRead(pin_on2_cmd_CH4);

PDU1_ON[0] = PDU1_ON[1];
if(tmr_pduCH1turnOFF != 0) tmr_pduCH1turnOFF--; //decrement each loop until zero
if(tmr_pduCH1turnOFF == 0) PDU1_ON[1] = 0; // if counter reaches zero (no pdu pulses), turn off
if(pducmdCH1[0]==LOW && pducmdCH1[1]==HIGH ||
   pducmdCH1[0]==HIGH && pducmdCH1[1]==LOW) tmr_pduCH1turnOFF = 15; // if pdu pulse train detected, reset to 40
if(pducmdCH1[0]==LOW && pducmdCH1[1]==HIGH && tmr_pduCH1turnON < 5) tmr_pduCH1turnON++;
if(tmr_pduCH1turnON == 5) PDU1_ON[1] = 1;
if(PDU1_ON[1]) tmr_pduCH1turnON = 0;

PDU2_ON[0] = PDU2_ON[1];
if(tmr_pduCH2turnOFF != 0) tmr_pduCH2turnOFF--; //decrement each loop until zero
if(tmr_pduCH2turnOFF == 0) PDU2_ON[1] = 0; // if counter reaches zero (no pdu pulses), turn off
if(pducmdCH2[0]==LOW && pducmdCH2[1]==HIGH ||
   pducmdCH2[0]==HIGH && pducmdCH2[1]==LOW) tmr_pduCH2turnOFF = 15; // if pdu pulse train detected, reset to 40
if(pducmdCH2[0]==LOW && pducmdCH2[1]==HIGH && tmr_pduCH2turnON < 5) tmr_pduCH2turnON++;
if(tmr_pduCH2turnON == 5) PDU2_ON[1] = 1;
if(PDU2_ON[1]) tmr_pduCH2turnON = 0;

PDU3_ON[0] = PDU3_ON[1];
if(tmr_pduCH3turnOFF != 0) tmr_pduCH3turnOFF--; //decrement each loop until zero
if(tmr_pduCH3turnOFF == 0) PDU3_ON[1] = 0; // if counter reaches zero (no pdu pulses), turn off
if(pducmdCH3[0]==LOW && pducmdCH3[1]==HIGH ||
   pducmdCH3[0]==HIGH && pducmdCH3[1]==LOW) tmr_pduCH3turnOFF = 15; // if pdu pulse train detected, reset to 40
if(pducmdCH3[0]==LOW && pducmdCH3[1]==HIGH && tmr_pduCH3turnON < 5) tmr_pduCH3turnON++;
if(tmr_pduCH3turnON == 5) PDU3_ON[1] = 1;
if(PDU3_ON[1]) tmr_pduCH3turnON = 0;

PDU4_ON[0] = PDU4_ON[1];
if(tmr_pduCH4turnOFF != 0) tmr_pduCH4turnOFF--; //decrement each loop until zero
if(tmr_pduCH4turnOFF == 0) PDU4_ON[1] = 0; // if counter reaches zero (no pdu pulses), turn off
if(pducmdCH4[0]==LOW && pducmdCH4[1]==HIGH ||
   pducmdCH4[0]==HIGH && pducmdCH4[1]==LOW) tmr_pduCH4turnOFF = 15; // if pdu pulse train detected, reset to 40
if(pducmdCH4[0]==LOW && pducmdCH4[1]==HIGH && tmr_pduCH4turnON < 5) tmr_pduCH4turnON++;
if(tmr_pduCH4turnON == 5) PDU4_ON[1] = 1;
if(PDU4_ON[1]) tmr_pduCH4turnON = 0;

if (PDU1_ON[0] == LOW && PDU1_ON[1] == HIGH){
  digitalWrite(pin_flt2_sts_CH1, HIGH); // PDU AC ON
  delay(100);
  digitalWrite(pin_on_sts_CH1, HIGH); // TDK/Lambda AC ON
}

if (PDU2_ON[0] == LOW && PDU2_ON[1] == HIGH){
  digitalWrite(pin_flt2_sts_CH2, HIGH);
  delay(100);
  digitalWrite(pin_on_sts_CH2, HIGH);
}

if (PDU3_ON[0] == LOW && PDU3_ON[1] == HIGH){
  digitalWrite(pin_flt2_sts_CH3, HIGH);
  delay(100);
  digitalWrite(pin_on_sts_CH3, HIGH);
}

if (PDU4_ON[0] == LOW && PDU4_ON[1] == HIGH){
  digitalWrite(pin_flt2_sts_CH4, HIGH);
  delay(100);
  digitalWrite(pin_on_sts_CH4, HIGH);
}

if (PDU1_ON[0] == HIGH && PDU1_ON[1] == LOW){
  digitalWrite(pin_flt2_sts_CH1, LOW); // PDU AC ON
  delay(100);
  digitalWrite(pin_on_sts_CH1, LOW); // TDK/Lambda AC ON
}

if (PDU2_ON[0] == HIGH && PDU2_ON[1] == LOW){
  digitalWrite(pin_flt2_sts_CH2, LOW);
  delay(100);
  digitalWrite(pin_on_sts_CH2, LOW);
}

if (PDU3_ON[0] == HIGH && PDU3_ON[1] == LOW){
  digitalWrite(pin_flt2_sts_CH3, LOW);
  delay(100);
  digitalWrite(pin_on_sts_CH3, LOW);
}

if (PDU4_ON[0] == HIGH && PDU4_ON[1] == LOW){
  digitalWrite(pin_flt2_sts_CH4, LOW);
  delay(100);
  digitalWrite(pin_on_sts_CH4, LOW);
}


if(CH1_ON[1]==0 || sumfault_CH1) CH1_ON[1] = 0;
if(CH2_ON[1]==0 || sumfault_CH2) CH2_ON[1] = 0;
if(CH3_ON[1]==0 || sumfault_CH3) CH3_ON[1] = 0;
if(CH4_ON[1]==0 || sumfault_CH4) CH4_ON[1] = 0;

if(CH1_ON[1] && !sumfault_CH1) digitalWrite(pin_flt1_sts_CH1, HIGH);
if(!CH1_ON[1] || sumfault_CH1) digitalWrite(pin_flt1_sts_CH1, LOW);

if(CH2_ON[1] && !sumfault_CH2) digitalWrite(pin_flt1_sts_CH2, HIGH);
if(!CH2_ON[1] || sumfault_CH2) digitalWrite(pin_flt1_sts_CH2, LOW);

if(CH3_ON[1] && !sumfault_CH3) digitalWrite(pin_flt1_sts_CH3, HIGH);
if(!CH3_ON[1] || sumfault_CH3) digitalWrite(pin_flt1_sts_CH3, LOW); 

if(CH4_ON[1] && !sumfault_CH4) digitalWrite(pin_flt1_sts_CH4, HIGH);
if(!CH4_ON[1] || sumfault_CH4) digitalWrite(pin_flt1_sts_CH4, LOW);


// reset fault
if (digitalRead(pin_reset_cmd_CH1) == HIGH) {
  ovc_CH1 = 0;
  ovt1 = 0;
  ON1faultCH1=0;
  tmr_ON1fltCH1=5000;
}

if (digitalRead(pin_reset_cmd_CH2) == HIGH) {
  ovc_CH2 = 0;
  ovt1 = 0;
  ON1faultCH2=0;
  tmr_ON1fltCH2=5000;
 }


if (digitalRead(pin_reset_cmd_CH3) == HIGH) {
  ovc_CH3 = 0;
  ovt2 = 0;
  ON1faultCH3=0;
  tmr_ON1fltCH3=5000;
}

if (digitalRead(pin_reset_cmd_CH4) == HIGH) {
  ovc_CH4 = 0;
  ovt2 = 0;
  ON1faultCH4=0;
  tmr_ON1fltCH4=5000;
 }

}



sumfault_CH1 = ovc_CH1 + ovt1 + SDfail + ON1faultCH1;
sumfault_CH2 = ovc_CH2 + ovt1 + SDfail + ON1faultCH2;
sumfault_CH3 = ovc_CH3 + ovt2 + SDfail + ON1faultCH3;
sumfault_CH4 = ovc_CH4 + ovt2 + SDfail + ON1faultCH4;
if(sumfault_CH1) sumfault_CH1=1;
if(sumfault_CH2) sumfault_CH2=1;
if(sumfault_CH3) sumfault_CH3=1;
if(sumfault_CH4) sumfault_CH4=1;


// 1 second stuff
if(tk - tk_1s > 1000){

// Uptime update
uptime_L0 = uptime_L1;
uptime_L1 = micros();
if (uptime_L1 < uptime_L0) uptime_H++; // if rollover, increment high word
uptime_s = (uptime_H*pow(2, 32) + uptime_L1)/1e6;

//get loop index for measuring loop rate
i1=i2;
i2 = i;
}
  
/*
//every 30 s
if(tk - clk9 > 30000){
  T1_fan[0] = T1_fan[1];// put temperature reading from 30 s ago into [0]
  T1_fan[1] = T1;// put current temperature reading into [1]
  T2_fan[0] = T2_fan[1];
  T2_fan[1] = T2;
  clk9 = tk;
}
*/

PSMODSTAT= sts_mod[0] + sts_mod[1]*2 + sts_mod[2]*4 + sts_mod[3]*8;
PSFLTSTAT= sumfault_CH1 + sumfault_CH2*2 + sumfault_CH3*4 + sumfault_CH4*8 +
           ovc_CH1*16 + ovc_CH2*32 + ovc_CH3*64 + ovc_CH4*128 + 
           ovt1*4096 + ovt2*8192 +
           SDfail*32768 + heartbeat*65536 + resetFlag*131072 + 
           ON1faultCH1*262144 + ON1faultCH2*524288 + ON1faultCH3*1048576 + ON1faultCH4*2097152;




HKdata[0] = 1000.0;
HKdata[1] = 5.1;
HKdata[2] = 5.2;
HKdata[3] = 5.1;
HKdata[4] = 5.0;
HKdata[5] = 5.0;
HKdata[6] = 5.2;
HKdata[7] = 5.1;
HKdata[8] = 5.1;
HKdata[9] = 0;
HKdata[10] = 0;
HKdata[11] = 0;
HKdata[12] = 0;
HKdata[13] = 0;
HKdata[14] = 0;
HKdata[15] = 0;
HKdata[16] = 0;
HKdata[17] = 0;

HKdata[44] = 24.1;
HKdata[45] = 24.2;

HKdata[48] = 53.0;
HKdata[49] = 54.0;

HKdata[52] = PSMODSTAT;
HKdata[53] = PSFLTSTAT;

HKdata[54] = mod_serial; // PS Model, S/N
HKdata[55] = buildnumber; // firmware version

HKdata[56] = i2-i1; // loops/s
HKdata[57] = ctr_packet;
HKdata[58] = uptime_s;
HKdata[59] = 1001.0;

/*
//Send Housekeeping Packet to Serial Port
if ( (tk - clk1) >= 200 ){
for (int i=0; i<86; i++){
Serial.print(HKdata[i]);
Serial.print(",");
}
Serial.print("\n");
clk1 = tk;
}
*/

//Send Housekeeping Packet to Ethernet with UDP protocol
int packetSize = Udp.parsePacket();

//if packet received
if(packetSize){
   Udp.read(packetBuffer, 100);
  //Serial.print(packetBuffer);
  
 // if message = Loop, send out housekeeping data
//packetBuffer[4] = '\0'; //terminate string after 4 characters to exclude UDP message padding
  
  if(strcmp(packetBuffer, "Loop")==0){
  //tk2=millis();

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort() );
  //recast array name HKdata from pointer to float array to pointer to char buffer
  Udp.write((char*)HKdata, sizeof(HKdata) );
  Udp.endPacket();
  //Serial.println(datagram);
  
  //tk3=millis();
  //HBfast = !HBfast;
  n++;
  ctr_packet++;
  }


//respond to ADC data query
if(strcmp(packetBuffer, "D15?\n")==0){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort() );
  //recast array name adc_data from pointer to float array to pointer to char buffer
  Udp.write((char*)adc_data, sizeof(adc_data) );
  Udp.endPacket();
  Serial.println("Received ADC query");
  Serial.println(adc_data[0], HEX);
  Serial.println(adc_data[1], HEX);
  Serial.println(adc_data[2], HEX);
  Serial.println(adc_data[3], HEX);
}

//DCCT faults
if( strcmp(packetBuffer, "D0\n")==0 ) digitalWrite(A8, LOW); // dcctflt_en
if( strcmp(packetBuffer, "D1\n")==0 ){
  digitalWrite(A8, HIGH); // dcctflt_en
  digitalWrite(pin_dcctflt_sel0, LOW);
  digitalWrite(pin_dcctflt_sel1, LOW);
  Serial.println("Setting DCCT Fault CH1");
}
if( strcmp(packetBuffer, "D2\n")==0 ){
  digitalWrite(A8, HIGH); // dcctflt_en
  digitalWrite(pin_dcctflt_sel0, HIGH);
  digitalWrite(pin_dcctflt_sel1, LOW);
  Serial.println("Setting DCCT Fault CH2");
}
if( strcmp(packetBuffer, "D3\n")==0 ){
  digitalWrite(A8, HIGH); // dcctflt_en
  digitalWrite(pin_dcctflt_sel0, LOW);
  digitalWrite(pin_dcctflt_sel1, HIGH);
  Serial.println("Setting DCCT Fault CH3");
}
if( strcmp(packetBuffer, "D4\n")==0 ){
  digitalWrite(A8, HIGH); // dcctflt_en
  digitalWrite(pin_dcctflt_sel0, HIGH);
  digitalWrite(pin_dcctflt_sel1, HIGH);
  Serial.println("Setting DCCT Fault CH4");
}

if( strncmp(packetBuffer, "Ignd", 4)==0){
  ignd_data = (uint16_t)( (atof(packetBuffer+5)*(-1)+5)/5*32767);
  ignd_msb = (uint8_t)(ignd_data>>8);
  ignd_lsb = (uint8_t)(ignd_data&0xff);
  digitalWrite(pin_ignd_dac_cs, LOW);
  digitalWrite(pin_ldac, LOW);
  SPI.transfer(ignd_msb);
  SPI.transfer(ignd_lsb);
  digitalWrite(pin_ignd_dac_cs, HIGH);
  digitalWrite(pin_ldac, HIGH);
  Serial.print("Setting Ignd DAC to ");
  Serial.println(ignd_data);
}

if( strncmp(packetBuffer, "Ignd1", 5)==0 ){
  digitalWrite(pin_ignd_sel0, LOW);
  digitalWrite(pin_ignd_sel1, LOW);
  Serial.println("Selecting Ignd DAC for CH1");
 }
if( strncmp(packetBuffer, "Ignd2", 5)==0 ){
  digitalWrite(pin_ignd_sel0, HIGH);
  digitalWrite(pin_ignd_sel1, LOW);
  Serial.println("Selecting Ignd DAC for CH2");
 }
if( strncmp(packetBuffer, "Ignd3", 5)==0 ){
  digitalWrite(pin_ignd_sel0, LOW);
  digitalWrite(pin_ignd_sel1, HIGH);
  Serial.println("Selecting Ignd DAC for CH3");
 }
if( strncmp(packetBuffer, "Ignd4", 5)==0 ){
  digitalWrite(pin_ignd_sel0, HIGH);
  digitalWrite(pin_ignd_sel1, HIGH);
  Serial.println("Selecting Ignd DAC for CH4");
 }

//set channel test/cal mode
if( !strcmp(packetBuffer, "T10\n")){
  digitalWrite(pin_test_cal_ch1, HIGH);
  Serial.println("CH1 Test mode");
}
if( !strcmp(packetBuffer, "T11\n")){
  digitalWrite(pin_test_cal_ch1, LOW);
  Serial.println("CH1 Cal mode");
}
if( !strcmp(packetBuffer, "T20\n")){
  digitalWrite(pin_test_cal_ch2, HIGH);
  Serial.println("CH2 Test mode");
}
if( !strcmp(packetBuffer, "T21\n")){
  digitalWrite(pin_test_cal_ch2, LOW);
  Serial.println("CH2 Cal mode");
}
if( !strcmp(packetBuffer, "T30\n")){
  digitalWrite(pin_test_cal_ch3, HIGH);
  Serial.println("CH3 Test mode");
}
if( !strcmp(packetBuffer, "T31\n")){
  digitalWrite(pin_test_cal_ch3, LOW);
  Serial.println("CH3 Cal mode");
}
if( !strcmp(packetBuffer, "T40\n")){
  digitalWrite(pin_test_cal_ch4, HIGH);
  Serial.println("CH4 Test mode");
}
if( !strcmp(packetBuffer, "T41\n")){
  digitalWrite(pin_test_cal_ch4, LOW);
  Serial.println("CH4 Cal mode");
}

//DI Fault Bits
if( !strncmp(packetBuffer, "DI1", 3) ){
fault_byte = (uint8_t)atoi(packetBuffer+3);
if (fault_byte == 1) digitalWrite(pin_on_sts_CH1, HIGH);
if (fault_byte == 11) digitalWrite(pin_flt1_sts_CH1, HIGH);
if (fault_byte == 21) digitalWrite(pin_flt2_sts_CH1, HIGH);
if (fault_byte == 31) digitalWrite(pin_spare_sts_CH1, HIGH);
if (fault_byte == 0) digitalWrite(pin_on_sts_CH1, LOW);
if (fault_byte == 10) digitalWrite(pin_flt1_sts_CH1, LOW);
if (fault_byte == 20) digitalWrite(pin_flt2_sts_CH1, LOW);
if (fault_byte == 30) digitalWrite(pin_spare_sts_CH1, LOW);
Serial.println("CH1 fault = ");
Serial.println(ovc_CH1);
//Serial.println(mps_CH1);
}

if( !strncmp(packetBuffer, "DI2", 3) ){
fault_byte = (uint8_t)atoi(packetBuffer+3);
if (fault_byte == 1) digitalWrite(pin_on_sts_CH2, HIGH);
if (fault_byte == 11) digitalWrite(pin_flt1_sts_CH2, HIGH);
if (fault_byte == 21) digitalWrite(pin_flt2_sts_CH2, HIGH);
if (fault_byte == 31) digitalWrite(pin_spare_sts_CH2, HIGH);
if (fault_byte == 0) digitalWrite(pin_on_sts_CH2, LOW);
if (fault_byte == 10) digitalWrite(pin_flt1_sts_CH2, LOW);
if (fault_byte == 20) digitalWrite(pin_flt2_sts_CH2, LOW);
if (fault_byte == 30) digitalWrite(pin_spare_sts_CH2, LOW);
Serial.println("CH2 fault = ");
Serial.println(ovc_CH2);
//Serial.println(mps_CH2);
}


if( !strncmp(packetBuffer, "DI3", 3) ){
fault_byte = (uint8_t)atoi(packetBuffer+3);
if (fault_byte == 1) digitalWrite(pin_on_sts_CH3, HIGH);
if (fault_byte == 11) digitalWrite(pin_flt1_sts_CH3, HIGH);
if (fault_byte == 21) digitalWrite(pin_flt2_sts_CH3, HIGH);
if (fault_byte == 31) digitalWrite(pin_spare_sts_CH3, HIGH);
if (fault_byte == 0) digitalWrite(pin_on_sts_CH3, LOW);
if (fault_byte == 10) digitalWrite(pin_flt1_sts_CH3, LOW);
if (fault_byte == 20) digitalWrite(pin_flt2_sts_CH3, LOW);
if (fault_byte == 30) digitalWrite(pin_spare_sts_CH3, LOW);
Serial.println("CH3 fault = ");
Serial.println(ovc_CH3);
//Serial.println(mps_CH3);
}

if( !strncmp(packetBuffer, "DI4", 3) ){
fault_byte = (uint8_t)atoi(packetBuffer+3);
if (fault_byte == 1) digitalWrite(pin_on_sts_CH4, HIGH);
if (fault_byte == 11) digitalWrite(pin_flt1_sts_CH4, HIGH);
if (fault_byte == 21) digitalWrite(pin_flt2_sts_CH4, HIGH);
if (fault_byte == 31) digitalWrite(pin_spare_sts_CH4, HIGH);
if (fault_byte == 0) digitalWrite(pin_on_sts_CH4, LOW);
if (fault_byte == 10) digitalWrite(pin_flt1_sts_CH4, LOW);
if (fault_byte == 20) digitalWrite(pin_flt2_sts_CH4, LOW);
if (fault_byte == 30) digitalWrite(pin_spare_sts_CH4, LOW);
Serial.println("CH4 fault = ");
Serial.println(ovc_CH4);
//Serial.println(mps_CH4);
}


// Vmon Imon digipots. Write Imon value first, then write Vmon to send Imon and Vmon over SPI
if( !( strncmp(packetBuffer, "I1", 2) &&  strncmp(packetBuffer, "I2", 2) &&
       strncmp(packetBuffer, "I3", 2) &&  strncmp(packetBuffer, "I4", 2) ) ){
  igain = (uint8_t)(atof(packetBuffer+2)*255);
  Serial.print("Updated Imon gain with value = ");
  Serial.println(igain);
}


if( !strncmp(packetBuffer, "V1", 2)){
  vgain = (uint8_t)(atof(packetBuffer+2)*255);
  digitalWrite(pin_vi_cs_ch1, LOW);
  SPI.beginTransaction(settingsSlow);
  SPI.transfer(igain);
  SPI.transfer(vgain);
  SPI.endTransaction();
  digitalWrite(pin_vi_cs_ch1, HIGH);
  Serial.print("Updated Vmon gain with value = ");
  Serial.println(vgain);
  Serial.print("Wrote to Ch1 V and I digipots\n");
}

if( !strncmp(packetBuffer, "V2", 2)){
  vgain = (uint8_t)(atof(packetBuffer+2)*255);
  digitalWrite(pin_vi_cs_ch2, LOW);
  SPI.beginTransaction(settingsSlow);
  SPI.transfer(igain);
  SPI.transfer(vgain);
  SPI.endTransaction();
  digitalWrite(pin_vi_cs_ch2, HIGH);
  Serial.print("Updated Vmon gain with value = ");
  Serial.println(vgain);
  Serial.print("Wrote to Ch2 V and I digipots\n");
}

if( !strncmp(packetBuffer, "V3", 2)){
  vgain = (uint8_t)(atof(packetBuffer+2)*255);
  digitalWrite(pin_vi_cs_ch3, LOW);
  SPI.beginTransaction(settingsSlow);
  SPI.transfer(igain);
  SPI.transfer(vgain);
  SPI.endTransaction();
  digitalWrite(pin_vi_cs_ch3, HIGH);
  Serial.print("Updated Vmon gain with value = ");
  Serial.println(vgain);
  Serial.print("Wrote to Ch3 V and I digipots\n");
}

if( !strncmp(packetBuffer, "V4", 2)){
  vgain = (uint8_t)(atof(packetBuffer+2)*255);
  digitalWrite(pin_vi_cs_ch4, LOW);
  SPI.beginTransaction(settingsSlow);
  SPI.transfer(igain);
  SPI.transfer(vgain);
  SPI.endTransaction();
  digitalWrite(pin_vi_cs_ch4, HIGH);
  Serial.print("Updated Vmon gain with value = ");
  Serial.println(vgain);
  Serial.print("Wrote to Ch4 V and I digipots\n");
}

//Cal Source ON/OFF
if (!strcmp(packetBuffer, "CAL0\n")) {
  digitalWrite(pin_on_cal, LOW);
  Serial.println("Calibration source OFF");
}
if (!strcmp(packetBuffer, "CAL1\n")) {
  digitalWrite(pin_on_cal, HIGH);
  Serial.println("Calibration source ON");
}

if( strncmp(packetBuffer, "CALDAC", 6)==0){
  caldac_data = (uint32_t)( (-(atof(packetBuffer+6)-0.01719)*0.992375+10)/10*524287);
  caldac_byte2 = (uint8_t)( (caldac_data>>12)&0xff);
  caldac_byte1 = (uint8_t)( (caldac_data>>4)&0xff);
  caldac_byte0 = (uint8_t)(caldac_data&0xf);
  digitalWrite(pin_ical_dac_cs, LOW);
  digitalWrite(pin_ldac, LOW);
  SPI.transfer(caldac_byte2);
  SPI.transfer(caldac_byte1);
  SPI.transfer(caldac_byte0);
  digitalWrite(pin_ical_dac_cs, HIGH);
  digitalWrite(pin_ldac, HIGH);
  Serial.print("Setting Ical DAC to ");
  Serial.println(caldac_data);
}

if( strcmp(packetBuffer, "F1\n")==0 ){
  ovc_CH1=1;
  Serial.println("Setting PC Fault CH1");
}
if( strcmp(packetBuffer, "F2\n")==0 ){
  ovc_CH2=1;
  Serial.println("Setting PC Fault CH2");
}
if( strcmp(packetBuffer, "F3\n")==0 ){
  ovc_CH3=1;
  Serial.println("Setting PC Fault CH3");
}
if( strcmp(packetBuffer, "F4\n")==0 ){
  ovc_CH4=1;
  Serial.println("Setting PC Fault CH4");
}


if( strcmp(packetBuffer, "P0\n")==0 ){
  mode=0;
  Serial.println("Selecting BPC mode");
}
if( strcmp(packetBuffer, "P1\n")==0 ){
  mode=1;
  Serial.println("Selecting UPC mode");
}
  
memset(packetBuffer, 0, 100); //Clear out the buffer array
Udp.stop();
}


if ( Serial.available()>0 ) { // if command received
  start = millis();
  byte_i = 0;
    while ( (cmd[byte_i] = Serial.read()) != '\n' && (millis()-start)<1000 ) {
      byte_i++;
    }

if (cmd[0] == 'a' && cmd[1] == '1'){
  digitalWrite(pin_dcctflt_sel0, LOW);
  digitalWrite(pin_dcctflt_sel1, LOW);
cmd[0]=0;
cmd[1]=0;
}

if (cmd[0] == 'a' && cmd[1] == '2'){
  digitalWrite(pin_dcctflt_sel0, HIGH);
  digitalWrite(pin_dcctflt_sel1, LOW);
cmd[0]=0;
cmd[1]=0;
}

if (cmd[0] == 'a' && cmd[1] == '3'){
  digitalWrite(pin_dcctflt_sel0, LOW);
  digitalWrite(pin_dcctflt_sel1, HIGH);
cmd[0]=0;
cmd[1]=0;
}

if (cmd[0] == 'a' && cmd[1] == '4'){
  digitalWrite(pin_dcctflt_sel0, HIGH);
  digitalWrite(pin_dcctflt_sel1, HIGH);
cmd[0]=0;
cmd[1]=0;
}


}





i++;

}
