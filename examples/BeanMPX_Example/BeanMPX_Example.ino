#include <BeanMPX.h>

BeanMPX bean;
uint32_t timer = 0;

// Messages           {DID,  MID,  DAT0, DAT1, DAT2}
uint8_t fuel[] =      {0x62, 0xA4, 0x3C}; // Fuel D0 153-40 
uint8_t engTemp[]  =  {0x62, 0x2C, 0xA5}; // Engine Temp D0 90-255 
uint8_t gear[] =      {0x62, 0x40, 0x08, 0x10}; // P,R,N,D,M,3,2,L D0 11111111 (128, 64, 32, 16, 8, 4, 2, 1); Manual mode: 5,4,3,2,L D1 ---11111 (16, 8, 4, 2, 1)
uint8_t ect[] =       {0x62, 0xD2, 0x08, 0x10}; // ECT PWR, SNOW, CRUISE D0 --11--1- (32,16,2); CRUISE-FLASH, ECT PWR-FLASH D1 -11----- (64,32);
uint8_t seatBelt[] =  {0x62, 0xDF, 0x10, 0x80}; // DOOR D0 ---1---- (16);  SEAT BELT D1 1------- (128)

// DOOR-FLASH
uint8_t batt[] = {0x62, 0xD4, 0x28}; // DOOR-FLASH, BATT, OIL D0 -111---- (64, 32, 16) 
uint8_t door[] = {0x62, 0xFA, 0xff}; // DOOR-FLASH D0 11111111 (255); *important mid:0xD4 D0:0x28 D0 -1------ must be set first

void setup() {
  bean.ackMsg((const uint8_t[]) {0xFE}); // Messages to acknowledge
  bean.begin();  
  
  Serial.begin(115200);
  Serial.println("BeanMPX");
}

void loop() {  
  if (bean.available()) {
    Serial.print(bean.msgType()); 
    Serial.print(" ");    
    while (bean.available()) {      
      Serial.print(bean.read(), HEX); 
      Serial.print(" ");    
    }
    Serial.print("\n");    
  }


   if (timer < millis()) {
    if (!bean.isBusy()) {      
      bean.sendMsg(engTemp, sizeof(engTemp));
      timer = millis() + 1000;
    }    
  }
  
}
