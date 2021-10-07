#include <BeanMPX.h>

#define RX_BUFFER_SIZE 18
#define TX_BUFFER_SIZE 15

BeanMPX bean0;
BeanMPX bean1;

uint32_t timer = 0;
uint8_t g = 0;

uint8_t bean0_rx_buffer[RX_BUFFER_SIZE];
uint8_t bean0_rx_index = 0;
uint8_t bean0_tx_buffer[TX_BUFFER_SIZE];
uint8_t bean0_tx_index = 0;
uint8_t bean0_send_buffer = 0;

uint8_t bean1_rx_buffer[RX_BUFFER_SIZE];
uint8_t bean1_rx_index = 0;
uint8_t bean1_tx_buffer[TX_BUFFER_SIZE];
uint8_t bean1_tx_index = 0;
uint8_t bean1_send_buffer = 0;

// Messages           {DID,  MID,  DAT0, DAT1, DAT2}
uint8_t fuel[] =      {0x62, 0xA4, 0x3C}; // Fuel D0 153-40 
uint8_t engTemp[]  =  {0x62, 0x2C, 0xA5}; // Engine Temp D0 90-255 
uint8_t gear[] =      {0xFE, 0x40, 0x08, 0x10}; // P,R,N,D,M,3,2,L D0 11111111 (128, 64, 32, 16, 8, 4, 2, 1); Manual mode: 5,4,3,2,L D1 ---11111 (16, 8, 4, 2, 1)
uint8_t ect[] =       {0x62, 0xD2, 0x08, 0x10, 0x00}; // ECT PWR, SNOW, CRUISE D0 --11--1- (32,16,2); CRUISE-FLASH, ECT PWR-FLASH D1 -11----- (64,32); Beep D2 ----1--- (8) toggle on/off 
uint8_t seatBelt[] =  {0x62, 0xDF, 0x10, 0x80}; // DOOR D0 ---1---- (16);  SEAT BELT D1 1------- (128)

// DOOR-FLASH
uint8_t batt[] = {0x62, 0xD4, 0x28}; // DOOR-FLASH, BATT, OIL D0 -111---- (64, 32, 16) 
uint8_t door[] = {0x62, 0xFA, 0xff}; // DOOR-FLASH D0 11111111 (255); *important mid:0xD4 D0:0x28 D0 -1------ must be set first //


void setup() {
  Serial.begin(115200);
  while (!Serial);  
  Serial.println("BeanMPX Bridge");
  
  bean0.ackMsg((const uint8_t[]) {0xFE}, 1); // Acknowledge Messages, Length
  bean0.begin(6, 7, 0); // RX, TX, Bean0 (uses Timer1)
  
  bean1.ackMsg((uint8_t const[]) {0x62, 0x98}, 2); // Acknowledge Messages, Length
  bean1.begin(8, 9, 1); // RX, TX, Bean1 (uses Timer2)  

  timer = millis() + 10000;
}

void loop() {  

  // listen for messages from 'Combination Meter' (1st bus)
  if (bean0.available()) {       
    bean0_rx_index = 0;
    while (bean0.available()) {      
      bean0_rx_buffer[bean0_rx_index++] = bean0.read();
      if (bean0_rx_index > RX_BUFFER_SIZE) { // safety
        bean0_rx_index = 0;
      }      
    }

	// process received message
    if (bean0_rx_buffer[2] == 0xFE) {
      bean0_send_buffer = true;
      bean0_tx_index = 0;
      for (int i = 2; i < (bean0_rx_index - 3) && i < TX_BUFFER_SIZE; i++) {
        bean0_tx_buffer[bean0_tx_index++] = bean0_rx_buffer[i];
      }      
    }
    
    // print received message
	Serial.print("Bean0 - "); 
	Serial.print(bean0.msgType()); 
    Serial.print(" ");
    for (int i = 0; i < bean0_rx_index; i++) {
	  Serial.print(bean0_rx_buffer[i], HEX); 
	  Serial.print(" ");    
    }    
    Serial.print("\n");        
  }

  // transmit processed message to 'ECU' (2nd bus)
  if (bean0_send_buffer) {
    if (!bean1.isBusy()) {
      bean0_send_buffer = false;
      bean1.sendMsg(bean0_tx_buffer, bean0_tx_index);     
    }
  } 

  // transmit gear message to 1st bus
  if (timer < millis()) {
    if (!bean0.isBusy()) {      
      bean0.sendMsg(gear, sizeof(gear));
      timer = millis() + 1000;
	  
	  // toggle gear manual mode
      gear[3] = 1 << g;
      g++;
      if (g == 5) {
       g = 0; 
      }
    }    
  }
  
  // listen for messages from 'ECU' (2st bus)
  if (bean1.available()) {       
    bean1_rx_index = 0;
    while (bean1.available()) {      
      bean1_rx_buffer[bean1_rx_index++] = bean1.read();
      if (bean1_rx_index > RX_BUFFER_SIZE) { // safety
        bean1_rx_index = 0;
      }      
    }

	// print received message
    if (bean1_rx_buffer[2] == 0x62) {
      bean1_send_buffer = true;
      bean1_tx_index = 0;
      for (int i = 2; i < (bean1_rx_index - 3) && i < TX_BUFFER_SIZE; i++) {
        bean1_tx_buffer[bean1_tx_index++] = bean1_rx_buffer[i];
      }      
    }

    // print received message
    Serial.print("Bean1 - "); 
    Serial.print(bean1.msgType()); 
    Serial.print(" ");
    for (int i = 0; i < bean1_rx_index; i++) {
	  Serial.print(bean1_rx_buffer[i], HEX); 
	  Serial.print(" ");    
    }    
    Serial.print("\n"); 
  }

  // transmit processed message to 'Combination Meter' (1st bus)
  if (bean1_send_buffer) {
    if (!bean0.isBusy()) {
      bean1_send_buffer = false;
      bean0.sendMsg(bean1_tx_buffer, bean1_tx_index);     
    }
  }  
}
