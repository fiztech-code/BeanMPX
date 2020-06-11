
#ifndef BeanMPX_h
#define BeanMPX_h

#include <inttypes.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define RX 8
#define TX 9
#define BUFFER_SIZE 18

class BeanMPX {
	private: 
		// Receive vars		
		// stages of frame
		volatile uint8_t msg_stage = 0;
		/*
		  0 - SOF                 // start of message ( 1 bit: 1 )
		  1 - PRI ML              // priority, message length ( PRI 4 bit: XXXX; ML 4 bits: XXXX )
		  2 - DST-ID MES-ID DATA  // destination id, message id, data
		  3 - CRC                 // cyclic redundancy check
		  4 - EOM                 // end of message ( 8 bits: 0111 1110 )
		  5 - RSP                 // response ( 2 bits: 10 ACK[Acknowledgement], 01 NAK[Non-Acknowledgement )
		  6 - EOF                 // end of frame ( 6 bits: 000000 )
		*/
		volatile bool is_listining = false;
		volatile uint8_t _receive_buffer[BUFFER_SIZE];
		volatile uint8_t _buffer_index = 0;
		volatile bool _buffer_overflow = false;
		volatile uint8_t d = 0, s0 = 0, s1 = 0; // s0 and s1 - Consecutive bit counters for Stuffing Bits
		volatile uint16_t i = 0x80;
		volatile uint8_t msg_length = 0;
		uint8_t _receiveBitMask;

		// Transmit vars
		volatile uint8_t tx_msg_stage = 0;
		uint8_t _transmitBitMask;
		volatile bool is_transmitting = false;
		volatile uint8_t _transmit_buffer[BUFFER_SIZE];
		volatile uint8_t _tx_buffer_index = 0;
		volatile uint8_t _tx_buffer_len = 0;
		volatile uint8_t tx_s0 = 0, tx_s1 = 0; // ts0 and ts1 - Consecutive bit counters for Stuffing Bits
		volatile uint16_t j = 0x80;
		volatile uint8_t tx_retry = 2;

		// Acknowledge vars
		uint8_t acknowledge_did[10];
		volatile bool is_receive_ack = false;
		volatile uint8_t rsp = 0;
		volatile uint16_t k = 0x80;
		volatile bool is_transmit_ack = false;
		volatile uint8_t ack = 0x40;
		volatile uint16_t l = 0x80;

		// static data
		static BeanMPX *active_object;
		uint8_t crctable[256]; // CRC: Polynomial = 8X + 4X + X + 1
		
		// Output data		
		uint8_t msg[BUFFER_SIZE];
		uint8_t msg_index = _buffer_index;
		uint8_t msg_len = _buffer_index;
		char msg_type;
	private:		
		// private methods
		void pciSetup(byte pin);		
		
		uint8_t gencrc(uint8_t bytes[], uint8_t length);
		uint8_t checkcrc(uint8_t bytes[], uint8_t length);
		
		void storeReceivedBit(uint8_t rx_pin_val, bool no_stuffing_bit = false);
		void storeReceivedByte();		
		void receive();
		void receiveAcknowledge();
		
		void transmit();
		void transmitAcknowledge();
		
		void syncPulse();		
		
public:
  // public methods
  BeanMPX();  
  
  void begin();
  void ackMsg(const uint8_t *data);
  void ackMessages();
  
  void sendMsg(const uint8_t *data, uint16_t datalen);
  
  bool isBusy() { 
	return is_listining || is_transmitting; 
  }
    
  virtual uint8_t available();
  virtual char msgType();
  virtual uint8_t read();
  
  // handlers
  static inline void handle_rx();
  static inline void handle_rx_ack();  
  static inline void handle_tx();
  static inline void handle_tx_ack();  
  static inline void handle_sync();
};

#endif