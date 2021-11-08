
#ifndef BeanMPX_h
#define BeanMPX_h

#include <inttypes.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define MAILBOX_SIZE 8
#define BUFFER_SIZE 18
#define BEAN_FRAME_LENGTH 18

// using progmem for CRC to save memory
// CRC: Polynomial = 8X + 4X + X + 1
const PROGMEM uint8_t crctable[] = {
	0x00, 0x13, 0x26, 0x35, 0x4C, 0x5F, 0x6A, 0x79, 0x98, 0x8B, 0xBE, 0xAD, 0xD4, 0xC7, 0xF2, 0xE1, 
	0x23, 0x30, 0x05, 0x16, 0x6F, 0x7C, 0x49, 0x5A, 0xBB, 0xA8, 0x9D, 0x8E, 0xF7, 0xE4, 0xD1, 0xC2, 
	0x46, 0x55, 0x60, 0x73, 0x0A, 0x19, 0x2C, 0x3F, 0xDE, 0xCD, 0xF8, 0xEB, 0x92, 0x81, 0xB4, 0xA7, 
	0x65, 0x76, 0x43, 0x50, 0x29, 0x3A, 0x0F, 0x1C, 0xFD, 0xEE, 0xDB, 0xC8, 0xB1, 0xA2, 0x97, 0x84, 
	0x8C, 0x9F, 0xAA, 0xB9, 0xC0, 0xD3, 0xE6, 0xF5, 0x14, 0x07, 0x32, 0x21, 0x58, 0x4B, 0x7E, 0x6D, 
	0xAF, 0xBC, 0x89, 0x9A, 0xE3, 0xF0, 0xC5, 0xD6, 0x37, 0x24, 0x11, 0x02, 0x7B, 0x68, 0x5D, 0x4E, 
	0xCA, 0xD9, 0xEC, 0xFF, 0x86, 0x95, 0xA0, 0xB3, 0x52, 0x41, 0x74, 0x67, 0x1E, 0x0D, 0x38, 0x2B, 
	0xE9, 0xFA, 0xCF, 0xDC, 0xA5, 0xB6, 0x83, 0x90, 0x71, 0x62, 0x57, 0x44, 0x3D, 0x2E, 0x1B, 0x08, 
	0x0B, 0x18, 0x2D, 0x3E, 0x47, 0x54, 0x61, 0x72, 0x93, 0x80, 0xB5, 0xA6, 0xDF, 0xCC, 0xF9, 0xEA, 
	0x28, 0x3B, 0x0E, 0x1D, 0x64, 0x77, 0x42, 0x51, 0xB0, 0xA3, 0x96, 0x85, 0xFC, 0xEF, 0xDA, 0xC9, 
	0x4D, 0x5E, 0x6B, 0x78, 0x01, 0x12, 0x27, 0x34, 0xD5, 0xC6, 0xF3, 0xE0, 0x99, 0x8A, 0xBF, 0xAC,
	0x6E, 0x7D, 0x48, 0x5B, 0x22, 0x31, 0x04, 0x17, 0xF6, 0xE5, 0xD0, 0xC3, 0xBA, 0xA9, 0x9C, 0x8F, 
	0x87, 0x94, 0xA1, 0xB2, 0xCB, 0xD8, 0xED, 0xFE, 0x1F, 0x0C, 0x39, 0x2A, 0x53, 0x40, 0x75, 0x66, 
	0xA4, 0xB7, 0x82, 0x91, 0xE8, 0xFB, 0xCE, 0xDD, 0x3C, 0x2F, 0x1A, 0x09, 0x70, 0x63, 0x56, 0x45, 
	0xC1, 0xD2, 0xE7, 0xF4, 0x8D, 0x9E, 0xAB, 0xB8, 0x59, 0x4A, 0x7F, 0x6C, 0x15, 0x06, 0x33, 0x20, 
	0xE2, 0xF1, 0xC4, 0xD7, 0xAE, 0xBD, 0x88, 0x9B, 0x7A, 0x69, 0x5C, 0x4F, 0x36, 0x25, 0x10, 0x03};

class BeanMPX {
	private: 
		// per object data
		uint8_t _rx_prev_val;
		uint8_t _receivePin;
		uint8_t _receiveBitMask;
		volatile uint8_t *_receivePortRegister;
		
		uint8_t _transmitBitMask;
		volatile uint8_t *_transmitPortRegister;
		
		volatile uint8_t *_pcint_maskreg;
		uint8_t _pcint_maskvalue;
		
		// timer registers
		volatile uint16_t *_timerCountRegister;	
		uint16_t _timerCompareValue;
		volatile uint8_t *_timerInterruptMaskRegister;	
	    uint8_t _timerInterruptMask;	
		bool _use_timer2;
		
	
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
		volatile bool ignore_pulse = false;
		volatile uint8_t _receive_buffer[BUFFER_SIZE];
		volatile uint8_t _buffer_index = 0;
		volatile bool _buffer_overflow = false;
		volatile uint8_t d = 0; // rx data byte
		volatile uint8_t s0 = 0, s1 = 0; // rx_s0 and rx_s1 - Consecutive bit counters for Stuffing Bits
		volatile uint16_t i = 1; //0x80; // rx bit iterator
		volatile uint8_t msg_length = 0;		
		

		// Transmit vars		
		volatile uint8_t tx_msg_stage = 0;		
		volatile bool is_transmitting = false;
		volatile uint8_t _transmit_buffer[BUFFER_SIZE];
		volatile uint8_t _tx_buffer_index = 0;
		volatile uint8_t _tx_buffer_len = 0;
		
		volatile uint8_t _tx_byte_index = 0;
		volatile uint8_t _tx_byte_val = 0;
		volatile uint8_t _tx_bit_index = 7;
		volatile uint8_t _tx_bit_count = 0;	
		
		volatile uint8_t tx_s0 = 0, tx_s1 = 0; // ts0 and ts1 - Consecutive bit counters for Stuffing Bits
		volatile uint16_t j = 0x80; // tx bit iterator
		volatile uint8_t tx_retry = 2;

		// Acknowledge vars
		uint8_t acknowledge_did[10];
		volatile bool is_receive_ack = false;
		volatile uint8_t rsp = 0;
		volatile uint16_t k = 0x80; // ack bit iterator
		volatile bool is_transmit_ack = false;
		volatile uint8_t ack = 0x40;
		volatile uint16_t l = 0x80;		

		// static data
		static BeanMPX *active_object;	
		static BeanMPX *active_object2;	
		
		// Output data		
		uint8_t mailbox[MAILBOX_SIZE][BUFFER_SIZE+2] = {0};		
		uint8_t mailbox_fill_level = 0; 

		uint32_t busy_timeout = 0;		
	private:		
		// private methods
		void pciSetup(byte pin);		
		
		uint8_t gencrc(uint8_t bytes[], uint8_t length);
		uint8_t checkcrc(uint8_t bytes[], uint8_t length);
		
		void storeReceivedBit(uint8_t rx_pin_val, bool no_stuffing_bit = false);
		void storeReceivedByte();	
		void storeMessage(uint8_t *msg, uint8_t len, uint8_t msg_type = 0);
		void receive();
		void receiveAcknowledge();		
		
		void storeTxBit(uint8_t bit_val);
		void transmit();
		void transmitAcknowledge();
		
		void syncPulse();	
		void txSafetyState();
		void setTimerCounter();	
		
	public:
		// public methods
		BeanMPX();  

		void begin(uint8_t rx, uint8_t tx, bool use_timer2 = false);
		void ackMsg(const uint8_t *data, uint8_t len);		

		bool isBusy() { 			
			if (is_listining || is_transmitting || is_receive_ack || is_transmit_ack) {
				return true;
			}
			
			return (millis() - busy_timeout) < 2;
		}

		virtual uint8_t available();		
		void sendMsg(const uint8_t *data, uint16_t datalen);
		void getMsg(uint8_t *buffer, uint8_t buffer_len);
		
		uint8_t getStatus();

		// handlers
		static inline void handle_rx();
		static inline void handle_rx_ack();  
		static inline void handle_tx();
		static inline void handle_tx_ack();  

		static inline void handle_rx2();
		static inline void handle_rx_ack2();  
		static inline void handle_tx2();
		static inline void handle_tx_ack2();

		static inline void handle_sync(); 
		
};

#endif