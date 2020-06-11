#define RX 8
#define TX 9

// 
// Includes
//
#include <Arduino.h>
#include <BeanMPX.h>

//
// Statics
//
BeanMPX *BeanMPX::active_object = 0;

#ifdef _DEBUG
uint8_t debug_pin1 = (1<<PD3); // pin 3
uint8_t debug_pin2 = (1<<PD4); // pin 4
uint8_t debug_pin3 = (1<<PD5); // pin 5
#endif


//
// CRC
//
uint8_t BeanMPX::gencrc(uint8_t bytes[], uint8_t length) { // calculate CRC by byte
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t b = bytes[i];
    /* XOR-in next input byte */
    uint8_t data = (b ^ crc);
    /* get current CRC value = remainder */
    crc = (crctable[data]);
  }
  return crc;
}

uint8_t BeanMPX::checkcrc(uint8_t bytes[], uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 1; i < length; i++) { // skips SOF 
    uint8_t b = bytes[i];
    uint8_t data = (b ^ crc);
    crc = (crctable[data]);
  }  
  return crc;
}

//
// Receive helpers
//
void BeanMPX::storeReceivedBit(uint8_t rx_pin_val, bool no_stuffing_bit = false) {
  if (no_stuffing_bit) {
    if (rx_pin_val) {
      d |= i;
    }

    i >>= 1;
    return;
  }

  if (s0 == 5 || s1 == 5) { // skip stuffing bit
    i <<= 1;
  } else {
    if (rx_pin_val) {
      d |= i;
    }
  }

  if (rx_pin_val) {
    s0++;
    s1 = 0;
  } else {
    s1++;
    s0 = 0;
  }

  i >>= 1;
}

void BeanMPX::storeReceivedByte() {
  _receive_buffer[_buffer_index] = d;
  d = 0;
  i = 0x80;
  _buffer_index++;
}


//
// The receive routine called by the interrupt handler
//
void BeanMPX::receive() {
  if (is_transmitting) {
    return;
  }

  uint8_t rx_pin_val;
  rx_pin_val = (PINB & BeanMPX::_receiveBitMask);

  switch (msg_stage) {
  case 0:
    storeReceivedBit(rx_pin_val, true);

    if (!i) {
      d &= 1;
      storeReceivedByte();
      msg_stage++;
    }
    break;
  case 1:
    storeReceivedBit(rx_pin_val);

    if (!i) {
      msg_length = d & 0x0f;
      if (msg_length > 13) {
        msg_stage = 0;
        is_listining = false;
        TIMSK1 = 0;
        _buffer_index = 0;
      }
      storeReceivedByte();
      msg_stage++;
      break;
    }
    break;
  case 2:
    storeReceivedBit(rx_pin_val);

    if (!i) {
      storeReceivedByte();
      if (msg_length == 1) {
        msg_stage++;
        break;
      }
      msg_length--;
    }
    break;
  case 3:
    storeReceivedBit(rx_pin_val);

    if (!i) {
      storeReceivedByte();
      s0 = 0;
      s1 = 0;
      msg_stage++;
      break;
    }
    break;
  case 4:
    storeReceivedBit(rx_pin_val, true);
    if (!i) {
      if (checkcrc(_receive_buffer, _buffer_index) == 0) {
        ack = 0x40;
      } else {
        ack = 0x80;
      }
      if (d == 0x7E) {
        for (int a = 0; a < sizeof(acknowledge_did); a++) {
          if (_receive_buffer[2] == acknowledge_did[a]) {			 
            is_transmit_ack = true;
            a = sizeof(acknowledge_did);
          }
        }
      }
      storeReceivedByte();
      msg_stage++;
      break;
    }
    break;
  case 5:
    storeReceivedBit(rx_pin_val, true);

    if (!i || (i & 0x1f) > 0) {
      storeReceivedByte();
      TIMSK1 = 0;
      msg_stage = 0;
      is_listining = false;

	  memcpy(msg, _receive_buffer, sizeof _receive_buffer);
	  msg_index = 0;
	  msg_len = _buffer_index;	  
	  _buffer_index = 0;
	  msg_type = 'R';
	  
      break;
    }
    break;
  }
}

void BeanMPX::receiveAcknowledge() {
  if (!is_receive_ack) return;

  uint8_t rx_pin_val;
  rx_pin_val = (PINB & _receiveBitMask);

  if (rx_pin_val) {
    rsp |= k;
  }

  k >>= 1;

  if (!k) {
    k = 0x80;
	
	memcpy(msg, _transmit_buffer, sizeof(_transmit_buffer));
	msg[_tx_buffer_index] = rsp;
	msg_index = 0;
	msg_len = _tx_buffer_index + 1;	
	msg_type = 'T';
	

    if (rsp != 0x40 && tx_retry) {
      _tx_buffer_index = 0;
      j = 1;
      tx_retry--;
    } else {
      _tx_buffer_len = 0;
      tx_retry = 2;
    }
    is_receive_ack = false;
    rsp = 0;    
  }
}


//
// The transmit routine called by the interrupt handler
//
void BeanMPX::transmit() {
  if (!(_tx_buffer_index < _tx_buffer_len)) {
    if (_tx_buffer_len > 1) {
      is_receive_ack = true;
      return;
    } else {
      if (is_transmitting) {
        is_transmitting = false;
        if (!is_listining) {
          TIMSK1 = 0;
        }
        PORTB &= ~_transmitBitMask; // safety
      }
      return;
    }
  }

  is_transmitting = true;

  uint8_t tx_pin_val;
  tx_pin_val = _transmit_buffer[_tx_buffer_index] & j;

  if (_tx_buffer_index < _tx_buffer_len - 1 && (tx_s0 == 5 || tx_s1 == 5)) {
    j <<= 1;
    if (tx_s0 == 5) {
      PORTB &= ~_transmitBitMask;
    } else {
      PORTB |= _transmitBitMask;
    }
  } else {
    if (tx_pin_val) {
      PORTB |= _transmitBitMask;
    } else {
      PORTB &= ~_transmitBitMask;
    }
  }

  if (tx_pin_val) {
    tx_s0++;
    tx_s1 = 0;
  } else {
    tx_s1++;
    tx_s0 = 0;
  }

  j >>= 1;
  if (!j) {
    j = 0x80;
    _tx_buffer_index++;
  }
}

void BeanMPX::transmitAcknowledge() {
  if (!is_transmit_ack) {
    return;
  }

  uint8_t tx_pin_val;
  tx_pin_val = ack & l;

  if (tx_pin_val) {
    PORTB |= _transmitBitMask;
  } else {
    PORTB &= ~_transmitBitMask;
  }

  l >>= 1;

  if (!l || (l & 0x1f) > 0) {
    is_transmit_ack = false;
    l = 0x80;
    PORTB &= ~_transmitBitMask; // safety   
  }
}


// 
//  sync pulse
//
void BeanMPX::syncPulse() {
  uint8_t rx_pin_val;
  rx_pin_val = PINB & _receiveBitMask; // RX pin is 8

  if (!is_listining && rx_pin_val && !is_transmitting) {
    msg_stage = 0;
    i = 2;
    is_listining = true;
    TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
  }
  TCNT1 = 830;
}


//
// Enable Pin Interrupt
//
void BeanMPX::pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


//
// Static handle functions
//
inline void BeanMPX::handle_rx() {
  if (active_object) {
    active_object->receive();	
  }  
}
inline void BeanMPX::handle_rx_ack() {
  if (active_object) {
    active_object->receiveAcknowledge();	
  }  
}

inline void BeanMPX::handle_tx() {
  if (active_object) {
    active_object->transmit();	
  }  
}
inline void BeanMPX::handle_tx_ack() {
  if (active_object) {
    active_object->transmitAcknowledge();	
  }  
}

inline void BeanMPX::handle_sync() {	
  if (active_object) {
    active_object->syncPulse();	
  }  
}

//
// Interrupt handling
//
ISR(TIMER1_COMPA_vect) {
  #ifdef _DEBUG
  PORTD ^= debug_pin1;
  #endif
  BeanMPX::handle_rx();
  BeanMPX::handle_rx_ack();
}

ISR(TIMER1_COMPB_vect) {
  #ifdef _DEBUG
  PORTD ^= debug_pin2;
  #endif
  BeanMPX::handle_tx();
  BeanMPX::handle_tx_ack();
}

ISR(PCINT0_vect) {
  #ifdef _DEBUG
  PORTD ^= debug_pin3;    
  #endif
  BeanMPX::handle_sync();  
}

BeanMPX::BeanMPX():
	crctable({
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
		0xE2, 0xF1, 0xC4, 0xD7, 0xAE, 0xBD, 0x88, 0x9B, 0x7A, 0x69, 0x5C, 0x4F, 0x36, 0x25, 0x10, 0x03}) 
{
  // set acknowledge DIDs?
}

//
// Public methods
//
void BeanMPX::begin() {	
  #ifdef _DEBUG
  DDRD |= (1<<PD3) | (1<<PD4) | (1<<PD5);	// Debug Pins
  #endif
  
  // Init Timer1  
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10) | (1 << WGM12);	// prescaler 1, CTC Mode
  OCR1B = 830;
  OCR1A = 1660;
  TIMSK1 = 0;    
  
  // Transmit pin as outout
  DDRB |= (1<<TX);	// RX/TX pins  	

  // Receive pin as interupt 	
  pciSetup(RX);
  _receiveBitMask = digitalPinToBitMask(RX);
  _transmitBitMask = digitalPinToBitMask(TX);  
  
  active_object = this;
}

void BeanMPX::ackMsg(const uint8_t *destintion_id) {
	memcpy(acknowledge_did, destintion_id, sizeof(acknowledge_did));	
}

// Read from buffer
uint8_t BeanMPX::available() {
	return msg_len - msg_index;	
}

char BeanMPX::msgType() {
	return msg_type;	
}

uint8_t BeanMPX::read() {		
	if (msg_index == msg_len) {		
		return -1;
	}
	
	uint8_t d = msg[msg_index]; // grab next byte
	msg_index = (msg_index + 1) % BUFFER_SIZE;
	return d;
}




void BeanMPX::sendMsg(const uint8_t *data, uint16_t datalen) {  
  uint8_t frame[datalen + 4];
  tx_s0 = 0;
  tx_s1 = 0;

  frame[0] = 0x20 + datalen; // PRI ML
  for (byte i = 0; i < datalen; i++) {
    frame[i + 1] = data[i]; // DID, MID, D0, D1, ...
  }

  frame[datalen + 1] = gencrc(frame, datalen + 1); // CRC
  frame[datalen + 2] = 0x7e; // EOF

  for (byte i = sizeof(frame); i > 0; i--) {
    frame[i] = frame[i - 1];
  }

  frame[0] = 1;

  memcpy(_transmit_buffer, frame, sizeof(frame));
  _tx_buffer_len = sizeof(frame);
  _tx_buffer_index = 0;

  j = 1;

  TCNT1 = 830;
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
}
