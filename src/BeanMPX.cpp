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
	crc = pgm_read_word_near(crctable + data);
  }
  return crc;
}

uint8_t BeanMPX::checkcrc(uint8_t bytes[], uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 1; i < length; i++) { // skips SOF 
    uint8_t b = bytes[i];
    uint8_t data = (b ^ crc);    
	crc = pgm_read_word_near(crctable + data);
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
  if (_buffer_index > BUFFER_SIZE) {
    _buffer_index = 0;
    is_listining = false;
    msg_stage = 0;	
  }
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
	  tx_pin_val = 0; // bug fix to count stuffing bit	        
      PORTB &= ~_transmitBitMask;
    } else {
	  tx_pin_val = 1; // bug fix to count stuffing bit	  
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

BeanMPX::BeanMPX() {
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
  for (uint8_t i = 0; i < datalen; i++) {
    frame[i + 1] = data[i]; // DID, MID, D0, D1, ...
  }

  frame[datalen + 1] = gencrc(frame, datalen + 1); // CRC
  frame[datalen + 2] = 0x7e; // EOF

  for (uint8_t i = sizeof(frame)-1; i > 0; i--) {
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
