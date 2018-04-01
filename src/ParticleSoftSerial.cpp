/*****************************************************************************
*  ParticleSoftSerial library (ParticleSoftSewrial.cpp)
*  Copyright (c) 2016 Free Software Foundation.  All right reserved.
*  Written by Andreas Rothenwänder (aka ScruffR)
*
*  This library provides a basic implementation of an interrupt timer
*  driven software serial port on any two digital GPIOs as RX/TX.
*
*  Notes:
*  Import SparkIntervalTimer library (by Paul Kourany)
*    Due to limited free timers and to avoid interrupt mashup, only
*    one active instance is allowed.
*  Soft RX pin needs to be interrupt enabled, so on Photon D0 & A5
*    won't work as RX
*  
*  This library is free software; you can redistribute it and/or
*  modify it under the terms of the GNU Lesser General Public
*  License as published by the Free Software Foundation; either
*  version 2.1 of the License, or (at your option) any later version.
*
*  This library is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public
*  License along with this library; if not, write to the Free Software
*  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*****************************************************************************/

#include "ParticleSoftSerial.h"

enum {
    PSS_INACTIVE = -2,
    PSS_STARTBIT = -1,
    PSS_DATA     =  0,
};

typedef struct 
{
  unsigned long baudrate;
  unsigned int  usStartBit;
  unsigned int  usBitLength;
} BAUD_TIMING;

static const BAUD_TIMING btTable[] =
{
//    baud  µs/start  µs/bit 1/baudrate 
//                    shorter due to call latency
//  { 115200,     3,     9 }, // exact    8.68056µs
  {  57600,     9,    16 }, //         17.36111µs
  {  38400,    24,    24 }, //         26.04167µs
  {  31250,    31,    31 }, //         32.00000µs
  {  28800,    37,    33 }, //         34.72222µs
  {  19200,    61,    51 }, //         52.08333µs
  {  14400,    90,    68 }, //         69.44444µs
  {   9600,   140,   104 }, //        104.16667µs
  {   4800,   295,   208 }, //        208.33333µs
  {   2400,   610,   417 }, //        416.66667µs
  {   1200,  1230,   833 }, //        833.33333µs
  {    600,  2500,  1667 }, //       1666.66667µs
  {    300,  5000,  3333 }, //       3333.33333µs
  {      0,     0,     0 }  // end mark
};


int      ParticleSoftSerial::_rxPin                = PSS_INACTIVE;
int      ParticleSoftSerial::_txPin                = PSS_INACTIVE;
boolean  ParticleSoftSerial::_halfduplex           =        false;
uint32_t ParticleSoftSerial::_usStartBit           =          140; // start bit with odd lengths due to EXTI latency
uint32_t ParticleSoftSerial::_usBitLength          =          104; // default 9600 baud = 104µs per bit
uint8_t  ParticleSoftSerial::_parity               =         0x00; // default NONE
uint8_t  ParticleSoftSerial::_dataBits             =            8; // default 8bit
uint8_t  ParticleSoftSerial::_totalBits            =            9; // ignore start bit, 8data + 1stop
  
char ParticleSoftSerial::_rxBuffer[_PSS_BUFF_SIZE] =           ""; 
volatile uint8_t ParticleSoftSerial::_rxBufferHead =            0;
volatile uint8_t ParticleSoftSerial::_rxBufferTail =            0;
volatile int8_t  ParticleSoftSerial::_rxBitPos     = PSS_INACTIVE;
  
char ParticleSoftSerial::_txBuffer[_PSS_BUFF_SIZE] =            "";
volatile uint8_t ParticleSoftSerial::_txBufferHead =            0; 
volatile uint8_t ParticleSoftSerial::_txBufferTail =            0; 
volatile int8_t  ParticleSoftSerial::_txBitPos     = PSS_INACTIVE; 

IntervalTimer ParticleSoftSerial::rxTimer;
IntervalTimer ParticleSoftSerial::txTimer;

ParticleSoftSerial* ParticleSoftSerial::pss = NULL;

ParticleSoftSerial::ParticleSoftSerial(int rxPin, int txPin)
{
  if (pss) 
  {
#if (SYSTEM_VERSION >= 0x00060000)
    Log.error("There is already an instance of ParticleSoftSerial running on pins RX%d / TX%d", _rxPin, _txPin);
#endif
    return;
  }
  pss = this;

  _halfduplex = (rxPin == txPin);
  _rxPin = rxPin;
  _txPin = txPin;

   _rxBufferTail = _rxBufferHead = 
  _txBufferTail = _txBufferHead = 0;
  
  _PSS_DEBUG_PIN(D0);
}

ParticleSoftSerial::~ParticleSoftSerial() 
{
  end();
  pss = NULL;
}

void ParticleSoftSerial::prepareRX(void)
{
  pinMode(_rxPin, INPUT_PULLUP);

  _rxBitPos = PSS_INACTIVE;
  rxTimer.begin(rxTimerISR, _usBitLength, uSec);
  //rxTimer.interrupt_SIT(INT_DISABLE);

  // prepare for FALLING edge of start bit 
  attachInterrupt(_rxPin, rxPinISR, FALLING);
}

void ParticleSoftSerial::prepareTX(void)
{
  pinMode(_txPin, OUTPUT);
  pinSetFast(_txPin);

  _txBitPos = PSS_INACTIVE;
  txTimer.begin(txTimerISR, _usBitLength, uSec);
  //txTimer.interrupt_SIT(INT_DISABLE);
}

void ParticleSoftSerial::begin(unsigned long baud)
{
  begin(baud, SERIAL_8N1);
}

void ParticleSoftSerial::begin(unsigned long baud, uint32_t config)
{
  if (config & SERIAL_DATA_BITS_9)
  {
    _dataBits = 9;
  }
  else if (config & SERIAL_DATA_BITS_7)
  {
    _dataBits = 7;
  }
  else
  {
    _dataBits = 8;
  }
  
  if (config & SERIAL_STOP_BITS_2)
  {
    _totalBits = _dataBits + 2;
  }
  else
  {
    _totalBits = _dataBits + 1;
  }

  if (config & SERIAL_PARITY)
  {
    _parity = (config & SERIAL_PARITY_ODD) ? 0x11 : 0x10;
    _totalBits++;
  }
  else
  {
    _parity = 0;
  }
    
  for (int i=0; btTable[i].baudrate > 0; i++)
  {
    if (btTable[i].baudrate <= baud)
    {
#if (SYSTEM_VERSION >= 0x00060000)
      if (btTable[i].baudrate != baud)
      {
        Log.info("%lu not available! Selected rate %lu", baud, btTable[i].baudrate);
      }
#endif
      _usStartBit  = btTable[i].usStartBit;
      _usBitLength = btTable[i].usBitLength;

      break;
    }
  }

  if (!_halfduplex)
  { // since in halfduplex mode the pin starts off as RX pin
    prepareTX();
  }
  prepareRX();
}

void ParticleSoftSerial::end(void)
{
  detachInterrupt(_rxPin);
  rxTimer.end();
  txTimer.end();
  flush();
}

int ParticleSoftSerial::availableForWrite(void)
{
  return (_txBufferHead - _txBufferTail + _PSS_BUFF_SIZE - 1) % _PSS_BUFF_SIZE; 
}

int ParticleSoftSerial::available(void)
{
  return (_rxBufferHead - _rxBufferTail + _PSS_BUFF_SIZE) % _PSS_BUFF_SIZE; 
}

size_t ParticleSoftSerial::write(uint8_t b)
{
  // try to add new character for up to one second
  for (uint32_t ms=millis(); !availableForWrite(); Particle.process())
  {
    if (millis()-ms > 1000) return 0;
  }
  
  _txBuffer[_txBufferHead] = b;
  _txBufferHead = (_txBufferHead + 1) % _PSS_BUFF_SIZE;

  if (_txBitPos <= PSS_INACTIVE)                                   // if txTimer is not yet running, start it
  {
    if (_halfduplex)
    {
      end();
      prepareTX();
    }
    txTimer.resetPeriod_SIT(_usStartBit, uSec);
    _txBitPos = PSS_STARTBIT;
  }
    
  return 1;
}

size_t ParticleSoftSerial::write(uint16_t b9)
{
  // 9 bit not yet implemented
  return write((uint8_t)b9);
}

// use version pulled from Print
//size_t ParticleSoftSerial::write(const uint8_t *buffer, size_t size)
//{
//  size_t bytesSent;
//
//  // probably a better way with memcpy() - but more complicated ;-)
//  for(bytesSent = 0; bytesSent < size && write(buffer[bytesSent]); bytesSent++);
//
//  return bytesSent;
//}

int ParticleSoftSerial::read(void)
{
  uint8_t d;

  // Empty buffer?
  if (_rxBufferHead == _rxBufferTail) return -1;

  d = _rxBuffer[_rxBufferTail]; 
  _rxBufferTail = (_rxBufferTail + 1) % _PSS_BUFF_SIZE;

  return d;
}

int ParticleSoftSerial::peek(void)
{
  if (_rxBufferHead == _rxBufferTail) return -1;

  return _rxBuffer[_rxBufferTail];
}

void ParticleSoftSerial::flush(void)
{
  _rxBufferTail = _rxBufferHead;
}

void ParticleSoftSerial::sendBreak(int bits)
{
  if (_halfduplex)
  {
    end();
    prepareTX();
  }
  pinResetFast(_txPin);
  delayMicroseconds(_usBitLength * bits);
}

#ifdef _PSS_DEBUG
volatile uint32_t usLast[12];
volatile uint8_t  b[12];
#endif

void ParticleSoftSerial::rxPinISR(void)
{ // start bit triggers read after 1.5 bits lengths (= middle of first data bit)
  if (_rxBitPos <= PSS_STARTBIT)
  {
#ifdef _PSS_DEBUG
    usLast[0] = micros();
    b[0] = HIGH;
#endif
    //rxTimer.interrupt_SIT(INT_ENABLE);
    rxTimer.resetPeriod_SIT(_usStartBit, uSec);
    _rxBitPos = PSS_DATA;
    detachInterrupt(_rxPin);
  }
}

void ParticleSoftSerial::rxTimerISR(void)
{
  static uint8_t parityErr = (_parity & 0x01);
  uint8_t bit;

  if (_rxBitPos <= PSS_STARTBIT) return;
  _PSS_DEBUG_HIGH(D0);
  
  if (_rxBitPos == PSS_DATA) // after start bit go for normal bit length
  {
    rxTimer.resetPeriod_SIT(_usBitLength, uSec);

    parityErr = (_parity & 0x01);
    _rxBuffer[_rxBufferHead] = 0;
  }
  
  bit = pinReadFast(_rxPin);
#ifdef _PSS_DEBUG
  b[_rxBitPos+1] = bit;
  usLast[_rxBitPos+1] = micros();
#endif
  if (_rxBitPos <= _dataBits)
  {
    _rxBuffer[_rxBufferHead] |= (bit << _rxBitPos);
  }

  if ((_parity & 0x10) && (_rxBitPos <= _dataBits))
  {
    parityErr ^= (bit & 0x01); // keep track of the parity
  }  
  
  if (++_rxBitPos >= _totalBits)
  {
    if (parityErr)
    {
      _rxBuffer[_rxBufferHead] = 0xFF;
    }
    else
    {
      _rxBufferHead = (_rxBufferHead + 1) % _PSS_BUFF_SIZE;
    }
    _rxBitPos = PSS_INACTIVE;
    //rxTimer.interrupt_SIT(INT_DISABLE);
    attachInterrupt(_rxPin, rxPinISR, FALLING);
  }
  _PSS_DEBUG_LOW(D0);
}

void ParticleSoftSerial::txTimerISR(void)
{
  static uint8_t parity = (_parity & 0x01);

  if(_txBitPos < PSS_STARTBIT) return;

  if(_txBitPos == PSS_STARTBIT)                                 // produce StartBit
  {
    txTimer.resetPeriod_SIT(_usBitLength, uSec);
    pinResetFast(_txPin);
    parity = (_parity & 0x01);
  }
  else if(PSS_DATA <= _txBitPos && _txBitPos < _dataBits)       // send data bits
  {
    if ((_txBuffer[_txBufferTail] >> _txBitPos) & 0x01)
    {
      pinSetFast(_txPin);
      parity ^= 0x01;                                           // keep track of the parity
    }
    else
    {
      pinResetFast(_txPin);
    }
  }
  else if(_parity && _txBitPos == _dataBits)                    // send parity (if required)
  {
    if (parity)
    {
      pinSetFast(_txPin);
    }
    else
    {
      pinResetFast(_txPin);
    }
  }
  else if(_dataBits <= _txBitPos && _txBitPos < _totalBits)     // produce StopBit(s)
  {
      pinSetFast(_txPin);
  }
  else if(_txBitPos >= _totalBits)
  { 
    _txBufferTail = (_txBufferTail + 1) % _PSS_BUFF_SIZE;
    _txBitPos = PSS_INACTIVE;
  }
  
  if (_txBufferTail == _txBufferHead)                           // more data to send?
  {
    _txBitPos = PSS_INACTIVE;
    //rxTimer.interrupt_SIT(INT_DISABLE);
    if (_halfduplex && pss) pss->prepareRX();                   // when TX in finished revert back to default RX mode
  }
  else
  {
    _txBitPos++;
  }
}
