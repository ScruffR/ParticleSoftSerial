/*****************************************************************************
*  ParticleSoftSerial library (ParticleSoftSewrial.h)
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

#pragma once
#define _PARTICLE_BUILD_IDE_					// comment for use with CLI or Particle Dev

#include "Particle.h"

// SparkIntervalTimer library needs to be imported seperately
#if defined(_PARTICLE_BUILD_IDE_)
#  include "SparkIntervalTimer/SparkIntervalTimer.h"
#else
#  include "SparkIntervalTimer.h"
#endif

#define X_PSS_DEBUG
#ifdef _PSS_DEBUG
#  define _PSS_DEBUG_PIN(_pin)  pinMode(_pin, OUTPUT)
#  define _PSS_DEBUG_HIGH(_pin) pinSetFast(_pin)
#  define _PSS_DEBUG_LOW(_pin)  pinResetFast(_pin)
#else
#  define _PSS_DEBUG_PIN(_pin)  //pinMode(D0, OUTPUT)
#  define _PSS_DEBUG_HIGH(_pin) //pinSetFast(D0)
#  define _PSS_DEBUG_LOW(_pin)  //pinResetFast(D0)
#endif

#define _PSS_BUFF_SIZE 64 // buffer size

class ParticleSoftSerial : public Print
{
private:
  static ParticleSoftSerial* pss; // only one instance allowed!

  static int      _rxPin;
  static int      _txPin;
  static boolean  _halfduplex;
  static uint32_t _usStartBit;
  static uint32_t _usBitLength;
  static uint8_t  _parity;
  static uint8_t  _dataBits;
  static uint8_t  _totalBits;
  
  static char _rxBuffer[_PSS_BUFF_SIZE]; 
  static volatile uint8_t _rxBufferHead;
  static volatile uint8_t _rxBufferTail;
  static volatile int8_t  _rxBitPos;
  
  static char _txBuffer[_PSS_BUFF_SIZE];
  static volatile uint8_t _txBufferHead; 
  static volatile uint8_t _txBufferTail; 
  static volatile int8_t  _txBitPos;

  static IntervalTimer rxTimer;
  static IntervalTimer txTimer;

  void   prepareRX(void);
  void   prepareTX(void);

public:
  // public methods
  ParticleSoftSerial(int rxPin, int txPin);
  ~ParticleSoftSerial();
  void begin(unsigned long baud);
  void begin(unsigned long baud, uint32_t config);

  void end(void);
  size_t  availableForWrite(void);
  size_t  available(void);
  virtual size_t write(uint8_t b); 
  virtual size_t write(const uint8_t *buffer, size_t size);
  int  read(void);
  int  peek(void);
  void flush(void);
  void sendBreak(int bits);

  // public only for easy access by interrupt handlers
  
  static inline void rxPinISR(void);   // to reallign the interval timer
  static inline void rxTimerISR(void); // called by rxTimer
  static inline void txTimerISR(void); // called by txTimer
};
