/*****************************************************************************
*  ParticleSoftSerial library (PSS_SimpleTest.ino)
*  Copyright (c) 2016 Free Software Foundation.  All right reserved.
*  Written by Andreas Rothenw�nder (aka ScruffR)
*
*  This sample shows sends data from Serial1 to ParticleSoftSerial(D2/D3)
*  
*    Prerequisites:
*      import SparkIntervalTimer library (by Paul Kourany)
*      wire   Serial1 TX to D2
*             Serial1 RX to D3 (for sending ParticleSoftSerial to Serial1)
*  
*    Due to relatively low interrupt priorities used, baudrates greater 31250 
*    may be prone to data corruption, depending on over all system load.
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
#include <ParticleSoftSerial.h>

#define SENDER   Serial1
#define RECEIVER SoftSer
#define PROTOCOL SERIAL_8N1

const uint32_t baud = 19200;

#if (SYSTEM_VERSION >= 0x00060000)
  SerialLogHandler logHandler;
#endif

#define PSS_RX D2 // RX must be interrupt enabled (on Photon/Electron D0/A5 are not)
#define PSS_TX D3
ParticleSoftSerial SoftSer(PSS_RX, PSS_TX); 

void setup()
{
  Serial.begin();
  SENDER.begin(baud, PROTOCOL);   // baud rates below 1200 can't be produced by USART
  RECEIVER.begin(baud, PROTOCOL); // but SoftSerial can ;-)
}

char szTX[64];
char szRX[64];

void loop()
{
  int len;
  
  memset(szRX, 0, sizeof(szRX));
  strcpy(szTX, "0123456789abcdefghijklmnopqrtstuvwxyz����\xFF"); // add some exotic chars too
  
  len = strlen(szTX) + 1;
  SENDER.write((uint8_t*)szTX, len);   

  for(uint32_t ms = millis(); millis() - ms < 1000 && RECEIVER.available() < len; Particle.process());
    
  for(int i = 0; i < len; i++)
  {
    szRX[i] = RECEIVER.read();
  }
  RECEIVER.flush();
    
  Serial.printlnf("%s\n\r%s", szTX, szRX);      // print out both strings
  if (len = strcmp(szRX, szTX))
  {
    delay(5000);
  }
}
