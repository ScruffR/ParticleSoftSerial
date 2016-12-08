  ParticleSoftSerial library
  Copyright (c) 2016 Free Software Foundation.  All right reserved.
  Written by Andreas Rothenwänder (aka ScruffR)

  This library provides a basic implementation of an interrupt timer
  driven software serial port on any two digital GPIOs as RX/TX.

  Notes:
  Import SparkIntervalTimer library (by Paul Kourany)
    Due to limited free timers and to avoid interrupt mashup, only
    one active instance is allowed.
  Soft RX pin needs to be interrupt enabled, so on Photon D0 & A5
    won't work as RX
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
