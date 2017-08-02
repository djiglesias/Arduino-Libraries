/*
  DataBuffer.h - Serial communication for two Ardinos - Version 1.
  Copyright (c) 2017 Duncan Iglesias.  All right reserved.

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
*/

/* 
  Create an instance of DataBuffer and assign it a data type on both
  Arduinos. Using SoftwareSerial, connect the RX/TX pins (inversed) 
  between the two Arduinos.

  Note: Do not use baud rates higher than 38400bps for SoftwareSerial.

  The methods are:

    serialize(arr, len) - Converts an array of data to an array of bytes.
    deserialize(msg, len) - Converts an array of bytes to an array of data.
    size() - Returns the size of the most recent data buffer.

*/
#ifndef _DATABUFFER_H
#define _DATABUFFER_H

#define START_BIT     0x01
#define STOP_BIT      0xFF

template<typename T>
class DataBuffer {
  public:
    uint8_t size();
    unsigned char* serialize(T* arr, int len);
    T* deserialize(unsigned char* buf, int len);    
    
  private:    
    uint8_t length;
    unsigned char *msg;
    T *data;
};

template<typename T>
unsigned char* DataBuffer<T>::serialize(T* arr, int len) {     

  // Unnallocate previous memory.
  delete [] msg;

  // Create checksum safe from ovf.
  T chksum = T(0);
  for (int i=0; i < len; i++)
    chksum += arr[i]/len;
  arr[len] = chksum;

  // Instantiate new message buffer.
  msg = new unsigned char[length = (len + 1) * sizeof(T) + 3];

  // Convert data to bytes/characters.
  memset(msg, START_BIT, sizeof(byte));                // Start byte.
  memset(&msg[1], byte(length), sizeof(byte));         // Length byte.
  for (int i=0; i < len + 1; i++)
    memcpy(&msg[sizeof(T)*i+2],  &arr[i], sizeof(T));  // Data bytes.    
  memset(&msg[length-1], STOP_BIT, sizeof(byte));      // Stop byte.

  // Return serialized array.
  return msg;            
}


template<typename T>
T* DataBuffer<T>::deserialize(unsigned char* buf, int len) {

  // Unallocate previous memory.
  delete [] data;

  // Instantiate array to store incoming data.
  data = new T[length = len/sizeof(T) - 1];
  T chksum = 0, tmpchk;

  // Loop through using memcpy/memset.
  for (int i=0; i < length; i++) {
    memcpy(&data[i], buf + i*sizeof(T), sizeof(T));  
    chksum += data[i]/length;
  }

  // Check quality of data.
  memcpy(&tmpchk, buf + length*sizeof(T), sizeof(T)); 
  if (chksum == tmpchk) 
    return data;    
  else 
    return NULL;
}


template<typename T>
uint8_t DataBuffer<T>::size() {
  return length;
}

#endif