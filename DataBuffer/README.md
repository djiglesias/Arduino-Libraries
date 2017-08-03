# DataBuffer Library for Arduino

Enables serialization of data over a serial connection between two
Arduinos. Create an array of your data type and pass it to the 
serialization function and it will return an array of bytes.

The organization of the bytes are arranged with a single leading and 
trailing byte to indicate the start and end of a data package. The 
second byte represents the length (in bytes) of the data contained.
After the data bytes, there is a checksum of length sizeof(type) to
verify the quality of the data received.

For more information about this library please visit us at
https://github.com/djiglesias/Arduino-Libraries

## Hardware Setup

![setup](https://user-images.githubusercontent.com/7607469/28936889-1b79b16a-7857-11e7-9358-74c9b432dfa7.PNG)

## License

Copyright (C) 2017 by Duncan Iglesias

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
