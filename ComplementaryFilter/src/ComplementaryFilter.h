/*
  ComplementaryFilter.h - Data Processing - Version 1.
  Copyright (c) 2017 Duncan Iglesias.  All right reserved.

  Source:

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
  Uses a complementary filter to estimate the state of an object with
  reduced noise. Takes two variables: 

    Data A: An absolute value such as position or angle.
    Data B: A rate of motion such as speed or angular rotation.

  Note: For 3DOF IMUs there will be an error/limitation for yaw about
  the z axis. Neglect this value.    

*/
#ifndef _COMPLEMENTARY_H
#define _COMPLEMENTARY_H

#include "Arduino.h"

template<typename T>
class ComplementaryFilter{
public:
  T get_filtered_result(unsigned long dt, T a, T b);
  inline void  set_time_constant(float tao) {this->tao = tao;}
  inline float get_time_constant() {return tao;}

private:
  unsigned long last_read_time = 0;
  T last_unfiltered_data_B = 0;
  T last_filtered_result = 0;
  float tao = 0.24;

};


template<typename T>
T ComplementaryFilter<T>::get_filtered_result(unsigned long t, T a, T b) {  
  
  // Update time.
  float delta_t = (t - last_read_time)/1000.0;
  float alpha   = tao/(tao+delta_t);  

  // Apply the complementart filter.
  T filtered_data_B   = b + last_filtered_result;
  T unfiltered_data_B = b + last_unfiltered_data_B;
  T filtered_result   = alpha*filtered_data_B + (1.0 - alpha)*a;

  // Update last read values.
  last_read_time         = t;
  last_filtered_result   = filtered_result;
  last_unfiltered_data_B = unfiltered_data_B;

  return filtered_result;

}

#endif