/* Send Data
 by Duncan Iglesias <http://duncanjiglesias.ca>
 This example code is in the public domain.
*/

#include <SoftwareSerial.h>
#include "DataBuffer.h"

#define TYPE         float  // Variable type.
#define NUM_VAR      3      // Number of variables.
#define RND_MAX      300    // Random maximum.
#define RND_MIN      1      // Random minimum.
#define SS_RX        10     // Software Serial RX.
#define SS_TX        11     // Software Serial TX.
#define REQUEST      0x00   // Request from client.

SoftwareSerial   mySerial(SS_RX,SS_TX);
DataBuffer<TYPE> data_buf;

TYPE arr[NUM_VAR] = {};

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  randomSeed(analogRead(0));          

  Serial.println("-------------------------------------------------"); 
  Serial.println("");
  Serial.println("Data Buffer Serialization.");
  Serial.println("");
  Serial.println("1. Upload the first Arduino with Data_Send.");
  Serial.println("2. Open the serial monitor.");
  Serial.println("3. Upload the second Arduino with Data_Receive.");
  Serial.println("4. Open the serial monitor."); 
  Serial.println("5. Switch to 'No line ending' in Serial Monitor");
  Serial.println(""); 
  Serial.println("-------------------------------------------------"); 
  Serial.println(""); 
}

void loop() {   
  if (mySerial.available()) {
    if (mySerial.read() == REQUEST) {
      for (int i=0; i<NUM_VAR; i++)
        arr[i] = TYPE(random(RND_MIN,RND_MAX));  
      unsigned char *msg = data_buf.serialize(arr, sizeof(arr)/sizeof(TYPE));            
      mySerial.write(msg, data_buf.size());           
      mySerial.flush();
    }
  }
}