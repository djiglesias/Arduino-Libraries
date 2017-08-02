/* Receive Data
 by Duncan Iglesias <http://duncanjiglesias.ca>
 This example code is in the public domain.
*/
#include <SoftwareSerial.h>
#include "DataBuffer.h"

#define TYPE         float  // Variable type.
#define DELAY        40     // Delay for reading.
#define SS_RX        10     // Software Serial RX.
#define SS_TX        11     // Software Serial TX.
#define BUFFER       256    // Length of buffer.
#define REQUEST      0x00   // Request from client.

SoftwareSerial   mySerial(SS_RX,SS_TX);
DataBuffer<TYPE> data_buf;

float pass = 0.0;
float fail = 0.0;
unsigned long lastRequestTime = 0;
unsigned long currentTime = 0;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);

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
  // Request information.
  currentTime = millis();
  if (currentTime - lastRequestTime > DELAY) {
    mySerial.write(REQUEST);
    lastRequestTime = currentTime;
  }

  // Collect incoming message.
  if (mySerial.available() > 3) {           
    // Look for start bit.
    while(mySerial.read() != START_BIT) {}
    int index = 0;
    int len = int(mySerial.read());
    unsigned char msg[len];
    
    // Populate buffer until end bit.
    while(index < len - 2) {
      delay(2);
      if (mySerial.available() > 0)
        msg[index++] = mySerial.read();
    }
    while (mySerial.available() > 0) {} 
  
    // Deserialize mesage and check quality.
    TYPE *in = data_buf.deserialize(msg, index);
    
    if (in != NULL) {
      // Data successfully received.
      // add code here...
      pass += 1;
    }
    else {
      // Bad packet.
      // add code here...
      fail += 1;
    }

    // Print success rate. (~98.6%)
    Serial.print((pass/(pass + fail)*100), 1);
    Serial.println("%\t");
  }
}
