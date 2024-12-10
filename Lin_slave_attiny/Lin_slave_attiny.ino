/*
USART0 : 0x0800

Name: RXDATAH
Offset: 0x01
Reset: 0x00

Bit 0 – DATA[8] Receiver Data Register
When the USART receiver is configured to LINAUTO mode, this bit indicates if the received data are within the
response space of a LIN frame. If the received data are in the protected identifier field, this bit will be read as ‘0’.
Otherwise, the bit will be read as ‘1’.
------------------------------------------------------------------------------------------------------------------------

Control B
Name: CTRLB
Offset: 0x06
Reset: 0x00

Bits 2:1 – RXMODE[1:0] Receiver Mode
In LINAUTO mode the SYNC character is constrained and found valid if every two bits falls within 32 ±6 baud samples
of the internal baud rate and match data value 0x55. The GENAUTO and LINAUTO modes are only supported for
USART operated in Asynchronous Slave mode.
Writing these bits select the receiver mode of the USART:
Value 0x03 LINAUTO LIN Constrained Auto-Baud mode

*/

//#include "HardwareSerial.h" 
#define USE_LIN_MODE 1
//#include <UART0.cpp>

#include <SoftwareSerial.h>

#include "lin_slave_uart.h"

SoftwareSerial mySerial(PIN_PA4, PIN_PA5);
LIN_slave myLin(10, 20000);

char lol = 0x00;



void setup() {

  //begin_LIN_Slave(20000);
  
  //Serial.begin(20000);
  mySerial.begin(9600);

  USART0.CTRLB |= (0x03<<1);  // bit [2,1]  LINAUTO LIN Constrained Auto-Baud mode

  mySerial.println("Start");

}

void loop() {

  /*if (Serial.available()){
      lol = Serial.read();
      mySerial.write(0x23);
      mySerial.write(lol);      
      mySerial.write(0x23);
      mySerial.write(lol & 0x3F);
      mySerial.write(0x23);
      mySerial.write(0x23);
  }*/
  /*if (Serial.available()){
    if(USART0.RXDATAH & 0x01)     // check bit 0 for reserved lin id (0b0) or normal id (0b1).
    {
      if (Serial.available()) {   
        mySerial.write(Serial.read());
        delay(1);
      }      
    }
    else
    {
      mySerial.write(0x23);
      mySerial.write(0x23);
      mySerial.write(Serial.read());
      mySerial.write(0x23);
      mySerial.write(0x23);
      delay(5);
      if (Serial.available()) {   //  flushing rx buffer to ignre the message with reserved id!
        Serial.read();
      }
      
    }
  }*/
  //mySerial.write(Serial.read());

    

}
