
#define PINOUT 1
#define speed 20000
#define CYCLE_TIME (1000000/20000)


uint16_t us_delay = CYCLE_TIME;

uint8_t id = 0;
uint8_t header_syncByte = 0x55;
uint8_t header_id;

uint8_t dataPacket[9];

static inline bool bit_value (uint8_t * origin_byte_pointer, uint8_t bit_position)  { return (((*origin_byte_pointer) >> bit_position) & 0b00000001);  }
static inline void fastDset(uint8_t port) { PORTD = PORTD | (0x01 << (port)); }
static inline void fastDres(uint8_t port) { PORTD = PORTD & (0xFF ^ ( 1 << port)); }


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PINOUT, OUTPUT);
  //pinMode(6, OUTPUT); 

  Serial.begin(19500);    // USART interface for normal bit transfer part of the messages...
  digitalWrite(PINOUT, HIGH);
  delay(1);
}

// the loop function runs over and over again forever
void loop() 
{
  calcID( &id, &header_id);

  Serial.end();
  digitalWrite(PINOUT, LOW);
  delayMicroseconds(us_delay << 4 );  //  16 times cycle time               

  digitalWrite(PINOUT, HIGH);                   // Delimiter
  delayMicroseconds( us_delay << 1 ); //  two times cycle time
  Serial.begin(19500);  

  sendByte(&header_syncByte);
  sendByte(&header_id);

  delayMicroseconds(us_delay << 2);

  dataPacket[0] = 0x25;
  dataPacket[1] = 0xE2;
  dataPacket[2] = 0x80;
  dataPacket[3] = 0xD5;
  dataPacket[4] = 0xB0;

  sendData(dataPacket, 1);

  delay(6);
        
  id++;
}

void sendData(uint8_t * dataArray, uint8_t byteN) {
  int i = 0;
  uint16_t checksum = 0x0000;
  for(; i<byteN; i++){
    checksum += (dataArray[i]);
    checksum <= 0xFF ? : checksum-= 0xFF ;
  }
  dataArray[i] = 0xFF^checksum;
  Serial.write(dataArray, byteN +1);
}

void calcID(uint8_t * id, uint8_t * header_id){
  bool p0 = bit_value(id, 0) ^ bit_value(id, 1) ^ bit_value(id, 2) ^ bit_value(id, 4);
  bool p1 = !(bit_value(id, 1) ^ bit_value(id, 3) ^ bit_value(id, 4) ^ bit_value(id, 5));  

  *header_id = (*id & 0b00111111) | (p0 << 6) | (p1 << 7);
}


static inline void sendByte(uint8_t byte){ 
  Serial.write(byte);  
}

static inline void sendByte(uint8_t * byte){ 
  sendByte(*byte);
}
