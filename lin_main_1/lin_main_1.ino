
#define PINOUT 1
#define speed 19200
#define CYCLE_TIME (1000000/speed)

#define LIN_MAX_BYTE 8

//#define LIN_VERSION_1_3 //13
#define LIN_VERSION_2_0 //20

static const uint16_t us_delay = CYCLE_TIME;
 

static inline bool bit_value (uint8_t * origin_byte_pointer, uint8_t bit_position)  { return (((*origin_byte_pointer) >> bit_position) & 0b00000001);  }
static inline uint8_t calc_message_lenght(uint8_t ID);

uint8_t id = 40;  // for LIN 2.0 it means 4 data bytes...
uint8_t header_syncByte = 0x55;
uint8_t PID;



uint8_t dataPacket[LIN_MAX_BYTE + 1];   //  the +1 is to accomodate the checksum and send everything together



// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PINOUT, OUTPUT);

  Serial.begin(speed);    // USART interface for normal bit transfer part of the messages...

  dataPacket[0] = 0x25;
  dataPacket[1] = 0xE2;
  dataPacket[2] = 0x80;
  dataPacket[3] = 0xD5;
  dataPacket[4] = 0xB0;

  delay(10);
}




// the loop function runs over and over again forever
void loop() 
{
  PID = calcHeader( &id);
  uint8_t message_lenght = calc_message_lenght(id);

  /*  Sending break condition - blocking for now*/ 
  Serial.flush();           //  make sure no tx operation is ongoing when changing baudrate.
  Serial.begin(speed/1.5);
  Serial.write(0x00);
  Serial.flush();           //  make sure brake condition is fully sent befor continuing
  Serial.begin(speed);

  Serial.write(header_syncByte);
  Serial.write(PID);

  delayMicroseconds(us_delay << 2);   // just for testing with only one device
  add_checkSum(PID, dataPacket, message_lenght );  
  Serial.write(dataPacket, message_lenght+1 );

  delay(500);
      
}


static inline void add_checkSum(uint8_t PID, uint8_t * dataArray, uint8_t byteN){
  uint16_t checksum = 0x0000;
  #ifdef LIN_VERSION_2_0        //  If it's LIN 2.0 the entire PID byte is included in the checksum calculation.
    checksum = PID;
  #endif  
  for(int i = 0; i<byteN; i++){
    checksum += (dataArray[i]);
    checksum <= 0xFF ? : checksum-= 0xFF ;
  }
  
  dataArray[byteN] = 0xFF^checksum;   // = ~checksum

}

uint8_t calcHeader(uint8_t * id){
  bool p0 = bit_value(id, 0) ^ bit_value(id, 1) ^ bit_value(id, 2) ^ bit_value(id, 4);
  bool p1 = !(bit_value(id, 1) ^ bit_value(id, 3) ^ bit_value(id, 4) ^ bit_value(id, 5));  

  return (*id & 0b00111111) | (p0 << 6) | (p1 << 7);
}

static inline uint8_t calc_message_lenght(uint8_t ID){
  #ifdef LIN_VERSION_1_3
    return 8;
  #else
    ID = ID>>4;
    switch (ID)
    {
    case 0x00:
      return 2;
    case 0x01:
      return 2;
    case 0x02:
      return 4;
    case 0x03:
      return 8;
    
    default:
      return 0;
    }
  #endif
}

