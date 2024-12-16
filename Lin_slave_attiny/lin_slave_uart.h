
#define MAX_ID 64
#define MAX_LENGHT 8
#define MIN_BAUS 1000
#define MAX_BAUS 20000
#define DEFAULT_BAUD 9600

//#define LIN_VERSION_1_3 //13
#define LIN_VERSION_2_0 //20




/**
 * @brief LIN frame structure,
 *  10 bytes.
 */
typedef struct LIN_frame{
  uint8_t ID;
  uint8_t data[8];
}LIN_frame;

/**
 * @brief indicates action associated with a ID.
 */
typedef enum ID_type{
  none = 0x00,        //  ID not assigned to action in this node. 
  RX_type = 0x40,     //  ID of message containing data to be read
  TX_type = 0x80,     //  ID requesting us to send data
}ID_type;


typedef struct ID_definition{
  ID_type type;
  uint8_t data_vec_address;
}ID_definition;

//  for now here, these are isr related functions.

void SM_PID();
void SM_listen();
void SM_respond();
void SM_ignore();
void SM_verify_sent_data();
void abort_transmission();



//  for now here, these are high level management functions and variables. 

class LIN_slave {
  private:
    unsigned long baud;
    uint8_t ID_total, ID_available, ID_next_empty;
    ID_definition ID_map[MAX_ID];                    //  will contain the ID linked to read/write action    

    void _set_pins_lin(uint8_t mod_nbr, uint8_t mux_set, uint8_t enmask);
    uint8_t add_to_list_ID(uint8_t _ID, ID_type _type);  //  will add a ID to the list
    uint8_t LIN_checksum(uint8_t* data, uint8_t* lenght, uint8_t PID_byte);  //  will calculate the checksum
    uint8_t calculate_message_lenght(uint8_t ID);  //  will calculate the lenght of a lin message based on the ID

    void _rx_irq();

  public:
    LIN_slave(uint8_t number_of_ID_used, uint16_t baud);  //  
    void begin_LIN_Slave(unsigned long baud );
    LIN_frame get_frame_fromBuffer(uint8_t ID);                //  will return the data of the assigned frame
    uint8_t write_data_toBuffer(uint8_t ID, uint8_t *data_8)   //  will write the data of the assigned frame
    void begin_LIN_Slave();
    void end_LIN_Slave();
    uint8_t add_RX_ID(uint8_t ID);              //  will add a ID to the RX list
    uint8_t add_TX_ID(uint8_t ID);              //  will add a ID to the TX list
    
    
  private:
    LIN_frame *Data_buffer;     //  will contain the data of the assigned frame
};
