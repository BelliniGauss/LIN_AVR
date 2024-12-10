
#define MAX_PID 64
#define MAX_LENGHT 8
#define MIN_BAUS 1000
#define MAX_BAUS 20000
#define DEFAULT_BAUD 9600

/**
 * @brief LIN frame structure,
 *  10 bytes.
 */
typedef struct LIN_frame{
  uint8_t PID;
  uint8_t data[8];
  uint8_t lenght = 0;
}LIN_frame;


typedef enum PID_type{
  none = 0x00,
  RX_type = 0x40,
  TX_type = 0x80,
}PID_type;

typedef struct PID_definition{
  PID_type type;
  uint8_t data_vec_address;
  uint8_t data_lenght;
}PID_definition;


void SM_PID();
void SM_listen();
void SM_respond();


class LIN_slave {
  private:
    unsigned long baud;
    uint8_t PID_total, PID_available, PID_next_empty;
    PID_definition PID_map[MAX_PID];                    //  will contain the PID linked to read/write action    

    void _set_pins_lin(uint8_t mod_nbr, uint8_t mux_set, uint8_t enmask);
    uint8_t add_to_list_PID(uint8_t _PID, PID_type _type);  //  will add a PID to the list
    

    void _rx_irq();

  public:
    LIN_slave(uint8_t number_of_PID_used, uint16_t baud);  //  
    void begin_LIN_Slave(unsigned long baud );
    LIN_frame read_data(uint8_t PID);             //  will return the data of the assigned frame
    uint8_t write_data(LIN_frame frame_in);   //  will write the data of the assigned frame
    void begin_LIN_Slave();
    void end_LIN_Slave();
    uint8_t add_RX_PID(uint8_t PID);              //  will add a PID to the RX list
    uint8_t add_TX_PID(uint8_t PID);              //  will add a PID to the TX list
    
    
  private:
    LIN_frame *Data_buffer;     //  will contain the data of the assigned frame
};
