#define USE_LIN_MODE

#ifdef USE_LIN_MODE
#ifndef LIN_HEADER_H
#define LIN_HEADER_H
  
#define MAX_ID 64
#define MAX_LENGHT 8
#define MIN_BAUD 1000
#define MAX_BAUD 20000
#define DEFAULT_BAUD 9600

//#define LIN_VERSION_1_3 //13
#define LIN_VERSION_2_0 //20



typedef enum LIN_ERROR{
  LIN_OK = 0,
  ERROR_NOT_INITIALIZED,
  ERROR_ALREADY_INITIALIZED,
  ERROR_ID_FULL,
  ERROR_ID_NOT_AVAILABLE,
  ERROR_ID_OUT_OF_BOUND,
  ERROR_WRONG_ID_TYPE,
  ERROR_BAUD_RATE,
  ERROR ,
}LIN_ERROR;


/**
 * @brief LIN frame structure,
 *  10 bytes.
 */
typedef struct LIN_frame{
  LIN_ERROR error;
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



//  for now here, these are isr related:





/**
 * @brief Contains all data, data structure, functions that gets used during the ISRs of the LIN slave.
 */
namespace lin_isr {

    /**
     * @brief Pointer to state function for LIN_SM state machine: void fn(void);
     */
    typedef void (*LIN_statemachine_fn)();

    /**
     * @brief LIN slave state machine.
     */ 
    typedef struct LIN__state_machine_st {
      LIN_statemachine_fn state;
      uint8_t PID = 255;            //  Protected ID (all 8 bits) byte of the current RX
      uint8_t data[8];              //  Data bytes received as current lin message.
      uint8_t data_index = 0;       //  Index of the next data byte to be Rx (yet to be filed)
      uint8_t new_byte = 0;         //  Latest byte received from the UART
      uint8_t data_lenght = 0;      //  Expected lenght of Data to be Tx, based on the ID, excluding checksum.
    }LIN__state_machine_st;


    /**
     * @brief Structure to hold the data to be transmitted over the LIN bus.
     */
    typedef struct LIN_TX_components{
      uint8_t data[9];              //  the data to be transmitted, comprised of checksum. (8+1 bytes max) 
      uint8_t index;                //  the index of the next byte to be transmitted.
      uint8_t length;               //  the length of the data to be transmitted, comprised of checksum.
      uint8_t verify_index;         //  the index of the next byte to be verified.
    }LIN_TX_components;

    static volatile LIN__state_machine_st LIN_SM;
    static volatile LIN_TX_components LIN_TX;

    /*  General functions used during communication: */
    uint8_t LIN_checksum(volatile uint8_t* data, volatile uint8_t lenght, volatile uint8_t PID_byte);  //  will calculate the checksum
    uint8_t calculate_message_lenght(uint8_t ID);                   //  will calculate the lenght of a lin message based on the ID
    uint8_t write_toBuffer(uint8_t ID, volatile uint8_t *data_8);   //  will write the data of the assigned frame
    LIN_frame read_fromBuffer(uint8_t ID);                          //  will return the data of the assigned frame
    void abort_transmission();

    /*  State Machine functions used by the LIN state machine: */
    /*  passed as pointer of the type: typedef void (*LIN_statemachine_fn)();*/
    void SM_PID();
    void SM_listen();
    void SM_respond();
    void SM_ignore();
    void SM_verify_sent_data();
}





class LIN_slave {
  private:
      
    static void _set_pins_lin(uint8_t mod_nbr, uint8_t mux_set, uint8_t enmask);
    static LIN_ERROR add_to_list_ID(uint8_t _ID, ID_type _type);  //  will add a ID to the list


  public:
    //LIN_slave(uint8_t number_of_ID_used, uint16_t baud);  //  
    LIN_slave();                                                //  default empty constructor
    static LIN_ERROR initialize_numberOfIDs(uint8_t number_of_ID_used);  //  will initialize the buffer depennding on # of IDs used
    static LIN_ERROR begin_LIN_Slave(unsigned long baud );
    static LIN_frame get_frame_from_RxBuffer(uint8_t ID);                 //  will return the data of the assigned frame
    static LIN_ERROR write_data_to_TxBuffer(uint8_t ID, uint8_t *data_8);   //  will write the data of the assigned frame
    static void end_LIN_Slave();
    static LIN_ERROR add_RX_ID(uint8_t ID);              //  will add a ID to the RX list
    static LIN_ERROR add_TX_ID(uint8_t ID);              //  will add a ID to the TX list    
    
  private:
    
};


#endif
#endif
