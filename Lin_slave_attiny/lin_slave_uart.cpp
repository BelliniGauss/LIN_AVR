#define USE_LIN_MODE

#ifdef USE_LIN_MODE
      /*
One-Wire Half Duplex Mode:
1. Internally connect the TXD to the USART receiver (the LBME bit in the USARTn.CTRLA register).
2. Enable internal pull-up for the RX/TX pin (the PULLUPEN bit in the PORTx.PINnCTRL register).
3. Enable Open-Drain mode (the ODME bit in the USARTn.CTRLB register).
4. Set the baud rate (USARTn.BAUD).
5. Set the frame format and mode of operation (USARTn.CTRLC).
6. Enable the transmitter and the receiver (USARTn.CTRLB).
*/


#include "Arduino.h"
#include "lin_slave_uart.h"


//using namespace lin_isr;


#define ID(pid) (pid & 0b00111111)  //  Masking the 2 MSb of the PID byte, to get the 6 bit ID.





static unsigned long baud;
static uint8_t ID_total = 0, ID_available, ID_next_empty;
static ID_definition ID_map[MAX_ID];                    //  will contain the ID linked to read/write action  
static LIN_frame *Data_buffer;     //  will contain the data of the assigned frame



/**
 * @brief ISR for the USART0 RXC interrupt.
 * Will call the state machine to handle the received data.
 */
ISR(USART0_RXC_vect) {
  // Needs to update the LIN_slave state machine...

  // Recieve data, pass it to the state machine function poiinted by the state pointer:
  uint8_t rxDataH = USART0.RXDATAH;
  uint8_t   data  = USART0.RXDATAL; 
  bool is_PID =  !(rxDataH &  USART_DATA8_bm );       //  If the DATA8 bit is 0, then it's a PID byte.

  
  if((rxDataH & USART_PERR_bm) & is_PID) {            //  If ther's a parity error and  is PID
    abort_transmission();                             //  if we were transmitting, stop that.
    return;                                           //  discard the byte.
  }
  
  if(rxDataH & USART_FERR_bm) {           // Frame error,
    abort_transmission();                 //  if we were transmitting, stop that.
    return;                               //  discard the byte.
  }
  
  if(is_PID){     //   I have to start over a new reception sequence:
    LIN_SM.state = SM_PID;    //  Sp I set again the current fn to the PID one.
  }

  LIN_SM.new_byte = data;     //  load new byte to state space of state machine
  LIN_SM.state();             //  call the state function of the state machine.
}



/**
 * @brief State entered when a new PID byte is received.
 */
void lin_isr::SM_PID() {
  uint8_t data_in = LIN_SM.new_byte;
  LIN_SM.PID = data_in;        
  LIN_SM.data_index = 0;            //  new frame is starting, so I reset the data index.
  LIN_SM.data_lenght = calculate_message_lenght(ID(LIN_SM.PID));
    
  
  //  I'll now have to check between the programmed IDs if I should Listen to, respond to or ignore the frame...
  switch (ID_map[ID(LIN_SM.PID)].type)
  {
    case RX_type:
      LIN_SM.state = SM_listen;
      break;
    case TX_type:
      LIN_SM.state = SM_respond;
      break;
    default:
      LIN_SM.state = SM_ignore;
      break;  
  }
}


/**
 *  @brief State entered if the current transmission does not interest this slave device. 
 *  Possible causes: ID not relevant to us or error detected. 
 *  In this state the slave will ignore the incoming data. It does not exit by itself,
 *  only the next PID byte will make us start back to listen. 
 */
void lin_isr::SM_ignore() {
}


/**
 * @brief State of active recieving data, we're accumulating new bytes.
 * When we're done we'll verify the checksum. 
 * If the checksum is correct, we'll store the data in the buffer.
 * Then we'll transition to ignore state, SM_ignore.
*/
void lin_isr::SM_listen() {

  //  Checks if we're still receiving data:
  if(LIN_SM.data_index < LIN_SM.data_lenght) {
    LIN_SM.data[LIN_SM.data_index] = LIN_SM.new_byte;           //  Add the new byte to the LIN_SM recieving buffer
    LIN_SM.data_index++;                                        //  Increment the index of the LIN_SM buffer.
  }
  else     // if we recieved the checksum byte:
  {    
    uint8_t checksum = LIN_checksum(LIN_SM.data , LIN_SM.data_lenght, LIN_SM.PID);
    
    //  Check if the recieved byte matches the calculated checksum. 
    if(checksum == LIN_SM.new_byte)                             //  The checksum just recieved is correct, 
      write_toBuffer(ID(LIN_SM.PID), LIN_SM.data);                // I can now store the data in the buffer.
    else                                                        //  The checksum is INCORRECT, I will ignore the frame.     
      LIN_SM.state = SM_ignore;                                   //  I'll set the state to the SM_ignore.    
  }
}

void lin_isr::SM_respond() {
  LIN_SM.state = SM_verify_sent_data;    //  from now on we'll have to transmit and verify transmitted data for anomalies. 

  //  I should now send the data in the buffer to the UART peripheral.
  LIN_frame temp_frame = read_fromBuffer(ID(LIN_SM.PID));       //  Fetch the data from the buffer.
  
  //  Setup the buffer and data needed for the tramission.
  LIN_TX.verify_index = 0;
  LIN_TX.index = 0;
  LIN_TX.length = calculate_message_lenght(temp_frame.ID);  
  
  for(uint8_t i = 0; i < LIN_TX.length; i++) {
    LIN_TX.data[i] = temp_frame.data[i];
  }
  LIN_TX.data[LIN_TX.length] = LIN_checksum(LIN_TX.data, LIN_TX.length, LIN_SM.PID);   //  The last byte is set as checksum byte.
  LIN_TX.length ++;            //  The lenght of the frame is now 1 byte longer, because of the checksum byte.

  // setting up UART for half duplex transmission
  
  uint8_t ctrla = USART0.CTRLA;
  //ctrla &= ~USART_RXCIE_bm; --->  we can't disable RXCIE, we need to keep listening to check correctness of the data TXed.
  ctrla |=  USART_TXCIE_bm | USART_DREIE_bm;    //  Enabling TX and DRE interrupt, we'll need it for sending the next byte.
  USART0.STATUS = USART_TXCIF_bm;         //  Clearing the TXCIF flag. by writing a 1 to it.
  USART0.CTRLA = ctrla;                   //  Setting the new value of the control register A
  /* by setting ctrla we enable Data Register Empty interrupt, that will immediately call the ISR and start pushing 
  bytes on the serial interface, right?  */  


  //  The interrupt enabled usart tx routine should now be active, we can return from this isr. 
  //  Interrup active at this point: Tx, Rx, DRE  :::<
  //  DRE -> will load next tx byte in the shift register,
  //  TX -> will clear the TXCIF flag. 
  //  RX -> will check the correctness of the data, and will abort transmission if error detected in loopback. 
}

/**
 * @brief State entered after setting up the transmission of a response frame.
 * This state will verify the data sent, by comparing it to the data received since we operate in 
 * half-duplex mode.
 * If the data sent is incorrect we'll abort the transmission. 
 */
void lin_isr::SM_verify_sent_data(){

  // Check the latest byte with the expected byte we should have transmitted.
  if ( LIN_SM.new_byte == LIN_TX.data[LIN_TX.verify_index]){
    //  The byte was correct, 
    //  I'll increment the index so next time I'll check the next byte of the tx buffer
    LIN_TX.verify_index++;                    
  }else{
    //  The byte read differs from what I should have transmitted, 
    //  I'll have to abort the transmission.
    abort_transmission();

    //  Also I'll set state to ignore since I won't have to verify new data, and I need to 
    //  wait for the next PID byte to start next frame activities
    LIN_SM.state = SM_ignore;
  }
}




/**
 * @brief Stop transmission at register level. 
 * Disables TX and DRE interrupts. 
 * Clean TXcif Interrupt Flag.  * 
 */
void lin_isr::abort_transmission(){

  uint8_t ctrlA = USART0.CTRLA;
  uint8_t status = USART0.STATUS;
  
  ctrlA = ctrlA & ~(USART_TXCIE_bm | USART_DREIE_bm);    //  Disabling TX and DRE interrupts.
  status = status | USART_TXCIF_bm;                      //  Clearing the TXCIF flag, if set, by writing a 1 to it.

  USART0.CTRLA = ctrlA;                   //  Setting the new value of the control register A
  USART0.STATUS = status;                 //  Setting the new value of the status register, after TX interrupt has been disabled.
}



ISR(USART0_DRE_vect) {

  // There is more data in the output buffer. Send the next byte
  uint8_t byte_out = LIN_SM.data[LIN_TX.index];

  // clear the TXCIF flag -- "can be cleared by writing a one to its bit location". This makes sure flush() 
  // won't return until the bytes actually got written. It is critical to do this BEFORE we write the next byte
  USART0.STATUS = USART_TXCIF_bm;
  USART0.TXDATAL = byte_out;             //we write the next byte

  LIN_TX.index = LIN_TX.index + 1;

  if (LIN_TX.index == LIN_TX.length) {    
    // Buffer empty, so disable "data register empty" interrupt, I won't need to add byte to TXDATAL anymore.
    uint8_t ctrla = USART0.CTRLA;
    ctrla &= ~(USART_DREIE_bm);
    USART0.CTRLA = ctrla;
  }  
}


/**
 * @brief TX isr (TXCIF flag in status register) gets callew when all data has been transmitted out 
 * of the shift register and the transmit buffer (TXDATA) is empty (in the DRE interrupt we couldn't 
 * add data to TXDATA)
 * So we have just to disable the TX and DRE interrupts, and clear TXCIF.
 * (DRE should be already disabled, we'll just clear it to be sure)
 */
ISR(USART0_TXC_vect){
  if(LIN_TX.index == LIN_TX.length) {
    //  All data has been really transmitted, we can now disable the TX and DRE interrupts.
    uint8_t ctrla = USART0.CTRLA;
    uint8_t status = USART0.STATUS;
    ctrla &= ~(USART_TXCIE_bm);           //  Disabling TX interrupt - clear TXCIE in CTRLA
    ctrla &= ~(USART_DREIE_bm);           //  Disabling DRE interrupt - clear DREIE in CTRLA
    status |= USART_TXCIF_bm;             //  Writing a ‘1’ to this bit will clear the flag.

    USART0.CTRLA = ctrla;                 //  Will disable the TX and DRE interrupt
    USART0.STATUS = status;               //  will clear the TXCIF flag. 
  }else{
    //  If for any reason ther's still data to be transmitted, setup the next byte to be transmitted.
    uint8_t byte_out = LIN_SM.data[LIN_TX.index];
    USART0.STATUS = USART_TXCIF_bm;
    USART0.TXDATAL = byte_out;             //we write the next byte
  }
}



uint8_t lin_isr::write_toBuffer(uint8_t ID, volatile uint8_t *data_8)
{
  //  if ID is in TX_ID, then write the data, return 0 for correct operation.
  if(ID_map[ID].type == RX_type){

    LIN_frame temp_frame;
    temp_frame.ID = ID;

    for (uint8_t i = 0; i < 8; i++){
      temp_frame.data[i] = data_8[i];
    } 

    Data_buffer[ID_map[ID].data_vec_address] = temp_frame;
    return 0;
  }
  else
    return -1;
}



/**
 * @brief Will retrieve the LIN_frame to be tx inside the buffer.
 * @param ID - the ID (6 bit) of the frame to be retrieved.
 */
LIN_frame lin_isr::read_fromBuffer(uint8_t ID)
{
  //  if ID is in RX_ID, then return the data
  //  else return an empty frame
  LIN_frame temp_frame;

  if(ID_map[ID].type == TX_type)
    return Data_buffer[ID_map[ID].data_vec_address];
  else

  return temp_frame;
}



/**
 * @brief Checksum calculation function.
 * @param data - POINTER to array of data bytes.
 * @param lenght - POINTER to the lenght of the data array.
 * @param PID_byte - the Protected ID (all 8 bits) byte. used in LIN 2.0
 */
uint8_t lin_isr::LIN_checksum(volatile uint8_t* data, volatile  uint8_t lenght, volatile  uint8_t PID_byte) {
  uint16_t checksum = 0;

  #ifdef LIN_VERSION_2_0        //  If it's LIN 2.0 the entire PID byte is included in the checksum calculation.
    checksum = PID_byte;
  #endif

  for (uint8_t i = 0; i < lenght; i++)
  {
    checksum += data[i];
    if(checksum > 0xFF){
      checksum -= 0xFF;
    }
    //    checksum <= 0xFF ? : checksum-= 0xFF ;
  }
  uint8_t checksum_8 = ~checksum;                             //  Algorithm ask for the checksum to be inverted.
  return checksum_8;
}



/**
 * @brief Calculate the lenght of a lin message based on the ID.
 * note, for LIN 1.3 will always return 8.
 */
uint8_t lin_isr::calculate_message_lenght(uint8_t ID){
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













/**
 * @brief Setup of registers to begin LIN communication.
 * @param baud - the baud rate of the LIN bus. Between 1'000 and 20'000
 */      
LIN_ERROR LIN_slave::begin_LIN_Slave(unsigned long baud ) {

  //checking if the dataBuffer has been initialized:
  if(ID_total < 1){
    return ERROR_NOT_INITIALIZED;
  }

  //  checking the baud rate validity:
  if(baud > MIN_BAUD && baud < MAX_BAUD){
    return ERROR_BAUD_RATE;
  }
      
  uint8_t   ctrla = 0, ctrlb = 0, ctrlc = 0;    //  Empty control register target

  ctrla |= USART_RXCIE_bm;          //  Enable RX ISR, not Tx ISR,  do not set loopback.
  
  ctrlb |= USART_RXEN_bm;          //  Enable RX module of USART - not the interrupt
  ctrlb |= USART_TXEN_bm;          //  Enable TX module of USART - not the interrupt
  ctrlb |= USART_RXMODE_0_bm;      //  Setting the LINAUTO mode writing 0x03 to RXMODE field of CTRLB
  ctrlb |= USART_RXMODE_1_bm;

  //  CTRLC = 00 - async. mode | 00 - parity dis. | 0 - one Stop bit | 011 - 0x03 - 8 bit mode
  //  using defined values for async, no parity, 8 bit 1 stop bit:
  ctrlc = 0b00000011;

  
  uint16_t baud_setting = 0;                // at this point it should be able to reuse those 2 registers that it received options in!
  baud_setting = (((4 * F_CPU) / baud));    // And now the registers that baud was passed in are done.
  
  //  ctrlb & 0b11001000
  uint8_t setpinmask = ctrlb & 0xC8;        // ODME in bit 3, TX and RX enabled in bit 6, 7
  
  #ifdef LOOPBACK                         // if it's open-drain and loopback, need to set state bit 2.
    setpinmask             |= 0x10;       // this tells _set_pins not to disturb the configuration on the RX pin. // Now we should be able to ST _state.
  #endif
  
  uint8_t oldSREG = SREG;                   //  Saving Status Registers, before disabling interrupts:
  cli();                                    //  Disablng interrupts
  //volatile USART_t* MyUSART = USART0; //_hwserial_module;
  USART0.CTRLB          = 0;            // gotta disable first - some things are enable-locked.
  USART0.CTRLC          = ctrlc;        // No reason not to set first.
  USART0.BAUD           = baud_setting; //  
  USART0.EVCTRL       = 0;              // This needs to be turned off - no IR comm used
  
                                        // finally strip out the SERIAL_EVENT_RX bit which is in the DREIE
  USART0.CTRLA          = ctrla & 0xDF; // position, which we never set in begin.
  
  USART0.CTRLB          = ctrlb;        // Set the all important CTRLB...
  _set_pins_lin(0, 0, setpinmask);      // set up the pin(s)
  SREG = oldSREG;                       // re-enable interrupts, and we're done.  
}       


void LIN_slave::_set_pins_lin(uint8_t mod_nbr, uint8_t mux_set, uint8_t enmask) {
  
  //  PORTMUX.CTRLB - Bit 0 – USART0 Write this bit to '1' to select alternative communication pins for USART 0.
  PORTMUX.CTRLB       &= 0xFE;   // &= 0b11111110 Will make sure to reset the first bit

    
  const uint8_t* muxrow = &(_usart_pins[0][0]);     // for 1614 standard pin. It was: _usart_pins[mod_nbr + mux_set][0]);
  
  
  if ((enmask & 0x40 && !(enmask & 0x08))) {      //  enmask & 0b01000000 && ! enmask & 0b00001000    --  never happens for LIN , right?!
    pinMode(muxrow[0], OUTPUT);                   // If and only if TX is enabled AND NOT open drain -  should the TX interrupt be used. .
  } 
  else if (enmask & 0x50) {   //    & 0101'0000 -> TXEN & LOOPBACK if it is enabled but is in open drain mode, or is disabled, but loopback is enabled
    // TX should be INPUT_PULLUP.
    pinMode(muxrow[0], INPUT_PULLUP);
  }
  
  //      1000'0000(rxen)   & not 0001'0000(LB)
  if (enmask & USART_RXEN_bm && !(enmask & 0x10)) {        //  if ctrlb  RXEN but NOT loopback
    // Likewise if RX is enabled, unless loopback mode is too (in which case we caught it above, it should be pulled up
    pinMode(muxrow[1], INPUT_PULLUP);
  }
  
}


/**
 * @brief Constructor for the LIN_slave class.
 */
LIN_slave::LIN_slave(){}





LIN_ERROR LIN_slave::initialize_numberOfIDs(uint8_t number_of_ID_used){

  //  checking if the Data_buffer has already bitialized: 
  if(ID_total > 0){
    return ERROR_ALREADY_INITIALIZED;
  }

  //  checking if number of IDs asked is within LIN specs range
  if(number_of_ID_used > MAX_ID){
    return ERROR_ID_OUT_OF_BOUND;
  }
  

  ID_total = number_of_ID_used;

  ID_available = ID_total;
  ID_next_empty = 0;

  Data_buffer = (LIN_frame *) malloc(ID_total * sizeof(LIN_frame));
  
  if ( Data_buffer == NULL){
    ID_total = 0;
    ID_available = MAX_ID;
    return ERROR;
  }
  else{
    return LIN_OK;
  }
  
}




/**
 * @brief public: Will retrieve the LIN_frame to be tx inside the buffer.
 * @param ID - the ID (6 bit) of the frame to be retrieved.
 */
LIN_frame LIN_slave::get_frame_from_RxBuffer(uint8_t ID)
{
  //  if ID is in RX_ID, then return the data
  //  else return an error frame
  LIN_frame error_frame;

  switch (ID_map[ID].type)
  {
  case RX_type:
    return Data_buffer[ID_map[ID].data_vec_address];
  
  case TX_type:
    error_frame.error = ERROR_WRONG_ID_TYPE;
    break;

  case none:
    error_frame.error = ERROR_ID_NOT_AVAILABLE;
    break;
  
  default:
    error_frame.error = ERROR;
    break;
  }

  
  return error_frame;
}



/**
 * @brief Write the 8 byte array to the buffer at given ID.
 * @param ID - the ID to be written at.
 * @param data_8 - pointer to array of data to be written to the buffer.
 */
LIN_ERROR LIN_slave::write_data_to_TxBuffer(uint8_t ID, uint8_t *data_8)
{
  //  if ID is in TX_ID, then write the data, 
  if(ID_map[ID].type == TX_type){

    LIN_frame temp_frame;
    temp_frame.ID = ID;

    for (uint8_t i = 0; i < 8; i++){
      temp_frame.data[i] = data_8[i];
    } 

    Data_buffer[ID_map[ID].data_vec_address] = temp_frame;
    return LIN_OK;
  }
  else
    return ERROR_WRONG_ID_TYPE;
}


LIN_ERROR LIN_slave::add_RX_ID(uint8_t ID)
{
  return add_to_list_ID(ID, RX_type);
}
LIN_ERROR LIN_slave::add_TX_ID(uint8_t ID)
{
  return add_to_list_ID(ID, TX_type);  
}

/**
 * @brief Will add a ID to the list of IDs that this node will respond to.
 * @param _ID - the ID to be added to the list.
 * @param _type - the type of action associated with the ID.
 */
LIN_ERROR LIN_slave::add_to_list_ID(uint8_t _ID,  ID_type _type){

  if(_ID >= MAX_ID)
    return ERROR_ID_OUT_OF_BOUND;

  if(ID_available < 1)  
      return ERROR_ID_FULL;

  //  check if id is still free:
  if(ID_map[_ID].type == none)
  {
    ID_map[_ID].type = _type;
    ID_map[_ID].data_vec_address = ID_next_empty;
    ID_next_empty++;
    ID_available--;
    return LIN_OK;
  }
  else
    return ERROR_ID_NOT_AVAILABLE;
}



#endif
