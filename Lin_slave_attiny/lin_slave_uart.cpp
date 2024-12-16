      
      /*
One-Wire Half Duplex Mode:
1. Internally connect the TXD to the USART receiver (the LBME bit in the USARTn.CTRLA register).
2. Enable internal pull-up for the RX/TX pin (the PULLUPEN bit in the PORTx.PINnCTRL register).
3. Enable Open-Drain mode (the ODME bit in the USARTn.CTRLB register).
4. Set the baud rate (USARTn.BAUD).
5. Set the frame format and mode of operation (USARTn.CTRLC).
6. Enable the transmitter and the receiver (USARTn.CTRLB).
*/

#include "core_devices.h"
#include <inttypes.h>
//#include "api/Stream.h"
#include "pins_arduino.h"
#include "UART_constants.h"
//#include "UART_check_pins.h"
//#include <HardwareSerial.h>
#include "Arduino.h"



#include "lin_slave_uart.h"

//#define LOOPBACK
#define USE_LIN_MODE




/**
 * @brief pointer to a state function of the Lin state machine.
 * @param data_in - the data received from the UART peripheral.
 * @param is_ID - true if the data is an ID byte, false otherwise.
 */
typedef void (*LIN_statemachine_fn)();


typedef struct LIN__state_machine_st {
  LIN_statemachine_fn state;
  uint8_t PID = 255;
  uint8_t ID = 255;
  uint8_t data[8];
  uint8_t data_index = 0;
  uint8_t new_byte = 0;
  uint8_t data_lenght = 0;
}LIN__state_machine_st;

volatile uint8_t tx_data[9];          //  8 data byte max + 1 checksum byte. 
volatile uint8_t tx_lenght;
volatile uint8_t tx_index;
volatile uint8_t verified_index;
volatile uint8_t current_data_lenght;


volatile LIN__state_machine_st LIN_SM;



ISR(USART0_RXC_vect) {
  // Needs to update the LIN_slave state machine...

  // Recieve data, pass it to the state machine function poiinted by the state pointer:
  uint8_t rxDataH = USART0.RXDATAH;
  uint8_t   data  = USART0.RXDATAL; 

  if(rxDataH & (USART_PERR_bm | 0x01)) {
    // Parity error in a PID byte, discard the byte.
    return;
  }
  
  if(rxDataH & USART_FERR_bm) {
    // Frame error, discard the byte.
    return;
  }
  
  //possibly valid bte valid onl for ID byte... still need to check.   
  bool PID_byte_flag = (rxDataH & 0x01);  // Checking if it's an ID byte

  if(PID_byte_flag)      //  if it's an ID byte    I have to start over the reception
  {
    LIN_SM.state = SM_PID;    //  Sp I set again the current fn to the PID one.
  /*  TODO: will need to make sure to stop transmission? no, during transmission rx isr is disabled. */
  }

  LIN_SM.new_byte = data;
  LIN_SM.state(); 
}

void SM_PID() {
  uint8_t data_in = LIN_SM.new_byte;
  LIN_SM.PID = data_in & 0b00111111;        //  I cancel out the 2 MSb to get rid of parity bits.
  LIN_SM.current_data_index = 0;            //  new frame is starting, so I reset the data index.
  LIN_SM.data_lenght = calc_message_lenght(LIN_SM.PID);
    
  
  //  I'll now have to check between the programmed IDs if I should Listen to, respond to or ignore the frame...
  switch (ID_map[LIN_SM.ID].type)
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
void SM_ignore() {
}


/**
 * @brief State of active recieving data, we're accumulating new bytes.
 * When we're done we'll verify the checksum. 
 * If the checksum is correct, we'll store the data in the buffer.
 * Then we'll transition to ignore state, SM_ignore.
*/
void SM_listen() {

  //  Checks if we're still receiving data:
  if(LIN_SM.data_index < LIN_SM.data_lenght) {
    LIN_SM.data[LIN_SM.data_index] = LIN_SM.new_byte;           //  Add the new byte to the LIN_SM recieving buffer
    LIN_SM.data_index++;                                        //  Increment the index of the LIN_SM buffer.
  }
  else     // if we recieved the checksum byte:
  {    
    uint8_t checksum = LIN_checksum(LIN_SM.data , &(LIN_SM.data_lenght), LIN_SM.PID);
    
    //  Check if the recieved byte matches the calculated checksum. 
    if(checksum == LIN_SM.new_byte)                             //  The checksum just recieved is correct, 
      write_data_toBuffer(LIN_SM.ID, LIN_SM.data);                // I can now store the data in the buffer.
    else                                                        //  The checksum is INCORRECT, I will ignore the frame.     
      LIN_SM.state = SM_ignore;                                   //  I'll set the state to the SM_ignore.    
  }
}

void SM_respond() {
  LIN_SM.state = SM_verify_sent_data;    //  from now on we'll have to transmit and verify transmitted data for anomalies. 

  //  I should now send the data in the buffer to the UART peripheral.
  LIN_frame temp_frame = get_frame_fromBuffer(LIN_SM.PID);       //  Fetch the data from the buffer.
  
  //  Setup the buffer and data needed for the tramission.
  verified_index = 0;
  tx_index = 0;
  tx_lenght = calc_message_lenght(temp_frame.ID);  
  
  for(uint8_t i = 0; i < tx_lenght; i++) {
    tx_data + i = temp_frame.data[i];
  }
  tx_data[tx_lenght] = LIN_checksum(tx_data, &tx_lenght, LIN_SM.PID);   //  The last byte is set as checksum byte.
  tx_lenght ++;            //  The lenght of the frame is now 1 byte longer, because of the checksum byte.

  // setting up UART for half duplex transmission, first byte, 
  if(tx_lenght == 1) {   //  anomaly, but will include not to break transmission if it happens???
    uint8_t ctrla = USART0.CTRLA;
    //ctrla &= ~USART_RXCIE_bm;    /* TODO verify */           //  Disabling rx interrupt, we'll check manually the correctness of the data.
    ctrla |=  USART_TXCIE_bm;               //  Enabling tx interrupt, we'll need it for sending the next byte.
    USART0.STATUS = USART_TXCIF_bm;         //  Clearing the TXCIF flag. by writing a 1 to it.
    USART0.CTRLA = ctrla;                   //  Setting the new value of the control register A
    /* MUST clear TXCIF **before** writing new char, 
    otherwise ill-timed interrupt can cause it to erase the flag after the new charchter has been sent!*/
    USART0.TXDATAL = tx_data[0];     //  Sending the only byte of the frame.
  }
  else {
    uint8_t ctrla = USART0.CTRLA;
    //ctrla &= ~USART_RXCIE_bm;       /* TODO verify */         //  Disabling rx interrupt, we'll check manually the correctness of the data.
    ctrla |=  USART_TXCIE_bm | USART_DREIE_bm;    //  Enabling tx and DRE interrupt, we'll need it for sending the next byte.
    USART0.STATUS = USART_TXCIF_bm;         //  Clearing the TXCIF flag. by writing a 1 to it.
    USART0.CTRLA = ctrla;                   //  Setting the new value of the control register A
    /* by setting ctrla we enable Data Register Empty interrupt, that will immediately call the ISR and start pushing 
    bytes on the serial interface, right?  TODO DRE isr */  
  }

  //  The interrupt enabled usart routine should now be active, we can return from this isr. 
}

/**
 * @brief State entered after setting up the transmission of a response frame.
 * This state will verify the data sent, by comparing it to the data received since we operate in 
 * half-duplex mode.
 * If the data sent is incorrect we'll abort the transmission. 
 */
void SM_verify_sent_data(){

}


//rivedere che cazzo fa sta roba esattamente 
ISR(USART0_TXC_vect){
  uint8_t _data;

  //Writing a ‘1’ to this bit will clear the flag.
  uint8_t status = USART0.STATUS;  
  status |= USART_TXCIF_bm;

 /*
      DI SICURO NON VERIFICO COSI LA CORRETTEZZA DI CIO CHE HO TRASMESSO!
  
  for(uint8_t i; i< 10; i++)
  {
    if(USART0.STATUS & USART_RXCIF_bm){
    _data = USART0.RXDATAL;
    break;
    }
  }

  if(_data != tx_data + verify_index)*/
  /* TODO: transmitted byte doesn't match recieved, fault, needs to stop the transmission! */

}

/*  TUTTO DA CONTROLLARE */
ISR(USART0_DRE_vect) {
  //USART_t* usartModule      = USART0; //(USART_t*)HardwareSerial._hwserial_module;  // reduces size a little bit
  uint8_t txTail  = 0;  //HardwareSerial._tx_buffer_tail;

  // There must be more data in the output buffer. Send the next byte
  uint8_t byte_out = LIN_SM.current_data + tx_index;

  // clear the TXCIF flag -- "can be cleared by writing a one to its bit location". This makes sure flush() 
  // won't return until the bytes actually got written. It is critical to do this BEFORE we write the next byte
  USART0->STATUS = USART_TXCIF_bm;
  USART0->TXDATAL = byte_out;             //we write the next byte

  tx_index = tx_index + 1;
  uint8_t ctrla = USART0->CTRLA;

  if (tx_index == tx_lenght) {
    // Buffer empty, so disable "data register empty" interrupt
    ctrla &= ~(USART_DREIE_bm);
    USART0->CTRLA = ctrla;
  }  
}





/**
 * @brief Checksum calculation function.
 * @param data - POINTER to array of data bytes.
 * @param lenght - POINTER to the lenght of the data array.
 * @param PID_byte - the Protected ID (all 8 bits) byte. used in LIN 2.0
 */
uint8_t LIN_checksum(uint8_t* data, uint8_t* lenght, uint8_t PID_byte) {
  uint16_t checksum = 0;

  #ifdef LIN_VERSION_2_0        //  If it's LIN 2.0 the entire PID byte is included in the checksum calculation.
    checksum = PID_byte;
  #endif

  for (uint8_t i = 0; i < *lenght; i++)
  {
    checksum += data[i];
    checksum <= 0xFF ? : checksum-= 0xFF ;
  }
  uint8_t checksum_8 = ~checksum;                             //  Algorithm ask for the checksum to be inverted.
  return checksum_8;
}


ISR(USART0_TXC_vect, ISR_NAKED) {
        __asm__ __volatile__(
              "push      r30"     "\n\t"
              "push      r31"     "\n\t"
              :::);
        __asm__ __volatile__(
#if PROGMEM_SIZE > 8192
              "jmp   _do_txc_lin"     "\n\t"
#else
              "rjmp   _do_txc_lin"    "\n\t"
#endif
              ::"z"(0x0800));//(&Serial0));
        __builtin_unreachable();
}

void __attribute__((naked)) __attribute__((used)) __attribute__((noreturn)) _do_txc_lin(void) {
        __asm__ __volatile__(
          "_do_txc_lin:"                      "\n\t" // We start out 11-13 clocks after the interrupt
            "push       r24"              "\n\t" // r30 and r31 pushed before this.
            "in         r24,      0x3f"   "\n\t"  // Save SREG
            "push       r24"              "\n\t"  //
            "push       r25"              "\n\t"  //
            "push       r28"              "\n\t"  //
            "push       r29"              "\n\t"  //
            "ldd        r28,   Z +  8"    "\n\t"  // Load USART into Y pointer, low byte
            "ldi        r29,     0x08"    "\n\t"  // all USARTs are 0x08n0 where n is an even hex digit.
            "ldd        r25,   Y +  5"    "\n\t"  // Y + 5 = USARTn.CTRLA read CTRLA
          "_txc_flush_rx_lin:"                "\n\t"  // start of rx flush loop.
            "ld         r24,        Y"    "\n\t"  // Y + 0 = USARTn.RXDATAL rx data
            "ldd        r24,   Y +  4"    "\n\t"  // Y + 4 = USARTn.STATUS
            "sbrc       r24,        7"    "\n\t"  // if RXC bit is clear...
            "rjmp       _txc_flush_rx_lin"    "\n\t"  // .... skip this jump to remove more from the buffer.
            "andi       r25,     0xBF"    "\n\t"  // clear TXCIE
            "ori        r25,     0x80"    "\n\t"  // set RXCIE
            "std     Y +  5,      r25"    "\n\t"  // store CTRLA
//          "ldd        r24,   Z + 12"    "\n\t"
//          "ahha,   always,     true"    "\n\t"  // wait, if we're in TXC, We are in half duplex mode, duuuuh
//          "sbrs       r24,        2"    "\n\t"  // if we're in half duplex skip...
//          "rjmp      .+ 6"              "\n\t"  // a jump over the next three instructoins. Do do them iff in half duplex only
//          "ori        r24,     0x10"    "\n\t"  // add the "there's an echo in here" bit
//          "std     Z + 12,      r24"    "\n\t"  // Store modified state
            "pop        r29"              "\n\t"
            "pop        r28"              "\n\t"
            "pop        r25"              "\n\t"
            "pop        r24"              "\n\t"  // pop r24 to get old SREG back
            "out       0x3F,      r24"    "\n\t"  // restore sreg.
            "pop        r24"              "\n\t"  // pop r24 restore it
            "pop        r31"              "\n\t"  // and r31
            "pop        r30"              "\n\t"  // Pop the register the ISR did
            "reti"                        "\n"    // return from the interrupt.
            ::
          );
        __builtin_unreachable();
      }

      
void LIN_slave::begin_LIN_Slave(unsigned long baud ) {
      
  uint8_t   ctrla = 0, ctrlb = 0, ctrlc = 0;    //  Empty control register target

  ctrla |= USART_RXCIE_bm;          //  Enable RX ISR
  //ctrla |= USART_TXCIE_bm;        //  Enable TX ISR

  #ifdef LOOPBACK
  //ctrla |= USART_LBME_bm;           //  Enable LoopBack ? 
  #endif
  
  ctrlb |= USART_RXEN_bm;          //  Enable RX  
  ctrlb |= USART_TXEN_bm;          //  Enable TX
  ctrlb |= USART_ODME_bm;          //  Set Open Drain Mode Enable - needed for loopback
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
  
   //  CTRLB - Bit 0 – USART0Write this bit to '1' to select alternative communication pins for USART 0.
  PORTMUX.CTRLB       &= 0xFE;   // &= 0b11111110 Will make sure to reset the first bit

    
  const uint8_t* muxrow = &(_usart_pins[0][0]);     //  It was: _usart_pins[mod_nbr + mux_set][0]);
  
  
  if ((enmask & 0x40 && !(enmask & 0x08))) {      //  enmask & 0b01000000 && ! enmask & 0b00001000    --  never happens for LIN , right?!
    pinMode(muxrow[0], OUTPUT);                   // If and only if TX is enabled AND NOT open drain -  should the TX interrupt be used. .
  } 
  else if (enmask & 0x50) {   //    & 0101'0000 -> TXEN & LOOPBACK if it is enabled but is in open drain mode, or is disabled, but loopback is enabled
    // TX should be INPUT_PULLUP.
    pinMode(muxrow[0], INPUT_PULLUP);
  }
  
  //      1000'0000(rxen)   & not 0001'0000(LB)
  if (enmask & 0x80 && !(enmask & 0x10)) {        //  if ctrlb  RXEN but NOT loopback
    // Likewise if RX is enabled, unless loopback mode is too (in which case we caught it above, it should be pulled up
    pinMode(muxrow[1], INPUT_PULLUP);
  }
  
}



LIN_slave::LIN_slave(uint8_t number_of_ID_used, uint16_t baud)
{  
  if(number_of_ID_used > MAX_ID)
    number_of_ID_used = MAX_ID;
  else
    ID_total = number_of_ID_used;

  ID_available = ID_total;
  ID_next_empty = 0;

  Data_buffer = malloc(ID_total * sizeof(LIN_frame));
  // Should I catch the possible exception here?

  //  Setting the baud rate
  if(baud > MIN_BAUS && baud < MAX_BAUS)
    baud = baud;
  else
    baud = DEFAULT_BAUD;    //  Default baud rate
}

/**
 * @brief Will retrieve the LIN_frame to be tx inside the buffer.
 */
LIN_frame LIN_slave::get_frame_fromBuffer(uint8_t ID)
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
 * @brief Write the 8 byte data_8 array to the buffer at address ID.
 */
uint8_t LIN_slave::write_data_toBuffer(uint8_t ID, uint8_t *data_8)
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


uint8_t LIN_slave::add_RX_ID(uint8_t ID)
{
  add_to_list_ID(ID, RX_type);
}
uint8_t LIN_slave::add_TX_ID(uint8_t ID)
{
  add_to_list_ID(ID, TX_type);  
}

uint8_t LIN_slave::add_to_list_ID(uint8_t _ID,  ID_type _type){

  if(_ID >= MAX_ID)
    return -1;

  if(ID_available < 1)  
      return -1;

  if(ID_map[_ID].type == none)
  {
    ID_map[_ID].type = _type;
    ID_map[_ID].data_vec_address = ID_next_empty;
    ID_next_empty++;
    ID_available--;
    return 0;
  }
  else
    return -1;
}


/**
 * @brief Calculate the lenght of a lin message based on the ID.
 * note, for LIN 1.3 will always return 8.
 */
uint8_t calc_message_lenght(uint8_t ID){
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


