      
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
  LIN_statemachine_fn current_state;
  uint8_t current_PID = 255;
  uint8_t current_data[8];
  uint8_t current_data_index = 0;
  uint8_t new_byte = 0;
}LIN__state_machine_st;

volatile tx_data[9];
volatile tx_lenght;
volatile tx_index;
volatile verify_index;
//volatile tx_checksum;


LIN__state_machine_st LIN_SM;



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
  bool PID_byte = (rxDataH & 0x01);  // Checking if it's an ID byte

  if(PID_byte)      //  if it's an ID byte    I have to start over the reception
  {
    LIN_SM.current_state = SM_PID;    //  Sp I set again the current fn to the PID one.
  /*  TODO: will need to make sure to stop transmission? no, during transmission rx isr is disabled. */
  }

  LIN_SM.new_byte = data;
  LIN_SM.current_state(); 
}

void SM_PID() {
  uint8_t data_in = LIN_SM.new_byte;
  uint8_t pid = data_in>>2;                     //  I shift the data_in 2 bits to the right to get rid of parity bits.
  LIN_SM.current_data_index = 0;     //  new frame is starting, so I reset the data index.

  // The parity check is done by the uart peripheral, so I can just discard the parity bits (6,7).
  LIN_SM.current_PID = pid;   
  
  //  I'll now have to check between the programmed IDs if I should Listen to, respond to or ignore the frame...
  switch (PID_map[pid].type)
  {
    case RX_type:
      LIN_SM.current_state = SM_listen;
      break;
    case TX_type:
      LIN_SM.current_state = SM_respond;
      break;
    default:
      LIN_SM.current_state = SM_ignore;
      break;  
  }
}
void SM_ignore() {
}

void SM_listen() {
  uint8_T current_data_lenght = PID_map[LIN_SM.current_PID].data_lenght;
  if(LIN_SM.current_data_index < current_data_lenght) {
    LIN_SM.current_data[LIN_SM.current_data_index] = LIN_SM.new_byte;
    LIN_SM.current_data_index++;
  }
  else {
    // I'ts the checksum byte, I should check it.
    if(LIN_checksum( LIN_SM.current_data , &current_data_lenght) == LIN_SM.new_byte) {
      //  The checksum is correct, I can now store the data in the buffer.
      LIN_frame temp_frame;
      temp_frame.PID = LIN_SM.current_PID;
      for (uint8_t i = 0; i < current_data_lenght; i++){
        temp_frame.data[i] = LIN_SM.current_data[i];
      }
      temp_frame.lenght = current_data_lenght;
      write_data(temp_frame);
    }
    else {
      //  The checksum is incorrect, I should discard the frame.
      //  Also, I'll set the current state to the SM_ignore. Reception will restart at next PID byte. 
      LIN_SM.current_state = SM_ignore;
    }
  }
}

void SM_respond() {
  LIN_SM.current_state = SM_ignore;     //  from now on we'll have to transmit only and ignore incoming data.? maybe not necessary

  //  I should now send the data in the buffer to the UART peripheral.
  LIN_frame temp_frame = read_data(LIN_SM.current_PID);       //  Fetch the data from the buffer.
  
  //  Setup the buffer and data needed for the tramission.
  verified_index = 0;
  tx_index = 0;
  tx_lenght = temp_frame.lenght;  
  
  for(uint8_t i = 0; i < tx_lenght; i++) {
    tx_data + i = temp_frame.data[i];
  }
  tx_data[tx_lenght] = LIN_checksum(tx_data, &tx_lenght);   //  The last byte is set as checksum byte.
  tx_lenght ++;       //  The lenght of the frame is now 1 byte longer, because of the checksum byte.

  // setting up UART for half duplex transmission, first byte, 
  if(tx_lenght == 1) {   //  anomaly, but will include not to break transmission if it happens. 
    uint8_t ctrla = USART0.CTRLA;
    ctrla &= ~USART_RXCIE_bm;               //  Disabling rx interrupt, we'll check manually the correctness of the data.
    ctrla |=  USART_TXCIE_bm;               //  Enabling tx interrupt, we'll need it for sending the next byte.
    USART0.STATUS = USART_TXCIF_bm;         //  Clearing the TXCIF flag. by writing a 1 to it.
    USART0.CTRLA = ctrla;                   //  Setting the new value of the control register A
    /* MUST clear TXCIF **before** writing new char, 
    otherwise ill-timed interrupt can cause it to erase the flag after the new charchter has been sent!*/
    USART0.TXDATAL = tx_data[0];     //  Sending the only byte of the frame.
  }
  else {
    uint8_t ctrla = USART0.CTRLA;
    ctrla &= ~USART_RXCIE_bm;               //  Disabling rx interrupt, we'll check manually the correctness of the data.
    ctrla |=  USART_TXCIE_bm | USART_DREIE_bm;    //  Enabling tx and DRE interrupt, we'll need it for sending the next byte.
    USART0.STATUS = USART_TXCIF_bm;         //  Clearing the TXCIF flag. by writing a 1 to it.
    USART0.CTRLA = ctrla;                   //  Setting the new value of the control register A
    /* by setting ctrla we enable Data Register Empty interrupt, that will immediately call the ISR and start pushing 
    bytes on the serial interface. */
  }

  //  The interrupt enabled usart routine should now be active, we can return from this isr. 
}



//Soluzione di merda. 
ISR(USART0_TXC_vect){
  uint8_t _data;

  //Writing a ‘1’ to this bit will clear the flag.
  uint8_t status = USART0.STATUS;  
  status |= USART_TXCIF_bm;

  
  for(uint8_t i; i< 10; i++)
  {
    if(USART0.STATUS & USART_RXCIF_bm){
    _data = USART0.RXDATAL;
    break;
    }
  }

  if(_data != tx_data + verify_index)
  /* TODO: transmitted byte doesn't match recieved, fault, needs to stop the transmission! */

}


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
 */
uint8_t LIN_checksum(uint8_t* data, uint8_t* lenght) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < *lenght; i++)
  {
    checksum += data[i];
    checksum <= 0xFF ? : checksum-= 0xFF ;
  }
  checksum = ~checksum;
  return checksum;
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



LIN_slave::LIN_slave(uint8_t number_of_PID_used, uint16_t baud)
{  
  if(number_of_PID_used > MAX_PID)
    number_of_PID_used = MAX_PID;
  else
    PID_total = number_of_PID_used;

  PID_available = PID_total;
  PID_next_empty = 0;

  Data_buffer = malloc(PID_total * sizeof(LIN_frame));
  // Should I catch the possible exception here?

  //  Setting the baud rate
  if(baud > MIN_BAUS && baud < MAX_BAUS)
    baud = baud;
  else
    baud = DEFAULT_BAUD;    //  Default baud rate
}

LIN_frame LIN_slave::read_data(uint8_t PID)
{
  //  if PID is in RX_PID, then return the data
  //  else return an empty frame
  LIN_frame temp_frame;

  if(PID_map[PID].type == RX_type)
    return Data_buffer[PID_map[PID].data_vec_address];
  else

  return temp_frame;
}

uint8_t LIN_slave::write_data(LIN_frame frame_in)
{
  //  if PID is in TX_PID, then write the data, return 0 for correct operation.
  if(PID_map[frame_in.PID].type == TX_type){
    Data_buffer[PID_map[frame_in.PID].data_vec_address] = frame_in;
    return 0;
  }
  else
    return -1;
}


uint8_t LIN_slave::add_RX_PID(uint8_t PID, uint8_t lenght)
{
  add_to_list_PID(PID, lenght, RX_type);
}
uint8_t LIN_slave::add_TX_PID(uint8_t PID)
{
  add_to_list_PID(PID, lenght, TX_type);  
}

uint8_t LIN_slave::add_to_list_PID(uint8_t _PID, uint8_t lenght,  PID_type _type){

  if(_PID >= MAX_PID)
    return -1;

  if(lenght > MAX_LENGHT)
    return -1;

  if(PID_available < 1)  
      return -1;
  

  if(PID_map[_PID].type == none)
  {
    PID_map[_PID].type = _type;
    PID_map[_PID].data_lenght = lenght;
    PID_map[_PID].data_vec_address = PID_next_empty;
    PID_next_empty++;
    PID_available--;
    return 0;
  }
  else
    return -1;
}

