/*
 * Author:  Jesus Minjares
 * Date:    11/22/2020
 * App:
 *     Get user input via keyboard using UART0 interrupt. Once
 *     the software captures an character it store it into a buffer.
 *     Once the interrupt detects an carriage return it display full message
 *     and then sends it to client via bluetooth. UART0 is set at 9600 baud rate with
 *     1 stop bit no parity. The system is running at 3Mhz.
 * */
#include "msp.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

#define LEN 20
/*Clock*/
void set3Mhz(); //set the DCO clock @ 3Mhz

/* UART0 */
void UART0Setup();//initialize UART0
void sendUART0(char *message); //send char * via UART0

/* UART2 | Bluetooth */
void bluetoothSetup(); //initialize UART2
void sendBluetooth(char *message); //send char * via UART2
void sendChar(char c); //send char via UART0

/* Global Variables */
char buffer[LEN],bluetoothBuffer[LEN]; //create buffers of 10 bytes
uint8_t index = 0, bluetoothIndex = 0; //integer varaible of 8 bytes
/* Encryption/Decryption variables
   const unsigned char *key = "TESTKEY";
   unsigned char encryption[8], decryption[8];
   unsigned char en[8], de[8];
*/
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    /*  DES initialization:
            DESSetting();
            des_init(key,cfg);
     */
    set3Mhz(); //set clock @ 3Mhz
    UART0Setup(); //initialize UART0
    bluetoothSetup(); //initialize UART2
    sendUART0("Server Module...\r\n"); //send message to UART0

    __enable_irq(); //enable global interrupts
    while(1){} //infinite loop
}
/*
 * UART0 INTERRUPT Subroutine:
 *      use:
 *          store keyboard input to buffer and send
 *          input via bluetooth
 */
void EUSCIA0_IRQHandler(void){
    if(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG){ //check RX flag
        buffer[index] = EUSCI_A0->RXBUF; //store character to the current index of the buffer
        sendChar(buffer[index]); //print char to UART0
        if(buffer[index++] == '\r'){ //check if carriage return ,and use post increment
            //print buffer
            sendUART0("Input: ");
            sendUART0(buffer);
            sendUART0("\r\n");
            //send via bluetooth
            sendBluetooth(buffer);
            sendBluetooth("\r\n");
            //use memset to clear the buffer
            memset(buffer,0,sizeof(buffer));
            index = 0; //reset index to 0
        }
    }
}
/*
 * @param   None
 * @return  None
 * @note    set3Mhz will set the main clock at 3Mhz
 */
void set3Mhz(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 |CS_CTL1_SELM_3; // Select ACLK = REFO// MCLK = DCO // SMCLK = DCO
    CS->KEY = 0; //lock CS module
}
/**
 * @param   None
 * @return  None
 * @note    UART0() set UART0 to send data via serial port/usb
 *          9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void UART0Setup(){
    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P1->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;  // Configure eUSCI clock source for SMCLK
    EUSCI_A0->BRW = 19;  // 3000000/16/9600  = 19.53125
    EUSCI_A0->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}
/**
 * @param   message(string) to sent via UART0
 * @return  None
 * @note    9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void sendUART0(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){ //iterate over the length of the string
     while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //check TX flag
            EUSCI_A0->TXBUF = message[i]; //send char
    }
    return;
}
/**
 * @param   None
 * @return  None
 * @note    bluetoothSetup() will set UART2
 *          9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void bluetoothSetup(){
    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P3->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A2->BRW = 19;  // 3000000/16/9600  = 19.53125
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}
/**
 * @param   message(string) to sent via bluetooth
 * @return  None
 * @note    9600 baud rate
 *          1 stop bit
 *          0 parity
 */
void sendBluetooth(char *message){
    int i;
    for(i = 0; i < strlen(message); i++){ //iterate over the length of the string
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); //check flag
            EUSCI_A2->TXBUF = message[i]; //send char
    }
    return;
}
/**
 * UART2 INTERRUPT Subroutine:
 *      use:
 *          store message sent via bluetooth
 */
void EUSCIA2_IRQHandler(void){
    if(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){ //check RX flag
        bluetoothBuffer[bluetoothIndex] = EUSCI_A2->RXBUF; //store RX
        if(bluetoothBuffer[bluetoothIndex++] == '\n'){ //check for '\n' & use post increment
            //print to UART0
            sendUART0("bluetooth: ");
            sendUART0(bluetoothBuffer);
            //use memset to clear bluetoothBuffer
            memset(&bluetoothBuffer,0,sizeof(bluetoothBuffer));
            bluetoothIndex = 0;
        }
    }
}
/**
 * @param   character to be sent via UART0
 */
void sendChar(char c){
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
           EUSCI_A0->TXBUF = c;
}
