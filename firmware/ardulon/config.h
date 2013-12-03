//CONFIG PART //////////////////////////////////////////////////////////////////////////////////////  
//debug mode - if avalon is presented not enable it!
//#define DEBUGGING

#define LED_PIN 13             //status led

//AVALON DATA PORTS definition
//RIG1//////////////////

//from 1 AVALON
#define REPORT_P1   25  //(D0)   //NOT CHANGE THIS !
#define REPORT_N1   26  //(D1)   //NOT CHANGE THIS !
#define REPORT_DATA_DETECTOR   30  //(D9)   //NOT CHANGE THIS !

//to AVALON
#define CONFIG_P1   27  //(D2)   //NOT CHANGE THIS !
#define CONFIG_N1   28  //(D3)   //NOT CHANGE THIS !

//reset avalon
#define RESET_PIN   29  //(D6)   //NOT CHANGE THIS !

////////////////////////
//AVALON CONFIGURATION
#define AVALON_FREQUENCY  1000     //MHz 
#define AVALON_ASIC_COUNT 1        
#define AVALON_DATA_SIZE  256    //bytes
#define AVALON_CLK_LOW_CFG    0x00000007            //(F + 1) = freq/25 , R = 0 , OD = 0
#define AVALON_CLK_HIGH_CFG   0x00000000            //reserved

//00000000000000000000000000000111

//////UART////////////////////////////
#define UART_BUFFER_SIZE 256          //bytes
#define MIDSTATE_LENGTH 32            //bytes
#define DATA_LENGTH 12                //bytes
#define JOB_DATA_LENGHT 1 + MIDSTATE_LENGTH + DATA_LENGTH 

//END OF CONFIG PART //////////////////////////////////////////////////////////////////////////////  
