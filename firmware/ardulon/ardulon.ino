/////////////////////////////////////////////////////////////////////////////////////////////////////////
//       ARDULON USB - ARDUINO BASED AVALON SHA256 USB MINER  v0.1b                                    //
//       coded & designed by Michal Maslik                                                             //
//                                                                                                     //
// Copyright (C) 2013, Michal Maslik, mobicek@gmail.com                                                //
//                                                                                                     //
// This program is free software: you can redistribute it and/or modify it under the terms of the      //
// Creative Commons NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)                            //
// http://creativecommons.org/licenses/by-nc-sa/3.0/                                                   // 
//                                                                                                     //
// ONLY FOR NON-COMMERCIAL USAGE WITH ORIGINAL MOBIDRONE OSD HARDWARE.                                 //
//                                                                                                     //
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE COMMONS PUBLIC LICENSE     //
// ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW.              //
// ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.     //
//                                                                                                     //
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO BE BOUND BY THE         //
// TERMS OF THIS LICENSE. TO THE EXTENT THIS LICENSE MAY BE CONSIDERED TO BE A CONTRACT,               //
// THE LICENSOR GRANTS YOU THE RIGHTS CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF            //
// SUCH TERMS AND CONDITIONS.                                                                          //
//                                                                                                     //
// This program is distributed in the hope that it will be useful,                                     //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                                      //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                                //  
//                                                                                                     //
//                                                                                                     //  
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "config.h"

//HW: Arduino Due 
//84Mhz - 100Mips
//1/100Mips = 10nanoSec for one cpu cycle 

#define delay10 __asm__("nop\n\t");   // nop - 10nS
#define delay60 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 10nS
#define delay100 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 10nS
#define delay120 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 10nS
#define delay130 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 10nS

//global variables////////////////////////////////////

//Initialize table of round constants
//first 32 bits of the fractional parts of the cube roots of the first 3 primes
static const volatile uint32_t k[] = {0x428a2f98, 0x71374491, 0xb5c0fbcf};

static volatile uint32_t w[DATA_LENGTH/4];     
static volatile uint32_t m[MIDSTATE_LENGTH/4];     
//comunication - ring buffers

//UART data TX 
static volatile uint8_t uartTxBuffer[UART_BUFFER_SIZE];
static volatile uint16_t uartHeadTx,uartTailTx;

//UART data RX
static volatile uint8_t uartRxBuffer[UART_BUFFER_SIZE];
static volatile uint16_t uartHeadRx,uartTailRx;
static volatile uint16_t newJobFromUart;
static volatile uint16_t dataSum;
static volatile uint16_t counter;
static volatile uint8_t b;

//avalon data TX
static volatile uint8_t avalonDataTX[AVALON_DATA_SIZE];
static volatile uint8_t avalonDataTXBitArray[AVALON_DATA_SIZE * 8];
static volatile uint16_t avalonHeadTx,avalonTailTx;
static volatile uint8_t avalonFrameTransfered;
static volatile uint16_t avalonBitCounterTX;
static volatile uint16_t avalonNumberOfBitsTX;
static volatile uint16_t tmp;
static volatile uint16_t tmp1;
static volatile uint16_t tmp2;
static volatile uint8_t  oneByte;
static volatile uint8_t firstPeriod;

//avalon data RX
static volatile uint8_t avalonDataRX[AVALON_DATA_SIZE];
static volatile int16_t avalonHeadRX;
static volatile uint16_t avalonTailRX;
static volatile uint16_t avalonBitCounterRX;
static volatile uint16_t avalonNumberOfBitsRX;
static volatile uint8_t avalonBitArrayRX[48];

//avalon processing
static volatile uint32_t goldenNonce,receivedNonce;
static volatile uint32_t a0,a1,a2,e0,e1,e2;
static volatile uint16_t nonceNotReaded;

//program variables
static volatile uint32_t mainTimer;
static volatile uint32_t sendTimer;
static volatile uint32_t receiverTimer;
static volatile uint8_t ledState;

void setup() {
  
  //initialise USB
  SerialUSB.begin(115200); // baud rate ignored for usb virtual serial real speed cca. 835KByte/s 
  // initialize serial: 
  Serial.begin(115200);
 
  #ifdef DEBUGGING
    Serial.println("Ardulon miner started");
  #endif
  
  //init values
  initValues();

  pinMode(LED_PIN, OUTPUT);

  pinMode(REPORT_P1,INPUT);
  pinMode(REPORT_N1,INPUT);
  pinMode(REPORT_DATA_DETECTOR,INPUT);

  pinMode(CONFIG_P1,OUTPUT);
  pinMode(CONFIG_N1,OUTPUT);

  pinMode(RESET_PIN,OUTPUT);
  digitalWrite(RESET_PIN,HIGH); //slower as direct port manipulation
  
  //Transmitter
  //direct pin manipulation for higher speed
  REG_PIOD_OWER = 0x0C;  //enable output write - only to D2,D3 (CONFIG_P1,CONFIG_N1)
  REG_PIOD_ODSR = 0x0C;  //d2,d3 to high

  digitalWrite(LED_PIN, ledState);
  
  //Receiver
  //enable interrupt on port D
  pmc_enable_periph_clk(ID_PIOD);
  NVIC_DisableIRQ(PIOD_IRQn);
  NVIC_ClearPendingIRQ(PIOD_IRQn);
  NVIC_SetPriority(PIOD_IRQn, 0);
  NVIC_EnableIRQ(PIOD_IRQn);
 
  //enable rising edge interrupt for D0,D1
  REG_PIOD_AIMER  = 0x03;   //  Aditional interrupt mode low,high,falling,rising
  REG_PIOD_ESR    = 0x03;   // "Edge" Select Register
  REG_PIOD_REHLSR = 0x03;   // "Rising Edge / High Level" Select Register
  REG_PIOD_IER    = 0x03;   // Interrupt enable register
  
  //enable falling edge interrupt for D9
  REG_PIOD_AIMER  |= 1<<9;   //  Aditional interrupt mode low,high,falling,rising
  REG_PIOD_ESR    |= 1<<9;   // "Edge" Select Register
  REG_PIOD_FELLSR  = 1<<9;   // "Falling Edge / Low Level" Select Register
  REG_PIOD_IER    |= 1<<9;   // Interrupt enable register
  
  // Disable interrupt
  //REG_PIOD_IDR = 0x03;  
}

void loop() {

  //for(uint8_t i = 0; i<10 ;i++)    //1 microsecond delay  //este odtestovat !!!!!
  //    delay100;
  
  //read data from avalon buffer and send to uart 
  if(avalonNumberOfBitsRX > 0){

    avalonFrameTransfered = 0; 
    avalonNumberOfBitsRX = 0;  
    
    #ifdef DEBUGGING 
      Serial.println();
      Serial.print("Golden nonce received");
    #endif 
    
    receivedNonce = 0;
    
    //build received nonce variable
    for(uint8_t x = 0; x < 32 ; x++){
        if(avalonBitArrayRX[x] == 1){
           receivedNonce |= (uint32_t) (1 << x);      
        }
    }   
     
    goldenNonce = receivedNonce - 0x180;
    serialize32(byteSwap(goldenNonce)); 
    uartSendGoldenNonce();   
    nonceNotReaded --; 

  }else if(avalonFrameTransfered == 0){      
    if(newJobFromUart > 0){ //send data to avalon  
      buildAvalonFrame(); 
      transmitDataToAvalon();    
      newJobFromUart --;
    }   
  }else if(avalonFrameTransfered == 1){    
    if(newJobFromUart > 0){ //new job arrived but old job is processing so close old job  
      #ifdef DEBUGGING 
        Serial.println("New job was received, delete old job.");
      #endif

      deleteJob();

      buildAvalonFrame(); 
      transmitDataToAvalon();                                    

      newJobFromUart --;   
    }
  }  
  
  checkSerialDataAvailable();
}

void checkSerialDataAvailable() {

  if(SerialUSB.available() >= JOB_DATA_LENGHT) {
      // get the new byte - round buffer
      while(SerialUSB.available()){
         b = SerialUSB.read();
    
         #ifdef DEBUGGING
         Serial.println(b);
         #endif
  
         dataSum += serialize8RX(b);
         counter++;
      } 
      if(dataSum == 0){ //ping only
        uartSendAck();
        counter = 0;
        deleteJob();  //stop job if exist
        //software_Reset();
        newJobFromUart = 0;
        uartRxClean();
        SerialUSB.flush();      
      }else{
        #ifdef DEBUGGING
          Serial.println("Job processing ...");
        #endif
        
        dataSum = 0;
        counter = 0;
        ledState != ledState;
        //digitalWrite(LED_PIN,ledState);
        checkReceivedDataFromUart();       
      }     
  } 
}

//uart ack
void uartSendAck(){
#ifdef DEBUGGING
  Serial.print("Sending ACK: ");
#endif  
  for(uint8_t i = 0; i < 100 ;i++)
     SerialUSB.write((byte)0x00);       
}
void uartSendAckJobReceived(){
#ifdef DEBUGGING
  Serial.print("Sending ACK: ");
#endif
  SerialUSB.write((byte)0x01);   
}
void uartSendAckJobFinished(){
#ifdef DEBUGGING
  Serial.print("Sending ACK: ");
#endif
  SerialUSB.write((byte)0x02);   
}
void uartSendAckJobOutOfSync(){
#ifdef DEBUGGING
  Serial.print("Sending ACK: ");
#endif
  SerialUSB.write((byte)0x03);   
}

//uart rx buffer clean
void uartRxClean(){
  uartTailRx = uartHeadRx;
}

void resetAvalon(){

  #ifdef DEBUGGING 
    Serial.println();
    Serial.println("RESET AVALONS...");
  #endif
  
  digitalWrite(RESET_PIN,LOW);
  delay(100);       //100 milliS
  digitalWrite(RESET_PIN,HIGH); 
}

void software_Reset() {
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
}

void deleteJob(){

  //disable timer interrupt if exist
  stopTimer(TC3_IRQn);
  //reset avalons
  //resetAvalon();  
  
  nonceNotReaded = 0;
  avalonBitCounterRX = 0;
  avalonNumberOfBitsRX = 0;
  avalonNumberOfBitsTX = 0;
  avalonBitCounterTX = 0;
  firstPeriod = 1;
  avalonFrameTransfered = 0;
  ledState = LOW;
}

void checkReceivedDataFromUart(){
   
  SerialUSB.flush();
  
  //if first byte of uart job = 1
  if(read8() == 1){        
    uartSendAckJobReceived();
    
    /*
    SerialUSB.write((byte)0x02);
    
    for(int i = 0; i< 44 ;i++) 
    SerialUSB.write((byte)read8());
    */
    
    //delay(10);
    //test nonce response
    //SerialUSB.write((byte)0x02);
    //SerialUSB.write((byte)0x04);
    //SerialUSB.write((byte)0x1f);
    //SerialUSB.write((byte)0xb0);
    //SerialUSB.write((byte)0x5e);
    
    newJobFromUart ++;
       
  }else{
    #ifdef DEBUGGING
      Serial.println("Job data corrupted");
    #endif
    uartSendAckJobOutOfSync();
    uartRxClean();
  } 
}

void initValues(){
  //avalon rx 
  avalonHeadRX = 0;
  avalonTailRX = 0;
  avalonBitCounterRX = 0;
  avalonNumberOfBitsRX = 0;
  
  //avalon tx
  avalonHeadTx = 0;
  avalonTailTx = 0;
  avalonFrameTransfered = 0;
  avalonBitCounterTX = 0;
  avalonNumberOfBitsTX = 0;
  tmp = 0;
  tmp1 = 0;
  tmp2 = 0;
  oneByte = 0;
  firstPeriod = 1;

  //uart rx
  uartHeadRx = 0;
  uartTailRx = 0;
  newJobFromUart = 0;
  dataSum = 0;
  counter = 0;

  //uart tx
  uartHeadTx = 0;
  uartTailTx = 0;

  //avalon processing
  nonceNotReaded = 0;
  goldenNonce = 0;
  receivedNonce = 0;
  a0 = 0;
  a1 = 0;
  a2 = 0;
  e0 = 0;
  e1 = 0;
  e2 = 0;

  //main variables
  mainTimer = 0;
  sendTimer = 0;
  receiverTimer = 0;
  ledState = LOW;
} 

// *******************************************************
// Helper routines
// *******************************************************
uint32_t byteSwap(uint32_t number) {
  
  uint32_t value = 0;
  
  value |= (uint8_t)  ((number>>24) & 0xFF);
  value |= (uint16_t)(((number>>16) & 0xFF) << 8 );
  value |= (uint32_t)(((number>> 8) & 0xFF) << 16);
  value |= (uint32_t)(((number    ) & 0xFF) << 24);
 
  return value;
}
// timer configuration
void initTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  float rc = (float) VARIANT_MCK/2/((float)frequency); //2 because we selected TIMER_CLOCK1
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_SetPriority(irq, 1);
  NVIC_EnableIRQ(irq);
}

inline void stopTimer(IRQn_Type irq)
{
  NVIC_DisableIRQ(irq);
  pmc_disable_periph_clk((uint32_t)irq);
}

void convertBytesToBitesForAvalon(){

    while(((avalonTailTx + (avalonBitCounterTX/8)) % AVALON_DATA_SIZE) != avalonHeadTx){ 
       
       tmp = (avalonBitCounterTX/8) + 1;
       tmp1 = avalonBitCounterTX%8;
       tmp2 = (avalonTailTx + tmp) % AVALON_DATA_SIZE;     
       oneByte = avalonDataTX[tmp2];
   
       if((oneByte&(1<<tmp1)) == 0)//logic 0
          avalonDataTXBitArray[avalonBitCounterTX] = 0x08;     
       else //logic 1
          avalonDataTXBitArray[avalonBitCounterTX] = 0x04; 
           
       avalonBitCounterTX ++;
       
       /*
       if(tmp1 == 0){
         char tmp1[2];
         sprintf(tmp1, "%02x", avalonDataTX[tmp2]);
         SerialUSB.print(tmp1);
       }*/
       
    }
    avalonTailTx = avalonHeadTx;  
}

void transmitDataToAvalon(){
  
  convertBytesToBitesForAvalon(); 
  //initialise timer for transfer data to avalon, timer is used only for switching to interrupt
  initTimer(TC1, 0, TC3_IRQn, 8000000);            //125nanoSec = 8000000Hz => 8Mhz 
}

//rotate 32bits
uint32_t ror32(uint32_t number, uint8_t bits) {
  return ((number >> bits) | (number << (32 - bits)));
}

/////////////////////////////////////////////////////////
//avalon 
void buildAvalonFrame(){

  buildClockSegment(AVALON_FREQUENCY);
  buildDataSegment();
  buildNonceConfiguration();
}

void avalonPrecalculations(){

  
  volatile uint32_t t1,t2,s0,s1,maj,ch;
  volatile uint32_t a,b,c,d,e,f,g,h;
 
  //BigEndian

  a = m[0];
  b = m[1];
  c = m[2];
  d = m[3];
  e = m[4];
  f = m[5];
  g = m[6];
  h = m[7];
  
  
  for(uint8_t i = 0 ; i < 3 ; i++)
  {
    s0 = (ror32(a,2)) ^ (ror32(a,13)) ^ (ror32(a, 22));
    maj = (a & b) ^ (a & c) ^ (b & c);

    s1 = (ror32(e,6)) ^ (ror32(e, 11)) ^ (ror32(e,25));
    ch = (e & f) ^ ((~e) & g);

    t1 = h + s1 + ch + k[i] + w[i];
    t2 = s0 + maj;

    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;

    if(i == 0){
      a0 = a;
      e0 = e;
    }
    else if(i == 1){
      a1 = a;
      e1 = e;
    }
    else if(i == 2){
      a2 = a;
      e2 = e; 
    }        
  }    
}

void buildClockSegment(uint32_t frequency){

  uint32_t clockConfig[2]; 
  clockConfig[0] = AVALON_CLK_LOW_CFG;
  clockConfig[1] = AVALON_CLK_HIGH_CFG;
  
  //uint32_t multiplier = frequency;

  //ip core frequency 256MHz = > divider R = 32 , divider N = 512 
  //external clock 32Mhz
  //frequency = XCLKIN *(N/(2*R))
  /*
   clockConfig = 0b00000000000000000000000101110        //[63 - 35] reserved (0x2e)  
   100000         //[34 - 29] clock input divider R
   00000000000   //[28 - 18] clock main divider N   
   001100000000 //[17 - 6]  reserved 0x300
   101111;//[5 - 0]
   */

  //clockConfig = 0b0000000000000000000000010111010000000000000000110000000000000111;
  
  clockConfig[0] |= (uint32_t) ((((frequency/25)-1) << 21) & 0xFFFFFFFF);
  
  serialize32TXAvalon(clockConfig[0]);
  serialize32TXAvalon(clockConfig[1]);
}

void buildDataSegment(){

  volatile uint32_t dataArray[DATA_LENGTH/4];
  volatile uint32_t midstateArray[MIDSTATE_LENGTH/4];

  
  #ifdef DEBUGGING 
    Serial.print("Midstate: 0x");
  #endif
  
  
  /*
  midstateArray[7] = 0x5fddb5bc;
  midstateArray[6] = 0xd2afbd00;  
  midstateArray[5] = 0x144684c7;
  midstateArray[4] = 0x19c68fa2;
  midstateArray[3] = 0x27d0a8e3;
  midstateArray[2] = 0x34ad84b2;
  midstateArray[1] = 0xa92c66be;
  midstateArray[0] = 0x3e99a4fd;
  */
  //midstateArray[0] = 0xbcb5dd5f; //midstateArray[7] = 0x5fddb5bc;                       
  //midstateArray[1] = 0x00bdafd2; //midstateArray[6] = 0xd2afbd00;
  //midstateArray[2] = 0xc7844614; //midstateArray[5] = 0x144684c7;
  //midstateArray[3] = 0xa28fc619; //midstateArray[4] = 0x19c68fa2;
  //midstateArray[4] = 0xe3a8d027; //midstateArray[3] = 0x27d0a8e3;
  //midstateArray[5] = 0xb284ad34; //midstateArray[2] = 0x34ad84b2;
  //midstateArray[6] = 0xbe662ca9; //midstateArray[1] = 0xa92c66be;
  //midstateArray[7] = 0xfda4993e; //midstateArray[0] = 0x3e99a4fd;
  
  
  //for(int i=0; i<8 ;i++)
  //   m[i] = midstateArray[i]; 
    
  
  //fill midstate to buffer
  for(uint8_t i = 0; i < MIDSTATE_LENGTH/4 ; i++){
    midstateArray[((MIDSTATE_LENGTH/4)-1)- i] = byteSwap(read32());
    m[((MIDSTATE_LENGTH/4)-1)- i] = (midstateArray[((MIDSTATE_LENGTH/4)-1)- i]);
    #ifdef DEBUGGING 
      char tmp[10];
      sprintf(tmp, "%04x", midstateArray[((MIDSTATE_LENGTH/4)-1)- i]);
      Serial.print(tmp);
    #endif 
  }

  #ifdef DEBUGGING 
    Serial.println();
    Serial.print("Data: 0x");
  #endif
  
 
  /*
  dataArray[2] = 0xbb8446f6;   //dataArray[2] = 0xf64684bb;
  dataArray[1] = 0x0815bc51;   //dataArray[1] = 0x51bc1508;
  dataArray[0] = 0x3713011a;   //dataArray[0] = 0x1a011337;
  
  for(int i=0; i<3 ;i++)
     w[i] = dataArray[i]; 
  */
  
  //fill data to buffer
  for(uint8_t i = 0; i < DATA_LENGTH/4 ; i++){
    dataArray[((DATA_LENGTH/4)-1)-i] = byteSwap(read32());
    w[((DATA_LENGTH/4)-1)-i] = (dataArray[((DATA_LENGTH/4)-1)-i]);

    #ifdef DEBUGGING 
      char tmp1[10];
      sprintf(tmp1, "%04x", dataArray[((DATA_LENGTH/4)-1)-i]);
      Serial.print(tmp1);
    #endif    
  }
  
  #ifdef DEBUGGING    
    Serial.println();
  #endif

  //data 12 bytes
  for(uint8_t i = 0; i < DATA_LENGTH/4 ; i++){
     serialize32TXAvalon(dataArray[i]);
  } 
  
  //precalculating values  
  avalonPrecalculations();

  serialize32TXAvalon(a1);
  serialize32TXAvalon(a0);
  serialize32TXAvalon(e2);
  serialize32TXAvalon(e1);
  serialize32TXAvalon(e0);

  //midstate 32 bytes
  for(uint8_t i = 0; i < MIDSTATE_LENGTH/4 ; i++)
    serialize32TXAvalon(midstateArray[i]);

  //precalculating value
  serialize32TXAvalon(a2);
}

void buildNonceConfiguration(){

  uint32_t base = (uint32_t)(pow(2,32)/AVALON_ASIC_COUNT);  //truncated decimal part
      
  for(uint16_t i = 0; i < AVALON_ASIC_COUNT; i++){
      serialize32TXAvalon(base*i); 
  }      
}

//multibyte variable to single bytes
void serialize64TXAvalon(uint64_t a) {
  serialize8TXAvalon((a    ) & 0xFF);
  serialize8TXAvalon((a>> 8) & 0xFF);
  serialize8TXAvalon((a>>16) & 0xFF);
  serialize8TXAvalon((a>>24) & 0xFF);
  serialize8TXAvalon((a>>32) & 0xFF);
  serialize8TXAvalon((a>>40) & 0xFF);
  serialize8TXAvalon((a>>48) & 0xFF);
  serialize8TXAvalon((a>>56) & 0xFF);
}
void serialize32TXAvalon(uint32_t a) {
  serialize8TXAvalon((a    ) & 0xFF);
  serialize8TXAvalon((a>> 8) & 0xFF);
  serialize8TXAvalon((a>>16) & 0xFF);
  serialize8TXAvalon((a>>24) & 0xFF);
}

void serialize16TXAvalon(int16_t a) {
  serialize8TXAvalon((a   ) & 0xFF);
  serialize8TXAvalon((a>>8) & 0xFF);
}

void serialize8TXAvalon(uint8_t a) {
  uint16_t t = avalonHeadTx;
  if (++t >= AVALON_DATA_SIZE) t = 0;
  avalonDataTX[t] = a;
  avalonHeadTx = t;   
}

//single bytes to multibyte variable
uint32_t read32Avalon() {
  uint32_t t = read16Avalon();
  t+= (uint32_t)read16Avalon()<<16;
  return t;
}
uint16_t read16Avalon() {
  uint16_t t = read8Avalon();
  t+= (uint16_t)read8Avalon()<<8;
  return t;
}
uint8_t read8Avalon() {
  volatile uint16_t t = avalonTailRX;
  if (++t >= AVALON_DATA_SIZE) t = 0;
  avalonTailRX = t;  
  return avalonDataRX[t]&0xff;
}
/////////////////////////////////////////////////////////
//uart 

//multibyte variable to single bytes
void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  //uart
  if (++uartHeadTx >= UART_BUFFER_SIZE) uartHeadTx = 0;
      uartTxBuffer[uartHeadTx] = a;
}

uint8_t serialize8RX(uint8_t a) {
  if (++uartHeadRx >= UART_BUFFER_SIZE) uartHeadRx = 0;
      uartRxBuffer[uartHeadRx] = a;
  
  return a;   
}

//single bytes to multibyte variable
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8() {
  volatile uint16_t t = uartTailRx;
  if (++t >= UART_BUFFER_SIZE) t = 0;
  uartTailRx = t;  
  return uartRxBuffer[t]&0xff;
}
////////////////////////////////////////////////////////////

// *******************************************************
// UART transmitter
// *******************************************************

void transmitWorkToUart(){
  uint16_t t;
  while(1){
    t = uartTailTx;
    if (uartHeadTx != t) {
      if (++t >= UART_BUFFER_SIZE) t = 0;

      volatile uint8_t oneByte = uartTxBuffer[t];
      SerialUSB.write((byte)oneByte);
      uartTailTx = t;
    }else{
      break;
    }  
  }
}

void uartSendGoldenNonce(){ 
  //job status - job finished
  uartSendAckJobFinished();
  //golden nonce 4B
  transmitWorkToUart();
}

// *******************************************************
// Interrupt driven Transmitter for Avalon
// *******************************************************
// time critical operations !!! dont add or change code this!!!
// transfering data to avalon
void TC3_Handler(){
    
   // TC_GetStatus to "accept" interrupt
   TC_GetStatus(TC1, 0); 
   stopTimer(TC3_IRQn);
   
   asm volatile( 
        ".equ PIOD_ODSR, 0x400E1438" "\n\t" //PORTD Output Data Status Register
        "mov r5, 0x0C" "\n\t"
        "ldr r6, = PIOD_ODSR" "\n\t"
        "mov r7, 0x00" "\n\t"                  
        "start:" "\n\t"                    //one bit transfer     
        "str r7, [r6]" "\n\t"                   
        "ldr r8, [%[array],%[count]]" "\n\t" 
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        //"nop" "\n\t" //pridane
        "str r8, [r6]" "\n\t"
        "nop" "\n\t"
        //"nop" "\n\t" //pridane
        "add %[count], %[count], 0x01" "\n\t" //count++
        "cmp %[count], %[comparator]" "\n\t"  
        "blt start" "\n\t"           // if count < comparator => start  
    
        //"nop" "\n\t" //pridane
          
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        "nop" "\n\t"
        "str r5, [r6]" "\n\t"               // idle mode
        
        ::[count] "r" (avalonNumberOfBitsTX),[comparator] "r" (avalonBitCounterTX),[array] "r" (avalonDataTXBitArray)  
   );
    
   //REG_PIOD_IER = 0x03 + (1<<9); 
    
   avalonBitCounterTX = 0;
   avalonNumberOfBitsTX = 0;
   avalonFrameTransfered = 1; 
}

// *******************************************************
// Interrupt driven receiver from Avalon
// *******************************************************
//time critical operations !!! dont add code this!!! 
//this handler is running on falling D9

void PIOD_Handler(void) {
     
   asm volatile( 
        ".equ PIOD_ISR, 0x400E144C" "\n\t" //PORTD Interrupt Status Register
        ".equ PIOD_PDSR, 0x400E143C" "\n\t" //PORTD Port Data Status Register 
        "ldr r4, = PIOD_PDSR" "\n\t"    
        "ldr r5, = PIOD_ISR" "\n\t"  
        
        "readbit:" "\n\t"                    //one bit receiving     
        "ldr r6, [r5]" "\n\t"
        "and r6, r6, 0x03" "\n\t"          //PIOD_ISR&3 
        "cmp r6, 0x00" "\n\t"              
        "beq check" "\n\t"                         
        "str r6, [%[array],%[count]]" "\n\t"
        "add %[count], %[count], 0x01" "\n\t" //count++
        "b readbit" "\n\t"             
        
        "check:" "\n\t"                          
        "ldr r7, [r4]" "\n\t"
        "and r7, r7, 0x03" "\n\t"          //PIOD_PDSR&3 
        "cmp r7, 0x03" "\n\t"            
        "bne readbit" "\n\t"                         
         
        "end:" "\n\t"                          
        "mov %[countout], %[count]" "\n\t"
        
        :[countout] "=r" (avalonNumberOfBitsRX) : [count] "r" (avalonNumberOfBitsRX), [array] "r" (avalonBitArrayRX)  
   );    
}
