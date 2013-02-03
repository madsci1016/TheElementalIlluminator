/******************************************************************
*  Dan's Perodic Table Display
*	 By Bill Porter
*		details:
*	        	http://www.billporter.info/?p=1750
*
*
*	 Contributers:
*		Just me so far
*
*  Version history
*    1.0 Tested working 

*
*
*This program is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by the 
Free Software Foundation, either version 3 of the License, or(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
<http://www.gnu.org/licenses/>

*This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
*To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or
*send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
*  
******************************************************************/

#include "Tlc5940.h"
#include "tlc_fades.h"
#include <EasyTransfer.h>
#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <EEPROM.h>


#define ADC_CHANNEL 0

byte mode;
#define SLAVE 0
#define EQ 1
#define BREATHING 2
#define TWINKLE 3

#define NUM_MODES 4



PROGMEM uint8_t address[] = {18,124,16,17,107,106,105,113,112,114,19,20,108,109,110,115,117,116,21,22,23,24,30,35,37,38,40,39,111,99,98,97,96,120,119,118,26,25,27,28,29,36,32,34,33,44,80,82,86,87,88,121,122,123,13,12,4,5,50,53,51,52,61,69,76,73,75,74,136,134,135,9,11,41,43,42,46,45,81,85,83,93,92,129,128,130,3,2,6,7,59,57,58,62,60,68,67,65,66,64,139,138,137,1,10,48,49,54,55,56,84,89,91,90,95,131,133,132,14,0};

PROGMEM uint8_t bargraphaddress[8][8][3] = {{{0,0},{3,2},{12,13},{25,26},{21,22},{19,20},{16,17},{18,0}},{{0,0},{6,7},{4,5},{1,10,0},{9,11,14},{28,29,27},{24,30,23},{0,0}},{{0,0},{59,57},{50,53},{48,49},{41,43},{36,32},{35,37},{0,0}},{{0,0},{58,62},{51,52},{54,55},{42,46},{34,33},{38,40},{0,0}},{{0,0},{60,68,67},{61,69,76},{56,84,89},{45,81,85},{44,80,82},{39,111,99},{0,0}},{{65,66},{73,75},{91,90},{83,93},{86,87},{98,97},{108,109},{107,106}},{{64,139},{74,136},{95,131},{92,129},{88,121},{96,120},{110,115},{105,113}},{{138,137},{134,135},{133,132},{128,130},{122,123},{119,118},{117,116},{112,114,124}}};

byte fresh;
int timeout;

//create object
EasyTransfer ET; 

struct RECEIVE_DATA_STRUCTURE{
  byte mode;
  byte frame[sizeof(address)];
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;

/*******************************************FFT Spectrum Stuff******************************************/
//create pointers
int16_t* capturePntr;
volatile byte* samplePosPntr;

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
PROGMEM uint8_t
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,215,210,178,137, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  *colData[] = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };
/*******************************************End FFT Stuff***********************************************/

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


void setup()
{
  //read last mode from EEPROM
  mode = EEPROM.read(0);
  //writeout next mode into EEPROM
  EEPROM.write(0, ++mode);
  
  //catch loop around
  if(mode >= NUM_MODES){
    mode = 0;
    EEPROM.write(0, mode);
  }
  
  //start LED driver Library
  Tlc.init();
  
  //start Serial
  Serial.begin(38400);
  
  //start binary communications library
  ET.begin(details(mydata), &Serial);
  
  randomSeed(analogRead(0));

}
void loop()
{
  
  switch(mode){
    
    case SLAVE:
      if(Serial.available()>0)
        processFrame();
      else
        delay(10);
    break;
    case EQ:
      runEQ();
    break;
    case BREATHING:
      animation1();
    break;
    case TWINKLE:
      animation2();
    break;

  }
   
}

  
  


void processFrame()
{
  //if(Serial.available()>0){
 //   Serial.print("Got Serial:");
    Serial.println(Serial.available());
    for(int y=0; y<500; y++){
      if(ET.receiveData()==true){
   //     Serial.println("Good Packet");
        y=0;
        Tlc.clear();
        for(int i=0; i<120; i++)
          Tlc.set(pgm_read_byte(&address[i]),mydata.frame[i]<<4); 
        Tlc.update();
      }
      else
        delay(2);
    }
}
      
void animation1(){
  
  for(int y=0; y<255; y++){
    
      for(int i=0; i<NUM_TLCS*16; i++){
        Tlc.set(i, y<<4);
      }
        
  Tlc.update();
  //POLLSERIAL
  
  delay(10);
  }
    for(int y=255; y>0; y--){
    
      for(int i=0; i<NUM_TLCS*16; i++){
        Tlc.set(i, y<<4);
      }
        
  Tlc.update();
  //POLLSERIAL
 delay(10);
  }
  
}

 ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023

  capturePntr[*samplePosPntr] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if(++*samplePosPntr >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}


void runEQ(){
  
  int16_t       capture[FFT_N];    // Audio capture buffer
  complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
  uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
  volatile byte samplePos = 0;     // Buffer position counter
  
  //set pointers
  capturePntr = capture;
  samplePosPntr = &samplePos;
  
  byte
    peak[8],      // Peak level of each column; used for falling dots
    dotCount = 0, // Frame counter for delaying dot-falling speed
    colCount = 0; // Frame counter for storing past column data
  int
    col[8][10],   // Column levels for the prior 10 frames
    minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
    maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
    colDiv[8];    // Used when filtering FFT output to 8 columns
    
    
      uint8_t i, j, nBins, binNum, *data;
  
    memset(peak, 0, sizeof(peak));
    memset(col , 0, sizeof(col));
  
    for(i=0; i<8; i++) {
      minLvlAvg[i] = 0;
      maxLvlAvg[i] = 512;
      data         = (uint8_t *)pgm_read_word(&colData[i]);
      nBins        = pgm_read_byte(&data[0]) + 2;
      binNum       = pgm_read_byte(&data[1]);
      for(colDiv[i]=0, j=2; j<nBins; j++)
        colDiv[i] += pgm_read_byte(&data[j]);
    }
  
    // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
    ADMUX  = (ADC_CHANNEL)| _BV(REFS0); // Channel sel, right-adj, use AREF pin
    ADCSRA = _BV(ADEN)  | // ADC enable
             _BV(ADSC)  | // ADC start
             _BV(ADATE) | // Auto trigger
             _BV(ADIE)  | // Interrupt enable
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
    ADCSRB = 0;                // Free run mode, no high MUX bit
    DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
    //TIMSK0 = 0;                // Timer0 off
  
    sei(); // Enable interrupts  
    
    for(;;){
      uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
      uint16_t minLvl, maxLvl;
      int      level, y, sum;
    
      while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish
    
      fft_input(capture, bfly_buff);   // Samples -> complex #s
      samplePos = 0;                   // Reset sample counter
      ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
      fft_execute(bfly_buff);          // Process complex data
      fft_output(bfly_buff, spectrum); // Complex -> spectrum
    
      // Remove noise and apply EQ levels
      for(x=0; x<FFT_N/2; x++) {
        L = pgm_read_byte(&noise[x]);
        spectrum[x] = (spectrum[x] <= L) ? 0 :
          (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
      }
    
      // Fill background w/colors, then idle parts of columns will erase
     // matrix.fillRect(0, 0, 8, 3, LED_RED);    // Upper section
     // matrix.fillRect(0, 3, 8, 2, LED_YELLOW); // Mid
     // matrix.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section

             Tlc.clear();
    //Serial.println(freeRam());
      // Downsample spectrum output to 8 columns:
      for(x=0; x<8; x++) {
        data   = (uint8_t *)pgm_read_word(&colData[x]);
        nBins  = pgm_read_byte(&data[0]) + 2;
        binNum = pgm_read_byte(&data[1]);
        for(sum=0, i=2; i<nBins; i++)
          sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
        col[x][colCount] = sum / colDiv[x];                    // Average
        minLvl = maxLvl = col[x][0];
        for(i=1; i<10; i++) { // Get range of prior 10 frames
          if(col[x][i] < minLvl)      minLvl = col[x][i];
          else if(col[x][i] > maxLvl) maxLvl = col[x][i];
        }
        // minLvl and maxLvl indicate the extents of the FFT output, used
        // for vertically scaling the output graph (so it looks interesting
        // regardless of volume level).  If they're too close together though
        // (e.g. at very low volume levels) the graph becomes super coarse
        // and 'jumpy'...so keep some minimum distance between them (this
        // also lets the graph go to zero when no sound is playing):
        if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
        minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
        maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)
    
        // Second fixed-point scale based on dynamic min/max levels:
        level = 10L * (col[x][colCount] - minLvlAvg[x]) /
          (long)(maxLvlAvg[x] - minLvlAvg[x]);
    
        // Clip output and convert to byte:
        if(level < 0L)      c = 0;
        else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
        else                c = (uint8_t)level;
    
        if(c > peak[x]) peak[x] = c; // Keep dot on top
    
        
    
        if(peak[x] <= 0) { // Empty column?
          //matrix.drawLine(x, 0, x, 7, LED_OFF);
          //for(int v=0; v<8; v++){
            //Tlc.set(bargraphaddress[x][v][0], 0);
            //Tlc.set(bargraphaddress[x][v][1], 0);
          //}
          continue;
        } else if(c < 8) { // Partial column?
          //matrix.drawLine(x, 0, x, 7 - c, LED_OFF);
          for(int v=c; v>=0; v--){
            Tlc.set(pgm_read_byte(&bargraphaddress[x][v][0]), 4095);
            Tlc.set(pgm_read_byte(&bargraphaddress[x][v][1]), 4095);
            Tlc.set(pgm_read_byte(&bargraphaddress[x][v][2]), 4095);
          }
        }
    
        // The 'peak' dot color varies, but doesn't necessarily match
        // the three screen regions...yellow has a little extra influence.
        y = peak[x];
        Tlc.set(pgm_read_byte(&bargraphaddress[x][y][0]), 4095);
        Tlc.set(pgm_read_byte(&bargraphaddress[x][y][1]), 4095);
        Tlc.set(pgm_read_byte(&bargraphaddress[x][y][2]), 4095);
        //if(y < 2)      Tlc.set(bargraphaddress[x][y][0], 4095)//matrix.drawPixel(x, y, LED_RED);
       // else if(y < 6) //matrix.drawPixel(x, y, LED_YELLOW);
       // else           //matrix.drawPixel(x, y, LED_GREEN);
      }
    
      //matrix.writeDisplay();
      Tlc.set(0,Tlc.get(1));
      Tlc.update();
    
      // Every third frame, make the peak pixels drop by 1:
      if(++dotCount >= 3) {
        dotCount = 0;
        for(x=0; x<8; x++) {
          if(peak[x] > 0) peak[x]--;
        }
      }
    
      if(++colCount >= 10) colCount = 0;
    
      
        }
}


void animation2(){

 TLC_CHANNEL_TYPE channel = random(0,144);
    if (!tlc_isFading(channel)) {
      uint16_t duration = random(512,1024);
      int maxValue = random(4095);
      uint32_t startMillis = millis() + 50;
      uint32_t endMillis = startMillis + duration;
      if(tlc_addFade(channel, 2048, maxValue, startMillis, endMillis) == 0)
        Tlc.set(channel, 2048);
      else
        tlc_addFade(channel, maxValue, 2048, endMillis, endMillis + duration);
    }
  
  tlc_updateFades();
}

void barGraphTest(){
    
  for(int y = 0; y<8; y++){
    for(int i=0; i<8; i++){
       Tlc.set(pgm_read_byte(&bargraphaddress[y][i][0]), 2048);
       Tlc.set(pgm_read_byte(&bargraphaddress[y][i][1]), 2048);
       Tlc.set(0,0);
       Tlc.update();
       delay(100);
    }
    
      for(int i=7; i>=0; i--){
       Tlc.set(pgm_read_byte(&bargraphaddress[y][i][0]), 0);
       Tlc.set(pgm_read_byte(&bargraphaddress[y][i][1]), 0);
       Tlc.set(0,0);
       Tlc.update();
       delay(100);
    }
  }
  
}

