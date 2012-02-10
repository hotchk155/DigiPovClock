///////////////////////////////////////////////////////////////////////
//
// DIGITAL POV CLOCK
// ARDUINO ATMEGA328
//
// http://hotchk155.blogspot.com/2012/02/digital-pov-clock-working-mostly.html
// 
// Feb 2012, Jason Hotchkiss
//
// Inspired by HDD slot POV clock code from Giles F. Hall & Paul McInnis 
//
///////////////////////////////////////////////////////////////////////
#include  <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>

// Define the digital output pins that drive each LED cluster
#define P_DIG_H1    14
#define P_DIG_H2    15
#define P_COLON1    16
#define P_DIG_M1    17
#define P_DIG_M2    4
#define P_COLON2    5
#define P_DIG_S1    6
#define P_DIG_S2    7

// Digital input pins for the switches
#define P_SWITCH1         8
#define P_SWITCH2         9
#define P_SWITCH3         10

// Define the digital output pin for the heartbeat LED
#define P_LED  3

// Define the I2C address of the realtime clock
#define RTC_ADDRESS 0b1101000

// The LEDs are driven by a buffer of 256 bytes, which indicate which 
// of the 8 LED clusters are switched on for each of 256 disk positions
//
// For each location, the following bitmasks define the LED clusters
// to illuminate at that position. Bit values chosen for easy mapping to 
// the I/O lines by simple shifting
//
#define M_DIG_H1    0x08
#define M_DIG_H2    0x04
#define M_COLON1    0x02
#define M_DIG_M1    0x01
#define M_DIG_M2    0x80
#define M_COLON2    0x40
#define M_DIG_S1    0x20
#define M_DIG_S2    0x10

// This macro is used to load an LED bitmap into the correct IO lines to
// drive the LED clusters. Using inlne code and writing to PORT latches
// instead of using digitalOut() function is much faster
#define WRITELEDS(c) { PORTC &= 0xf0; PORTC |= ((c) & 0x0f); PORTD &= 0x0f; PORTD |= ((c) & 0xf0); }

// Define the display buffers. Double buffering is used so that the next 
// display can be prepared while the current one is being rendered by 
// the hardware. Just doing a 6 digit clock, we might have managed without 
// these buffers but the approach is easy to work with and has  potential for 
// all kinds of funky visual effects
#define NUM_SECTORS   256
byte Buffer1[NUM_SECTORS] = {0};
byte Buffer2[NUM_SECTORS] = {0};

// This is the width of a single digit in sectors. Its not a whole number but it doesn't
// matter (just helps stop cumulative errors)
#define DIGIT_WIDTH ((float)23.2)

// Pointers are used to flag the "display" and "update" (off 
// screen) buffers
byte *pDisplayBuffer = Buffer1;
byte *pUpdateBuffer = Buffer2;

// Flag to tell interrupt handler the buffers can be swapped 
// at the start of the next revolution
volatile byte bSwitchBuffers = 0;

// The current sector (256 per rev)
volatile byte bCurrentSector = 0;


//////////////////////////////////////////////////////////////////////////
// BCD2DEC
// Convert binary coded decimal into decimal. (i.e. 0x12 -> 12 decimal)
// Helper function for the RTC
byte BCD2DEC(byte b)
{
    return 10*(b/16) + b%16;
}

//////////////////////////////////////////////////////////////////////////
// DEC2BCD
// Convert decimal into binary coded decimal. (i.e. 10 decimal -> 0x10 hex)
// Helper function for the RTC
byte DEC2BCD(byte b)
{
    return ((b/10)<<4)|(b%10);
}

//////////////////////////////////////////////////////////////////////////
// readRTC
// Read the time from the RTC chip (M41T00)
//////////////////////////////////////////////////////////////////////////
void readRTC(byte *pHH, byte *pMM, byte *pSS)
{
  Wire.beginTransmission(RTC_ADDRESS); //                               
  Wire.send(0x00);            // sends address 
  Wire.endTransmission();     // stop transmitting
  Wire.requestFrom(RTC_ADDRESS, 3);
  
  *pSS=BCD2DEC(Wire.receive());
  *pMM=BCD2DEC(Wire.receive());
  *pHH=BCD2DEC(Wire.receive());
}

//////////////////////////////////////////////////////////////////////////
// writeRTC
// Write new time to the RTC chip (M41T00)
//////////////////////////////////////////////////////////////////////////
void writeRTC(byte HH, byte MM, byte SS)
{
  Wire.beginTransmission(RTC_ADDRESS); //                               
  Wire.send(0x00);            // sends address 
  Wire.send(DEC2BCD(SS));
  Wire.send(DEC2BCD(MM));
  Wire.send(DEC2BCD(HH));  
  Wire.endTransmission();     // stop transmitting
}

//////////////////////////////////////////////////////////////////////////
// incTime
// Adjust the time UPWARDS when user presses button
// The RTC registers are updated thru I2C
//////////////////////////////////////////////////////////////////////////
void incTime()
{
  byte HH,MM,SS;
  readRTC(&HH,&MM,&SS);  
  SS=0;
  if(++MM>59) 
  {
      MM=0;
      if(++HH>23)
        HH=0;      
  }
  writeRTC(HH,MM,SS);
}

//////////////////////////////////////////////////////////////////////////
// decTime
// Adjust the time DOWNWARDS when user presses button
// The RTC registers are updated thru I2C
//////////////////////////////////////////////////////////////////////////
void decTime()
{
  byte HH,MM,SS;
  readRTC(&HH,&MM,&SS);  
  SS=0;
  if(MM>0) 
  {
      MM--;
  }
  else
  {      
      MM=59;      
      if(HH>0)
      {
         HH--;
      }
      else
      {
        HH=23;
      }      
  }
  writeRTC(HH,MM,SS);
}


//////////////////////////////////////////////////////////////////////////
// updateClock
// Read the time from the RTC and update the drawing buffers
//////////////////////////////////////////////////////////////////////////
void updateClock()
{
  byte ss;
  byte mm;
  byte hh;
  readRTC(&hh,&mm,&ss);

  // clear the buffer
  for(int i=0; i<256; ++i)
      pUpdateBuffer[i] = 0;
  
  // HOURS DIGIT 1
  int q;        
  q = ((hh / 10) * DIGIT_WIDTH + 235);
  pUpdateBuffer[q&0xff] |= M_DIG_H1;
  
  // HOURS DIGIT 2
  q = ((hh % 10) * DIGIT_WIDTH + 214);
  pUpdateBuffer[q&0xff] |= M_DIG_H2;    

  // FIRST COLON (even number of seconds
  // only - makes colons flash)
  if(!(ss%2))
  {
    pUpdateBuffer[173] |= M_COLON1;
  }
  
  // MINUTES DIGIT 1
  q = ((mm / 10) * DIGIT_WIDTH + 182);
  pUpdateBuffer[q&0xff] |= M_DIG_M1;        
  
  // MINUTES DIGIT 2
  q = ((mm % 10) * DIGIT_WIDTH + 162);
  pUpdateBuffer[q&0xff] |= M_DIG_M2;
  
  // SECOND COLON
  if(!(ss%2))
  {
    pUpdateBuffer[122] |= M_COLON2;
  }

  // SECONDS DIGIT 1
  q = ((ss / 10) * DIGIT_WIDTH + 131);
  pUpdateBuffer[q&0xff] |= M_DIG_S1;    

  // SECONDS DIGIT 2
  q = ((ss % 10) * DIGIT_WIDTH + 111);
  pUpdateBuffer[q&0xff] |= M_DIG_S2;    
  
  // flag for buffer switch
  bSwitchBuffers=1; 
  
  // wait for the interrupt handlers to action it
  while(bSwitchBuffers);
}

//////////////////////////////////////////////////////////////////////////
//
// SETUP
//
//////////////////////////////////////////////////////////////////////////
void setup(void)
{
  // Setup I2C  
  Wire.begin(); 
  
  // setup pin modes  
  pinMode(P_DIG_H1, OUTPUT);
  pinMode(P_DIG_H2, OUTPUT);
  pinMode(P_COLON1, OUTPUT);
  pinMode(P_DIG_M1, OUTPUT);
  pinMode(P_DIG_M2, OUTPUT);
  pinMode(P_COLON2, OUTPUT);
  pinMode(P_DIG_S1, OUTPUT);
  pinMode(P_DIG_S2, OUTPUT);
  
  pinMode(P_LED, OUTPUT);

  pinMode(P_SWITCH1, INPUT);
  pinMode(P_SWITCH2, INPUT);
  pinMode(P_SWITCH3, INPUT);

  // set weak pullups on switch inputs
  digitalWrite(P_SWITCH1, HIGH);
  digitalWrite(P_SWITCH2, HIGH);
  digitalWrite(P_SWITCH3, HIGH);

  // clear the buffers
  memset(Buffer1, 0, sizeof(Buffer1));
  memset(Buffer2, 0, sizeof(Buffer2));

  // disable interrupts
  cli();
  
  // setup timer0 - 8bit
  // resonsible for timing the LEDs
  TCCR2A = 0;
  TCCR2B = 0;  
  // select CTC mode
  TCCR2A |= (1<<WGM21);
  // select prescaler clk /8
  TCCR2B |= (1<<CS21);
  // enable compare interrupt
  TIMSK2 |= (1<<OCIE2A);

  // 16 bit timer1 times the rotation of the disk
  TCCR1B = 0;
  TCCR1A = 0;
  // select prescaler clk / 8
  TCCR1B |= (1<<CS11);
  // reset timer
  TCNT1 = 0;
  // enable overflow interrupt
  TIMSK1 |= (1<<TOIE1);
  
  // configure the index interrupt (once per rotation)
  EICRA = _BV(ISC01);
  EIMSK |= _BV(INT0);
  
  // enable interrupts
  sei();
}

//////////////////////////////////////////////////////////////////////////
//
// LOOP
//
//////////////////////////////////////////////////////////////////////////
unsigned long nextClockUpdate = 0;
unsigned long nextHeartBeat = 0;
byte bHeartBeat = 0;
void loop(void)
{
  unsigned long m = millis();

  // update the clock?
  if(m > nextClockUpdate)
  {
    updateClock();
    
    // read buttons
    if(!digitalRead(P_SWITCH2))
      incTime();
    if(!digitalRead(P_SWITCH1))
      decTime();
      
    // set delay time till the clock will be updated again
    nextClockUpdate = m + 50;
  }
  
  // update the heartbeat LED?
  if(m > nextHeartBeat)
  {
    digitalWrite(P_LED, bHeartBeat);  
    bHeartBeat = !bHeartBeat;
    nextHeartBeat = m + 200;
  }      
}


//////////////////////////////////////////////////////////////////////////
// INT0_vect
// Called when the rotation sensor is triggered (at the start of a new
// revolution
//////////////////////////////////////////////////////////////////////////
ISR(INT0_vect)
{
  // Store the rev time
  int iRevTime = TCNT1;
  
  // Reset the timer which measures rev time
  TCNT1 = 0;
  
  // Reset the timer which controls LED switching
  TCNT2 = 0;

  // switch the buffers if needed  
  if(bSwitchBuffers)
  {
    byte *pBuffer = pUpdateBuffer;
    pUpdateBuffer = pDisplayBuffer;
    pDisplayBuffer = pBuffer;
    bSwitchBuffers = 0;
  }

  // start from first sector
  bCurrentSector = 0;
  
  // Set LEDs for first sector
  WRITELEDS(pDisplayBuffer[bCurrentSector]);
  bCurrentSector++;
  
  // Set the period register to 1/256th of rev time  
  OCR2A = iRevTime>>8;
}

//////////////////////////////////////////////////////////////////////////
// TIMER2_COMPA_vect
// Called when timer 2 reaches period register (this happens at the start
// of each segment)
//////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) 
{
    // display the new slice
    WRITELEDS(pDisplayBuffer[bCurrentSector]);
    bCurrentSector++;
}

//////////////////////////////////////////////////////////////////////////
// TIMER1_0VF_vect
//////////////////////////////////////////////////////////////////////////
ISR(TIMER1_0VF_vect) 
{
}
