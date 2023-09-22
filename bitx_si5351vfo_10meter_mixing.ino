/*
  YD2UTC
  A part of this program is taken from Jason Mildrum, NT7S.
  All extra functions are written by me, Rob Engberts PA0RWE

  References:
  http://nt7s.com/
  http://sq9nje.pl/
  http://ak2b.blogspot.com/
  http://pa0rwe.nl/?page_id=804

 *  SI5351_VFO control program for Arduino NANO
 *  Copyright PA0RWE Rob Engberts
 *  
 *  Using the old Si5351 library by Jason Mildrun nt7s
 *
 *  Functions:
 *  - CLK0 - Tx frequency = Display frequency
 *  - CLK1 - Rx / RIT frequency = Tx +/- BFO (upper- or lower mixing)
 *           When RIT active, RIT frequency is displayed and is tunable.
 *           When RIT is inactive Rx = Tx +/- BFO
 *  - CLK2 - BFO frequency, tunable
 *
 *  - Stepsize:  select (pushbutton)
 *  - Calibrate: (pushbutton) calculates difference between X-tal and measured
 *               x-tal frequency, to correct x-tal frequency.
 *  - Selection: (pushbutton) Switch between TRx and BFO mode
 *  - RIT switch: tunable Rx frequency, while Tx frequency not changed
 *  - Programming PIC by ICSP
 *  
 *  Si5351 settings: I2C address is in the .h file
 *                   X-tal freq is in the .h file but set in line 354
 *
***************************************************************************
 *  02-04-2015   1.0    Start building program based on the PIC version
 *  18-06-2019   1.1    Extend frequency range to 10 KHz down
 *                      Update frequency display. Added 'MHz' and 'KHz'
 *                      Set max frequency to 100 MHz.
 *  21-06-2020   2.0    Changed text BFO to IF
 *                      Changed bfo_t => vfo_if, vfo_t => vfo_hf, vfo_r => vfo_lo
 *                      Make possible to use the vfo for upper or lower mixing
 *                        by using a #define statement (upper or lower)
 *                      Store of actual frequency after 5 seconds of not tuning  
 *                      Set Hz digit of frequency at start to zero (0)
 *                      Stepsize was not saved. Repaired
 *  06-04-2021   3.0    Added backward stepsize by pressing stepsize button longer                      
 *
***************************************************************************
*  Includes
**************************************************************************/
#include <LiquidCrystal.h>
#include <Rotary.h>
#include <RWE_si5351.h> 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
/**************************************************************************
*  Define Upper / Lower mixing  (Remark // if not true)
**************************************************************************/
#define upper
//#define lower

/**************************************************************************
*  (Pin) Definitions
**************************************************************************/
#define ENCODER_A     2       // Encoder pin A INT0/PCINT18 D2
#define ENCODER_B     3       // Encoder pin B INT1/PCINT19 D3
#define ENCODER_BTN   4       // Encoder pushbutton D4
#define IF_SHIFT_BTN   6    //if shift
#define IF_BTN     7       // Select TRx or BFO
#define CALL_BTN  5  //call freq
#define USB_SWITCH  A2 
#define LSB_SWITCH  A3

//      I2C-SDA       A4      // I2C-SDA
//      I2C-SCL       A5      // I2C-SCL



#define F_MIN            10000UL      // Lower frequency limit  10 KHz
#define F_MAX        100000000UL      // Upper frequency limit 100 MHz

/**************************************************************************
*  EEPROM data locations 
**************************************************************************/
#define EE_SAVED_RADIX  0   // Stepsize pointer
#define EE_SAVED_AFREQ  4   // Actual Tx Frequency (CLK0)
#define EE_SAVED_BFREQ  8   // BFO (IF) Frequency  (CLK2)
#define EE_SAVED_XFREQ  12  // X-tal frequency  (25 or 27 MHz)
#define EE_SAVED_OFSET  16  // store correction
#define EE_SAVED_CALBR  20  // calibrated indicator
#define EE_SAVED_IF_SHIFT 24 // if shift save freq
#define EE_SAVED_FR_CALL 28 // save callfreq reference

#define SM_SAMPLING_INTERVAL  33
#define SM_SAMPLE_CNT 15
byte pep[SM_SAMPLE_CNT];                      // s-meter readings storage
long lastMilis = 0;      


Si5351 si5351;
Rotary r = Rotary(ENCODER_A, ENCODER_B);

/**************************************************************************
*  Declarations
**************************************************************************/
volatile uint32_t vfo_if = 999720000ULL / SI5351_FREQ_MULT;   // CLK0 start IF
volatile uint32_t vfo_hf = 706000000ULL / SI5351_FREQ_MULT;   // CLK2 start Tx freq
volatile uint32_t vfo_lo = vfo_hf - vfo_if;                    // CLK1 start Rx freq
volatile uint32_t vfo_s = vfo_hf;                              // Saved for RIT
volatile uint32_t if_freq_out = vfo_if;
uint32_t vco_c = 0;                                            // X-tal correction factor
uint32_t xt_freq;
uint32_t test = 0;
uint32_t radix = 100ULL, old_radix = 100ULL;                   //start step size
boolean changed_f = 0, stepflag = 0, calflag = 0, modeflag = 0, ritset = 0, callFreq=0,shiftFlag = 0;
boolean calibrate = 0;
boolean lsb_flag = 0, usb_flag = 0;
boolean shiftMode= 0;  
boolean calibrate_flag = 0;

int  act_clk = 0, disp_txt = 0;

unsigned long startTime;
unsigned long endTime;
unsigned long duration;
byte timerRunning;
unsigned long currentMillis = millis();
unsigned long startMillis = 0;
boolean  actupdt = false;

/*
 * freq  and if shift reference
 */
volatile uint32_t if_shift =  1500ULL;                            // Saved for RIT
volatile uint32_t freq_correction= 80500ULL;



/**
 * smeter
 */

byte meter_s1[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B11000
};

byte meter_s2[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B11011
};

byte meter_s3[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B11000,
B11000
};

byte meter_s4[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B00000,
B11011,
B11011
};

byte meter_s5[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B11000,
B11000,
B11000,
};

byte meter_s6[8] = {
B00000,
B00000,
B00000,
B00000,
B00000,
B11011,
B11011,
B11011,
};
byte meter_s7[8] = {
B00000,
B00000,
B00000,
B00000,
B11000,
B11000,
B11000,
B11000,
};

byte meter_s8[8] = {
B00000,
B00000,
B00000,
B11011,
B11011,
B11011,
B11011,
B11011,
};
byte meter_s9[8] = {
B11000,
B00000,
B11000,
B11000,
B11000,
B11000,
B11000,
B11000,
};
byte meter_s10[8] = {
B11011,
B00000,
B11011,
B11011,
B11011,
B11011,
B11011,
B11011,
};
byte meter_s20[8] = {
B11000,
B00000,
B11000,
B11000,
B11000,
B11000,
B11000,
B11000,
};
byte meter_s30[8] = {
B11011,
B00000,
B11011,
B11011,
B11011,
B11011,
B11011,
B11011,
};

void mySMeter ( int level, char label ){
   static byte lastpos=0;
   byte i;

   // Write the S meter indicator 2n line first column
   lcd.setCursor(0,1);
   lcd.write( label );
   
   // Note the need to stick to createChar# per bar type written. 
   // You can't use single char creation e.g. 0 or 1 etc as it will show directly on already written chars elsewhere.
   // Resulting in strange looking s-meter graph.
   
   switch( level ){
     case 13: lcd.createChar(6, meter_s30); lastpos=7; lcd.setCursor(lastpos,1); lcd.write( 6 ); break;
     case 12: lcd.createChar(6, meter_s30); lastpos=6; lcd.setCursor(lastpos,1); lcd.write( 6 ); break; // lcd.write(" "); break;
     case 11: lcd.createChar(6, meter_s20); lastpos=6; lcd.setCursor(lastpos,1); lcd.write( 6 ); break; // lcd.write(" "); break;
     case 10: lcd.createChar(5, meter_s10); lastpos=5; lcd.setCursor(lastpos,1); lcd.write( 5 ); break; // lcd.write("  "); break;
     case 9:  lcd.createChar(5, meter_s9); lastpos=5; lcd.setCursor(lastpos,1); lcd.write( 5 );  break; // lcd.write("  "); break;
     case 8:  lcd.createChar(4, meter_s8); lastpos=4; lcd.setCursor(lastpos,1); lcd.write( 4 );  break; // lcd.write("   "); break;
     case 7:  lcd.createChar(4, meter_s7); lastpos=4; lcd.setCursor(lastpos,1); lcd.write( 4 );  break; // lcd.write("   "); break;
     case 6:  lcd.createChar(3, meter_s6); lastpos=3; lcd.setCursor(lastpos,1); lcd.write( 3 );  break; // lcd.write("    ");break;
     case 5:  lcd.createChar(3, meter_s5); lastpos=3; lcd.setCursor(lastpos,1); lcd.write( 3 );  break; // lcd.print("    "); break;
     case 4:  lcd.createChar(2, meter_s4); lastpos=2; lcd.setCursor(lastpos,1); lcd.write( 2 );  break; // lcd.write("     "); break;
     case 3:  lcd.createChar(2, meter_s3); lastpos=2; lcd.setCursor(lastpos,1); lcd.write( 2 );  break; // lcd.write("     "); break;
     case 2:  lcd.createChar(1, meter_s2); lastpos=1; lcd.setCursor(lastpos,1); lcd.write( 1 );  break; // lcd.write("     "); break;
     case 1:  lcd.createChar(1, meter_s1); lastpos=1; lcd.setCursor(lastpos,1); lcd.write( 1 );  break; // lcd.write("      "); break;
     case 0:                               lastpos=1; lcd.setCursor(lastpos,1); break; //lcd.write("       "); break;
     default: break;
   }
   // Clear the remaining bars
   for (i=lastpos;i<7;i++)
   {
     lcd.setCursor(i+1,1); lcd.write(' ');
   }
}

void showBarGraph() {
    static byte old_ave = 0;
    byte ave = 0, i;
    
    // find the average
    for (i=0; i<SM_SAMPLE_CNT; i++) {
        ave += pep[i];
    }
    ave /= SM_SAMPLE_CNT;
    
    // print the bars
    // Serial.print("Value averaged: ");Serial.println(ave);
    if (old_ave != ave)
    {
      mySMeter(ave,'S');
      old_ave = ave;
    }
}

void smeter() {
    // contador para el ciclo de lecturas en el array
    static byte smeterCount = 0; // Count the measurements done 
    word val = 0;                // Input value read from the input pin

    // sample and process the S-meter in RX & TX
    // Serial.print("milliseconds passed :"); Serial.println(millis()-lastMilis);
    if ((millis() - lastMilis) >= SM_SAMPLING_INTERVAL) {
      lastMilis = millis();
      // it has rotated already?
      if (smeterCount < SM_SAMPLE_CNT) {
         // take a measure and rotate the array

         // we are sensing a value that must move in the 0-1.1v so internal reference
         analogReference(INTERNAL);
         // read the value and map it for 13 chars (0-12) in the LCD bar
        
          val = analogRead(A1);
         
         // reset the reference for the buttons handling
         analogReference(DEFAULT);

         // watchout !!! map can out peaks, so smooth
         if (val > 1023) val = 1023;
         // Serial.print("Value unmapped: ");Serial.println(val);

         // scale it to 13 blocks (0-14)
         val = map(val, 0, 1023, 0, 14);

         // push it in the array
         for (byte i = 0; i < SM_SAMPLE_CNT-1; i++) {
             pep[i] = pep[i+1];
             // Serial.print("Value pep array: ");Serial.println(pep[i]);
         }
         pep[SM_SAMPLE_CNT-1] = val;
         // Serial.print("Value mapped: ");Serial.println(val);

         // increment counter
         smeterCount += 1;
         
      } else {
        // rise the flag about the need to show the bar graph and reset the count
        showBarGraph();
        smeterCount = 0;
      }
    }
}


/**
 * usb lsb
 */
 boolean changeMode() {

   if(!digitalRead(LSB_SWITCH)) {
    if(lsb_flag==0) {
       lsb_flag=1;
       usb_flag=0;
       shiftMode=0;
    }
   }



   if(!digitalRead(USB_SWITCH)) {
    if(usb_flag==0) {
       usb_flag=1;
       lsb_flag=0;
       shiftMode=1;
    }
   }

   
  return 1;
 }


/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
/**************************************/
ISR(PCINT2_vect) {
  char result = r.process();
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);
}


/**************************************/
/* Change the frequency               */
/* dir = 1    Increment               */
/* dir = -1   Decrement               */
/**************************************/
void set_frequency(short dir)
{
  switch (act_clk)
  {
    case 0:                 // HF frequency
      if (dir == 1)
        vfo_hf += radix;
      if (dir == -1) {
        if (vfo_hf < radix) break; // to prevent negative value
        vfo_hf -= radix;
      }
      break;
    case 1:                 // HF frequency (only if RIT is on)
      if (dir == 1)
        vfo_hf += radix;
      if (dir == -1) {
        if (vfo_hf < radix) break; // to prevent negative value      
        vfo_hf -= radix;
      }
      break;
    case 2:                 // IF frequency
      if (dir == 1)
        vfo_if += radix;
      if (dir == -1) {
        if (vfo_if < radix) break; // to prevent negative value      
        vfo_if -= radix;
      }
      break;

      case 3:                 // if  shift
      if (dir == 1)
        if_shift += radix;
      if (dir == -1) {
      if (if_shift < radix) break; // to prevent negative value      
        if_shift -= radix;
      }
      break;
      
      case 4:                 // Freq correction
      if (dir == 1)
        freq_correction += radix;
      if (dir == -1) {
      if (freq_correction < radix) break; // to prevent negative value      
        freq_correction -= radix;
      }
      break;
  }

  if(vfo_hf > F_MAX)
    vfo_hf = F_MAX;
  if(vfo_hf < F_MIN)
    vfo_hf = F_MIN;

  changed_f = 1;
}


/**************************************/
/* Read the buttons with debouncing   */
/**************************************/
boolean get_button()
{
  if (!digitalRead(ENCODER_BTN))            // Stepsize
  {
    delay(20);
    startTime = millis();                   // Start counting    
    if (!digitalRead(ENCODER_BTN))
    {
      while (!digitalRead(ENCODER_BTN));
      stepflag = 1;      
    }
    duration = millis() - startTime;        // Total time pressed
  }
 
  else if (!digitalRead(IF_BTN))         // Selection
  {
    delay(20);
    if (!digitalRead(IF_BTN))
    {
      while (!digitalRead(IF_BTN));
      modeflag = 1;
    }
  }  

  else if (!digitalRead(CALL_BTN))         // call freq mode
  {
    delay(20);
    if (!digitalRead(CALL_BTN))
    {
     while (!digitalRead(CALL_BTN));
       callFreq = 1;
    } 
  } 

 else if (!digitalRead(IF_SHIFT_BTN))         // if shift
  {
    delay(20);
    if (!digitalRead(IF_SHIFT_BTN))
    {
     while (!digitalRead(IF_SHIFT_BTN));
      shiftFlag = 1;
    }
  } 



 
  if (stepflag | calflag | modeflag | callFreq | shiftFlag) return 1;
  else return 0;
}


/**************************************/
/* Displays the frequency and stepsize*/
/**************************************/
void display_frequency()
{
  char LCDstr[10];
  char Hertz[7];
  int p,q = 0;
  unsigned long freq;
 lcd.setCursor(0, 0);

 if(shiftMode == 1) {
    lcd.print("U");
  } else {
    lcd.print("L");
  }
  switch(act_clk)
  {
    case 0:                               // HF frequency
      freq = vfo_hf;
      break;
    case 1:                               // HF frequency (Used in RIT Mode)
      freq = vfo_hf;
      break;
    case 2:                               // IF frequency
      freq = vfo_if;
      break;
    case 3:                               // if shift freq
      freq = if_shift;
      break;
    case 4:                               // Callibration freq
      freq = freq_correction;
      break;
  }

  Hertz[1]='\0';                           // empty array

  sprintf(LCDstr, "%ld", freq);           // convert freq to string
  p=strlen(LCDstr);                       // determine length
  lcd.setCursor(16, 2); 
  if (p>6){                               // MHz
    lcd.print(F("MHz"));
    q=p-6;
    strcpy(Hertz,LCDstr);                 // get Herz digits (6)
    strcpy(LCDstr+q,Hertz+(q-1));         // copy into LCDstr and add to MHz
    LCDstr[q]='.';                        // decimal point
  }
  else {                                  // KHz
     lcd.print(F("KHz"));
    q=p-3;
    strcpy(Hertz,LCDstr);                 // get Herz digits (3)
    strcpy(LCDstr+q,Hertz+(q-1));         // copy into LCDstr and add to KHz
    LCDstr[q]='.';                        // decimal point
  }

  switch (p)                              // display stepsize
  {
    case 5:                               //  10 KHZ
      lcd.setCursor(36,0);
      break;
    case 6:                               // 100 KHZ
      lcd.setCursor(24,0);
      break;
    case 7:                               //   1 MHZ
      lcd.setCursor(12,0);
      break;
    case 8:                               //  10 MHZ
      lcd.setCursor(0,0);
      break;
    case 9:                               // 100 MHZ
      lcd.setCursor(0,0);
      break;
  }
    
  lcd.setCursor(3, 0);
  lcd.print(LCDstr);
  display_settings();
}


/**************************************/
/* Displays step, mode and version    */
/**************************************/
void display_settings()
{
// Stepsize  
   lcd.setCursor(12, 0);   
 // display.print(F("Step:"));
  switch (radix)
  {
    case 1:
      lcd.print(F("   1Hz"));
      break;
    case 10:
       lcd.print(F("  10Hz"));
      break;
    case 100:
       lcd.print(F(" 100Hz"));
      break;
    case 1000:
       lcd.print(F("  1kHz"));
      break;
    case 10000:
       lcd.print(F(" 10kHz"));
      break;
    case 100000:
       lcd.print(F("100kHz"));
      break;
    case 1000000:
       lcd.print(F("  1MHz"));
      break;
  }

// Mode
  lcd.setCursor(13, 1);
  switch (act_clk)
  {
    case 0:
      lcd.print(F("Trx"));
      break;
    case 2:
     lcd.print(F("IF "));
      break;
    case 3:
      lcd.print(F("IF.S"));
      break;
    case 4:
      lcd.print(F("CAL"));
      break;
  }
  

  lcd.setCursor(0, 1);
  switch (disp_txt)
  {
    case 0:
    
      break;
    case 1:
      lcd.print(F("** Turn RIT Off *"));
      break;
    case 2:
      lcd.print(F("*** Set to TRx **"));
      break;
    case 3:
      lcd.print(F("** Calibration **"));
      break;      
    case 4:
      lcd.print(F("* Calibration OK!"));
      break;      
  }

 // display.display();
}


/**
 * Set VFO Frequencies
 */
void setLoFrequency() {
  #if defined(upper)
     if(shiftMode==0) {
            vfo_lo = vfo_hf + vfo_if-if_shift;                //lsb
            if_freq_out = vfo_if-if_shift;
          } else if(shiftMode ==1) {
             vfo_lo = vfo_hf + vfo_if+if_shift;             //usb
              if_freq_out = vfo_if+if_shift;
          } else {  
             vfo_lo = vfo_hf + vfo_if;
       
      }              
  #endif
  #if defined(lower) 
          vfo_lo = vfo_hf - vfo_if;    
   #endif
  si5351.set_freq(vfo_lo * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK1);   

  //enable bfo
 // si5351.set_freq((if_freq_out * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK2);

  
}



/**
 * usb lsb
 */
 /*
 boolean changeMode() {

   if(!digitalRead(LSB_SWITCH)) {
    if(lsb_flag==0) {
       lsb_flag=1;
       usb_flag=0;
       shiftMode=0;
    }
   }



   if(!digitalRead(USB_SWITCH)) {
    if(usb_flag==0) {
       usb_flag=1;
       lsb_flag=0;
       shiftMode=1;
    }
   }

   
  return 1;
 }


*/
/**
 * callmode
 * 
 */
  boolean callMode() {
    if(!digitalRead(CALL_BTN)) {
      if(calibrate_flag == 0)  {
          
            calibrate_flag = 1;
       } 

       if(act_clk == 4 && calibrate_flag == 1 ) {
        calibrate_flag = 0;
       
      }     
    } 
    
    return 1;
  }
  



/**************************************/
/*            S E T U P               */
/**************************************/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)

// Read EEPROM
  radix = eeprom_read_dword((const uint32_t *)EE_SAVED_RADIX);
  if ((radix < 10ULL) | (radix > 1000000ULL)) radix = 100ULL;  
  
  vfo_hf = eeprom_read_dword((const uint32_t *)EE_SAVED_AFREQ);
  if ((vfo_hf < F_MIN) | (vfo_hf > F_MAX)) vfo_hf = 14000000ULL;
  
  test = (vfo_hf / 10);                       // Round to 10Hz
  vfo_hf = test * 10;

  vfo_if = eeprom_read_dword((const uint32_t *)EE_SAVED_BFREQ);
  if ((vfo_if < F_MIN) | (vfo_if > F_MAX)) vfo_if = 10000000ULL;  

  test = (vfo_if / 10);                       // Round to 10Hz
  vfo_if = test * 10;

  vco_c = 0;
  if (eeprom_read_dword((const uint32_t *)EE_SAVED_CALBR) == 0x60)  {
    vco_c = eeprom_read_dword((const uint32_t *)EE_SAVED_OFSET);
  }  
  xt_freq = SI5351_XTAL_FREQ + vco_c;

  /**
 * load if shift freq from memory
 */
  if_shift  =  eeprom_read_dword((const uint32_t *)EE_SAVED_IF_SHIFT);
  if ((if_shift < F_MIN) | (if_shift > F_MAX)) if_shift = 1500ULL;  

  test = (if_shift / 10);                       // Round to 10Hz
  if_shift = test * 10;


/**
 * load freq  ref from memory
 */
  freq_correction  =  eeprom_read_dword((const uint32_t *)EE_SAVED_FR_CALL);
  if ((freq_correction < F_MIN) | (freq_correction > F_MAX)) freq_correction =  80500ULL; 
  test = (freq_correction / 10);                       // Round to 10Hz
  freq_correction = test * 10;

//initialize the Si5351
  si5351.set_correction(freq_correction); // Set to zero because I'm using an other calibration method
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, xt_freq);    // Frequency get from settings in VFO_si5351.h file
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  

  setLoFrequency();
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);

  //enable bfo
  //si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);


  
// Encoder setup
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);                       // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

  sei();

// Pin Setup
  
  pinMode(IF_BTN, INPUT_PULLUP);   // Select TRx or BFO
  pinMode(IF_SHIFT_BTN, INPUT_PULLUP);
  
  pinMode(CALL_BTN, INPUT_PULLUP);
  pinMode(LSB_SWITCH, INPUT_PULLUP);
  pinMode(USB_SWITCH, INPUT_PULLUP);


//  Timer
  startMillis = millis();               // Reset actual freq store timer
  actupdt = true;    
  
// Display first time  
 lcd.begin(16, 2);
  display_frequency();  // Update the display


}

/**************************************/
/*             L O O P                */
/**************************************/
void loop()
{
  if (disp_txt == 4) {
    delay(3000);                  // Display calibration OK and wait 3 seconds
    disp_txt = 0;
  }


Serial.println(act_clk);
  /**
   * change mode usb or lsb
   */
  if(changeMode()){
     setLoFrequency();
  }
  

  smeter();
  showBarGraph();

if(callMode()) {

  if(calibrate_flag ==1){
 
     si5351.set_correction(freq_correction); // Set to zero because I'm using an other calibration method
     si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
     eeprom_write_dword((uint32_t *)EE_SAVED_FR_CALL, freq_correction);
   
  } 
}
  
  
// Update the display if the frequency has been changed
  if (changed_f)  {
    display_frequency();   
    // Update LO frequency    
   setLoFrequency();

    changed_f = 0;
    disp_txt = 0;                   // Clear line
    startMillis = millis();         // Reset actual freq store timer
    actupdt = false;  
  }



// Store actual freq and other settings once per 5 seceonds after frequency change
  if((millis() - startMillis) > 5000 && actupdt == false) {
    eeprom_write_dword((uint32_t *)EE_SAVED_AFREQ, vfo_hf);
    startMillis = millis();                   // Reset actual freq store timer   
    actupdt = true;
    Serial.println("Act freq stored"); 
  }
  
// Button press
// Also stored the last used frequency together with the step size before store
//
  if (get_button()) { 
    if (stepflag) {                 // Stepsize button
      if (duration > 500)  {        // For a long time then prev step
        switch (radix)
        {
        case 1000000:
          radix = 100000;
          break;
        case 100000:
          radix = 10000;
          break;
        case 10000:
          radix = 1000;
          break;
        case 1000:
          radix = 100;
          break;
        case 100:
          radix = 10;
          break;
        case 10:
          radix = 1;
          break;
        case 1:
          radix = 1000000;
          break;               
        }
      }  
      else {
        switch (radix)
        {
        case 1:
          radix = 10;
          break;
        case 10:
          radix = 100;
          break;
        case 100:
          radix = 1000;
          break;
        case 1000:
          radix = 10000;
          break;
        case 10000:
          radix = 100000;
          break;
        case 100000:
          radix = 1000000;
          break;
        case 1000000:
          radix = 1;
          break;       
        }
      }     
      eeprom_write_dword((uint32_t *)EE_SAVED_RADIX, radix);  // Store stepsize
      stepflag  = 0;
    }
    else if (modeflag)  {         // Mode button
      if (act_clk == 0) act_clk = 2; else act_clk = 0;
      eeprom_write_dword((uint32_t *)EE_SAVED_BFREQ, vfo_if);
      modeflag = 0;  
      disp_txt = 0;                                 // Clear line
    }

     /**
   * if shif
   */
   else if (shiftFlag)  {         // if shift
      if (act_clk == 0) act_clk = 3; else act_clk = 0;
      eeprom_write_dword((uint32_t *)EE_SAVED_IF_SHIFT, if_shift);
      shiftFlag = 0;  
      disp_txt = 0;                                 // Clear line
    }


  /**
  * calibrate mode
  */
    else if (callFreq)  {         // calibrate button
      if (act_clk == 0) act_clk = 4; else act_clk = 0;
        eeprom_write_dword((uint32_t *)EE_SAVED_FR_CALL, freq_correction);
         si5351.set_correction(freq_correction); 
         si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
        callFreq = 0;  
        disp_txt = 0;   

    }

 

  } // end get_button
  
  display_frequency();    
  // Update display

  
} // end while loop
