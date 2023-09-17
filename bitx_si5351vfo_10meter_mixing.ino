
// include the library code:
#include <LiquidCrystal.h>
#include <Rotary.h>
#include <RWE_si5351.h> 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// initialize the library with the numbers of the interface pins
// rs: 13, en: 12, d4:11, d5:10, d6: 9, d7: 8
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

/**************************************************************************
*  (Pin) Definitions
**************************************************************************/
#define ENCODER_A     2       // Encoder pin A INT0/PCINT18 D2
#define ENCODER_B     3       // Encoder pin B INT1/PCINT19 D3
#define ENCODER_BTN   4       // Encoder pushbutton D4
#define Calibrbtn     5       // Calibrate
#define RIT_Switch    6       // RIT Switch
#define TX_Switch     7       // Select TRx or BFO
#define BAND_Switch   A0
//      I2C-SDA       A4      // I2C-SDA
//      I2C-SCL       A5      // I2C-SCL

#define F_MIN            10000UL      // Lower frequency limit  10 KHz
#define F_MAX        100000000UL      // Upper frequency limit 100 MHz

/**************************************************************************
*  EEPROM data locations 
**************************************************************************/
#define EE_SAVED_RADIX  1000UL   // Stepsize pointer
#define EE_SAVED_AFREQ  4   // Actual Tx Frequency (CLK0)
#define EE_SAVED_BFREQ  8   // BFO (IF) Frequency  (CLK2)
#define EE_SAVED_XFREQ  12  // X-tal frequency  (25 or 27 MHz)
#define EE_SAVED_OFSET  16  // store correction
#define EE_SAVED_CALBR  20  // calibrated indicator


#define SMETER_IN  A1



Si5351 si5351;
Rotary r = Rotary(ENCODER_A, ENCODER_B);

/**************************************************************************
*  Declarations
**************************************************************************/
volatile uint32_t bfo_f =  1000000000ULL / SI5351_FREQ_MULT;   // CLK0 start IF
volatile uint32_t vfo_t =  706000000ULL / SI5351_FREQ_MULT;   // CLK2 start Tx freq
volatile uint32_t vfo_r = bfo_f + vfo_t;                      // CLK1 start Rx freq
volatile uint32_t vfo_s = vfo_t;                              // Saved for RIT
volatile uint32_t vfo_lo;
uint32_t vco_c = 0;                                           // X-tal correction factor
uint32_t xt_freq;
long radix = 1000L, old_radix = 1000L;                          //start step size
boolean changed_f = 0, stepflag = 0, calflag = 0, modeflag = 0, ritset = 0;
boolean calibrate = 0;
byte  act_clk = 0, disp_txt = 0;

//smeter
unsigned long previousMillis = 0;
unsigned long starttime = 0;



byte bar5[8]={B00000,
B11011,
B11011,
B11011,
B11011,
B11011,
B11011,
B00000};

byte bar1[8]={B10000,
B11000,
B11100,
B11110,
B11110,
B11100,
B11000,
B10000};


/*****************************/
/* display S meter           */
/****************************/
void display_smeter(int strength)

{

 // Serial.println(analogRead(SMETER_IN));
// range is 0 to 1024 of adc  
//meter run from position 3 to 14 (12 )
int scale = (strength*60)/1024;
Serial.println(strength);
Serial.println(scale);
lcd.setCursor(0, 1);
lcd.print("S");
int smeter=(scale*11)/15;
if(smeter<10)
lcd.print(smeter);
else
lcd.print("9");
lcd.print(">");
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
//clear all after a small interval
lcd.setCursor(3, 1);
lcd.print("            ");
lcd.noCursor();
}


// write it and show for 500 milli sec
for (int i = 3; i<scale; i++)
  {
      lcd.setCursor(i, 1);
      lcd.write(byte(0));
      if(smeter>9)
      lcd.print("+");
  }

  
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
    case 0:                 // Tx frequency
      if (dir == 1)
        vfo_t += radix;
      if (dir == -1) {
        if (vfo_t < radix) break; // to prevent negative value
        vfo_t -= radix;
      }
      break;
    case 1:                 // Tx frequency (only if RIT is on)
      if (dir == 1)
        vfo_t += radix;
      if (dir == -1) {
        if (vfo_t < radix) break; // to prevent negative value      
        vfo_t -= radix;
      }
      break;
    case 2:                 // BFO frequency
      if (dir == 1)
        bfo_f += radix;
      if (dir == -1) {
        if (bfo_f < radix) break; // to prevent negative value      
        bfo_f -= radix;
      }
      break;
  }

  if(vfo_t > F_MAX)
    vfo_t = F_MAX;
  if(vfo_t < F_MIN)
    vfo_t = F_MIN;

  
  if(vfo_t <= (450000000ULL / SI5351_FREQ_MULT)) {
    digitalWrite(BAND_Switch, HIGH);
  } else {
    digitalWrite(BAND_Switch, LOW);
  }

  changed_f = 1;
}


/**************************************/
/* Read the buttons with debouncing   */
/**************************************/
boolean get_button()
{
  if (!digitalRead(ENCODER_BTN))            // Stepsize
  {
    //delay(20);
    delay(1);
    if (!digitalRead(ENCODER_BTN))
    {
      long strttime=millis();
      while (!digitalRead(ENCODER_BTN));
      if((millis() - strttime) > 1000)    // check if it was a long press
      {   
         modeflag = 1;
      } else 
      stepflag = 1;      
    }
  }
  else if (!digitalRead(Calibrbtn))         // Calibrate
  {
  //  delay(20);
   delay(1);
    if (!digitalRead(Calibrbtn))
    {
      while (!digitalRead(Calibrbtn));
      calflag = 1;
    }
  }
  else if (!digitalRead(TX_Switch))         // Selection
  {
   // delay(20);
    delay(1);
    if (!digitalRead(TX_Switch))
    {
      while (!digitalRead(TX_Switch));
      modeflag = 1;
    }
  }  
  if (stepflag | calflag | modeflag) return 1;
  else return 0;
}


/********************************************************************************
 * RIT switch handling
 * Switch to small stepsize (100 Hz)
 *******************************************************************************/
void RIT_switch()                               // Read RIT_switch
{
  if (!digitalRead(RIT_Switch) && ritset == 0){     // RIT on
    act_clk = 1;
    ritset = 1;
    vfo_s = vfo_t;                              // Save Tx freq
    old_radix = radix;                          // Save actual stepsize
    radix = 100;                                // Set stepsize to 100 Hz
  }
  else if (digitalRead(RIT_Switch) && ritset == 1){ // RIT 0ff
    act_clk = 0;                                // RTx mode
    ritset = 0;
    vfo_t = vfo_s;                              // Restore to original vco_t
    radix = old_radix;                          // Back to old stepsize
    disp_txt = 0;                               // Clear line
    
// Update Rx frequency based on the restored Tx frequency 
// update use -    
     if (vfo_t <= bfo_f) vfo_r = bfo_f + vfo_t;                                  // Upper / lower mixing
         else vfo_r = vfo_t - bfo_f;
        

    si5351.set_freq((vfo_r * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK1);
  }
}

void display_welcome_screen() {
  int i; 
  lcd.setCursor(0, 0);
  lcd.print("     SPECIAL FOR    ");
  lcd.setCursor(0, 1);
  lcd.print("INDONESIAN RED CROSS");
   for (i = 0 ; i < 16; i++) 
   {
    lcd.scrollDisplayRight(); 
    delay(250);
   } 

    //delay(1000);
    lcd.clear();
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
  // Print a message to the LCD.
 

 if(vfo_t < 8000000ULL) {
    lcd.print("L");
  } else {
    lcd.print("U");
  }



  switch(act_clk)
  {
    case 0:                               // Tx frequency
      freq = vfo_t+6190;
      break;
    case 1:                               // Tx frequency (Used in RIT Mode)
      freq = vfo_t;
      break;
    case 2:                               // MF frequency
      freq = bfo_f;
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

  switch (p)
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
    
 // display.setTextSize(2);  
   lcd.setCursor(3, 0);
 // display.println(LCDstr);
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
 // display.setTextSize(1);  
 // lcd.print(F("st:"));
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
      lcd.print(F("TRx"));
      break;
    case 1:
      lcd.print(F("RIT"));
      break;
    case 2:
      lcd.print(F("BFO"));
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
  

}




void setup() {

// Read EEPROM
  radix = eeprom_read_dword((const uint32_t *)EE_SAVED_RADIX);
  if ((radix < 10UL) | (radix > 1000000UL)) radix = 1000UL;  
  
  vfo_t = eeprom_read_dword((const uint32_t *)EE_SAVED_AFREQ);
  if ((vfo_t < F_MIN) | (vfo_t > F_MAX)) vfo_t = 14000000ULL;

  bfo_f = eeprom_read_dword((const uint32_t *)EE_SAVED_BFREQ);
  if ((bfo_f < F_MIN) | (bfo_f > F_MAX)) bfo_f = 10000000ULL;  

  vco_c = 0;
  if (eeprom_read_dword((const uint32_t *)EE_SAVED_CALBR) == 0x60)  {
    vco_c = eeprom_read_dword((const uint32_t *)EE_SAVED_OFSET);
  }  
  xt_freq = SI5351_XTAL_FREQ + vco_c;

//initialize the Si5351
  si5351.set_correction(0); // Set to zero because I'm using an other calibration method
 // si5351.set_correction(125000);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, xt_freq);    // Frequency get from settings in VFO_si5351.h file
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
// Set CLK0 to output the starting "vfo" frequency as set above by vfo = ?
//  si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
//  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA);
// Set CLK1 to output the Rx frequncy = vfo +/- bfo frequency
     if (vfo_t <= bfo_f) vfo_r = bfo_f + vfo_t;                                  // Upper / lower mixing
         else vfo_r = vfo_t - bfo_f;
  si5351.set_freq((vfo_r * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK1);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);
// Set CLK2 to output bfo frequency
 /**if(vfo_t < 8000000ULL) {
       vfo_lo=  bfo_f;//+ 3000ULL;
      } else {
        vfo_lo= bfo_f; //- 3000ULL;
      }   

        */
               
    if(vfo_t > 1000000000ULL) {
        vfo_lo=  bfo_f;//-1500ULL;
      } else {
        vfo_lo= bfo_f; 
      }  
      

//  si5351.set_freq((vfo_lo * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK2);
//  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);

// Encoder setup
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);                       // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

  sei();

// Pin Setup
  pinMode(Calibrbtn, INPUT_PULLUP);   // Calibrate
  pinMode(RIT_Switch, INPUT_PULLUP);  // RIT Switch
  pinMode(TX_Switch, INPUT_PULLUP);   // Select TRx or BFO
  pinMode(BAND_Switch, INPUT_PULLUP);

  digitalWrite(BAND_Switch, LOW);
  
// Display first time  
    lcd.begin(16, 2);
   // display_welcome_screen();
   display_frequency();  // Update the display
        lcd.createChar(0, bar5); 
  lcd.createChar(1, bar1);  

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

  get_button();
  
// Update the display if the frequency has been changed
  if (changed_f)  {
    display_frequency();

    if (act_clk == 0 && !calibrate)                   // No Tx update during calibrate
     // si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
      int i=1;
    else if (act_clk == 2)  
    // BFO update
    
  /**  if(vfo_t < 8000000ULL) {
       vfo_lo=  bfo_f;//+ 3000ULL;
      } else {
        vfo_lo= bfo_f; //- 3000ULL;
      }  
      */

                
    if(vfo_t > 1000000000ULL) {
        vfo_lo=  bfo_f;//-1500ULL;
      } else {
        vfo_lo= bfo_f; 
      }  
      
//        si5351.set_freq(vfo_lo * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK2);   // correct BFO frequency
      //  si5351.set_freq(vfo_t * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK0);   // Correct Tx freq
         if (vfo_t <= bfo_f) vfo_r = bfo_f + vfo_t;                                  // Upper / lower mixing
         else vfo_r = vfo_t - bfo_f;
        si5351.set_freq(vfo_r * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK1);   // correct Rx frequency
    changed_f = 0;
    disp_txt = 0;                   // Clear line
  }

  RIT_switch();                     // read RIT switch
  
// Button press
// Also stored the last used frequency together with the step size before store
//
  if (get_button()) {
    if (stepflag) {                 // Stepsize button
      eeprom_write_dword((uint32_t *)EE_SAVED_RADIX, radix);  // Store frequency and stepsize
      eeprom_write_dword((uint32_t *)EE_SAVED_AFREQ, vfo_t);
  
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
        radix = 10;
        break;       
      }
      stepflag  = 0;
    }
    else if (modeflag)  {         // Mode button
      if (act_clk == 0) act_clk = 2; else act_clk = 0;
      eeprom_write_dword((uint32_t *)EE_SAVED_BFREQ, bfo_f);
      modeflag = 0;  
      disp_txt = 0;                                 // Clear line
    }
    else if (calflag) {                             // Calibrate button
      if (!digitalRead(RIT_Switch)){                // RIT is on
        disp_txt = 1;                               // Message RIT off
      }
      else if (act_clk == 2){                       // BFO mode on
        disp_txt = 2;                               // Message BFO off        
      }
      else if (!calibrate)  {                       // Start calibrate
        vfo_s = vfo_t;                              // Save actual freq
        old_radix = radix;                          // and stepsize
        vfo_t = SI5351_XTAL_FREQ;                   // en set to default x-tal
        disp_txt = 3;                               // Message Calibrate
        calibrate = 1;
        radix = 10;                                 // Set to 10 Hz        
      //  si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0); // Set CLK0
      }
      else if (calibrate) {                         // after tuning x-tal freq
        calibrate = 0;
        vco_c = vfo_t - SI5351_XTAL_FREQ;           // difference
        vfo_t = vfo_s;                              // restore freq
        radix = old_radix;                          // and stepsize
        disp_txt = 4;                               // Message Calibrate OK
        
        eeprom_write_dword((uint32_t *)EE_SAVED_OFSET, vco_c);        // store correction
        xt_freq = SI5351_XTAL_FREQ + vco_c;                           // Calibrated x-tal freq
        eeprom_write_dword((uint32_t *)EE_SAVED_CALBR, 0x60);         // Calibrated
        si5351.init(SI5351_CRYSTAL_LOAD_8PF, xt_freq);                // Initialize
        si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);   
        
    if(vfo_t > 1000000000ULL) {
       vfo_lo=  bfo_f;//-1500ULL;
      } else {
        vfo_lo= bfo_f; 
      }     
      
//        si5351.set_freq(vfo_lo * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK2);   // correct BFO frequency
      //  si5351.set_freq(vfo_t * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK0);   // Correct Tx freq
         if (vfo_t <= bfo_f) vfo_r = bfo_f + vfo_t;                                  // Upper / lower mixing
         else vfo_r = vfo_t - bfo_f;
        si5351.set_freq(vfo_r * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK1);   // correct Rx frequency
      }
      calflag = 0;
    }
  }    
  
  display_frequency(); 
  //show smeter
 //  barplot(analogRead(analogInput));// Update displa
//

//display_smeter(analogRead(SMETER_IN));




} // end while loop
