///////////////////////////////////////////////
//                                           //
//   10 MHZ GPSDO                            //
//   v. 2020.11                              //
//   Victor Marinov /LZ1NY                   //
//                                           //
///////////////////////////////////////////////



// include the library code:
#include <SoftwareSerial.h>
#include <string.h>
#include <ctype.h> 




#define txPin A1	// mySerial
#define testPin  13	// 


#define RS 12	// lcd
#define RW 11	// lcd
#define ENA   10  //lcd
//#define OC1B  10 // PWM

#define rxPin A0   // mySerial from GPS
#define OC1A  9 // PWM

#define ICP1 8	// 1 Hz PPS
// 7
#define D4 6	// lcd
#define T1 5	// TMR 1 clock - 10 MHz
#define D5 4	// lcd
#define D6 3	// lcd
#define D7 2	// lcd
#define DUTY_MIN 6500
#define DUTY_MAX 0xFFF0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

//#include <LiquidCrystal.h>
//LiquidCrystal lcd(RS, RW, ENA, D4, D5, D6, D7);

#include <Adafruit_CharacterOLED.h>
Adafruit_CharacterOLED lcd(OLED_V2, RS, RW, ENA, D4, D5, D6, D7);

void printFloat(float value, int places);
unsigned long t1count=0, t1countp=0, current_time=0, timep=0, sum10=0;
uint8_t window=0, ii=0;
unsigned int byte_lp=0, byte_hp=0;
long int delta;
boolean toggle, pps_toggle=false, small, screen_cnange=false, home_screen=true, first_pps=true, pps_flag = false;

char console[100];

 int byteGPS=-1;
 char GPSline[350] = "";
 char comandoGPR[7] = "$GPRMC", time[13];
 int cont=0;
 int bien=0;
 int conta=0;
 int indices[13];
 unsigned int duty = 0x0, corr;
 uint8_t hour, min,sec;

#define base  10
#define scale 10

byte deltasign[8] = {  B00000, B00100, B01010, B01010, B10001, B10001, B11111, B00000  };

long int counts10mhz=1000000000;
uint8_t coef;


boolean coarse=true;


SoftwareSerial mySerial( rxPin, txPin, false); // RX, TX
//------------------------------------------------------
void tmr_setup() {

    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
 
    // Timer1 clock source - external clock, rising edge


    TCCR1B |= (1 << ICES1);
    TCCR1B |= (1 << ICNC1);

    //TCCR1B |= (1 << CS10);
    //TCCR1B |= (1 << CS11);
    //TCCR1B |= (1 << CS12);

    bitClear(TCCR1B, CS10);
    bitSet(TCCR1B, CS11);
    bitSet(TCCR1B, CS12);
    OCR1A = 0x22FF;

   
      // Interrupt setup
      // ICIE1: Input capture, TOIE1: Timer1 overflow
      TIFR1 = (1<<ICF1) | (1<<TOV1) ;// clear pending interrupts
      TIMSK1 = (1<<ICIE1) | (1<<TOIE1) ; // enable interupts

      // Set up the Input Capture pin, ICP1, Arduino Uno pin 8
      pinMode(ICP1, INPUT);
      digitalWrite(ICP1, 1);  // pull-up   
       
      pinMode(OC1A, OUTPUT);

    bitSet(TCCR1A,COM1A0);
    bitClear(TCCR1A,COM1A1);
   
   


    duty=0xffff/4;
    //duty=19300;

    // enable global interrupts:
    //sei();
    interrupts(); 
     }

//----------------------------------------------------------------------------------------- 
ISR(TIMER1_CAPT_vect) {
        unsigned int byte_l, byte_h;
        byte_l = ICR1L;   // grab captured timer value
        byte_h = ICR1H;
    		
        if (first_pps){  ICR1L =0; ICR1H=0; first_pps = false; ii = 0;   return; }

        window++;
    		ii++;
        
        pps_toggle = !pps_toggle; pps_flag =  true;



        if (coarse) { if (ii < base*2 )       {  return; }          coef=base;} 
        else {        if (ii < base*2*scale ) {  return; }          coef=base*scale;     }

        counts10mhz=10000000*coef;


       	ii = 0;
           
  

        unsigned long  counts;
        if (t1count < t1countp) { counts = (0xffffffff - t1countp + t1count ) << 16;  }
        else { counts = ((t1count - t1countp)<<16); }

        
        unsigned long tmr = (byte_h<<8) + byte_l;
        unsigned long tmrp = (byte_hp<<8) + byte_lp;

        counts += (0xffffffff - tmrp + tmr+1);

        
        

        delta =  (counts - counts10mhz);
        
        uint8_t k;
        char buffer[10];

        byte_lp = byte_l; byte_hp = byte_h;
        t1countp = t1count;
      

        if(delta>coef*5 or delta<-coef*5) {lcd.print("n.a.     ");}
        else
           {     if (coarse) {
                    if(delta>5 or delta<-5) { corr = delta*530;}   // >1Hz
                    else if(delta>1 or delta<-1) { corr = delta*300;} //
                    else { corr = delta*150;}  }

                  else {
                    if(delta>5 or delta<-5) { corr = delta*53;}
                    else if(delta>2 or delta<-2) { corr =  delta*30;}
                    else { corr = delta*15;}  }


        duty = duty - corr;

        dtostrf(  ((float) delta) /coef, 4,2,&console[0]);
        strcat(&console[0], "Hz, coef: ");
        itoa (coef, buffer, 10);
        strcat(&console[0], buffer);
        strcat(&console[0], ", corr: ");  
        itoa (-corr, buffer, 10);
        strcat(&console[0], buffer);
        strcat(&console[0], ", PWM: ");
        itoa (duty, buffer, 10);
        strcat(&console[0], buffer);
        strcat(&console[0], "\r\n");
        screen_cnange =  true;
          }
       
       
        home_screen =  false;
        
        if (coarse and  (delta < 5 and delta>-5)) {coarse =  false;} // 0.5Hz  coef=10
        
        if (!coarse and  (delta< scale*5 and delta <-5*scale)) {coarse =  true;} //   coef = 100
              
        //coarse = true;
       
   
    }       
//---------------------------------------------------
ISR(TIMER1_OVF_vect) {
      t1count++;
      toggle = !toggle;

      if( digitalRead(OC1A)==0)   { OCR1A = - duty;}
      else                        { OCR1A = duty;}
      return; 
}
//-------------------------------------------------------------------------
void setup() {
            uint8_t c;

            tmr_setup();
            pinMode(testPin, OUTPUT);
            // set up the LCD's number of columns and rows:
            lcd.begin(16, 2);
            lcd.createChar(0, deltasign);

            // Print a message to the LCD.
            //delay(200);
            lcd.clear();          //  delay(200);
            //lcd.home();          //  delay(200);
            lcd.setCursor(0,0);
            lcd.print("   LZ1NY"); 

            lcd.setCursor(0,1);
            lcd.print("10 MHz GPSDO");  
            //lcd.print("0123456789ABCDEF");

            Serial.begin(38400);
            Serial.print("10 MHz GPSDO\r\n");
            mySerial.begin(9600);

            delay(5000);

            lcd.clear(); 

            lcd.setCursor(0,0);
            lcd.print("Sync Wait"); 
                      }

//---------------------------------------------------------------------------
void print_data_lcd() {

            char line_delta[15];
            lcd.setCursor(0,0);
            lcd.write(byte(0));
            lcd.print("F=");
            dtostrf(  ((float) delta) /coef, 4,2,line_delta);

            lcd.print(line_delta);


            lcd.print("Hz  ");
            


                            }


//-------------------------------------------------------------------------
void print_time_pwm_lcd() {
            lcd.setCursor(0,1);

            if (ii%10<6)  {   lcd.print(time); }
            else          {   lcd.print("PWM:");  lcd.print(duty);  lcd.print(" ");}


                          }
//-----------------------------------------------------------------------------
void gps_data(void){
         byteGPS=mySerial.read();         // Read a byte of the serial port

         
         GPSline[conta]=byteGPS;        
         conta++;                      
          
         if (byteGPS==13){            // If the received byte is = to 13, end of transmission   // note: the actual end of transmission is <CR><LF> (i.e. 0x13 0x10)
                   cont=0;
                   bien=0;
                   
                   for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPR, // The following for loop starts at 1, because this code is clowny and the first byte is the <LF> (0x10) from the previous transmission.
                     if (GPSline[i]==comandoGPR[i-1]){    bien++;       }
                                         }       


                   if(bien==6){               // If yes, continue and process the data
                     for (int i=0;i<300;i++){
                       if (GPSline[i]==','){  indices[cont]=i;  cont++;  }
                       if (GPSline[i]=='*'){  indices[12]=i;    cont++; }
                                            }
                        
                        int j=indices[0];
                        
                        time[0] = 0;
                        
                        
                        strncat(&time[0],&GPSline[j+1],1);
                        strncat(time,&GPSline[j+2],1);
                        strncat(time,":",1);
                        strncat(time,&GPSline[j+3],1);
                        strncat(time,&GPSline[j+4],1);
                        strncat(time,":",1);
                        strncat(time,&GPSline[j+5],1);
                        strncat(time,&GPSline[j+6],1);  
                        strncat(time,"z",4);          

                              }
                   
                         
                         conta=0;                    // Reset the buffer
                         for (int i=0;i<340;i++){     GPSline[i]=' '; }                 
                                }
                      

}
//********************************************************************************************
void loop() {

      if (mySerial.available() > 0) {
               //byteGPS=mySerial.read();        
               //Serial.write(byteGPS);
               gps_data();


        ;
      }

      if (screen_cnange){
                  print_data_lcd();
                  Serial.write(time);
                  Serial.write("\t");
                  Serial.write(console);
                  screen_cnange = false;        }


       if (pps_flag) {

                  lcd.setCursor(10,0);
                  if(pps_toggle){lcd.print(" *");}
                  else {lcd.print("  ");}



                  lcd.setCursor(10,1);
                  lcd.print("    ");
                  int progress;

                  if(coarse) {progress = ii*5;}
                  else       {progress = ii/2;}



                  if(progress<10) {lcd.setCursor(11,1);}
                  else{lcd.setCursor(10,1);}

                  lcd.print(progress);



                  if ( !home_screen) {
                      lcd.setCursor(15,1);
                      if(coarse){lcd.print("C");}
                      else {lcd.print("F");} } 


                  print_time_pwm_lcd();
                  pps_flag =  false;

                           }


}










