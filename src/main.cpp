#include <avr/io.h>
#include <Arduino.h>
#include <Wire.h>
#define numberOfButtons 4
#include <ButtonPress.h>
#include <LiquidCrystal_I2C.h> 
#include <util/delay.h>
#include <SoftwareSerial.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define SERVO_MIN_PULSE_WIDTH 500  // 1ms pulse width (minimum position)
#define SERVO_MAX_PULSE_WIDTH 1500  // 2ms pulse width (maximum position)
#define SERVO_PERIOD 20000          // 20ms total period (50Hz)


SoftwareSerial espSerial(1, 0); // RX, TX
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16 columns, 2 rows

void basicTimer();
void breakTimer();
void connectToWIFI();
void printESPResponse();
void setupI2C_LCD(); 
void resetTimer();
void selectCategory();
byte convertSeconds();
void setServoAngle(uint16_t pulseWidth);
void custom_delay_us(uint16_t us);
void ocBox();
void printCat();

enum Category {
  Study, 
  Test
};

unsigned long segundos = 1500; 
int breakSeconds = 500;
bool timerIsRunning = false; 
bool breakTimerIsRunning = false;
bool isBoxOpen = false;
Category selectedCategory = 0;
byte count  = 59;
// variable malloc para almacenar los timers terminados
// o si no también meter un postrequest siempre que termine un timer

// TODO: track timers completados en una variable


int main(void) {
  sei();
init();
  Wire.begin();     
  
  setupI2C_LCD();

  // PORTD
  DDRD = 0b00011101;

  // LED on PORTD2 and Buzzer on PORTD3
 // DDRD = (1 << DDD2) | (1 << DDD3);  
  PORTD &= ~(1 << PORTD2);           
  PORTD &= ~(1 << PORTD3);     

  // PORTB (botones)
  DDRB = 0b00000000; 
  PORTB = (1 << PINB0) | (1 << PINB1) | (1 << PINB2) | (1 << PINB3);  

 

 // connectToWIFI();    


    TCCR1A = 0; // Set Timer1 to Normal mode (WGM13:0 = 0)
    TCCR1B = (1 << CS12); // Set prescaler to 64


    setServoAngle(500);

  while (1) {
    
    if (ButtonPressed(0, PINB, 0, 100)) {
      lcd.clear();
      lcd.print("Coloca tu celu");
      lcd.setCursor(0, 1);
      lcd.print("en la caja");
      if(isBoxOpen == false)
      {
        setServoAngle(1500);
       _delay_ms(4000);
        setServoAngle(500);
      }
      else {
        _delay_ms(4000);
        setServoAngle(500);
      }
     

      PORTD ^= (1 << PORTD2);  // Toggle LED
      PORTD ^= (1 << PORTD3);  // Toggle Buzzer
      if(!timerIsRunning) {timerIsRunning = true;}
     else {timerIsRunning = false;}
    } 


    if(ButtonPressed(1, PINB, 1, 100)) 
    {
      selectCategory();
    }

    if(ButtonPressed(2, PINB, 2, 100))
    {
      isBoxOpen = true; 
      setServoAngle(1500);
      timerIsRunning = false;
      PORTD ^= 1 << PORTD3;
      lcd.clear();
      lcd.print("Timer reseteado");
      resetTimer();
    }

    basicTimer();
    
  }

}

void basicTimer()
{
   if(segundos > 0 && timerIsRunning) 
   {
    if (TCNT1 > 63000)
      {
          TCNT1 = 0;
          segundos--;
          // TODO Fix lcd clear
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Timer -> ");
          lcd.print(segundos / 60);
          lcd.print(":");
          lcd.print(count--);
          printCat();
          if(count == 0) {count = 59;}
      }
   }
   else if( segundos == 0 && !timerIsRunning)
   {
    timerIsRunning = false;
    breakTimerIsRunning = true;
    breakTimer();
   }
}

void breakTimer()
{
  setServoAngle(1500);
  if(breakSeconds > 0 && breakTimerIsRunning)
  {
    if (TCNT1 > 63000)
      {
          TCNT1 = 0;
          breakSeconds--;
          lcd.clear();
          lcd.print(breakSeconds);
      }
  }
  else 
  {
    breakTimerIsRunning = false;
    setServoAngle(500);
    resetTimer();
  }
}

void resetTimer()
{
  segundos = 1500;
  count = 59;
}

void connectToWIFI() {
  espSerial.begin(115200);

  espSerial.println("AT");
  _delay_ms(1000);
  espSerial.println("AT+CWJAP=\"Fibertel WiFi131 2.4GHz\",\"00439434465\"");
}

void setupI2C_LCD() {
  lcd.init();        
  lcd.backlight();   
  lcd.clear();       
  lcd.print("Ready");  
}

void selectCategory() {
  lcd.setCursor(0,1);
  lcd.print("Cat: ");

    switch (selectedCategory)
    {
    case 0:
      selectedCategory = 1;
      lcd.print("Test 1");
      break;
    case 1: 
      lcd.print("Test 2");
      selectedCategory = 2;
      break;
    case 2: 
    selectedCategory = 0; 
    lcd.print("Test 3");
    break;
    default:
      break;
    }
}


void setServoAngle(uint16_t pulseWidth) {
    // Ensure pulse width is within valid range (1ms to 2ms)
    if (pulseWidth < SERVO_MIN_PULSE_WIDTH) pulseWidth = SERVO_MIN_PULSE_WIDTH;
    if (pulseWidth > SERVO_MAX_PULSE_WIDTH) pulseWidth = SERVO_MAX_PULSE_WIDTH;
    
    // Generate the PWM signal manually by toggling the pin
    PORTD |= (1 << PORTD4);  // Set PORTD4 high (start pulse)
    custom_delay_us(pulseWidth);   // Wait for the pulse width duration
    PORTD &= ~(1 << PORTD4); // Set PORTD4 low (end pulse)

    // Wait for the rest of the 20ms period
    custom_delay_us(SERVO_PERIOD - pulseWidth);
}

void custom_delay_us(uint16_t us) {
    // Loop to create a delay for the given number of microseconds
    while (us--) {
        // Each iteration of this loop takes approximately 4 clock cycles
        _delay_loop_1(4); 
    }
}

void printCat() 
{
  lcd.setCursor(0, 1);
  lcd.print("Cat: ");
  lcd.print(selectedCategory);
}