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

unsigned long segundos = 5; // 1500 segundos = 25 minutos
short int breakSeconds = 5; // 500 = 5 minutos
bool timerIsRunning = false; 
bool breakTimerIsRunning = false;
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

  // LED on PORTD2 and LED on PORTD3 Buzzer on PORTD4
 // DDRD = (1 << DDD2) | (1 << DDD3);  
  PORTD &= ~(1 << PORTD2);           
  PORTD &= ~(1 << PORTD3);     
  PORTD &= ~(1 << PORTD4);

  // PORTB (botones) 0 al 3
  DDRB = 0b00000000; 
  PORTB = (1 << PINB0) | (1 << PINB1) | (1 << PINB2) | (1 << PINB3);  

 

 // connectToWIFI();    


    TCCR1A = 0; // Set Timer1 to Normal mode (WGM13:0 = 0)
    TCCR1B = (1 << CS12); // Set prescaler to 64



  while (1) {
    
    if (ButtonPressed(0, PINB, 0, 100)) {
      if(!timerIsRunning)  
      {
        timerIsRunning = true;
        PORTD ^= (1 << PORTD2);  // Toggle LED
        PORTD ^= (1 << PORTD4);  // Toggle Buzzer
      }
      else 
      {
        PORTD ^= (1 << PORTD2);
        timerIsRunning = false;
        }
    } 


    if(ButtonPressed(1, PINB, 1, 100)) 
    {
      selectCategory();
    }

    if(ButtonPressed(2, PINB, 2, 100))
    {
      PORTD &= (1 << 1);
      lcd.clear();
      lcd.print("Timer reseteado");
      resetTimer();
    }

    basicTimer();
  }

}

void basicTimer()
{
   if(segundos > 0) 
   {
    if (TCNT1 > 63000 && timerIsRunning)
      {
          PORTD &= (1<<2); // Turn off buzzer
          TCNT1 = 0;
          segundos--;
          // TODO Fix lcd clear
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Timer -> ");
          lcd.print(segundos);
          lcd.print(":");
          lcd.print(count--);
          printCat();
          if(count == 0) {count = 59;}
      }
   }
   else if( segundos == 0 )
   {
    breakTimer();
   }
}

void breakTimer()
{
  // TODO: Implementar break timer LCD
  if(breakSeconds > 0)
  {
    // TODO: Leds Apagar 2 y prender 3
    if (TCNT1 > 63000 && timerIsRunning)
      {
          TCNT1 = 0;
          breakSeconds--;
          lcd.clear();
          lcd.print(breakSeconds);
      }
  }
  else 
  {
    resetTimer();
  }
}

void resetTimer()
{
  breakTimerIsRunning = false; 
  timerIsRunning = false;
  segundos = 1500;
  count = 59;
}

void connectToWIFI() {
  espSerial.begin(115200);

  espSerial.println("AT");
  _delay_ms(1000);
  espSerial.println("AT+CWJAP=\"Fibertel  2.4GHz\",\"\"");
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




void printCat() 
{
  lcd.setCursor(0, 1);
  lcd.print("Cat: ");
  lcd.print(selectedCategory);
}