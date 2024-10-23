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
#include <Config.h>


SoftwareSerial espSerial(6 ,7); // RX, TX
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16 columns, 2 rows

void basicTimer();
void breakTimer();
void connectToWIFI();
void printESPResponse();
void setupI2C_LCD(); 
void resetTimer();
void continueTimer();
void selectCategory();
byte convertSeconds();
void setServoAngle(uint16_t pulseWidth);
void custom_delay_us(uint16_t us);
void printCat();
void clearRow(byte row);
void selectPomodoro(uint16_t pV);


enum Category {
  Study, 
  Test
};

#define ADC_CHANNEL 7 // A7 corresponds to ADC channel 7


int segundos = 5; // 1500 segundos = 25 minutos
short int breakSeconds = 5; // 500 = 5 minutos
int selectSeconds = 5; // Valor por default
int selectBreakSeconds = 5; // Valor por default
bool timerIsRunning = false; 
bool breakTimerIsRunning = false;
Category selectedCategory = 0;
byte count  = 59;
byte pomos = 0; 

// TODO: track timers completados en una variable

void printESPResponse() {
  while (espSerial.available()) {
    String response = espSerial.readStringUntil('\n'); // Lee hasta un salto de línea
    Serial.println(response); // Muestra la respuesta en el monitor serial
  }
}

void ADC_Init() {
    ADMUX = (1 << REFS0) | (ADC_CHANNEL); // AVCC with external capacitor at AREF pin, select ADC channel
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler = 128
}

uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADCW; // Read ADC value
}

int main(void) {
  sei();
  init();
  Wire.begin();   
  ADC_Init();  
  

  // PORTD
  DDRD = 0b10011101;

  // LED on PORTD2 and LED on PORTD3 Buzzer on PORTD4
 // DDRD = (1 << DDD2) | (1 << DDD3);  
  PORTD &= ~(1 << PORTD2);           
  PORTD &= ~(1 << PORTD3);     
  PORTD &= ~(1 << PORTD4);

  // PORTB (botones) 0 al 3
  DDRB = 0b00000000; 
  PORTB = (1 << PINB0) | (1 << PINB1) | (1 << PINB2) | (1 << PINB3);  

 




    TCCR1A = 0; // Set Timer1 to Normal mode (WGM13:0 = 0)
    TCCR1B = (1 << CS12); // Set prescaler to 64

    Serial.begin(9600);
    espSerial.begin(9600);
    Serial.println("Estableciendo conexión...");
    // connectToWIFI();

    setupI2C_LCD();
    _delay_ms(2000);

    clearRow(0);
    clearRow(1);
    lcd.setCursor(0, 0);
    lcd.print("Pomodoro 25/5");
    lcd.setCursor(0, 1);
    lcd.print("Cat: ");
    lcd.print(selectedCategory);
  while (1) {

    uint16_t potValue = ADC_Read(ADC_CHANNEL);

    selectPomodoro(potValue);


  if (Serial.available()) {
    char c = Serial.read();
    espSerial.write(c); 
  }

  if (espSerial.available()) {
    char c = espSerial.read();
    Serial.write(c); 
  }

  printESPResponse(); 
    
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
      espSerial.println("AT");
    }
    if(ButtonPressed(2, PINB, 2, 100))
    {
      PORTD &= (1 << 1);
      clearRow(0);
      lcd.setCursor(0, 0);
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
    if (timerIsRunning)
    {
      if (!(PORTD & (1 << PORTD2))) {
          PORTD |= (1 << PORTD2); 
        }
    }
    if (TCNT1 > 63000 && timerIsRunning)
      {
          PORTD &= (1<<2); // Turn off buzzer
          TCNT1 = 0;
          segundos--;
          // TODO Fix lcd clear
          clearRow(0);
          lcd.setCursor(0, 0);
          lcd.print("Timer -> ");
          lcd.print(segundos/60);
          lcd.print(":");
          lcd.print(count--);
          if(count == 0) {count = 59;}
      }
   }
   else if( segundos == 0 )
   {
    if (PORTD & (1 << PORTD2)) {
    PORTD &= ~(1 << PORTD2); 
    }
    
    breakTimer();
   }
}

void breakTimer()
{

  PORTD &= (1<<2);
  if (!(PORTD & (1 << PORTD3))) {
    PORTD |= (1 << PORTD3); 
  }
  if(breakSeconds > 0)
  {
    if (TCNT1 > 63000 && timerIsRunning)
      {
          TCNT1 = 0;
          breakSeconds--;
          clearRow(0);
          lcd.setCursor(0, 0);
          lcd.print("Descanso -> ");
          lcd.print(breakSeconds/60);
          lcd.print(":");
          lcd.print(count--);
          if(count == 0) {count = 59;}
      }
  }
  else 
  {
     if (PORTD & (1 << PORTD3)) {
    PORTD &= ~(1 << PORTD3); 
    }
    clearRow(0);
    pomos = pomos + 1;  
    lcd.setCursor(0,1);
    lcd.print("           - P=");
    lcd.print(pomos);
    continueTimer();
  }
}

void resetTimer()
{
  timerIsRunning = false;
  segundos = selectSeconds;
  breakSeconds = selectBreakSeconds; 
}

void continueTimer()
{
  segundos = selectSeconds;
  breakSeconds = selectBreakSeconds;
  timerIsRunning = true; 
}

void connectToWIFI() {
  espSerial.println("AT");
  _delay_ms(500);
  String command =  "AT+CWJAP=\"" + String(WIFI_SSID) + "\",\"" + String(WIFI_PWD) + "\"";
  espSerial.println(command);
  _delay_ms(1000);
}

void setupI2C_LCD() {
  lcd.init();        
  lcd.backlight();   
  lcd.clear();       
  lcd.print("- POMODUINO V1 -");  
  lcd.setCursor(0, 1);
  lcd.print("- Lucas Perata -");
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
  lcd.print((int)pomos);
}

void clearRow(byte row)
{
  lcd.setCursor(0, row);
  lcd.print("                ");
}

void selectPomodoro(uint16_t pV)
{
 if(!timerIsRunning) 
 {
  if(pV == 12)
    {
      selectSeconds = 5; 
      segundos = selectSeconds;
      selectBreakSeconds = 5; 
      breakSeconds = selectBreakSeconds;
      clearRow(0);
  lcd.setCursor(0, 0);
      lcd.print("Pomodoro 25/5");
    } 
    else if(pV == 547)
    {
      selectSeconds = 6; 
      segundos = selectSeconds;
      selectBreakSeconds = 6;
      breakSeconds = selectBreakSeconds;
      clearRow(0);
      lcd.setCursor(0, 0);
      lcd.print("Pomodoro 35/10");
    } 
    else if(pV == 1001)
    {
      selectSeconds = 7; 
      segundos = selectSeconds;
      selectBreakSeconds = 7; 
      breakSeconds = selectBreakSeconds;
      clearRow(0);
      lcd.setCursor(0, 0);
      lcd.print("Pomodoro 50/10");
    }
 }
}

// TODO: Buzzer cambio entre tiempo y break 
// TODO: Post request / Sync 
// TODO: Detecting multiple button pressed 
// TODO: lcd backlight off auto 
