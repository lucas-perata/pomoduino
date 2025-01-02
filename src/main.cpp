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
void welcomeMessage();
void sendPostRequest(int minutes);


enum Category {
  Estudio,// 0
  Trabajo, // 1
  Programacion, // 2
  Ejercicio, // 3
  Meditacion, // 4
  Lectura // 5
};

#define ADC_CHANNEL 7 // A7


int segundos = 5; // 1500 segundos = 25 minutos
short int breakSeconds = 5; // 500 = 5 minutos
int selectSeconds = 1; // Valor por default
int selectBreakSeconds = 1; // Valor por default
bool timerIsRunning = false; 
bool breakTimerIsRunning = false;
Category selectedCategory = Estudio;
byte count  = 59;
byte pomos = 0; 
bool buzzer = false; 
bool screen = true; 
bool sync = false; 
short int syncAttempts = 0; 
bool backlight = true; 

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
    lcd.init();
    lcd.backlight();
    connectToWIFI();
    setupI2C_LCD();
    _delay_ms(2000);
    welcomeMessage();

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
      
      // Start-Stop timer 
      if (ButtonPressed(0, PINB, 0, 100)) {
        if(!timerIsRunning)  
        {
          timerIsRunning = true;
          PORTD ^= (1 << PORTD4);  // Toggle Buzzer

          if(segundos > 0)
          {
             PORTD ^= (1 << PORTD2);
          }
          else 
          {
            PORTD ^= (1 << PORTD3);
          }
        }
        else 
        {
          timerIsRunning = false;
          if(segundos > 0)
          {
             PORTD ^= (1 << PORTD2);
          }
          else 
          {
            PORTD ^= (1 << PORTD3);
          }
        }
      } 

      // Selección categoría 
      if(ButtonPressed(1, PINB, 1, 100)) 
      {
        selectCategory();
      }

      // Reseteo timer
      if(ButtonPressed(2, PINB, 2, 100))
      {
        timerIsRunning = false;
        resetTimer();
      }
    
      // Apagado-encendido de pantalla 

      // Sync 

      if(!timerIsRunning && ButtonPressed(3, PINB, 3, 100))
      {
          // syncData();    
          sendPostRequest(segundos / 60); 

      } 
      else if(ButtonPressed(3, PINB, 3, 100))
      {
        if(backlight)
        {
          lcd.noBacklight();
          backlight = false; 
        }
        else 
        {
          lcd.backlight();
          backlight = true; 
        }
      }

      basicTimer();
    }
}

void basicTimer()
{
   if(segundos > 0 && timerIsRunning) 
   {
    if (TCNT1 > 63000 && timerIsRunning)
      {
        if (!(PORTD & (1 << PORTD2))) {
          PORTD |= (1 << PORTD2); 
        }
          PORTD &= (1<<2); // Turn off buzzer
          sync = false; 
          TCNT1 = 0;
          segundos--;
          clearRow(0);
          lcd.setCursor(0, 0);
          lcd.print("Timer -> ");
          lcd.print(segundos/60);
          lcd.print(":");
          if(count < 10)
          {
            lcd.print("0");
          }
          lcd.print(count--);
          if(count == 0) {count = 59;}
      }
   }
   else if( segundos == 0 )
   {
    if(!sync && syncAttempts == 0)
    {
      sendPostRequest(selectSeconds / 60);
    }

    if (PORTD & (1 << PORTD2)) {
    PORTD &= ~(1 << PORTD2); 
    }

    if(!buzzer)
    {
      PORTD |= (1 << PORTD4);
    }
    
    breakTimer();
   }
}

void breakTimer()
{
  
  if(breakSeconds > 0 && timerIsRunning)
  {
    if (TCNT1 > 63000 && timerIsRunning)
      {

        if (!(PORTD & (1 << PORTD3))) {
          PORTD |= (1 << PORTD3);
          }
          if(!buzzer)
          {
            PORTD ^= 1 << PORTD4; 
            buzzer = true; 
          }
          PORTD &= (1<<2);
          PORTD ^= (1 << PORTD3);


          TCNT1 = 0;
          breakSeconds--;
          clearRow(0);
          lcd.setCursor(0, 0);
          lcd.print("Descanso -> ");
          lcd.print(breakSeconds/60);
          lcd.print(":");
          if(count < 10)
          {
            lcd.print("0");
          }
          lcd.print(count--);
          if(count == 0) {count = 59;}
      }
  }
  else if (breakSeconds == 0)
  {
    clearRow(1);
    pomos = pomos + 1;  
    lcd.setCursor(0,1);
    lcd.print("Cat: ");
    lcd.print(selectedCategory);
    lcd.print(" - P: ");
    lcd.print(pomos);
    if (PORTD & (1 << PORTD3))
    {
      PORTD &= ~(1 << PORTD3);
    }
    buzzer = false;
     if(!buzzer)
    {
      PORTD |= (1 << PORTD4);
    }
    sync = false; 
    continueTimer();
  }
}

void resetTimer()
{
  PORTD &= (1 << 1);
  segundos = selectSeconds;
  breakSeconds = selectBreakSeconds; 
  count = 59; 
  clearRow(0);
  lcd.setCursor(0, 0);
  lcd.print("Timer reseteado");
}

void continueTimer()
{
  segundos = selectSeconds;
  breakSeconds = selectBreakSeconds;
  timerIsRunning = true; 
}

void connectToWIFI() {
  lcd.clear(); 
  lcd.print("Conectando WiFi");
  
  String command = "AT+CWJAP=\"" + String(WIFI_SSID) + "\",\"" + String(WIFI_PWD) + "\"";
  espSerial.println(command);
  
  unsigned long startTime = millis();
  bool connected = false;

  while (millis() - startTime < 15000) { // Timeout after 15 seconds
    if (espSerial.available()) {
      String response = espSerial.readStringUntil('\n'); // Read the response
      Serial.println(response); // Print the response for debugging
      
      // Check for connection success or failure
      if (response.indexOf("OK") != -1) {
        lcd.clear();
        lcd.print("Conectado!"); // Connection successful
        connected = true;
        break;
      } else if (response.indexOf("FAIL") != -1) {
        lcd.clear();
        lcd.print("Falló la conexión"); // Connection failed
        break;
      }
    }
  }
  
  if (!connected) {
    lcd.clear();
    lcd.print("Timeout!"); 
  }
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
  selectedCategory = static_cast<Category>((selectedCategory + 1) % 6);
  lcd.setCursor(0,1);
  lcd.print("Cat: ");
  
    switch (selectedCategory)
    {
    case Estudio:
      lcd.print("0");
      break;
    case Trabajo: 
      lcd.print("1");
      break;
    case Programacion: 
    lcd.print("2");
    break;
    case Ejercicio: 
    lcd.print("3");
    break; 
    case Meditacion: 
    lcd.print("4");
    break; 
    case Lectura: 
    lcd.print("5");
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
  if(pV == 19)
    {
      selectSeconds = 1500; 
      segundos = selectSeconds;
      selectBreakSeconds = 300; 
      breakSeconds = selectBreakSeconds;
      clearRow(0);
      lcd.setCursor(0, 0);
      lcd.print("Pomodoro 25/5");
    } 
    else if(pV == 547)
    {
      selectSeconds = 2100; 
      segundos = selectSeconds;
      selectBreakSeconds = 600;
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

void welcomeMessage()
{
    clearRow(0);
    clearRow(1);
    lcd.setCursor(0, 0);
    lcd.print("Pomodoro 25/5");
    lcd.setCursor(0, 1);
    lcd.print("Cat: ");
    lcd.print(selectedCategory);
    lcd.print(" - P: ");
    lcd.print(pomos);
}

void sendPostRequest(int minutes) {
    String payload = "{"
                     "\"minutes\":" + String(minutes) + ","
                     "\"category\":" + String(selectedCategory) + "}";
                     
    espSerial.flush();
    Serial.print(payload);
    Serial.print(payload.length());

      clearRow(0); 
      lcd.setCursor(0, 0); 
      lcd.print("Sincronizando...");
    
    espSerial.println("AT+CIPSTART=\"TCP\",\"192.168.0.235\",7241");
    
    unsigned long start = millis();
    while (millis() - start < 10000) {
        if (espSerial.find("CONNECT")) break;
        if (espSerial.find("ERROR")) { 
            espSerial.println("AT+CWQAP");
            return;
        }
    }
    
    String request = String("POST /api/TimeEntry HTTP/1.1\r\n") +
                     "Host: 192.168.0.235:7241\r\n" +
                     "Content-Type: application/json\r\n" +
                     "Content-Length: " + String(payload.length()) + "\r\n" +
                     "Connection: close\r\n\r\n" + payload;
                     
    espSerial.println("AT+CIPSEND=" + String(request.length()));

    if (espSerial.find(">")) {
        espSerial.print(request);
        if (espSerial.find("SEND OK")) {
            espSerial.println("AT+CIPCLOSE");
            clearRow(0); 
            lcd.setCursor(0, 0); 
            lcd.print("Sincronizado!");
            sync = true; 
            syncAttempts = 0; 
        } 
    }

    if(!sync)
    {
       espSerial.println("AT+CIPCLOSE");
        clearRow(0); 
        lcd.setCursor(0, 0); 
        lcd.print("Error de sync...");
        syncAttempts++;
    }
} 



