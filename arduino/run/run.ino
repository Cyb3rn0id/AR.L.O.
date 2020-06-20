/*
 * ARLO
 * Robot Educativo basato su Arduino UNO
 * (c)2020 Giovanni Bernardo (https://www.settorezero.com)
 * 
 * progetto distribuito con licenza CC-BY-SA- NC 4.0
 * https://www.github.com/CyB3rn0id/arlo
 * 
 * 
 * CREDITS
 * 
 * Steve Garrat
 * Per l'idea dell'utilizzo del sonar HC-SR04 tramite interrupt
 * (https://homediyelectronics.com/projects/arduino/arduinoprogramminghcsr04withinterrupts/)
 * Il segnale di pulse viene schedulato mediante l'interrupt generato dal Timer 1
 * sfruttando una libreria apposita chiamata TimerOne scaricabile da:
 * https://github.com/PaulStoffregen/TimerOne
 * Si ha un interrupt del Timer1 ogni 50uS, per cui la durata del segnale di pulse è
 * pari a 50uS (sono necessari minimo 10uS per l'HC-SR04) e contemporaneamente si ha
 * conteggio che arriva fino a 200mS: arrivati a 200mS si invia un nuovo segnale e
 * così via, per cui la distanza tra un pulse e l'altro è di 200mS
 * La durata del segnale di echo viene misurata mediante la valutazione della distanza
 * di tempo che intercorre tra due interrupt generati dal cambio di stato
 * 
 * Nick Bontranger
 * Per la libreria che permette di pilotare i servo mediante il Timer 2
 * https://github.com/nabontra/ServoTimer2
 * questa libreria accetta il pilotaggio dei servo passando la durata dell'impulso di pilotaggio
 * piuttosto che l'angolazione come fa la libreria standard di Arduino.
 * Utilizzando servo modificati per la rotazione continua è controproducente utilizzare il valore
 * di angolo. Inoltre usando il Timer2 è possibile liberare il Timer1 (utilizzato per la libreria 
 * standard Servo) ed utilizzarlo per la generazione di interrupt
 * 
 * Adafruit
 * Perchè produce continuamente librerie per Arduino, e qui utilizziamo la loro per pilotare
 * un display OLED I2C 128x32
 */

// librerie utilizzate dal display oled
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Libreria per sfruttare l'interrupt sul Timer 1
#include <TimerOne.h>
// Libreria per utilizzare i servo con il Timer 2
#include <ServoTimer2.h>
// Libreria per utilizzare la memoria EEprom interna dell'ATmega328
#include <EEPROM.h>

// utilizzo pins
#define MotorRPin 9       // servocomando destro
#define MotorLPin 10      // servocomando sinitro
#define P1 6              // pulsante 1
#define P2 7              // pulsante 2
#define trigPin 8         // sonar, pin di trigger
#define echoPin 2         // sonar, pin di echo

// definizioni usate dai servocomandi
ServoTimer2 MotorL;       // oggetto motore sinistro
ServoTimer2 MotorR;       // offetto motore destro
uint16_t servoL_center=1500; // valore default centro per servo sinistro
uint16_t servoR_center=1500; // valore default centro per servo destro
uint8_t servoL_eeprom=0; // locazione di memoria in cui è contenuto il valore di centro del servo sinistro
uint8_t servoR_eeprom=2; // locazione di memoria in cui è contenuto il valore di centro del servo destro
#define SPEED  200 // velocità normale, in avanti
#define SPEED_SLOW 50 // velocità usata per le manovre

// definizioni usate dal sonar
#define SONAR_ECHO_INTERRUPT_ID 0 // ID interrupt su Arduino UNO
#define TIMER_US 50 // interrupt timer ogni 50uS
#define TICK_COUNTS 4000 // 4000*50uS = 200mS, distanza tra un pulse e il successivo
#define OBSTACLE 13 // a 13 cm rallento e mi fermo

// definizioni per display oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variabili globali
volatile long distance=0; // distanza rilevata dal sonar, espressa in cm
// enumerazione usata per la modalità di funzionamento
enum arlo_mode
  {
  configuration,
  normal  
  };
arlo_mode mode=normal;// modalità funzionamento ARLO
// enumerazione usata per le pagine del menu di configurazione
enum config_pages
  {
  left_motor=0,
  right_motor=1,
  config_end=2  
  };
  
void setup() 
  {
  // setup sonar
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // setup servo
  MotorL.attach(MotorLPin);   
  MotorR.attach(MotorRPin);
  // setup pulsanti
  pinMode(P1,INPUT_PULLUP);
  pinMode(P2,INPUT_PULLUP);

  // interrupts
  Timer1.initialize(TIMER_US); // Initialise timer 1
  Timer1.attachInterrupt(timer1_ISR); // Attach interrupt to the timer service routine 
  attachInterrupt(SONAR_ECHO_INTERRUPT_ID, sonarEcho_ISR, CHANGE); // Attach interrupt to the sensor echo input
  
  delay(2000);
  Serial.begin(9600);
  
  randomSeed(analogRead(6)); // inizializza il generatore di numeri casuali con un ingresso analogico disconnesso

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while(1); // Don't proceed, loop forever
    }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK);

  Serial.println("ARLO Startup");
  // valori di centro dei servocomandi, presi dall'eeprom
  // impostati ai valori di default se l'eeprom contiene valori anomali
  // o semplicemente non è mai stata inizializzata
  uint16_t LV=EEPROM.read(servoL_eeprom)<<8;
  LV|=EEPROM.read(servoL_eeprom+1);
  uint16_t RV=EEPROM.read(servoR_eeprom)<<8;
  RV|=EEPROM.read(servoR_eeprom+1);
  if (LV<2501 && LV>499) servoL_center=LV;
  if (RV<2501 && RV>499) servoR_center=RV;
  Serial.print("Valore centrale sinistra: ");
  Serial.println(LV);
  Serial.print("Valore centrale destra: ");
  Serial.println(RV);

  // se all'avvio P1 risulta premuto, avvio la modalità setup
  if (digitalRead(P1)==0) 
    {
      mode=configuration;
      while(digitalRead(P1)==0) {continue;}
      Serial.println("Configurazione");
    }
  }

void loop()
  {
  while (mode==configuration) config_menu();
  
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("ARLO RUN");
  display.print(distance);
  display.print("cm       ");
  display.display();  

  if(distance>OBSTACLE)
    {
    dritto(SPEED);
    }
  else
    {
    uint8_t randomNum=random(0,1);
    Serial.println("stop");
    // rampa di decelerazione
    dritto(SPEED-50);
    delay(100);
    dritto(SPEED-100);
    delay(100);
    dritto(SPEED-150);
    delay(100);
    // stop
    fermo(100);
    indietro(1000);
    fermo(100);
    if (randomNum)
        {
        destra(1000);
        }
    else
        {
        sinistra(1000);
        }
    fermo(100);
    }
  }

void config_menu(void)
  {
  static config_pages actual_page=0;
  static bool splash=true;

  if (splash)
    {
    display.setCursor(0,0);
    display.println("ARLO SETUP");
    display.display();  
    delay(2000);
    display.clearDisplay();
    display.setCursor(0,0);
    display.display(); 
    splash=false;
    delay(1000);
    }
  
  uint16_t val=analogRead(0);
  // mappo 0-1023 a 500-2500
  uint16_t pos=map(val, 0, 1023, 500, 2500);
  
  switch(actual_page)
    {
    case left_motor: // setup motore sinistro
      display.setCursor(0,0);
      display.setTextSize(1);
      display.print("LEFT MOTOR (");
      display.print(servoL_center);
      display.println(")");
      display.setTextSize(2);
      display.print(pos);
      display.print("  ");
      display.println();
      display.setTextSize(1);
      display.print("P1: save P2: next");
      display.display();  
      MotorL.write(pos);
    break;

    case right_motor: // setup motore destro
      display.setCursor(0,0);
      display.setTextSize(1);
      display.print("RIGHT MOTOR (");
      display.print(servoR_center);
      display.println(")");
      display.setTextSize(2);
      display.print(pos);
      display.print("  ");
      display.println();
      display.setTextSize(1);
      display.print("P1: save P2: next");
      display.display();  
      MotorR.write(pos);
    break;

    case config_end:
      display.setTextSize(2);
      display.setCursor(0,0);
      display.println("P1: Exit");
      display.println("P2: Return to Setup");
      display.display();
      break;
    } // switch

  // pulsante 1 premuto: conferma valore acquisito o uscita dal menù
  if (digitalRead(P1)==0)
    {
    delay(50);
    if(digitalRead(P1)==0)
      {
      while(digitalRead(P1)==0) {continue;}
      switch(actual_page)
        {
          case 0:
            servoL_center=pos;
            // divido il valore a 16 bit in due bytes
            // e li memorizzo in due locazioni di eeprom
            EEPROM.update(servoL_eeprom, (uint8_t)(pos>>8));
            delay(4); // an eeprom.write takes 3.3mS
            EEPROM.update(servoL_eeprom+1, (uint8_t)(pos&0x00FF));
            delay(4);
            Serial.print("Valore centrale sinistro: ");
            Serial.println(pos);
            break;

          case 1:
            servoR_center=pos;
            // divido il valore a 16 bit in due bytes
            // e li memorizzo in due locazioni di eeprom
            EEPROM.update(servoR_eeprom, (uint8_t)(pos>>8));
            delay(4);
            EEPROM.update(servoR_eeprom+1, (uint8_t)(pos&0x00FF));
            delay(4);
            Serial.print("Valore centrale destro: ");
            Serial.println(pos);
            break;

          case 2:
            // esco dalla configurazione
            mode=normal;
            display.clearDisplay();
            display.display();
            Serial.println("Uscita da configurazione");
            return;
            break;
        }
      }
    }

  // pulsante 2 premuto: cicla la pagina
  if (digitalRead(P2)==0)
    {
    delay(50);
    if(digitalRead(P2)==0)
      {
      while(digitalRead(P2)==0) {continue;}
      // dal momento che il C++ tratta le enumerazioni come un tipo a se stante
      // bisogna convertirla in un intero per poter usare l'operatore ++
      uint8_t tmp=(uint8_t)actual_page;
      tmp++;
      if (tmp>(uint8_t)config_end) tmp=0;
      actual_page=(config_pages)tmp;
      
      display.clearDisplay();
      display.display();
      delay(10);
      }
    }
  
  }

  
void dritto(uint16_t vel)
  {
  if (vel>SPEED) vel=SPEED;
  MotorL.write(servoL_center+vel);
  MotorR.write(servoR_center-vel);
  }

void indietro(long ms)
  {
  long timenow=millis();
  while((millis()-timenow)<ms)
    {
    MotorL.write(servoL_center-SPEED_SLOW);
    MotorR.write(servoR_center+SPEED_SLOW);
    }
  }

void destra(long ms)
  {
  long timenow=millis();
  while((millis()-timenow)<ms)
    {
    MotorL.write(servoL_center+SPEED_SLOW);
    MotorR.write(servoR_center+SPEED_SLOW);
    }
  }
  
void sinistra(long ms)
  {
  long timenow=millis();
  while((millis()-timenow)<ms)
    {
    MotorL.write(servoL_center-SPEED_SLOW);
    MotorR.write(servoR_center-SPEED_SLOW);
    }
  }

void fermo(long ms)
  {
  long timenow=millis();
  while((millis()-timenow)<ms)
    {
    MotorL.write(servoL_center);
    MotorR.write(servoR_center);
    }
  }
  
// Interrupt su Timer1 ogni 50uS : invio segnale di pulse
void timer1_ISR()
  {
  static volatile int state=0; // stato attuale dell'invio del pulse
  static volatile int trigger_time_count=0; // countdown per reinvio trigger
  
  if (!(--trigger_time_count)) // conta fino ad arrivare a 200mS
    {
    trigger_time_count=TICK_COUNTS; // ricarica il contatore
    state=1; // possiamo inviare un nuovo segnale di pulse
    }
    
  switch(state)
    {
    case 0: // non fa nulla: siamo in attesa che passino i 200mS di pausa
            // tra un invio e il successivo
      break;

    case 1:  // bisogna inviare il segnale di pulse
      digitalWrite(trigPin, HIGH);  // pin di trigger a livello alto: invio il segnale
      state=2; // passiamo allo stato successivo
      break;
        
    case 2: // bisogna spegnere il segnale di pulse (ci ritroviamo qui al successivo interrupt dallo stato 1)
    default:      
      digitalWrite(trigPin, LOW); // spengo il segnale di pulse
      state=0; // ritorno a fare nulla, ricominceremo tutto tra 200mS
      break;
    }
  }

// routine di interrupt sul cambio di stato del pin collegato all'echo
void sonarEcho_ISR()
  {
  static long echo_start=0; // salva il tempo in cui il segnale di echo è partito
  static long echo_end=0; // salva il tempo in cui il segnale di echo è terminato
  long echo_duration=0; // durata dell'echo espressa in microsecondi
  switch (digitalRead(echoPin)) // controlliamo se il pin di pulse è a livello alto o basso
    {
    case HIGH: // il segnale di pulse è appena arrivato
      echo_end=0;
      echo_start=micros(); // tempo di inizio
      break;
      
    case LOW: // il segnale di pulse è terminato
      echo_end=micros(); // tempo di fine
      if (echo_end>echo_start) // mi assicuro che non ci sia stato un overflow di micros()
        {
        echo_duration=echo_end-echo_start; // durata del segnale di echo, in microsecondi
        distance=echo_duration/58; // distanza espressa in centimetri
        }
      break;
    }
  }
  
