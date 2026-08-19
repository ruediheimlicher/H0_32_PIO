#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

///
/// @mainpage	Robot_32
///
/// @details	PWM for Servos
///
/// @file		Robot_32.ino
/// @brief		Main sketch
///
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
///
/// @copyright	(c) Ruedi Heimlicher, 2019
////// @see		ReadMe.txt for references
///

// https://github.com/MaximilianBlase/EA-DogLibrary

// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu

#include "Arduino.h"

#include "main.h"
#include "defines.h"
#include <ADC.h>
#include <ADC_util.h>

#include <SPI.h>
#include "gpio_MCP23S17.h"

// #include <Wire.h>
// #include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

// #include <hd44780.h>
////#include <hd44780ioClass/hd44780_I2Cexp.h>

// hd44780_I2Cexp lcd;

// #include "lcd.h"
// #include "analog.h"

#include <EEPROM.h>

#include <RF24.h>
// #include <RF24Network.h>

// Load Wi-Fi library
// #include <ESP8266WiFi.h>

////#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// #define OLED_DC     6
// #define OLED_CS     7
// #define OLED_RESET  8
// Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

// instantiate an object for the nRF24L01 transceiver

#define CE_PIN 3 // Teensy_FS: Pin 9
#define CSN_PIN 23
RF24 radio(CE_PIN, CSN_PIN);
uint16_t errcounter = 0;
uint16_t errcounterA = 0;
uint16_t errcounterB = 0;
uint16_t errcounterC = 0;
uint16_t radiocounter = 0;
const uint64_t pipeOut = 0xABCDABCD71LL; // NOTE: The address in the Transmitter and Receiver code must be the same "0xABCDABCD71LL" | Verici ve Alıcı kodundaki adres aynı olmalıdır

// ********************
// ACK data ***********
uint8_t ackData[4] = {31, 32, 33, 34};
// ********************
// ********************

elapsedMillis zeitintervall;
uint8_t sekundencounter = 0;
elapsedMillis sinceLastBlink = 0;

uint8_t weichentastencounter = 0;

uint8_t firstruncounter = 0;

Signal data;
void ResetData()
{
   data.task = 0;
   data.A = 127; // pot
   data.B = 127;
   data.C = 127;
}
/*
void OSZIA_HI(void)
{
   digitalWriteFast(OSZIA_PIN, HIGH);
}
void OSZIA_LO(void)
{
   digitalWriteFast(OSZIA_PIN, LOW);
}
void OSZIA_TOG()
{
   digitalWriteFast(OSZIA_PIN, !(digitalRead(OSZIA_PIN)));
}
*/
uint16_t lerp(uint16_t a, uint16_t b, float t)
{
   return a * (1 - t) + b * t;
}

uint16_t lerpint(uint16_t a, uint16_t b, uint16_t t)
{
   /*
   Use an integer for your parameter F, with F from 0 to 1024 instead of a float from 0 to 1. Then you can just do:
  */
   return (a * (1024 - t) + b * t) >> 10;
}

ADC *adc = new ADC(); // adc object
// Set parameters

#define TEST 1

// Define structures and classes

// Define variables and constants
// #define STARTWERT  2840 // Mitte
#define STARTWERT 2080 // Nullpunkt
#define MAXWERT 4096   // Nullpunkt

#define LOOPLED 0 //

#define CONTROL_PIN 6

#define CURR_PIN A6

#define ANZLOKS 6

#define ANZLOKALLOKS 6 // anz loks bei lokalem Betrieb
#define ANZLOKALPOTS 4

#define IMPULSTASK 1
#define PAUSETASK 2

#define POT_0_PIN A0
#define POT_1_PIN A1
#define POT_2_PIN A2
#define POT_3_PIN A3

// #define SOURCECONTROL    6 // Eingang, HI wenn local

#define OSZI_PULS_A 8
#define OSZI_PULS_B 9
#define OSZI_PULS_C 2

volatile uint8_t loopstatus = 0;
volatile uint8_t loopcounter = 0;
volatile uint8_t sourcestatus = 1; // local/USB
uint8_t oldsourcestatus = 1;
#define LOCAL 0
#define USB 1
#define FIRSTRUN 1
#define PAUSEBIT 2 // Pause zwischen Datenserie
#define WAITBIT 3

#define SERIAL_OK 2

byte buffer[64];

byte sendbuffer[64];

elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

volatile uint8_t usbtask = 0;
volatile uint8_t looptask = 0;

volatile uint8_t teensytask = 0;

volatile uint16_t aktualcommand = 0;

volatile uint16_t commandarray0[32] = {0};

volatile uint16_t taskarray[8][32] = {0};

volatile uint16_t adressearray[4];
volatile uint16_t eepromadressearray[8][4];
volatile uint16_t speedarray[4];

volatile uint16_t loknummerTRITarray[4] = {0};
volatile uint8_t loknummer = 0;

volatile uint8_t speed = 0;
volatile uint8_t speed_raw = 0;

volatile uint8_t richtungcounter = 0; // mehrere richtungdatenpakete bei Richtungswechsel

volatile uint8_t richtungstatus = 0;
#define MAXRICHTUNGCOUNTER 4 // max anzahl richtungdatenpakete
#define RICHTUNGSTART 0
#define RICHTUNGEND 1

uint8_t minanzeige = 0xFF;
// let GET_U:UInt8 = 0xA2
// let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;

elapsedMillis sinceweiche;

float sinpos = 0;
#define pi 3.14
#define SIN_START 0xE0
#define SIN_STOP 0xE1

#define SPI_CLK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_MCP_CS 10

#define SPI_SR_CS 22

// cs 10
// DIP-tasten, function
gpio_MCP23S17 mcp0(SPI_MCP_CS, 0x20); // instance 0 (address A0,A1,A2 tied to 0)

// Weichen A
gpio_MCP23S17 mcp1(SPI_SR_CS, 0x20); // instance 1 (address A0=1, A1 = A2 = 0)

// Weichen B
gpio_MCP23S17 mcp2(SPI_SR_CS, 0x22); // instance 2 (address A0=0, A1 = 1, A2 = 0)

uint8_t regA = 0x0;
uint8_t regB = 0;

#define ANZWEICHENGRUPPEN 1
#define GRUPPE_0 0
#define GRUPPE_1 1
uint8_t weichenaddressarray[2][4] = {{2, 2, 2, 1}, {2, 2, 1, 1}};
uint8_t weichenposition[2] = {}; // 0: gerade 1: ablenkung

#define RINGBUFFER_SIZE 32
#define RINGBUFFER_MASK (RINGBUFFER_SIZE - 1)

typedef struct
{
   uint8_t weiche;
   uint8_t richtung;
} weichendata;

typedef struct
{
   weichendata data[RINGBUFFER_SIZE];
   volatile uint8_t head;
   volatile uint8_t tail;
} RingBuffer; // ringbuffer fuer weichenposition

void rb_init(RingBuffer *rb)
{
   rb->head = 0;
   rb->tail = 0;
}

bool rb_is_empty(RingBuffer *rb)
{
   return rb->head == rb->tail;
}

bool rb_is_full(RingBuffer *rb)
{
   return ((rb->head + 1) & RINGBUFFER_MASK) == rb->tail;
}

bool rb_push(RingBuffer *rb, weichendata value)
{
   uint16_t next = (rb->head + 1) & RINGBUFFER_MASK;

   if (next == rb->tail)
      return false; // voll

   rb->data[rb->head] = value;
   rb->head = next;

   return true;
}

int rb_pop(RingBuffer *rb, weichendata *value)
{
   if (rb_is_empty(rb))
      return -1; // leer

   *value = rb->data[rb->tail];
   rb->tail = (rb->tail + 1) & RINGBUFFER_MASK;

   return 0;
}

uint16_t rb_count(RingBuffer *rb)
{
   if (rb->head >= rb->tail)
      return rb->head - rb->tail;

   return RINGBUFFER_SIZE - rb->tail + rb->head;
}

RingBuffer weichenringbuffer = {0};
uint8_t rinbuffercounter = 0;

volatile uint8_t tastencodeA = 0;
volatile uint8_t tastencodeB = 0;
volatile uint8_t tastencodeC = 0;
volatile uint8_t tastencodeD = 0;

volatile uint8_t weichentastencodeC = 0;
volatile uint8_t weichentastencodeD = 0;
volatile uint8_t oldweichentastencodeC = 0;
volatile uint8_t oldweichentastencodeD = 0;

// mcp2
volatile uint8_t weichentastencodeE = 0;
volatile uint8_t weichentastencodeF = 0;
volatile uint8_t oldweichentastencodeE = 0;
volatile uint8_t oldweichentastencodeF = 0;

uint8_t weichentastenstatusC = 0;
uint8_t weichentastenstatusD = 0;

uint8_t oldweichentastenstatusC = 0;
uint8_t oldweichentastenstatusD = 0;

uint8_t weichenxor0 = 0;
uint8_t weichenxor1 = 0;
uint8_t weichenxor2 = 0;
uint8_t weichenxor3 = 0;
uint8_t weichenxor4 = 0;

uint8_t weichennummer = 0; // 0..7
uint8_t weichenrichtung = 0;

uint8_t oldweichennummer = 0xFF; // 0..7
uint8_t oldweichenrichtung = 0xFF;

uint8_t tastenstatusA = 0;
// pi.__BEGIN_DECLS

volatile uint8_t tastenadresseA = 0;
volatile uint8_t tastenadresseB = 0;
volatile uint8_t tastenadresseC = 0;
volatile uint8_t tastenadresseD = 0;

volatile uint8_t diptastenadresseA = 0; // dipschalter wird von links gelesen, bit 0 ist ganz links
volatile uint8_t diptastenadresseB = 0;
volatile uint8_t diptastenadresseC = 0; // dipschalter wird von links gelesen, bit 0 ist ganz links
volatile uint8_t diptastenadresseD = 0;

volatile uint8_t lokaladressearray[ANZLOKALLOKS] = {}; // Lok-Adressen
volatile uint8_t lokalcodearray[ANZLOKALLOKS] = {};    // Lok-Codes (Richtung, Funktion)

volatile uint8_t usbadressearray[4] = {}; // Lok-Adressen
volatile uint8_t usbcodearray[4] = {};    // Lok-Codes (Richtung, Funktion)

uint8_t tastenstatusB = 0;

uint8_t localpotarray[ANZLOKALPOTS] = {};
volatile uint8_t speedraw0 = 0;
volatile uint8_t speedraw1 = 0;
volatile uint8_t speedraw2 = 0;
volatile uint8_t speedraw3 = 0;
uint8_t speedrawcounter = 0;

uint8_t lokalstatus = 0;
#define LOKALRICHTUNGBIT0 0
#define LOKALRICHTUNGBIT1 1
#define LOKALRICHTUNGBIT2 2
#define LOKALRICHTUNGBIT3 3
elapsedMillis sincelocalrichtung;

uint8_t potpinarray[4] = {POT_0_PIN, POT_1_PIN, POT_2_PIN, POT_3_PIN};

const char *buffercode[4] = {"BUFFER_FAIL", "BUFFER_SUCCESS", "BUFFER_FULL", "BUFFER_EMPTY"};

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa

#define HI 0xFEFE   // 1111111011111110
#define LO 0x0202   // 0000001000000010
#define OPEN 0x02FE // 0000001011111110

#define TIMERINTERVALL 24 // Zeitfenster für Serie
#define PAUSE 5           // Abstand zwischen Paketen
// Utilities
elapsedMillis sinceringbuffer;

elapsedMillis sincewegbuffer;

elapsedMillis sinceemitter;

elapsedMillis sincemcp;

uint16_t abschnittindex = 0; // aktuelles Element in positionsarray

// Create an IntervalTimer object
IntervalTimer paketTimer;
volatile uint16_t timerintervall = TIMERINTERVALL;

IntervalTimer stromTimer;
volatile uint16_t emitter = 0;
volatile uint16_t emitterarray[8] = {0};
volatile uint16_t emittermittel = 0;
volatile uint8_t emittermittelcounter = 0;
volatile uint16_t emitterNULL = 330;
volatile uint8_t pause = PAUSE;
volatile uint8_t richtung = 1; // vorwaerts

volatile uint8_t paketpos = 0;
volatile uint8_t paketmax = 2 * ANZLOKS;

volatile uint8_t commandpos = 0; // pos im command
volatile uint8_t bytepos = 0;    // pos im Ablauf

uint16_t tritarray[] = {LO, OPEN, HI};

int achse0_startwert = 0;
uint8_t asciicounter = 0;

volatile uint8_t weichebyte6 = 0; // letztes Adressbit
volatile uint8_t weichebyte7 = 0;
volatile uint8_t weichenstatus = 0;
volatile uint16_t weichencounter = 0;
volatile uint8_t tastencounter = 0;

volatile uint16_t usbweichencounter = 0;
volatile uint8_t usbgeradecounter = 0;
volatile uint8_t usbablenkungcounter = 0;



uint8_t aktuelleweiche = 0xFF;

uint8_t weichenXORcounterA = 0;
uint8_t weichenXORcounterB = 0;
uint8_t weichenXORcounterC = 0;
uint8_t weichenXORcounterD = 0;

volatile uint8_t weichenbuffer[4] = {};

#define WEICHESTART 0
#define WEICHERUN 1
#define WEICHEDELAY 2
#define WEICHEREADY 3

#define WEICHEOFF 5
#define ABLENKUNG 6
#define GERADE 7
#define WEICHENIMPULSDAUER 0xAF
#define MAXWEICHENCOUNTER  125

LiquidCrystal_I2C lcd(
    0x27,       // I2C-Adresse
    2, 1, 0,    // EN, RW, RS  (PCF8574 Bits)
    4, 5, 6, 7, // D4–D7      (PCF8574 Bits)
    3,          // Backlight Bit
    POSITIVE);

void printHex8(uint8_t data) // prints 8-bit data in hex with leading zeroes
{
   // Serial.print("0x");
   // for (int i=0; i<length; i++)
   {
      if (data < 0x10)
      {
         // Serial.print("0");
      }
      // Serial.print(data,HEX);
      // Serial.println(" ");
   }
}
// Functions

void OSZI_A_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A, LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A, HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B, LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B, HIGH);
}

void OSZI_B_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_B, !digitalRead(OSZI_PULS_B));
}

void OSZI_C_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C, LOW);
}

void OSZI_C_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C, HIGH);
}

void OSZI_C_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_C, !digitalRead(OSZI_PULS_C));
}

void pakettimerfunction()
{
   /*
    commands
    LO     0x0202  // 0000001000000010
    OPEN   0x02FE  // 0000001011111110
    HI     0xFEFE  // 1111111011111110
    */

   aktualcommand = (paketpos < ANZLOKS) ? taskarray[paketpos][bytepos] : 0; // zu schickendes command, 16 bit; während Pause (paketpos >= ANZLOKS) keinen OOB-Read
                                                                            // errcounter++;
   // OSZI_C_TOGG();
   if ((bytepos) == 0)
   {
      // errcounter++; // 140
      if (paketpos == 0) // syncsignal
      {
         // errcounter++; // 20
         looptask = IMPULSTASK;
         OSZI_A_LO(); // sync
         OSZI_B_HI();
   
      }

      if ((paketpos == 1))
      {
         OSZI_B_LO();
      }

      if (paketpos == ANZLOKS)
      {
         looptask = PAUSETASK;
      }

      if (paketpos == ANZLOKS - 1) // Weiche
      {
         // weichebyte6 = taskarray[paketpos][2];
         // weichebyte7 = taskarray[paketpos][3];
         // sinceweiche = 0;
      }
   }

   if ((aktualcommand & (1 << commandpos)) && (paketpos < ANZLOKS)) // bit an pos commandpos: zB LO  0000001000000010
   {
      digitalWriteFast(CONTROL_PIN, HIGH);
   }
   else
   {
      digitalWriteFast(CONTROL_PIN, LOW);
   }

   if ((paketpos == 1))
   {
      OSZI_B_LO();
   }

   if ((paketpos == ANZLOKS))
   {
      OSZI_B_LO();
   }
   commandpos++;
   if (commandpos > 19)
   {
      commandpos = 0;
      OSZI_A_HI(); // sync, erster Impuls fertig generiert
      errcounter++;
      bytepos++;
      if (bytepos >= 20 + pause) // Paket fertig
      {
         errcounterB++;
         bytepos = 0;
         paketpos += 1;
         // jede Lok ein Paket
         if (paketpos >= paketmax - 1) // Paketserie fertig
         {
            paketpos = 0;
         }
      }
   }
}

/*
void LCD_init(void)
{
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   digitalWrite(LCD_RSDS_PIN,1);
   digitalWrite(LCD_ENABLE_PIN,1);
   digitalWrite(LCD_CLOCK_PIN,1);

}
*/
void lcdputint3(uint16_t zahl) // int bis 1000
{
   char string[4];
   int8_t i; // schleifenzähler

   string[3] = '\0'; // String Terminator
   for (i = 2; i >= 0; i--)
   {
      string[i] = (zahl % 10) + '0'; // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   lcd.print(string);
}

void lcdputint1(uint16_t zahl) // int bis 1000
{
   char string[2];
   string[0] = (zahl) + '0';
   string[1] = '\0';
   lcd.print(string);
}

void ADC_init(void)
{
   emitter = 0; //

   adc->adc0->setAveraging(4);  // set number of averages
   adc->adc0->setResolution(8); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
   // adc->adc0->enableInterrupts(ADC_0);

   //   delay(100);
}

void loadtaskarray(uint8_t pos)
{
   volatile uint16_t aa[4];
   aa[0] = HI;
   aa[1] = HI;
   aa[2] = HI;
   aa[3] = OPEN;
   taskarray[pos][0] = aa[0];
   taskarray[pos][1] = aa[1];
   taskarray[pos][2] = aa[2];
   taskarray[pos][3] = aa[3];
   taskarray[pos][4] = HI; // Lampe
   taskarray[pos][5] = 0;  // speedarray[0];
   taskarray[pos][6] = 1;  // speedarray[1];
   taskarray[pos][7] = 2;  // speedarray[2];
   taskarray[pos][8] = 3;  // speedarray[3];

   // pause
   taskarray[pos][9] = 0;
   taskarray[pos][10] = 0;
   taskarray[pos][11] = 0;

   // wiederholung
   taskarray[pos][12] = taskarray[pos][0];
   taskarray[pos][13] = taskarray[pos][1];
   taskarray[pos][14] = taskarray[pos][2];
   taskarray[pos][15] = taskarray[pos][3];
   taskarray[pos][16] = taskarray[pos][4];
   taskarray[pos][17] = taskarray[pos][5];
   taskarray[pos][18] = taskarray[pos][6];
   taskarray[pos][19] = taskarray[pos][7];
   taskarray[pos][20] = taskarray[pos][8];

   taskarray[pos][21] = 0;
   taskarray[pos][22] = 0;
   taskarray[pos][23] = 0;
}

uint8_t checkDoubleAddress(void)
{
   for (uint8_t i = 0; i < ANZLOKALLOKS; i++)
   {
      for (uint8_t k = 0; k < ANZLOKALLOKS; k++)
      {
         if (k != i)
         {
            if (lokaladressearray[i] == lokaladressearray[k])
            {
               return (lokaladressearray[k]);
            }
         }
      }
   }
   return 0;
}

void stromtimerfunction()
{
   emitter = adc->analogRead(CURR_PIN);
   emitterarray[emittermittelcounter & 0x07] = emitter;
   emittermittelcounter++;
}

// Add setup code
void setup()
{
   // Serial.begin(9600);
   // Serial.begin(115200);
   Wire.begin();
   delay(100);
   lcd.begin(20, 4);
   delay(100);

   lcd.home(); // go home
   // lcd.print("H032 ESP");

   loopstatus |= (1 << FIRSTRUN); // Bit fuer tasks in erster Runde
   delay(100);
   //  analogWriteResolution(16); // 32767

   lcd.setCursor(18, 0);
   lcd.print("D");

   // paketTimer.begin(pakettimerfunction,timerintervall);
   // paketTimer.priority(0);

   // stromTimer.begin(stromtimerfunction, 5000);

   pinMode(SPI_SR_CS, OUTPUT);
   digitalWrite(SPI_SR_CS, HIGH);

   pinMode(LOOPLED, OUTPUT);

   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4
   // FTM2   25, 32
   // analogWriteFrequency(5, 50);
   // Serial.println(F("RawHID H0"));

   // pinMode(TAKT_PIN, OUTPUT);
   // digitalWriteFast(TAKT_PIN, LOW); // LO, OFF

   // Control
   pinMode(CONTROL_PIN, OUTPUT);
   digitalWriteFast(CONTROL_PIN, HIGH); // HI, OFF

   pinMode(OSZI_PULS_A, OUTPUT);
   digitalWriteFast(OSZI_PULS_A, HIGH);
   pinMode(OSZI_PULS_B, OUTPUT);
   digitalWriteFast(OSZI_PULS_B, HIGH);
   pinMode(OSZI_PULS_C, OUTPUT);
   digitalWriteFast(OSZI_PULS_C, HIGH);

   // ghpinMode(SOURCECONTROL, INPUT);

   // LCD_init();
   lcd.setCursor(18, 0);
   lcd.print("E");
   //                Configure the NRF24 module  | NRF24 modül konfigürasyonu
   radio.begin();

   radio.openWritingPipe(pipeOut);

   radio.setChannel(124);
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available

   radio.setPALevel(RF24_PA_MAX); // Output power is set for maximum range  |  Çıkış gücü maksimum menzil için ayarlanıyor.

   radio.setPALevel(RF24_PA_MIN);
   radio.setPALevel(RF24_PA_MAX);
   radio.enableAckPayload();
   radio.setRetries(0, 0);
   radio.stopListening(); // Start the radio comunication for Transmitter | Verici için sinyal iletişimini başlatır.
   if (radio.failureDetected)
   {
      radio.failureDetected = false;
      delay(250);
      lcd.setCursor(19, 0);
      lcd.println("-");
   }
   else
   {
      lcd.setCursor(19, 0);

      lcd.println("+");
   }
   // Serial.println("printDetails:");
   // radio.printDetails();
   lcd.setCursor(18, 0);
   lcd.print("+");
   ResetData();

   mcp0.begin(0);
   /*
    • PortA registeraddresses range from 00h–0Ah
    • PortB registeraddresses range from 10h–1Ah
    PortA output, PortB input: Direction 1 output: direction 0
    0x0F: A: out B: in
    */

   // mcp0.gpioPinMode(0x00FF); // A Ausgang, B Eingang
   // delay(100);
   lcd.setCursor(18, 0);
   lcd.print("b");
   mcp0.gpioPinMode(0xFFFF); // alle input
   lcd.setCursor(18, 0);
   lcd.print("c");
   mcp0.portPullup(0x00FF);
   //

   mcp0.gpioPort(0xCC33);

   /* *********************************** */
   // mcp1
   /* *********************************** */
   lcd.setCursor(18, 0);
   lcd.setCursor(18, 0);

   // lcd.print("d");

   /*
   OUTPUT   : 0
   INPUT    : 1
   Breadboard
   pin mode:
   Bank B
   7  : out w0
   5,6: in
   4  : out w1
   3,2: in
   1,0: aux, default out


   Bank A
   15    :out w2
   14,13 : in
   12    : out w3
   11,10 : in
   9,8   : aux, default out

   bin: 0110 1100 0110 1100
   hex: 6C6C
   */

   /*
  Print:
  OUTPUT   : 0
  INPUT    : 1
  pin mode:
  Bank B
  7  : set C
  6  : set A
  5  : Out 0
  4  : Out 1
  3  : Set B
  2  : Set D
  1,0: aux, default out
  1100 1100 > 0xCC


  Bank A
  15    :out w2
  14,13 : in
  12    : out w3
  11,10 : in
  9,8   : aux, default out

  bin: 1100 1100 1100 1100
  hex: 0xCCCC
  */
   /*
       MCP23S17 Pin	Library Number original
       GPA0	0
       GPA1	1
       GPA2	2
       GPA3	3
       GPA4	4
       GPA5	5
       GPA6	6
       GPA7	7
       GPB0	8
       GPB1	9
       GPB2	10
       GPB3	11
       GPB4	12
       GPB5	13
       GPB6	14
       GPB7	15
   */

#define PIN_MODE 0xCCCC //
   /*
   // PIN-Nummern regulaer:
   #define SET_A_A  6   // 1
   #define SET_A_B  3   // 1
   #define OUT_A_A  5   // 0

   #define SET_A_C  7   // 1
   #define SET_A_D  2   // 1
   #define OUT_A_B  4   // 0

   #define SET_B_A  14   // 1
   #define SET_B_B  11   // 1
   #define OUT_B_A  13   // 0

   #define SET_B_C  15   // 1
   #define SET_B_D  10   // 1
   #define OUT_B_B  12   // 0
   */

   // PIN-Nummern Bank vertauscht:

#define SET_A_A 9  // 1
#define SET_A_B 12 // 1
#define OUT_A_A 10 // 0

#define SET_A_C 8  // 1
#define SET_A_D 13 // 1
#define OUT_A_B 11 // 0

#define SET_B_A 6 // 1
#define SET_B_B 3 // 1
#define OUT_B_A 5 // 0

#define SET_B_C 7 // 1
#define SET_B_D 2 // 1
#define OUT_B_B 4 // 0

#define SET_A_A_BIT 1 // 1
#define SET_A_B_BIT 4 // 1

#define SET_A_C_BIT 0 // 1
#define SET_A_D_BIT 5 // 1

#define SET_B_A_BIT 6 // 1
#define SET_B_B_BIT 3 // 1

#define SET_B_C_BIT 7 // 1
#define SET_B_D_BIT 2 // 1

   mcp1.begin(0);
   lcd.setCursor(18, 0);
   lcd.print("e");
   //_delay_ms(100);

   // mcp1.gpioPinMode(PIN_MODE);// A7 output, A6,A5 input 0110 1100 0110 1100
   mcp1.gpioPinMode(0x33CC);
   mcp1.gpioPinMode(0, 0);

// mcp1.gpioPinMode(10,OUTPUT);
// mcp1.gpioPinMode(11,OUTPUT);
#define CHECK 15
#define BLINK 0
   mcp1.gpioPinMode(CHECK, OUTPUT);
   mcp1.gpioPinMode(BLINK, OUTPUT);
   // 11001100
   mcp1.gpioPort(0xFFFF); // alle HI

   // ***********************************
   // mcp2
   // ***********************************
   lcd.setCursor(18, 0);
   lcd.print("f");
   mcp2.begin(0);
   lcd.setCursor(18, 0);
   lcd.print("g");
   // mcp2.gpioPinMode(PIN_MODE);// A7 output, A6,A5 input
   mcp2.gpioPinMode(0x33CC);
   mcp2.gpioPinMode(10, OUTPUT);
   mcp2.gpioPinMode(11, OUTPUT);

   mcp2.gpioPort(0xFFFF); // alle HI

   EEPROM.begin();
   lcd.setCursor(18, 0);
   lcd.print("h");
   // delay(100);
   usbtask = 0;
   adressearray[0] = OPEN;
   adressearray[1] = HI;
   adressearray[2] = LO;
   adressearray[3] = OPEN;

   speed = 0;

   // init_analog();
   for (uint8_t i = 0; i < 4; i++)
   {

      if (speed & (1 << i))
      {
         speedarray[i] = HI;
      }
      else
      {
         speedarray[i] = LO;
      }
   }

   lcd.setCursor(18, 0);
   lcd.print("g");
   eepromadressearray[0][0] = tritarray[buffer[8]];

   for (uint8_t p = 0; p < 8; p++)
   {
      loadtaskarray(p);
   }

   aktualcommand = OPEN;

   pinMode(POT_0_PIN, INPUT);
   pinMode(POT_1_PIN, INPUT);
   pinMode(POT_2_PIN, INPUT);
   // pinMode(CURR_PIN, INPUT);

   // lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   lcd.setCursor(18, 0);
   lcd.print("i");
   //_delay_ms(100);
   lcd.setCursor(18, 0);
   lcd.print("j");

   ADC_init();
   lcd.setCursor(18, 0);
   lcd.print("k");
   delay(100);
   // Serial.print("setup: ");
   //  lcd.init();
   //  lcd.backlight();
   //  lcd.setCursor(0,0);
   //  lcd.print("H0-32");
   //_delay_ms(200);
   //  lcd.clear();
   uint8_t eepromtimerintervall = EEPROM.read(0xA0);
   if (eepromtimerintervall < 0xFF) // schon ein Wert gespeichert
   {
      timerintervall = eepromtimerintervall;
   }
   //  lcd.setCursor(0,0);
   //  lcd.print(timerintervall);
   uint8_t eeprompos = 0x00;
   uint8_t eepromadressbyte = 0;
   /*
   eepromadressbyte = EEPROM.read(eeprompos++);
   // Serial.print("adresse: ");
   // Serial.print(eeprompos);
   // Serial.print(" byte: ");
   // Serial.println(eepromadressbyte);
    // Serial.print("\n");
*/
   lcd.setCursor(18, 0);
   lcd.print("F");
   taskarray[0][0] = tritarray[buffer[8]];
   taskarray[0][1] = tritarray[buffer[9]];
   taskarray[0][2] = tritarray[buffer[10]];
   taskarray[0][3] = tritarray[buffer[11]];

   /*
      delay(50);
         eepromadressbyte = EEPROM.read(eeprompos);
      taskarray[0][0] = tritarray[eepromadressbyte];
         // Serial.print("adresse: ");
         // Serial.print(eeprompos);
         // Serial.print(" byte: ");
         // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
        delay(50);
        eeprompos++;
        eepromadressbyte = EEPROM.read(eeprompos);
      taskarray[0][1] = tritarray[eepromadressbyte];
            // Serial.print("adresse: ");
            // Serial.print(eeprompos);
            // Serial.print(" byte: ");
            // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
        delay(10);
        eeprompos++;
        eepromadressbyte = EEPROM.read(eeprompos);
      taskarray[0][2] = tritarray[eepromadressbyte];
            // Serial.print("adresse: ");
            // Serial.print(eeprompos);
            // Serial.print(" byte: ");
            // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
        delay(10);
        eeprompos++;
        eepromadressbyte = EEPROM.read(eeprompos);
        taskarray[0][3] = tritarray[eepromadressbyte];
            // Serial.print("adresse: ");
            // Serial.print(eeprompos);
            // Serial.print(" byte: ");
            // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);

      // repetition address
        taskarray[0][12] = taskarray[0][0] ;
        taskarray[0][13] = taskarray[0][1] ;
        taskarray[0][14] = taskarray[0][2] ;
        taskarray[0][15] = taskarray[0][3] ;
      // pause
       taskarray[0][9] = 0;
       taskarray[0][10] = 0;
       taskarray[0][11] = 0;

      delay(50);

      eeprompos = 0x08;
          eepromadressbyte = EEPROM.read(eeprompos);
       taskarray[1][0] = tritarray[eepromadressbyte];
          // Serial.print("adresse: ");
          // Serial.print(eeprompos);
          // Serial.print(" byte: ");
          // Serial.println(eepromadressbyte);
       //  lcd.print(" ");
      //  lcd.print(eepromadressbyte);

         delay(50);
         eeprompos++;
         eepromadressbyte = EEPROM.read(eeprompos);
       taskarray[1][1] = tritarray[eepromadressbyte];
             // Serial.print("adresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
         delay(10);
         eeprompos++;
         eepromadressbyte = EEPROM.read(eeprompos);
       taskarray[1][2] = tritarray[eepromadressbyte];
             // Serial.print("adresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
         delay(10);
         eeprompos++;
         eepromadressbyte = EEPROM.read(eeprompos);
         taskarray[1][3] = tritarray[eepromadressbyte];
             // Serial.print("adresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);

      delay(50);

       eeprompos = 0x10;
           eepromadressbyte = EEPROM.read(eeprompos);
        taskarray[2][0] = tritarray[eepromadressbyte];
           // Serial.print("adresse: ");
           // Serial.print(eeprompos);
           // Serial.print(" byte: ");
           // Serial.println(eepromadressbyte);
       //  lcd.print(" ");
      //  lcd.print(eepromadressbyte);

          delay(50);
          eeprompos++;
          eepromadressbyte = EEPROM.read(eeprompos);
        taskarray[2][1] = tritarray[eepromadressbyte];
              // Serial.print("adresse: ");
              // Serial.print(eeprompos);
              // Serial.print(" byte: ");
              // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
          delay(10);
          eeprompos++;
          eepromadressbyte = EEPROM.read(eeprompos);
        taskarray[2][2] = tritarray[eepromadressbyte];
              // Serial.print("adresse: ");
              // Serial.print(eeprompos);
              // Serial.print(" byte: ");
              // Serial.println(eepromadressbyte);
      //  lcd.print(eepromadressbyte);
          delay(10);
          eeprompos++;
          eepromadressbyte = EEPROM.read(eeprompos);
          taskarray[2][3] = tritarray[eepromadressbyte];

              // Serial.print("adresse: ");
              // Serial.print(eeprompos);
              // Serial.print(" byte: ");
              // Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
    */
   lcd.clear();
   lcd.setCursor(19, 1);
   lcd.print("X");
   paketTimer.begin(pakettimerfunction, timerintervall);

   paketTimer.priority(0);

   loopstatus |= (1 << WAITBIT);
   looptask = IMPULSTASK;
}

// Add loop code
void loop()
{
   // #pragma mark mcp
   if (loopstatus & (1 << FIRSTRUN))
   {

      // firstruncounter++;
      /*
      if (firstruncounter > 10)
      {
         //paketTimer.begin(pakettimerfunction, timerintervall);

         lcd.setCursor(18, 0);
         lcd.print("X");
         //paketTimer.priority(0);
         loopstatus &= ~(1 << FIRSTRUN);
         lcd.setCursor(18, 0);
         lcd.setCursor(18, 0);

         lcd.print("F");
         //errcounter++;
      }
      */
   }

   // if (sincemcp > 10)
   // if((loopstatus & (1<<PAUSEBIT)) && !(loopstatus & (1<<WAITBIT)))
   if (looptask == PAUSETASK)
   {
      OSZI_C_LO();

      // digitalWriteFast(SPI_SR_CS, !digitalReadFast(SPI_SR_CS)); // CS aktivieren

      if (radio.write(&data, sizeof(data)))
      {
         radiocounter++;

         // ********************
         // ACK Payload ********

         if (radio.isAckPayloadAvailable())
         {

            radio.read(&ackData, sizeof(ackData));
            radio.flush_rx();

            // speed
            localpotarray[2] = ackData[0]; // von nRF pot 2
            localpotarray[3] = ackData[2]; // von nRF pot 3

            // code
            tastencodeC = ackData[1]; // data von diptasten

            uint8_t tastencodeC_raw = (tastencodeC & 0xF0) >> 4; // oberste 4 Bit diptasten

            tastenadresseC = (tastencodeC & 0xF0) >> 4;
            lokaladressearray[2] = 0xFF - tastencodeC_raw;

            lokalcodearray[2] = (tastencodeC & 0x0F); // Bit 0-3: Richtung (Bit 1) und Lampe (Bit 0)

            tastencodeD = ackData[3];
            uint8_t tastencodeD_raw = (tastencodeD & 0xF0) >> 4; // oberste 4 Bit diptasten
            lokaladressearray[3] = 0xFF - tastencodeD_raw;
            lokalcodearray[3] = tastencodeD & 0x0F; // Bit 0-3: Richtung (Bit 1) und Lampe (Bit 0)
         }
         else
         {
            // errcounter++;
         }

         // ********************
         // ********************
      }
      else
      {
         // Serial.println("radio error\n");
         digitalWrite(BUZZPIN, !(digitalRead(BUZZPIN)));
         // errcounter++;
      }

      sincemcp = 0;
      // bit 0: Funktion
      // bit 1: Richtungsimpuls
      // bit 4-7: Adresse lesen: SPI MCP23S17
      tastencodeA = 0xFF - mcp0.gpioReadPortA(); // active taste ist LO > invertieren

      // 240702: Tastencode invertiert, analog Trafo und H0-Interface
      uint8_t tastencodeA_raw = (tastencodeA & 0xF0) >> 4; // oberste 4 Bit diptasten

      tastenadresseA = (tastencodeA & 0xF0) >> 4;

      lokaladressearray[0] = 0xFF - tastencodeA_raw;

      lokalcodearray[0] = tastencodeA & 0x0F; // Bit 0-3 Richtung und Lampe

      for (uint8_t i = 0; i < 4; i++)
      {
         if (tastenadresseA & (1 << (i)))
         {
            diptastenadresseA &= ~(1 << 2 * i);
            diptastenadresseA &= ~(1 << (2 * i + 1));
         }
         else
         {
            diptastenadresseA |= (1 << 2 * i);
            diptastenadresseA |= (1 << (2 * i + 1));
         }
      }

      tastencodeB = 0xFF - mcp0.gpioReadPortB(); // active taste ist LO > invertieren
      tastenadresseB = (tastencodeB & 0xF0) >> 4;

      // 240702: Tastencode invertieren, > DIP-code wird analog Trafo und H0-Interface
      uint8_t tastencodeB_raw = (tastencodeB & 0xF0) >> 4;

      // lokaladressearray[1] = (tastencodeB & 0xF0) >> 4;

      lokaladressearray[1] = 0xFF - tastencodeB_raw; // oberste 4 bit sind 1

      lokalcodearray[1] = tastencodeB & 0x0F; // Bit 0-3

      for (uint8_t i = 0; i < 4; i++)
      {
         // if (tastenadresseB & (1<<(3-i)))
         if (tastenadresseB & (1 << (i)))
         {
            diptastenadresseB &= ~(1 << 2 * i);
            diptastenadresseB &= ~(1 << (2 * i + 1));
         }
         else
         {
            diptastenadresseB |= (1 << 2 * i);
            diptastenadresseB |= (1 << (2 * i + 1));
         }
      }

      tastenstatusA |= tastencodeB;

      // Weichen

      weichenposition[GRUPPE_0] = 0;

      weichendata wC; // data in ringbuffer

      weichentastencodeC = mcp1.gpioReadPortA();
      weichenxor0 = weichentastencodeC ^ oldweichentastencodeC;
      if (weichenxor0) // neue Daten
      {
         weichenXORcounterA++;
         // Weiche A Position 0
         if ((weichentastencodeC & (1 << SET_A_A_BIT)) == 0) // 6   Taste 6 gedrueckt , weiche0
         {

            mcp1.gpioDigitalWrite(OUT_A_A, HIGH);   // GPA7
            weichenposition[GRUPPE_0] &= ~(1 << 0); // bit fuer weiche loeschen
            wC = {.weiche = 0, .richtung = 0};
            rb_push(&weichenringbuffer, wC);
         }

         if ((weichentastencodeC & (1 << SET_A_B_BIT)) == 0) //  3    Taste 5 gedrueckt weiche0
         {
            mcp1.gpioDigitalWrite(OUT_A_A, LOW);   //
            weichenposition[GRUPPE_0] |= (1 << 0); // bit fur weiche setzen
            wC = {.weiche = 0, .richtung = 1};
            rb_push(&weichenringbuffer, wC);
         }

         // Weiche B Position 1
         if ((weichentastencodeC & (1 << SET_A_C_BIT)) == 0) //  7    Taste 3 gedrueckt , weiche1
         {
            mcp1.gpioDigitalWrite(OUT_A_B, HIGH); // GPA7
            weichenposition[GRUPPE_0] &= ~(1 << 1);
            wC = {.weiche = 1, .richtung = 0};
            rb_push(&weichenringbuffer, wC);
         }

         if ((weichentastencodeC & (1 << SET_A_D_BIT)) == 0) // 2    Taste 2 gedrueckt weiche1
         {
            mcp1.gpioDigitalWrite(OUT_A_B, LOW); //
            weichenposition[GRUPPE_0] |= (1 << 1);
            wC = {.weiche = 1, .richtung = 1};
            rb_push(&weichenringbuffer, wC);
         }

         oldweichentastencodeC = weichentastencodeC;
      } // if(weichentastencodeD ^

      weichendata wD;                                   // data in ringbuffer
      weichentastencodeD = mcp1.gpioReadPortB() & 0xFE; // Blink exkl.
      weichenxor1 = weichentastencodeD ^ oldweichentastencodeD;
      if (weichenxor1) // neue Daten
      {
         weichenXORcounterB++;
         // Weiche C Position 2
         if ((weichentastencodeD & (1 << SET_B_A_BIT)) == 0) // Taste 6 gedrueckt
         {
            mcp1.gpioDigitalWrite(OUT_B_A, HIGH); // GPB7
            wD = {.weiche = 2, .richtung = 0};
            rb_push(&weichenringbuffer, wD);
         }

         if ((weichentastencodeD & (1 << SET_B_B_BIT)) == 0) // Taste 5 gedrueckt
         {
            mcp1.gpioDigitalWrite(OUT_B_A, LOW); //
            wD = {.weiche = 2, .richtung = 1};
            rb_push(&weichenringbuffer, wD);
         }

         // Weiche D Position 1
         if ((weichentastencodeD & (1 << SET_B_C_BIT)) == 0) // Taste 3 gedrueckt , weiche3
         {
            mcp1.gpioDigitalWrite(OUT_B_B, HIGH); // GPA7
            wD = {.weiche = 3, .richtung = 0};
            rb_push(&weichenringbuffer, wD);
         }

         if ((weichentastencodeD & (1 << SET_B_D_BIT)) == 0) // Taste 2 gedrueckt weiche3
         {
            mcp1.gpioDigitalWrite(OUT_B_B, LOW); //
            wD = {.weiche = 3, .richtung = 1};
            rb_push(&weichenringbuffer, wD);
         }

         oldweichentastencodeD = weichentastencodeD;
         // mcp1.gpioDigitalWrite(OUT_B_A, 1);

      } // if(weichentastencodeD ^
      else
      {
      }

      weichendata wE;                            // data in ringbuffer
      weichentastencodeE = mcp2.gpioReadPortA(); //& 0x7F;

      weichenxor2 = weichentastencodeE ^ oldweichentastencodeE;
      if (weichenxor2) // neue Daten
      {
         weichenXORcounterC++;
         // Weiche E Position 4
         if ((weichentastencodeE & (1 << SET_A_A_BIT)) == 0) // Taste 3 gedrueckt
         {
            mcp2.gpioDigitalWrite(OUT_A_A, HIGH);   //
            weichenposition[GRUPPE_0] &= ~(1 << 4); // bit fuer weiche loeschen

            wE = {.weiche = 4, .richtung = 0};
            rb_push(&weichenringbuffer, wE);
         }

         if ((weichentastencodeE & (1 << SET_A_B_BIT)) == 0) // Taste 2 gedrueckt
         {
            mcp2.gpioDigitalWrite(OUT_A_A, LOW);   //
            weichenposition[GRUPPE_0] |= (1 << 4); // bit fur weiche setzen

            wE = {.weiche = 4, .richtung = 1};
            rb_push(&weichenringbuffer, wE);
         }

         // Weiche F Position 5
         if ((weichentastencodeE & (1 << SET_A_C_BIT)) == 0) // Taste 3 gedrueckt , weiche1
         {
            mcp2.gpioDigitalWrite(OUT_A_B, HIGH);  // GPA7
            weichenposition[GRUPPE_0] |= (1 << 5); // bit fur weiche setzen

            wE = {.weiche = 5, .richtung = 0};
            rb_push(&weichenringbuffer, wE);
         }

         if ((weichentastencodeE & (1 << SET_A_D_BIT)) == 0) // Taste 2 gedrueckt weiche1
         {
            mcp2.gpioDigitalWrite(OUT_A_B, LOW);   //
            weichenposition[GRUPPE_0] |= (1 << 5); // bit fur weiche setzen

            wE = {.weiche = 5, .richtung = 1};
            rb_push(&weichenringbuffer, wE);
         }

         oldweichentastencodeE = weichentastencodeE;
      } // if(weichentastencodeDE ^

      weichendata wF;                            // data in ringbuffer
      weichentastencodeF = mcp2.gpioReadPortB(); //& 0xFE;
      weichenxor3 = weichentastencodeF ^ oldweichentastencodeF;
      if (weichenxor3) // neue Daten
      {
         weichenXORcounterD++;
         if ((weichentastencodeF & (1 << SET_B_A_BIT)) == 0) // Taste 6 gedrueckt
         {
            mcp2.gpioDigitalWrite(OUT_B_A, HIGH); //
            weichenposition[GRUPPE_0] &= ~(1 << 6);
            wF = {.weiche = 6, .richtung = 0};
            rb_push(&weichenringbuffer, wF);
         }

         if ((weichentastencodeF & (1 << SET_B_B_BIT)) == 0) // Taste 5 gedrueckt
         {
            mcp2.gpioDigitalWrite(OUT_B_A, LOW); //
            weichenposition[GRUPPE_0] |= (1 << 6);
            wF = {.weiche = 6, .richtung = 1};
            rb_push(&weichenringbuffer, wF);
         }
         // Weiche F Position 1
         if ((weichentastencodeF & (1 << SET_B_C_BIT)) == 0) // Taste 3 gedrueckt , weiche3
         {
            mcp2.gpioDigitalWrite(OUT_B_B, HIGH); // GPA7
            weichenposition[GRUPPE_0] &= ~(1 << 7);
            wF = {.weiche = 7, .richtung = 0};
            rb_push(&weichenringbuffer, wF);
         }

         if ((weichentastencodeF & (1 << SET_B_D_BIT)) == 0) // Taste 2 gedrueckt weiche3
         {
            mcp2.gpioDigitalWrite(OUT_B_B, LOW); //
            weichenposition[GRUPPE_0] |= (1 << 7);
            wF = {.weiche = 7, .richtung = 1};
            rb_push(&weichenringbuffer, wF);
         }

         oldweichentastencodeF = weichentastencodeF;
      } //

      // Pot auslesen

      for (uint8_t i = 0; i < ANZLOKALPOTS - 2; i++) // lokal 2 kanaele
      {
         if (i < 2) // teensy4, nur 4 can
         {
            localpotarray[i] = adc->analogRead(potpinarray[i]); // 8 bit
            sendbuffer[16 + i] = localpotarray[i];
         }
      }

      // Weichen Start

      // Weichen End

      OSZI_C_HI();

      looptask = IMPULSTASK;

      
      taskarray[ANZLOKS - 1][3] = HI; // OPEN entfernen, Adresse auf 2,2,2,2 stellen
      taskarray[ANZLOKS - 1][15] = HI;


   } // if (looptask == PAUSETASK )

   // #pragma mark EMITTER

   sinceemitter = 0;
   if (sinceemitter > 200)
   {
      sinceemitter = 0;

      //     emitter = adc->analogRead(CURR_PIN); // in stromtimerfunktion

      //// Serial.print(" data: \t");
      emittermittel = 0;
      for (uint8_t i = 0; i < 8; i++)
      {

         //       // Serial.print(emitterarray[i]);
         //       // Serial.print("\t");
         emittermittel += emitterarray[i];
      }

      //    // Serial.print("\t");
      emittermittel /= 8;
      //     // Serial.print("emittermittel: ");
      //     // Serial.print(emittermittel);
      // // Serial.print(byte(0));

      //     // Serial.print("\n");

      //  lcd.setCursor(4,0);
      if (emitter < 0xFF)
      {

         uint8_t anzeige = (emittermittel);
         anzeige = anzeige ^ 0xFF;
         if (anzeige < minanzeige)
         {
            minanzeige = anzeige;
         }
      }
      else
      {
         //  lcd.print("         ");
      }
      ////  lcd.setCursor(18,0);
      uint8_t pos = (emittermittel) / 10;

      ////  lcd.print("*");
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = emitter & 0x00FF;
      sendbuffer[13] = (emitter & 0xFF00) >> 8;

      /*
       uint16_t pot0 = readPot(A0);
       //  lcd.setCursor(12,0);
       if (pot0 < 10)
       {
       //  lcd.print("  ");
       }
       else if (pot0 < 100)
       {
       //  lcd.print(" ");
       }
       //  lcd.setCursor(12,0);
       //  lcd.print(pot0);
       */

   } // if sinceemitter
   // errcounter++;
   /*
   if((sinceweiche > 2) && (weichenstatus & (1<<WEICHESTART)))
   {
       loknummer = ANZLOKS - 1; // letztes Paket, Weichen
      //  address
      taskarray[loknummer][0] = weichenbuffer[0];
      taskarray[loknummer][1] = weichenbuffer[1];
      taskarray[loknummer][2] = weichenbuffer[2];
      taskarray[loknummer][3] = weichenbuffer[3];

      taskarray[loknummer][12] = weichenbuffer[0];
      taskarray[loknummer][13] = weichenbuffer[1];
      taskarray[loknummer][14] = weichenbuffer[2];
      taskarray[loknummer][15] = weichenbuffer[3];

   }
   */

   if (weichenstatus & (1 << WEICHESTART))
   {
      if (weichencounter < MAXWEICHENCOUNTER)
      {
         weichencounter++;

         if ((weichencounter > 0x20) ) //&& (weichenstatus & (1 << WEICHERUN)))
         {
           taskarray[ANZLOKS - 1][3] = HI; // OPEN entfernen, Adresse auf 2,2,2,2 stellen
           taskarray[ANZLOKS - 1][15] = HI;

            weichenstatus &= ~(1 << WEICHERUN);

         }
      }
      else if (weichencounter >= MAXWEICHENCOUNTER) // Pause
      {
         // weichencounter = 64;
         weichenstatus &= ~(1 << WEICHESTART);
      }
   }

   // errcounter++;
   /*
   if (rb_count(&weichenringbuffer))
         {
            weichendata w;

            int erfolg = rb_pop(&weichenringbuffer, &w);
            if (erfolg == 0)
            {
               
               lcd.setCursor(6, 2);
               lcd.print("+");
               lcd.setCursor(0, 2);
               lcd.print(w.weiche, HEX);
               lcd.setCursor(4, 2);
               lcd.print(w.richtung, HEX);
               
               uint8_t weichenadresse[4] = {2, 2, 2, 1};
               uint8_t weichenpos = ANZLOKS - 1;
               taskarray[weichenpos][0] = tritarray[weichenadresse[0]];
               taskarray[weichenpos][1] = tritarray[weichenadresse[1]];
               taskarray[weichenpos][2] = tritarray[weichenadresse[2]];
               taskarray[weichenpos][3] = tritarray[weichenadresse[3]]; // OPEN
               // rep
               taskarray[weichenpos][12] = taskarray[weichenpos][0];
               taskarray[weichenpos][13] = taskarray[weichenpos][1];
               taskarray[weichenpos][14] = taskarray[weichenpos][2];
               taskarray[weichenpos][15] = taskarray[weichenpos][3];

               uint8_t weichennummer = w.weiche;
               aktuelleweiche = w.weiche;
               aktuelleweiche |= (w.richtung<<4);

               if (!(weichenstatus & (1 << WEICHESTART)))
               {
                  weichenstatus |= (1 << WEICHESTART);
                  weichenstatus |= (1 << WEICHERUN);
                  weichencounter = 0;
                  sinceweiche = 0;

                  for (uint8_t i = 3; i != 255; i--) // i decrement 3..0
                  {
                     if (weichennummer & (1 << i))
                     {
                        speedarray[i] = HI;
                        taskarray[weichenpos][5 + i] = HI;
                        // lcd.print("1");
                        // speed_send |= (1<<i);
                     }
                     else
                     {
                        speedarray[i] = LO;
                        taskarray[weichenpos][5 + i] = LO;
                        // lcd.print("0");
                        // speed_send &= ~(1<<i);
                     }
                  }
                  taskarray[weichenpos][17] = taskarray[weichenpos][5];
                  taskarray[weichenpos][18] = taskarray[weichenpos][6];
                  taskarray[weichenpos][19] = taskarray[weichenpos][7];
                  taskarray[weichenpos][20] = taskarray[weichenpos][8];

                  // richtung
                  if (w.richtung == 1)
                  {
                     taskarray[weichenpos][4] = HI;  // Ablenkung
                     taskarray[weichenpos][16] = HI; // Ablenkung
                  }
                  else
                  {
                     taskarray[weichenpos][4] = LO;  // Gerade
                     taskarray[weichenpos][16] = LO; // Gerade
                  }
               }
            }
            else
            {
               lcd.setCursor(6, 2);
               lcd.print("*");
            }
         }
         else
         {
            
            lcd.setCursor(0, 2);
            lcd.print("  ");
            lcd.setCursor(4, 2);
            lcd.print("  ");
            lcd.setCursor(6, 2);
            lcd.print("-");
            
         } 
         // pop
   */
   // sinceblink = 0;
   if (sinceblink > 500)
   {
      // OSZI_C_TOGG();

      //  mcp2.gpioDigitalWrite(15,0); //
      if (sourcestatus & 0x01) // local
      {

         // errcounter++;

         lcd.setCursor(0, 1);
         lcdputint3(lokaladressearray[0]);
         lcd.setCursor(4, 1);
         lcdputint3(lokaladressearray[1]);
         lcd.setCursor(8, 1);
         lcdputint3(lokaladressearray[2]);
         lcd.setCursor(12, 1);
         lcdputint3(lokaladressearray[3]);

         // uint8_t doubleadress = checkDoubleAddress();

         lcd.setCursor(0, 3);
         lcd.print(tastencodeA, HEX);
         lcd.setCursor(3, 3);
         lcd.print(tastencodeB, HEX);

         lcd.setCursor(6, 3);
         lcd.print(weichentastencodeC, HEX);
         lcd.setCursor(9, 3);
         lcd.print(weichentastencodeD, HEX);

         // weichenXORcounterA
         lcd.setCursor(12, 3);
         lcd.print(weichenXORcounterA,HEX);
         lcd.setCursor(15, 3);
         lcd.print(weichenXORcounterB, HEX);

         // asciicounter++;
         // asciicounter &= 0x0F;

         /*
         lcd.setCursor(12, 3);
         lcd.print((weichentastencodeE), HEX);
         lcd.setCursor(15, 3);
         lcd.print((weichentastencodeF, HEX);
         */

         // mcp2.gpioDigitalWrite(15,1); //
         // lcd.print(weichentastenstatusD, HEX);
         // oldweichentastencodeD = weichentastencodeD;
         // mcp1.gpioDigitalWrite(7,0); //

         
         if (rb_count(&weichenringbuffer))
         {
            weichendata w;

            int erfolg = rb_pop(&weichenringbuffer, &w);
            if (erfolg == 0)
            {
               lcd.setCursor(6, 2);
               lcd.print("+");
               lcd.setCursor(0, 2);
               lcd.print(w.weiche, HEX);
               lcd.setCursor(4, 2);
               lcd.print(w.richtung, HEX);

               uint8_t weichenadresse[4] = {2, 2, 2, 1};
               uint8_t weichenpos = ANZLOKS - 1;
               taskarray[weichenpos][0] = tritarray[weichenadresse[0]];
               taskarray[weichenpos][1] = tritarray[weichenadresse[1]];
               taskarray[weichenpos][2] = tritarray[weichenadresse[2]];
               taskarray[weichenpos][3] = tritarray[weichenadresse[3]]; // OPEN
               // rep
               taskarray[weichenpos][12] = taskarray[weichenpos][0];
               taskarray[weichenpos][13] = taskarray[weichenpos][1];
               taskarray[weichenpos][14] = taskarray[weichenpos][2];
               taskarray[weichenpos][15] = taskarray[weichenpos][3];

               uint8_t weichennummer = w.weiche;

               if (!(weichenstatus & (1 << WEICHESTART)))
               {
                  weichenstatus |= (1 << WEICHESTART);
                  weichenstatus |= (1 << WEICHERUN);
                  weichencounter = 0;
                  sinceweiche = 0;

                  for (uint8_t i = 3; i != 255; i--) // i decrement 3..0
                  {
                     if (weichennummer & (1 << i))
                     {
                        speedarray[i] = HI;
                        taskarray[weichenpos][5 + i] = HI;
                        // lcd.print("1");
                        // speed_send |= (1<<i);
                     }
                     else
                     {
                        speedarray[i] = LO;
                        taskarray[weichenpos][5 + i] = LO;
                        // lcd.print("0");
                        // speed_send &= ~(1<<i);
                     }
                  }
                  taskarray[weichenpos][17] = taskarray[weichenpos][5];
                  taskarray[weichenpos][18] = taskarray[weichenpos][6];
                  taskarray[weichenpos][19] = taskarray[weichenpos][7];
                  taskarray[weichenpos][20] = taskarray[weichenpos][8];

                  // richtung
                  if (w.richtung == 1)
                  {
                     taskarray[weichenpos][4] = HI;  // Ablenkung
                     taskarray[weichenpos][16] = HI; // Ablenkung
                  }
                  else
                  {
                     taskarray[weichenpos][4] = LO;  // Gerade
                     taskarray[weichenpos][16] = LO; // Gerade
                  }
               }
            }
            else
            {
               lcd.setCursor(6, 2);
               lcd.print("*");
            }
         }
         else
         {
            lcd.setCursor(0, 2);
            lcd.print("  ");
            lcd.setCursor(4, 2);
            lcd.print("  ");
            lcd.setCursor(6, 2);
            lcd.print("-");
         } 
         
         // pop
         
         /*
         lcd.setCursor(16,3);
         lcd.print("   ");
         lcd.setCursor(14,3);
         lcd.print(doubleadress);
         */

         lcd.setCursor(0, 0);
         lcdputint3(localpotarray[0]);
         lcd.setCursor(4, 0);
         lcdputint3(localpotarray[1]);
         lcd.setCursor(8, 0);
         lcdputint3(localpotarray[2]); // von nRF pot 2
         lcd.setCursor(12, 0);
         lcdputint3(localpotarray[3]); // von nRF pot 3
         // errcounter++;
      }
      else if (sourcestatus & 0x02)
      {
         
      }
      lcd.setCursor(0, 2);
      lcdputint3(usbweichencounter);
      lcd.setCursor(4, 2);
      lcdputint3(usbablenkungcounter);
      lcd.setCursor(8, 2);
      lcdputint3(usbgeradecounter);

      /*
      lcd.setCursor(8, 2);
      lcdputint3(weichencounter);
      lcd.print(' ');
      lcdputint3(tastencounter);
      lcd.print(' ');
      lcdputint3(weichenstatus);
      */
      /*
      lcd.setCursor(0,1);
      lcdputint3(speedraw0);
      lcd.setCursor(4,1);
      lcdputint3(speedraw1);
      lcd.setCursor(8,1);
      lcdputint3(speedraw2);
      lcd.setCursor(12,1);
      lcdputint3(speedraw3);
      lcd.setCursor(16,1);
      lcdputint3(speedrawcounter);
      */

      sinceblink = 0;
      loopcounter++;

      if (loopcounter % 2 == 0)
      {
         // mcp1.gpioDigitalWrite(CHECK, 1); //

         mcp1.gpioDigitalWrite(BLINK, 1);
         // mcp1.gpioDigitalWrite(10, 1); //
      }
      else
      {
         // mcp1.gpioDigitalWrite(CHECK, 0); //

         mcp1.gpioDigitalWrite(BLINK, 0); //
                                          // mcp1.gpioDigitalWrite(10, 0); //
      }
      lcd.setCursor(19, 0);
      lcd.print(char('A' + asciicounter));
      asciicounter++;
      asciicounter &= 0x0F;
      data.A = asciicounter;

      /*
      char buf[4];
      uint8_t ack0 = ackData[0];
      if(ack0 < 10)
      {
         sprintf(buf, "%1d",ack0);
      }
      else if (ack0 < 100)
      {
         sprintf(buf, "0%2d",ack0);
      }
      else
      {
         sprintf(buf, "0%d",ack0);
      }
      //sprintf(buf, "0%1d",ackData[0]);
      */
      /*
      lcd.setCursor(0,2);
      lcdputint3(ackData[0]);
      lcd.setCursor(4,2);
      lcdputint3(ackData[1]);

      lcd.setCursor(8,2);
      lcdputint3(ackData[2]);
      lcd.setCursor(12,2);
      lcdputint3(ackData[3]);
      */
      /*
       uint8_t index = 3;
       lcd.setCursor(0,2);
       lcdputint3(taskarray[index][0]);
       lcd.setCursor(4,2);
       lcdputint3(taskarray[index][1]);

       lcd.setCursor(8,2);
       lcdputint3(taskarray[index][2]);
       lcd.setCursor(12,2);
       lcdputint3(taskarray[index][3]);
       */

      /*
      lcd.setCursor(0,3);
      lcd.print('R');
      lcd.print(radiocounter);
      lcd.setCursor(10,3);
      lcd.print('E');
      lcd.print(errcounter);
      */

      // lcd.print(' ');
      // lcd.print(diptastenadresseB);

      pinMode(LOOPLED, OUTPUT);
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));

      if (sourcestatus == 2)
      {
         lcd.setCursor(17, 3);
         lcd.print("USB");
         if (loknummer == 0)
         {
         }
      }
      else if (sourcestatus == 1)
      {
         lcd.setCursor(17, 3);
         lcd.print("loc");

         // lcd.setCursor(0,3);
      }

      // Kanal A

      // Kanal B:

      lcd.setCursor(16, 1);
      if (taskarray[1][4] == LO)
      {
         lcd.print("OF");
      }
      else if (taskarray[1][4] == HI)
      {
         lcd.print("ON");
      }

      if (speed > 15)
      {
         speed = 0;
      }
      // errcounter++;

   } // if sincblinkk 500

   // loknummerTRITarray[0] = 3;
   //  #pragma mark USB
   int n = 0;
   n = RawHID.recv(buffer, 10); //
   if (n > 0)
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //// Serial.print(F("Received packet, erstes byte: "));
      //// Serial.println((int)buffer[0]);
      //     for (int i=0; i<8; i++)
      {
         //       int b = buffer[0] & (1 << i);
         //       digitalWrite(i, b);
      }

      // ************************************
      usbtask = buffer[0]; // Auswahl lok
      // ************************************
      loknummer = buffer[20];

      sourcestatus = buffer[21];

      if((sourcestatus != oldsourcestatus))
      {
         //if(sourcestatus == 2)
         {

         
         usbablenkungcounter = 0;
         usbgeradecounter = 0;
         usbweichencounter = 0;
         }
         oldsourcestatus = sourcestatus;
      }
      

      /*
      //// Serial.println(" ");
      // Serial.print("******************  usbtask *** ");
      printHex8(usbtask);
      // Serial.printf("USB sourcestatus: %d loknummer: %d\n",sourcestatus,loknummer);
       */
      if (timerintervall != buffer[18])
      {

         paketTimer.update(buffer[18]);
         timerintervall = buffer[18];
         //   // Serial.print(timerintervall);
         EEPROM.update(0xA0, timerintervall);

         if (loopstatus & (1 << FIRSTRUN))
         {
            loopstatus &= ~(1 << FIRSTRUN);
         }
      }

      // // Serial.printf("USB sourcestatus 2: %d\n ",sourcestatus);
      // #pragma mark TASK
      if (sourcestatus & 0x02) // USB
      {
         usbadressearray[0] = buffer[8];
         usbadressearray[1] = buffer[9];
         usbadressearray[2] = buffer[10];
         usbadressearray[3] = buffer[11];

         speed_raw = buffer[17];

         switch (usbtask)
         {
         case 0xBF: // Weiche
         {

            loknummer = ANZLOKS - 1; // letztes Paket, Weichen
            //  address
            weichendata wC; // data in ringbuffer
            uint8_t rawrichtung = buffer[16];
            if(rawrichtung)
            {
               usbablenkungcounter++;
            }
            else
            {
               usbgeradecounter++;
            }

            wC = {.weiche = buffer[17], .richtung = rawrichtung};
            rb_push(&weichenringbuffer, wC);
            usbweichencounter++;
            break;


            taskarray[loknummer][0] = tritarray[buffer[8]];
            taskarray[loknummer][1] = tritarray[buffer[9]];
            taskarray[loknummer][2] = tritarray[buffer[10]];
            taskarray[loknummer][3] = tritarray[buffer[11]]; // OPEN

            // repetition address

            taskarray[loknummer][12] = taskarray[loknummer][0];
            taskarray[loknummer][13] = taskarray[loknummer][1];
            taskarray[loknummer][14] = taskarray[loknummer][2];
            taskarray[loknummer][15] = taskarray[loknummer][3];

            // weichenbuffer speichern
            weichenbuffer[0] = tritarray[buffer[8]];
            weichenbuffer[1] = tritarray[buffer[9]];
            weichenbuffer[2] = tritarray[buffer[10]];
            weichenbuffer[3] = tritarray[buffer[11]]; // OPEN

            tastencounter++;

            if (!(weichenstatus & (1 << WEICHESTART)))
            {
               weichenstatus |= (1 << WEICHESTART);
               weichencounter = 0;
            }

            speed_raw = buffer[17]; // Weiche 0..7, 3 bit
            speed = speed_raw;



            // uint8_t  speed_send = 0;

            for (uint8_t i = 3; i != 255; i--) // i decrement 3..0
            {
               if (speed & (1 << i))
               {
                  speedarray[i] = HI;
                  // taskarray[0][8-i] = HI;
                  taskarray[loknummer][5 + i] = HI;
                  // lcd.print("1");
                  // speed_send |= (1<<i);
               }
               else
               {
                  speedarray[i] = LO;
                  // taskarray[0][8-i] = LO;
                  taskarray[loknummer][5 + i] = LO;
                  // lcd.print("0");
                  // speed_send &= ~(1<<i);
               }
            }

            // rep speed

            taskarray[loknummer][17] = taskarray[loknummer][5];
            taskarray[loknummer][18] = taskarray[loknummer][6];
            taskarray[loknummer][19] = taskarray[loknummer][7];
            taskarray[loknummer][20] = taskarray[loknummer][8];

            // Funktion
            if (buffer[16] & 0x01)
            {
               taskarray[loknummer][4] = HI;
               taskarray[loknummer][16] = HI;
               //lcd.setCursor(16, 2);
               //lcd.print("ON ");
            }
            else
            {
               taskarray[loknummer][4] = LO;
               taskarray[loknummer][16] = LO;
               //lcd.setCursor(16, 2);
               //lcd.print("OFF");
            }
         }

         case 0xA0: // address
         {
            /*
             uint8_t eeprompos = 0;
             uint8_t eepromadressbyte = 0;

             eepromadressbyte = EEPROM.read(eeprompos);
             // Serial.print("\nadresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);

             eeprompos++;
             eepromadressbyte = EEPROM.read(eeprompos);
             // Serial.print("adresse: ");
             // Serial.print(eeprompos);uint8_t weichengruppe = 0;
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);

             eeprompos++;
             eepromadressbyte = EEPROM.read(eeprompos);
             // Serial.print("adresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);

             eeprompos++;
             eepromadressbyte = EEPROM.read(eeprompos);

             // Serial.print("adresse: ");
             // Serial.print(eeprompos);
             // Serial.print(" byte: ");
             // Serial.println(eepromadressbyte);

             */

            taskarray[loknummer][0] = tritarray[buffer[8]];
            taskarray[loknummer][1] = tritarray[buffer[9]];
            taskarray[loknummer][2] = tritarray[buffer[10]];
            taskarray[loknummer][3] = tritarray[buffer[11]];

            if (loknummer == 0)
            {
               loknummerTRITarray[0] = tritarray[buffer[8]];
               loknummerTRITarray[1] = tritarray[buffer[9]];
               loknummerTRITarray[2] = tritarray[buffer[10]];
               loknummerTRITarray[3] = tritarray[buffer[11]];
            }

            uint8_t eeprompos = 0;

            eeprompos = 0;
            eepromadressearray[loknummer][0] = tritarray[buffer[8]];
            //        EEPROM.update(eeprompos++,buffer[8]);
            // delay(10);
            eepromadressearray[loknummer][1] = tritarray[buffer[9]];
            //        EEPROM.update(eeprompos++,buffer[9]);
            // delay(10);
            eepromadressearray[loknummer][2] = tritarray[buffer[10]];
            //        EEPROM.update(eeprompos++,buffer[10]);
            // delay(10);
            eepromadressearray[loknummer][3] = tritarray[buffer[11]];
            //        EEPROM.update(eeprompos,buffer[11]);
            // delay(10);

            // repetition
            taskarray[loknummer][12] = taskarray[loknummer][0];
            taskarray[loknummer][13] = taskarray[loknummer][1];
            taskarray[loknummer][14] = taskarray[loknummer][2];
            taskarray[loknummer][15] = taskarray[loknummer][3];
            //// Serial.print(" usb adress: ");
            loknummer = 0;
            /*
            // Serial.print(eepromadressearray[loknummer][0],HEX);
            // Serial.print(" ");
            // Serial.print(eepromadressearray[loknummer][1],HEX);
            // Serial.print(" ");
            // Serial.print(eepromadressearray[loknummer][2],HEX);
            // Serial.print(" ");
            // Serial.print(eepromadressearray[loknummer][3],HEX);
            // Serial.print("\n");
            */
            // EEPROM.update(0,eepromadressearray);
         }
         break;

         case 0xB0: // speed

         {

            // Adresse mitgeben
            taskarray[loknummer][0] = tritarray[buffer[8]];
            taskarray[loknummer][1] = tritarray[buffer[9]];
            taskarray[loknummer][2] = tritarray[buffer[10]];
            taskarray[loknummer][3] = tritarray[buffer[11]];

            eepromadressearray[loknummer][0] = tritarray[buffer[8]];
            eepromadressearray[loknummer][1] = tritarray[buffer[9]];
            eepromadressearray[loknummer][2] = tritarray[buffer[10]];
            eepromadressearray[loknummer][3] = tritarray[buffer[11]];

            // repetition address
            taskarray[loknummer][12] = taskarray[loknummer][0];
            taskarray[loknummer][13] = taskarray[loknummer][1];
            taskarray[loknummer][14] = taskarray[loknummer][2];
            taskarray[loknummer][15] = taskarray[loknummer][3];

            //// Serial.print("usbtaskask 0xB0");
            //     taskarray[0][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0

            speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
            uint8_t speed_red = 0;
            //             // Serial.print("speed_raw 0: ");
            //             // Serial.println(speed_raw);

            if (speed_raw < 10)
            {
               //  lcd.print(" ");
            }
            else
            {
               // //  lcd.print("speed ");
            }
            //  lcd.print(speed_raw);

            if (speed_raw < 2) // stillstand oder Richtungswachsel
            {

               for (uint8_t i = 0; i < 4; i++)
               {
                  //// Serial.println("speed_raw 0: HALT");
                  //// Serial.println(speed_raw);

                  taskarray[loknummer][5 + i] = LO;
               }
               if (speed_raw == 1)
               {
                  //// Serial.println("speed_raw 0: WENDEN");
                  taskarray[loknummer][5] = HI; // richtungswechsel fuer speed = 1
               }
            }
            else
            {
               speed = speed_raw;

               //// Serial.print("speed 0: ");
               //// Serial.println(speed);
               for (uint8_t i = 0; i < 4; i++)
               {
                  //// Serial.print(" i: "); // Serial.print(i);
                  //// Serial.print(" data: ");// Serial.print(speed & (1<<i));
                  //// Serial.print("\n");
                  if (speed & (1 << i))
                  {
                     //// Serial.print("HI");
                     speedarray[i] = HI;
                     // taskarray[0][8-i] = HI;
                     taskarray[loknummer][5 + i] = HI;
                  }
                  else
                  {
                     //// Serial.print("LO");
                     speedarray[i] = LO;
                     // taskarray[0][8-i] = LO;
                     taskarray[loknummer][5 + i] = LO;
                  }
                  //// Serial.print("\n");
               }
            }

            for (int i = 5; i < 9; i++)
            {
               if (taskarray[loknummer][i] == 0xFEFE)
               {
                  //// Serial.print("1");
               }
               else
               {
                  //// Serial.print("0");
               }
               //// Serial.print(taskarray[loknummer][i]);
            }
            //// Serial.print("\n");

            // rep speed
            taskarray[loknummer][17] = taskarray[loknummer][5];
            taskarray[loknummer][18] = taskarray[loknummer][6];
            taskarray[loknummer][19] = taskarray[loknummer][7];
            taskarray[loknummer][20] = taskarray[loknummer][8];

            lcd.setCursor(0, 2);
            lcd.print("t");
            lcd.print(loknummer);
            lcd.print("w");
            lcd.print(buffer[17]);
         }
         break;

         case 0xC0: // Richtung
         {
            //  // Serial.print("Richtung b 17: ");
            //  // Serial.println(buffer[17]);

            // Adresse mitgeben
            taskarray[loknummer][0] = tritarray[buffer[8]];
            taskarray[loknummer][1] = tritarray[buffer[9]];
            taskarray[loknummer][2] = tritarray[buffer[10]];
            taskarray[loknummer][3] = tritarray[buffer[11]];

            eepromadressearray[loknummer][0] = tritarray[buffer[8]];
            eepromadressearray[loknummer][1] = tritarray[buffer[9]];
            eepromadressearray[loknummer][2] = tritarray[buffer[10]];
            eepromadressearray[loknummer][3] = tritarray[buffer[11]];

            // speed auf 0 setzen
            for (uint8_t i = 1; i < 4; i++)
            {
               // Serial.println("speed_raw 0: HALT");
               taskarray[loknummer][5 + i] = LO;
            }

            if (buffer[17] == 1) // Richtung Toggeln
            {
               taskarray[loknummer][5] = HI;
               //  lcd.setCursor(15,0);
               //  lcd.print("T");

               /*
                for (uint8_t i=1;i<4;i++)
                {
                // Serial.println("speed_raw 0: HALT");

                taskarray[loknummer][5+i] = LO;
                }
                */
            }

            else if (buffer[17] == 0)
            {
               taskarray[loknummer][5] = LO;
               //  lcd.setCursor(15,0);
               //  lcd.print(" ");
            }

            // repetition speed 0
            taskarray[loknummer][17] = taskarray[loknummer][5]; // auch richtung
            taskarray[loknummer][18] = taskarray[loknummer][6];
            taskarray[loknummer][19] = taskarray[loknummer][7];
            taskarray[loknummer][20] = taskarray[loknummer][8];

            // repetition address
            taskarray[loknummer][12] = taskarray[loknummer][0];
            taskarray[loknummer][13] = taskarray[loknummer][1];
            taskarray[loknummer][14] = taskarray[loknummer][2];
            taskarray[loknummer][15] = taskarray[loknummer][3];
         }
         break;

         case 0xD0: // Funktion
         {
            //// Serial.print("D0 Funktion b16: ");
            //// Serial.println(buffer[16]);

            // Adresse mitgeben
            taskarray[loknummer][0] = tritarray[buffer[8]];
            taskarray[loknummer][1] = tritarray[buffer[9]];
            taskarray[loknummer][2] = tritarray[buffer[10]];
            taskarray[loknummer][3] = tritarray[buffer[11]];

            // repetition address
            taskarray[loknummer][12] = taskarray[loknummer][0];
            taskarray[loknummer][13] = taskarray[loknummer][1];
            taskarray[loknummer][14] = taskarray[loknummer][2];
            taskarray[loknummer][15] = taskarray[loknummer][3];

            eepromadressearray[loknummer][0] = tritarray[buffer[8]];
            eepromadressearray[loknummer][1] = tritarray[buffer[9]];
            eepromadressearray[loknummer][2] = tritarray[buffer[10]];
            eepromadressearray[loknummer][3] = tritarray[buffer[11]];

            if (buffer[16] & 0x01)
            {
               //// Serial.println("D0 Funktion HI");
               taskarray[loknummer][4] = HI;
               taskarray[loknummer][16] = HI;
               //  lcd.setCursor(12,1);
               //  lcd.print("ON ");
            }
            else
            {
               //// Serial.println("D0 Funktion LO");
               taskarray[loknummer][4] = LO;
               taskarray[loknummer][16] = LO;
               //  lcd.setCursor(12,1);
               //  lcd.print("OFF");
            }
         }
         break;

         case 0xE0: // Pause
         {
            //// Serial.print("E0 Pause b19: ");
            //// Serial.println(buffer[19]);
            pause = buffer[19];
         }
         break;
            /*
            case 0xE1: // Timerintervall
            {
               // Serial.print("E1 timerintervall b19: ");
               // Serial.println(buffer[18]);
               timerintervall = buffer[18];


            }break;
            */

         case 0xA1: // Lok 1
         {
            taskarray[1][0] = tritarray[buffer[8]];
            taskarray[1][1] = tritarray[buffer[9]];
            taskarray[1][2] = tritarray[buffer[10]];
            taskarray[1][3] = tritarray[buffer[11]];

            eepromadressearray[1][0] = tritarray[buffer[8]];
            eepromadressearray[1][1] = tritarray[buffer[9]];
            eepromadressearray[1][2] = tritarray[buffer[10]];
            eepromadressearray[1][3] = tritarray[buffer[11]];

            //// Serial.println("\nLok 1: ");

            // repetition adresse
            taskarray[1][12] = taskarray[1][0];
            taskarray[1][13] = taskarray[1][1];
            taskarray[1][14] = taskarray[1][2];
            taskarray[1][15] = taskarray[1][3];

            //// Serial.print("Lok 1: ");
            //// Serial.println(buffer[16]);

            //   EEPROM.update(0,eepromadressearray);
         }
         break;

         case 0xB1: // speed 1
         {
            taskarray[1][0] = tritarray[buffer[8]];
            taskarray[1][1] = tritarray[buffer[9]];
            taskarray[1][2] = tritarray[buffer[10]];
            taskarray[1][3] = tritarray[buffer[11]];

            eepromadressearray[1][0] = tritarray[buffer[8]];
            eepromadressearray[1][1] = tritarray[buffer[9]];
            eepromadressearray[1][2] = tritarray[buffer[10]];
            eepromadressearray[1][3] = tritarray[buffer[11]];

            // repetition adresse
            taskarray[1][12] = taskarray[1][0];
            taskarray[1][13] = taskarray[1][1];
            taskarray[1][14] = taskarray[1][2];
            taskarray[1][15] = taskarray[1][3];
            //// Serial.print("usbtaskask 0xB0");
            //     taskarray[1][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0

            speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
            uint8_t speed_red = 0;
            //// Serial.print("speed_raw 0: ");
            //// Serial.println(speed_raw);
            //  lcd.setCursor(0,0);
            ////  lcd.print("Lok0");
            if (speed_raw < 10)
            {
               //  lcd.print(" ");
            }
            else
            {
               // //  lcd.print("speed ");
            }
            //  lcd.print(speed_raw);

            if (speed_raw < 2) // stillstand oder Richtungswachsel
            {

               for (uint8_t i = 0; i < 4; i++)
               {
                  //// Serial.println("speed_raw 1: HALT");
                  //// Serial.println(speed_raw);

                  taskarray[1][5 + i] = LO;
               }
               if (speed_raw == 1)
               {
                  //// Serial.println("speed_raw 0: WENDEN");
                  taskarray[1][5] = HI; // richtungswechsel fuer speed = 1
               }
            }
            else
            {
               speed = speed_raw;

               //   // Serial.print("speed 0: ");
               //   // Serial.println(speed);
               for (uint8_t i = 0; i < 4; i++)
               {
                  //// Serial.print(" i: "); // Serial.print(i);
                  //// Serial.print(" data: ");// Serial.print(speed & (1<<i));
                  // Serial.print("\n");
                  if (speed & (1 << i))
                  {
                     //// Serial.print("HI");
                     speedarray[i] = HI;
                     // taskarray[0][8-i] = HI;
                     taskarray[1][5 + i] = HI;
                  }
                  else
                  {
                     //// Serial.print("LO");
                     speedarray[i] = LO;
                     taskarray[1][5 + i] = LO;
                  }
                  // Serial.print("\n");
               }
            }

            //
            // Ausgabe bin
            /*
             for (int i=5; i<9; i++)
             {
             if (taskarray[1][i] == 0xFEFE)
             {
             // Serial.print("1");
             }
             else
             {
             // Serial.print("0");
             }
             //// Serial.print(taskarray[1][i]);

             }
             // Serial.print("\n");
             */
            // rep speed
            taskarray[1][17] = taskarray[1][5];
            taskarray[1][18] = taskarray[1][6];
            taskarray[1][19] = taskarray[1][7];
            taskarray[1][20] = taskarray[1][8];
            /*
            lcd.setCursor(8,2);
            lcd.print("t");
            lcd.print(loknummer);
            lcd.print("w");
            lcd.print(buffer[17]);
            */
         }
         break;

         case 0xC1: // richtung 1
         {
            //  // Serial.print("Richtung b 17: ");
            //  // Serial.println(buffer[17]);

            // Adresse mitgeben
            taskarray[1][0] = tritarray[buffer[8]];
            taskarray[1][1] = tritarray[buffer[9]];
            taskarray[1][2] = tritarray[buffer[10]];
            taskarray[1][3] = tritarray[buffer[11]];

            eepromadressearray[1][0] = tritarray[buffer[8]];
            eepromadressearray[1][1] = tritarray[buffer[9]];
            eepromadressearray[1][2] = tritarray[buffer[10]];
            eepromadressearray[1][3] = tritarray[buffer[11]];

            // repetition adresse
            taskarray[1][12] = taskarray[1][0];
            taskarray[1][13] = taskarray[1][1];
            taskarray[1][14] = taskarray[1][2];
            taskarray[1][15] = taskarray[1][3];

            // speed auf 0 setzen
            for (uint8_t i = 1; i < 4; i++)
            {
               // // Serial.println("speed_raw 0: HALT");

               taskarray[1][5 + i] = LO;
            }

            if (buffer[17] == 1) // Richtung Toggeln
            {
               taskarray[1][5] = HI;
               //  lcd.setCursor(15,0);
               //  lcd.print("T");

               /*
                for (uint8_t i=1;i<4;i++)
                {
                // Serial.println("speed_raw 0: HALT");

                taskarray[loknummer][5+i] = LO;
                }
                */
            }

            else if (buffer[17] == 0)
            {
               taskarray[1][5] = LO;
               //  lcd.setCursor(15,0);
               //  lcd.print(" ");
            }

            // repetition speed 0
            taskarray[1][17] = taskarray[1][5];
            taskarray[1][18] = taskarray[1][6];
            taskarray[1][19] = taskarray[1][7];
            taskarray[1][20] = taskarray[1][8];
         }
         break;

         case 0xD1:
         {
            taskarray[1][0] = tritarray[buffer[8]];
            taskarray[1][1] = tritarray[buffer[9]];
            taskarray[1][2] = tritarray[buffer[10]];
            taskarray[1][3] = tritarray[buffer[11]];

            eepromadressearray[1][0] = tritarray[buffer[8]];
            eepromadressearray[1][1] = tritarray[buffer[9]];
            eepromadressearray[1][2] = tritarray[buffer[10]];
            eepromadressearray[1][3] = tritarray[buffer[11]];

            // repetition adresse
            taskarray[1][12] = taskarray[1][0];
            taskarray[1][13] = taskarray[1][1];
            taskarray[1][14] = taskarray[1][2];
            taskarray[1][15] = taskarray[1][3];

            //// Serial.print("D1 Funktion b16: ");
            //// Serial.println(buffer[16]);

            if (buffer[16] & 0x01)
            {
               //   // Serial.println("D0 Funktion HI");
               taskarray[1][4] = HI;
               taskarray[1][16] = HI;
            }
            else
            {
               //   // Serial.println("D0 Funktion LO");
               taskarray[1][4] = LO;
               taskarray[1][16] = LO;
            }
         }
         break;

            // Lok 2
         case 0xA2: // address
         {

            taskarray[2][0] = tritarray[buffer[8]];
            taskarray[2][1] = tritarray[buffer[9]];
            taskarray[2][2] = tritarray[buffer[10]];
            taskarray[2][3] = tritarray[buffer[11]];

            eepromadressearray[2][0] = tritarray[buffer[8]];
            eepromadressearray[2][1] = tritarray[buffer[9]];
            eepromadressearray[2][2] = tritarray[buffer[10]];
            eepromadressearray[2][3] = tritarray[buffer[11]];

            //// Serial.println("\nLok 2: ");

            //// Serial.println(buffer[8]);
            //// Serial.println(buffer[9]);
            //// Serial.println(buffer[10]);
            //// Serial.println(buffer[11]);

            uint8_t eeprompos = 0x10;
            EEPROM.update(eeprompos++, buffer[8]);
            delay(10);
            EEPROM.update(eeprompos++, buffer[9]);
            delay(10);
            EEPROM.update(eeprompos++, buffer[10]);
            delay(10);
            EEPROM.update(eeprompos++, buffer[11]);
            delay(10);

            // repetition
            taskarray[2][12] = taskarray[2][0];
            taskarray[2][13] = taskarray[2][1];
            taskarray[2][14] = taskarray[2][2];
            taskarray[2][15] = taskarray[2][3];

            //          EEPROM.update(0,eepromadressearray);
         }
         break;

         case 0xB2: // speed
         {

            // Adresse mitgeben
            taskarray[2][0] = tritarray[buffer[8]];
            taskarray[2][1] = tritarray[buffer[9]];
            taskarray[2][2] = tritarray[buffer[10]];
            taskarray[2][3] = tritarray[buffer[11]];

            eepromadressearray[2][0] = tritarray[buffer[8]];
            eepromadressearray[2][1] = tritarray[buffer[9]];
            eepromadressearray[2][2] = tritarray[buffer[10]];
            eepromadressearray[2][3] = tritarray[buffer[11]];

            // repetition address
            taskarray[2][12] = taskarray[2][0];
            taskarray[2][13] = taskarray[2][1];
            taskarray[2][14] = taskarray[2][2];
            taskarray[2][15] = taskarray[2][3];

            //// Serial.print("usbtaskask 0xB0");
            //     taskarray[2][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0

            speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
            uint8_t speed_red = 0;
            //// Serial.print("speed_raw 0: ");
            //// Serial.println(speed_raw);

            if (speed_raw < 2) // stillstand oder Richtungswachsel
            {

               for (uint8_t i = 0; i < 4; i++)
               {
                  //// Serial.println("speed_raw 0: HALT");
                  //// Serial.println(speed_raw);

                  taskarray[2][5 + i] = LO;
               }
               if (speed_raw == 1)
               {
                  //// Serial.println("speed_raw 0: WENDEN");
                  taskarray[2][5] = HI; // richtungswechsel fuer speed = 1
               }
            }
            else
            {
               speed = speed_raw;

               //                // Serial.print("speed 0: ");
               //                // Serial.println(speed);
               for (uint8_t i = 0; i < 4; i++)
               {
                  //                   // Serial.print(" i: "); // Serial.print(i);
                  //                   // Serial.print(" data: ");// Serial.print(speed & (1<<i));
                  //                   // Serial.print("\n");
                  if (speed & (1 << i))
                  {
                     //                     // Serial.print("HI");
                     speedarray[i] = HI;
                     taskarray[2][5 + i] = HI;
                  }
                  else
                  {
                     //                   // Serial.print("LO");
                     speedarray[i] = LO;
                     taskarray[2][5 + i] = LO;
                  }
                  //                // Serial.print("\n");
               }
            }
            /*
             for (int i=5; i<9; i++)
             {
             if (taskarray[2][i] == 0xFEFE)
             {
             // Serial.print("1");
             }
             else
             {
             // Serial.print("0");
             }
             //// Serial.print(taskarray[2][i]);

             }
             // Serial.print("\n");
             */
            // rep speed
            taskarray[2][17] = taskarray[2][5];
            taskarray[2][18] = taskarray[2][6];
            taskarray[2][19] = taskarray[2][7];
            taskarray[2][20] = taskarray[2][8];
         }
         break;

         case 0xC2:
         {
            //  // Serial.print("Richtung b 17: ");
            //  // Serial.println(buffer[17]);

            // Adresse mitgeben
            taskarray[2][0] = tritarray[buffer[8]];
            taskarray[2][1] = tritarray[buffer[9]];
            taskarray[2][2] = tritarray[buffer[10]];
            taskarray[2][3] = tritarray[buffer[11]];

            eepromadressearray[2][0] = tritarray[buffer[8]];
            eepromadressearray[2][1] = tritarray[buffer[9]];
            eepromadressearray[2][2] = tritarray[buffer[10]];
            eepromadressearray[2][3] = tritarray[buffer[11]];

            // speed auf 0 setzen
            for (uint8_t i = 1; i < 4; i++)
            {
               // // Serial.println("speed_raw 0: HALT");

               taskarray[2][5 + i] = LO;
            }

            if (buffer[17] == 1) // Richtung Toggeln
            {
               taskarray[2][5] = HI;
               //  lcd.setCursor(15,0);
               //  lcd.print("T");

               /*
                for (uint8_t i=1;i<4;i++)
                {
                // Serial.println("speed_raw 0: HALT");

                taskarray[2][5+i] = LO;
                }
                */
            }

            else if (buffer[17] == 0)
            {
               taskarray[2][5] = LO;
               //  lcd.setCursor(15,0);
               //  lcd.print(" ");
            }

            // repetition speed 0
            taskarray[2][17] = taskarray[2][5];
            taskarray[2][18] = taskarray[2][6];
            taskarray[2][19] = taskarray[2][7];
            taskarray[2][20] = taskarray[2][8];

            // repetition address
            taskarray[2][12] = taskarray[2][0];
            taskarray[2][13] = taskarray[2][1];
            taskarray[2][14] = taskarray[2][2];
            taskarray[2][15] = taskarray[2][3];
         }
         break;

         case 0xD2:
         {
            //// Serial.print("D0 Funktion b16: ");
            //// Serial.println(buffer[16]);

            // Adresse mitgeben
            taskarray[2][0] = tritarray[buffer[8]];
            taskarray[2][1] = tritarray[buffer[9]];
            taskarray[2][2] = tritarray[buffer[10]];
            taskarray[2][3] = tritarray[buffer[11]];

            // repetition address
            taskarray[2][12] = taskarray[2][0];
            taskarray[2][13] = taskarray[2][1];
            taskarray[2][14] = taskarray[2][2];
            taskarray[2][15] = taskarray[2][3];

            eepromadressearray[2][0] = tritarray[buffer[8]];
            eepromadressearray[2][1] = tritarray[buffer[9]];
            eepromadressearray[2][2] = tritarray[buffer[10]];
            eepromadressearray[2][3] = tritarray[buffer[11]];

            if (buffer[16] & 0x01)
            {
               // Serial.println("D0 Funktion HI");
               taskarray[2][4] = HI;
               taskarray[2][16] = HI;
               //  lcd.setCursor(12,1);
               //  lcd.print("ON ");
            }
            else
            {
               // Serial.println("D0 Funktion LO");
               taskarray[2][4] = LO;
               taskarray[2][16] = LO;
               //  lcd.setCursor(12,1);
               //  lcd.print("OFF");
            }
         }
         break;

         case 0xE2: // Pause
         {
            //// Serial.print("E0 Pause b19: ");
            //// Serial.println(buffer[19]);
            pause = buffer[19];
         }
         break;

         } // switch

      } // if localstatus & 0x02
      //// Serial.println("USB END");

   } // n>0

   // #pragma mark local

   else if (sourcestatus & 0x01) // local
   {
      // errcounter++;
      // errcounter++;
      //  Weichen

      /*
      pin mode:
      Bank B
      7  : out w0
      5,6: in
      4  : out w1
      3,2: in
      1,0: aux, default out


      Bank A
      15    :out w2
      14,13 : in
      12    : out w3
      11,10 : in
      9,8   : aux, default out

      bin: 0110 1100 0110 1100
      hex: 6C6C

      */
      // Adresse setzen
      // weichengruppe 0: 0..7
      // weichengruppe 1: 8..15

      /*
      for (uint8_t weichengruppe = 0; weichengruppe < ANZWEICHENGRUPPEN; weichengruppe++)
      {
         loknummer = ANZLOKALLOKS - weichengruppe; // letztes Paket für 1 Gruppe, Weichen

         for (uint8_t i = 0; i < 4; i++)
         {
            taskarray[loknummer][i] = tritarray[weichenaddressarray[0][i]]; // 0..3

            // rep
            taskarray[loknummer][i + 12] = tritarray[weichenaddressarray[0][i]]; // 12..15
         }
         if (weichenposition[0] & (1 << 0))
         {
            taskarray[loknummer][4] = 1;  // Ablenkung
            taskarray[loknummer][16] = 1; // Ablenkung
         }
         else
         {
            taskarray[loknummer][4] = 0;  // gerade
            taskarray[loknummer][16] = 0; // gerade
         }
         taskarray[loknummer][5] = speedarray[0];
         taskarray[loknummer][6] = speedarray[1];
         taskarray[loknummer][7] = speedarray[2];
         taskarray[loknummer][8] = speedarray[3];

         // pause
         taskarray[loknummer][9] = 0;
         taskarray[loknummer][10] = 0;
         taskarray[loknummer][11] = 0;

         taskarray[loknummer][17] = speedarray[0];
         taskarray[loknummer][18] = speedarray[1];
         taskarray[loknummer][19] = speedarray[2];
         taskarray[loknummer][20] = speedarray[3];
      }
      */
      // end Weichen

      for (uint8_t localnum = 0; localnum < ANZLOKALLOKS - 1; localnum++) // Loks, ohne Weichen
      {

         loknummer = localnum;
         // loknummer = !loknummer;
         for (uint8_t i = 0; i < 4; i++)
         {
            if (lokaladressearray[localnum] & (1 << i))
            {
               taskarray[localnum][i] = tritarray[0]; // war vertauscht
            }
            else
            {
               taskarray[localnum][i] = tritarray[2];
            }
         }

         // repetition address
         taskarray[localnum][12] = taskarray[localnum][0];
         taskarray[localnum][13] = taskarray[localnum][1];
         taskarray[localnum][14] = taskarray[localnum][2];
         taskarray[localnum][15] = taskarray[localnum][3];

         // speed
         speed_raw = 0;
         if (localnum < ANZLOKALPOTS)
         {
            speed_raw = (localpotarray[localnum]) >> 4; // 0: halt 1: richtung 2-5: speed
         }

         if (speed_raw > 0)
         {
            speed_raw += 1; // speed 1 ist Richtungsumschaltung
         }
         if (speed_raw > 15)
         {
            speed_raw = 15;
         }

         /*
         if (richtungstatus & (1<<RICHTUNGSTART)) // Richtungswechsel im Gang
         {
            richtungcounter++;
            if (richtungcounter > 4)
            {
               richtungstatus &= ~(1<<RICHTUNGSTART); // Richtungswechsel beenden
            taskarray[localnum][5] = HI; // Richtungbit reset
            richtungcounter = 0;
            }
         }

         */

         if (speed_raw < 2) // stillstand oder Richtungswachsel
         {

            // speed auf 0 setzen
            for (uint8_t i = 0; i < 4; i++)
            {
               //// Serial.println("speed_raw 0: HALT");
               //// Serial.println(speed_raw);

               taskarray[localnum][5 + i] = LO;
            }

            // tritt nicht auf, 1 wird uebersprungen
            if (speed_raw == 1)
            {
               //// Serial.println("speed_raw 0: WENDEN");
               taskarray[localnum][5] = HI; // richtungswechsel fuer speed = 1
            }
         } // if speed_raw < 2
         else
         {
            // uint8_t speed_full = localpotarray[localnum] ; //8-bit Wert,
            speed = speed_raw;

            // speed setzen

            for (uint8_t i = 0; i < 4; i++)
            {

               if (speed & (1 << i))
               {
                  //                 // Serial.print("1");
                  speedarray[i] = HI;
                  taskarray[localnum][5 + i] = HI;
               }
               else
               {
                  //              // Serial.print("0");
                  speedarray[i] = LO;
                  taskarray[localnum][5 + i] = LO;
               }
               //           // Serial.print("\n");
            }
         } // speed_raw >= 2

         // rep speed
         /*
         taskarray[localnum][17] = taskarray[localnum][5];
         taskarray[localnum][18] = taskarray[localnum][6];
         taskarray[localnum][19] = taskarray[localnum][7];
         taskarray[localnum][20] = taskarray[localnum][8];
         */

         // Richtung

         if (lokalcodearray[localnum] & 0x02) // von debounce,  Richtungsimpuls, bit 1
         {

            if (!(richtungstatus & (1 << RICHTUNGSTART))) // start richtungtask
            {
               //// Serial.println("RICHTUNGSTART gesetzt");
               richtungstatus |= (1 << RICHTUNGSTART); // Beginn Richtungswechsel
               richtungcounter = 0;
            }
            else
            {
               // richtungcounter++;
            }

            // speed auf 0 setzen
            for (uint8_t i = 1; i < 4; i++)
            {
               //// Serial.println("speed_raw 0: HALT");
               // if(localnum < 2) // test: nur fuer 0,1
               {
                  taskarray[localnum][5 + i] = LO;
               }
            }

            taskarray[localnum][5] = HI; // Richtungbit set

            lokalstatus |= (1 << LOKALRICHTUNGBIT0);

            sincelocalrichtung = 0;
         }
         else // if ((lokalstatus & (1<<LOKALRICHTUNGBIT0)) && (sincelocalrichtung > 1000)) // nach 0.5s zuruecksetzen

         {
            if ((lokalstatus & (1 << LOKALRICHTUNGBIT0)) && (sincelocalrichtung > 4000)) // nach 0.5s zuruecksetzen
            {
               lokalstatus &= ~(1 << LOKALRICHTUNGBIT0);
               sincelocalrichtung = 0;
               taskarray[localnum][5] = LO;
               //           //  lcd.setCursor(6,0);
               //           //  lcd.print("W1");
            }
         }

         // repetition speed
         taskarray[localnum][17] = taskarray[localnum][5]; // auch richtung
         taskarray[localnum][18] = taskarray[localnum][6];
         taskarray[localnum][19] = taskarray[localnum][7];
         taskarray[localnum][20] = taskarray[localnum][8];

         // Funktion, bit 0
         if (lokalcodearray[localnum] & 0x01)
         {
            taskarray[localnum][4] = HI;
            taskarray[localnum][16] = HI; // rep
            //            //  lcd.setCursor(12,1);
            //            //  lcd.print("ON ");
         }
         else
         {
            taskarray[localnum][4] = LO;
            taskarray[localnum][16] = LO; // rep
            //           //  lcd.setCursor(12,1);
            //           //  lcd.print("OFF");
         }
      } // for localnum

      // exp
      /*
      {
         taskarray[2][5] = taskarray[0][5]; // auch richtung
         taskarray[2][6] = taskarray[0][6];
         taskarray[2][7] = taskarray[0][7];
         taskarray[2][8] = taskarray[0][8];

         taskarray[2][17] = taskarray[0][5]; // auch richtung
         taskarray[2][18] = taskarray[0][6];
         taskarray[2][19] = taskarray[0][7];
         taskarray[2][20] = taskarray[0][8];

      }

      {
         taskarray[3][5] = taskarray[1][5]; // auch richtung
         taskarray[3][6] = taskarray[1][6];
         taskarray[3][7] = taskarray[1][7];
         taskarray[3][8] = taskarray[1][8];

         taskarray[3][17] = taskarray[1][5]; // auch richtung
         taskarray[3][18] = taskarray[1][6];
         taskarray[3][19] = taskarray[1][7];
         taskarray[3][20] = taskarray[1][8];

      }
      */

   } // local

   // #pragma mark sincewegbuffer

   if ((sincewegbuffer > 1000) && (sourcestatus == 1)) // && (usbtask == SET_WEG)) // naechster Schritt
   {
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = emitter;

      n = RawHID.send(sendbuffer, 100);
      if (n > 0)
      {
         //        // Serial.print(F("Transmit packet "));
         //        // Serial.println(packetCount );
         packetCount = packetCount + 1;
      }
      else
      {
         // Serial.println(F("Unable to transmit packet"));
      }

      sincewegbuffer = 0;

      //     // Serial.print(" abschnittindex: ");
      //     // Serial.print(abschnittindex);

      //     // Serial.print(" aktuellepos index: ");
      //    // Serial.println(aktuellepos.index);

      // if (schrittecount == 0)
      if (usbtask == 0)
      {

         // **************
         // if ((schrittecount == 0)  && (!(usbtask == END_WEG)))
      }

      // **************

      // if ((schrittecount < anzschritte ) && (wegstatus & (1<<WEG_OK)))//&& ((abschnittindex+1) == aktuellepos.index))
   }

   // #pragma mark sinceringbuffer

   if ((sinceringbuffer > 32)) // && (usbtask == SET_RING)) // naechster Schritt
   {
      sinceringbuffer = 0;
      //     // Serial.print(" abschnittindex: ");
      //     // Serial.print(abschnittindex);

      //     // Serial.print(" aktuellepos index: ");
      //    // Serial.println(aktuellepos.index);

      // if (schrittecount == 0)
   }

   // every 4 seconds, send a packet to the computer

   if (msUntilNextSend > 4000)
   {
      msUntilNextSend = msUntilNextSend - 2000;
   }
} // loop
