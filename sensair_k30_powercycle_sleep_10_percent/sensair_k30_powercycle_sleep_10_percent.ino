/*
 Author: Joseph M. Myre (joemyre@gmail.com)
 Modified 8/29/2016 by Matt Covington to add power cycling capability
 
 This program connects an Arduino to a Cozir CO2 ambient sensor 
 and reports readings back over the USB serial interface and 
 to an SD card. The SD card is attached to the arduino using the 
 Adafruint Data Logger Shield.
 
 Pin connections:
 Arduino________Vaisala Sensor
 a0  ------------ V_out
 
 The SoftwareSerial object used to initialize the Cozir 
 object is initialized with the Arduino communication pin
 definitions.
 
 */
#include <SPI.h>
#include <Wire.h>

#include <SD.h>
#include <SoftwareSerial.h>
#include "RTClib.h"
//#include "kSeries.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


// Defines the number of milliseconds between 
// grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  2400000 //= 40 min 
// ms between entries (reduce to take more/faster data)
#define MAX_SLEEP_ITERATIONS  LOG_INTERVAL / 8000 //watchdog timer will wait 8 sec for each wakeup

int sleepIterations = 0;
volatile bool watchdogActivated = false;



// The number of milliseconds before writing the logged data 
// permanently to disk.
// Set it to the LOG_INTERVAL to write each time (safest)
// Set it to 10*LOG_INTERVAL to write all data every 10 datareads, 
//   you could lose up to the last 10 reads if power is lost 
//   but it uses less power and is much faster!
//
// ms between calls to flush() -- to write data to the card
//#define SYNC_INTERVAL 30000
// Time of the last sync() call
//uint32_t syncTime = 0;
//new method to determine SD write (always at end of power on cycle)
boolean stay_on = true;

//time to keep sensor powered on to warm up
#define WARMUP_TIME 1200000 // = 20 min * 60 sec * 1000 msec
//(approximate) time between readings of sensor during warming up period
#define READ_INTERVAL 10000 // = 1 min

// Echo the data to the USB serial line
// 1 for true, 0 for false
#define ECHO_TO_SERIAL   1 

// Define the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

RTC_PCF8523 RTC; // define the Real Time Clock object//DS1307 old RTC

// Digital pin 10 is used for the SD cs line for the 
// data logging shield.
const int chipSelect = 10;

// Define relay connections
#define relay0 7

// Pins 12/13 are reserved for the SD card
SoftwareSerial K_30_Serial(8,9);  //Sets up a virtual serial port
                     //Using pin 8 for Rx and pin 9 for Tx

byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
float valMultiplier = 10;

// The logging file
File logfile;

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}


/**
 * Function for reporting errors.
 **/
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  Serial.flush();
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

/**
 * The setup function performs multiple tasks:
 * 1) Connects the USB serial line with the host
 * 2) Initializes the SD card
 * 3) Opens the log file for writing
 * 4) Connects to the RTC for timestamping
 **/
void setup(){
  Serial.begin(9600); //Start Serial connection with host
  K_30_Serial.begin(9600);
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  // set up relay
  pinMode(relay0, OUTPUT);
  digitalWrite(relay0, LOW);

#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  RTC.begin();
  
  if (! RTC.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  
  // This section grabs the current datetime and compares it to
  // the compilation time.  If necessary, the RTC is updated.
  DateTime dt = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (dt.unixtime() < compiled.unixtime()) {
    Serial.println("RTC is older than compile time! Updating");
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }


  // Initialize the SD card
  //Serial.print("Initializing SD card...");
  // Make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("SD Card failed, or not present");
  }
  Serial.println("SD card initialized.");
  
  // Create a new logging file
  // or open a previously existing file
  char filename[] = "salogr00.csv";
  for (uint8_t i = 0; i < 1000; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    Serial.print(filename);
    Serial.print(" exists: ");
    Serial.println(SD.exists(filename));
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  
  if (!logfile) {
    error("Failed to create logging file");
  }

  logfile.println("millis,stamp,datetime,CO2[ppm]");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,CO2[ppm]");
#endif //ECHO_TO_SERIAL

  Serial.println("Starting up myArdsalaLogger");
  //Start Serial connection with Sensor


  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
}


/**
 *
 **/
void loop(){

  if (watchdogActivated)
  {
  watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;

// Switch relay -- Turn it on
  digitalWrite(relay0, HIGH);
  //record time we are starting logging loop
  uint32_t loop_start_time = millis();
  
  while (stay_on)//(millis() - loop_start_time <WARMUP_TIME )  
  {
  DateTime dt;
  
  // This indicates that logging is occurring
  // It would be good to turn this off for field
  // deployment to save power
  digitalWrite(greenLEDpin, HIGH);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif


  // fetch the time
  dt = RTC.now();
  // log time
  logfile.print(dt.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  //logfile.print('"');
  logfile.print(dt.year(), DEC);
  logfile.print("/");
  logfile.print(dt.month(), DEC);
  logfile.print("/");
  logfile.print(dt.day(), DEC);
  logfile.print(" ");
  logfile.print(dt.hour(), DEC);
  logfile.print(":");
  logfile.print(dt.minute(), DEC);
  logfile.print(":");
  logfile.print(dt.second(), DEC);
  //logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(dt.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  //Serial.print('"');
  Serial.print(dt.year(), DEC);
  Serial.print("/");
  Serial.print(dt.month(), DEC);
  Serial.print("/");
  Serial.print(dt.day(), DEC);
  Serial.print(" ");
  Serial.print(dt.hour(), DEC);
  Serial.print(":");
  Serial.print(dt.minute(), DEC);
  Serial.print(":");
  Serial.print(dt.second(), DEC);
  //Serial.print('"');
#endif //ECHO_TO_SERIAL

  /***
  *
  * Read data from sensor and apply multiplier to get CO2 ppm
  *
  ***/
  sendRequest(readCO2);
  unsigned long valCO2 = getValue(response);
  if (millis() - loop_start_time > WARMUP_TIME)
  {
    stay_on = false;
  }
  report(valCO2); 

  digitalWrite(greenLEDpin, LOW);
  delay(READ_INTERVAL);

  }

// Switch relay -- Turn it off  
  digitalWrite(relay0, LOW);
  stay_on = true;
  // delay for the amount of time we want between readings
  //delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));  
    }
  
  }
  //go to sleep
  sleep();
}

void report(unsigned long co2){

  logfile.print(",");
  logfile.println(co2);
#if ECHO_TO_SERIAL
  Serial.print(",");
  Serial.println(co2);
  Serial.flush();
#endif
  
  // Now we write data to disk! 
  // Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  // The sync interval should be tuned to your field needs 
  if (stay_on==false){//((millis() - syncTime) < SYNC_INTERVAL) return;
//  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  }
}

void sendRequest(byte packet[])
{
  //don't need delay below if we are powering on for long warm-up, though first reading will be 0
  //delay(5000);//K 30 needs a few seconds to respond to power up
  while(!K_30_Serial.available())  //keep sending request until we start to get a response
  {
    K_30_Serial.write(readCO2,7);
    delay(50);
  }
  
  int timeout=0;  //set a timeout counter
  while(K_30_Serial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;  
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(K_30_Serial.available())  //flush whatever we have
          K_30_Serial.read();
          
          break;                        //exit and try again
      }
      delay(50);
  }
  
  Serial.println("");
  for (int i=0; i < 7; i++)
  {
    response[i] = K_30_Serial.read();
    
    Serial.print(response[i]);
    Serial.print(",");
  }  
  Serial.println("");
}

unsigned long getValue(byte packet[])
{
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];    //low byte for value is 5th byte in the packet

 //   Serial.print("Raw high: ");
 //   Serial.print(high);
 //   Serial.print("    Raw low: ");
 //   Serial.println(low);
  
    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
 //   Serial.print("Raw val: ");
 //   Serial.println(val);
    return val*valMultiplier;
}


