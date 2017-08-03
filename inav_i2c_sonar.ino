/*
 * Decide on rangefinder type here. Only one type should be uncommented
 */
 #define USE_US100
 // #define USE_HCSR04

/*
 * Set I2C Slave address
 */
#define I2C_SLAVE_ADDRESS 0x14

// #define DEBUG;

#define TRIGGER_PIN 3
#define ECHO_PIN 4
#define LED_PIN 1

#define STATUS_OK 0
#define STATUS_OUT_OF_RANGE 1

#include <TinyWireS.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

/*
 * Configuration magic, do not touch
 */
#ifdef USE_US100
  #define PULSE_TO_CM 59 //Multiplier pulse length to distance in [cm]
  #define MEASUREMENT_PERIOD_RATIO 6 // measurement rate = MEASUREMENT_PERIOD_RATIO * 16, 96ms in this case
  #define MAX_RANGE 300 //Range of 4 meters
#endif

#ifdef USE_HCSR04
  #define PULSE_TO_CM 58 //Multiplier pulse length to distance in [cm]
  #define MEASUREMENT_PERIOD_RATIO 5 // measurement rate = MEASUREMENT_PERIOD_RATIO * 16, 80ms in this case
  #define MAX_RANGE 300 //Range of 4 meters
#endif

#define PULSE_TIMEOUT (MAX_RANGE * PULSE_TO_CM) //this is an equivalent of 4 meters range


volatile uint8_t i2c_regs[] =
{
    0, //status
    0, //older 8 of distance
    0, //younger 8 of distance
};

const byte reg_size = sizeof(i2c_regs);

/*
 * 0=16ms
 * 1=32ms
 * 2=64ms
 * 3=128ms
 * 4=250ms
 * 5=500ms
 * 6=1 sec,
 * 7=2 sec,
 * 8=4 sec,
 * 9= 8sec
 */
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

volatile uint8_t wakeCounter = 0;

/*
 * Watchdog Interrupt Service is executed when  watchdog timed out
 */
ISR(WDT_vect) {
  wakeCounter++;
}

volatile byte reg_position = 0;

/**
 * This function is executed when there is a request to read sensor
 * To get data, 2 reads of 8 bits are required
 * First requests send 8 older bits of 16bit unsigned int
 * Second request send 8 lower bytes
 * Measurement is executed when request for first batch of data is requested
 */
void requestEvent()
{  
  if (reg_position >= reg_size) {
    reg_position = 0;
  }
  
  TinyWireS.send(i2c_regs[reg_position]);

  reg_position++;
}

void receiveEvent(uint8_t howMany) {

    if (howMany < 1) {
        // Sanity-check
        return;
    }

    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();

    howMany--;

    if (!howMany) {
        // This write was only to set the buffer for next read
        return;
    }

    // Everything above 1 byte is something we do not care, so just get it from bus as send to /dev/null
    while(howMany--) {
      TinyWireS.receive();
    }
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  /*
   * Setup I2C
   */
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);
  TinyWireS.onReceive(receiveEvent);

  /*
   * Start watchdog timer
   */
  setup_watchdog(0);
}

long microsecondsToCentimeters(long microseconds){
//  return microseconds / PULSE_TO_CM;
  return (microseconds * 34 / 100 / 2) / 10;
}

void loop() {

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();
  sleep_disable(); 

  /*
   * Measurement is done every 6th wakeup, that gives more less 10Hz update rate (96ms)
   */
  if (wakeCounter == MEASUREMENT_PERIOD_RATIO) {
    
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);

    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);

    if (duration > 0) {
      i2c_regs[0] = STATUS_OK;
    } else {
      i2c_regs[0] = STATUS_OUT_OF_RANGE;
    }
    
    uint16_t cm = (uint16_t) microsecondsToCentimeters(duration);

#ifdef DEBUG
    if (cm <10) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
 #endif
    
    i2c_regs[1] = cm >> 8;
    i2c_regs[2] = cm & 0xFF;
    
    wakeCounter = 0;
  }
}
