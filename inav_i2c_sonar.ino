/*
 * Set I2C Slave address
 */
#define I2C_SLAVE_ADDRESS 0x14

#define MAX_READOUT_TIME 5 //in ms

#define TRIGGER_PIN 3
#define ECHO_PIN 4

#include <TinyWireS.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

volatile uint8_t i2c_regs[] =
{
    0, //older 8
    0 //younger 8
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

volatile unsigned long lastRequestMillis;
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

  if (millis() - lastRequestMillis > MAX_READOUT_TIME || reg_position >= reg_size) {
    reg_position = 0;
  }
  
  TinyWireS.send(i2c_regs[reg_position]);

  reg_position++;
  lastRequestMillis = millis();
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  /*
   * Setup I2C
   */
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);

  /*
   * Start watchdog timer
   */
  setup_watchdog(0);
}

long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
}

void loop() {

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();
  sleep_disable(); 

  /*
   * Measurement is done every 3rd wakeup, that gives more less 20Hz update rate (48ms)
   */
  if (wakeCounter == 3) {
    
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);

    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    uint16_t cm = (uint16_t) microsecondsToCentimeters(duration);
    
    i2c_regs[0] = cm >> 8;
    i2c_regs[1] = cm & 0xFF;
    
    wakeCounter = 0;
  }
}
