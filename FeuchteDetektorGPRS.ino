// H-Sniffler
// FeuchteDetektorGPRS
//
// This program measures the humidity of the environment and sends an SMS to a defined phone number
// if a defined threshold is exceeded. The intervall between the measurements is abt. 6 h.
//
// V1 Zi  Initial version

// environmental definitions
#define PHONE_NUMBER "+492345678910" // place your dial number here
#define MEASINTERVALL 0.004 // h
#define HUMIDITYTHRESHOLD 75 // % rH
#define VOLTAGETHRESHOLD 4.4 // Supply Voltage
#define FIRSTTIME_SMS true


#include <string.h>
#include "DHT.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <GPRS_Shield_Arduino.h>



#define PIN_TX    2
#define PIN_RX    3
#define BAUDRATE  19200

#define DHTPIN 4     // what digital pin we're connected to
#define GPRSPOWERPIN 5 // switch by False to On
#define DHT5VPIN A0
#define VINMEASUREPIN A1
#define ALIVEPIN 13
volatile int count = 0;
volatile int waitCount = 0;
volatile float oldHumidity = 0;
volatile float newHumidity = 0;
volatile float oldVoltage = 0;
volatile float newVoltage = 5.0;
boolean firstMessage = FIRSTTIME_SMS;
const char myHumAlarmString[] = "Feuchte-Alarm:       %rH \nTemperatur:       *C";
const char myVoltAlarmString[] = "Spannungs-Alarm:       V";
unsigned char ADCSRASave = 0;
unsigned char ADCEnableMask = bit (ADEN);

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#define DEBUG_PRINTLN2(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN2(x, y)
#endif
// Uncomment whatever type you're using!
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


DHT dht(DHTPIN, DHTTYPE);
GPRS gprs(PIN_TX,PIN_RX,BAUDRATE);//RX,TX,BaudRate

ISR(WDT_vect)
  /* Watchdog immer Interrupt Service Routine */
  {
  DEBUG_PRINT("WDT awakened!");
  wdt_disable();
  }

void sendMySMS(const char* phonenumber, const char* message) {
  int repeatCnt = 0;
  boolean gprsFlag = false;
  // wake up GPRS
  digitalWrite(PIN_RX,HIGH);
  digitalWrite(PIN_TX,HIGH);
  digitalWrite(GPRSPOWERPIN, LOW); // switch module on
  wdt_reset();
  delay(5000); // time for connecting to mobile net
  wdt_reset();
  delay(5000);
  wdt_reset();
  while (!(gprsFlag = gprs.wake()) and repeatCnt < 5) {
    DEBUG_PRINT("Wake not successfull!");
    repeatCnt ++;
    wdt_reset();
    delay(100);
  }
  // now send SMS
  wdt_reset();
  if (gprsFlag) {
    gprs.checkPowerUp();
    repeatCnt = 0;
    while(!(gprsFlag = gprs.init()) and repeatCnt < 5) {
        delay(100);
        DEBUG_PRINT("Initialization failed!");
        wdt_reset();
        repeatCnt ++;
    }  
    wdt_reset();
    repeatCnt = 0;
    while(!(gprsFlag = gprs.isNetworkRegistered())  and repeatCnt < 5) {
      delay(100);
      DEBUG_PRINT("Network has not registered yet!");
      wdt_reset();
      repeatCnt ++;
    }
    if (gprsFlag) {
      DEBUG_PRINT("gprs initialize done!");
      DEBUG_PRINT("start to send message ...");
      wdt_reset();
      
      if(gprs.sendSMS(phonenumber,message)) { //define phone number and text
        DEBUG_PRINT("Send SMS Succeed!");
      }
      else {
        DEBUG_PRINT("Send SMS failed!");
      }
      
    }
    // set GPRS to sleep
    wdt_reset();
    delay(5000);
    wdt_reset();
  }
  digitalWrite(GPRSPOWERPIN, HIGH); // siwtch module off
  digitalWrite(PIN_TX,LOW);
  digitalWrite(PIN_RX,LOW);
}


void enter_sleep(void)
  /* Arduino schlafen legen */
  {
  DEBUG_PRINT("Enter Sleep Mode Arduino"); 
  // disable ADC
  DEBUG_PRINT(ADCSRA);
  DEBUG_PRINT(ADCSRA & ~(ADCEnableMask));
  ADCSRASave = ADCSRA;
  ADCSRA = ADCSRA & ~(ADCEnableMask); // disable ADC
  #ifdef DEBUG
  delay(2000);
  #endif
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();  
  // cancel sleep as a precaution
  sleep_disable();
  ADCSRA = ADCSRASave; // enable ADC again with old settings
  DEBUG_PRINT("Sleep mode recovered!");
  }

void setup() {
  char i;
  unsigned char onTime = 100;
  boolean ok;
  Serial.begin(19200);                 // the GPRS baud rate     Serial.begin(9600);
  Serial.println("GPRS H-Sniffler V1.0!");
  dht.begin();  
  pinMode(DHT5VPIN, OUTPUT);
  pinMode(GPRSPOWERPIN, OUTPUT);
  pinMode(ALIVEPIN, OUTPUT);
  digitalWrite(GPRSPOWERPIN, LOW); // switch GPRS Power On for Init
  digitalWrite(DHT5VPIN, HIGH);
  delay(5000); // to power up GPRS completely
  /* Setup des Watchdog Timers */
  //MCUSR &= ~(1<<WDRF);             /* WDT reset flag loeschen */
  //WDTCSR |= (1<<WDCE) | (1<<WDE);  /* WDCE setzen, Zugriff auf Presclaler etc. */
  //WDTCSR = 1<<WDP0 | 1<<WDP3;      /* Prescaler auf 8.0 s */
  //WDTCSR |= 1<<WDIE;               /* WDT Interrupt freigeben */
  waitCount = int(MEASINTERVALL * 3600 / 8); /* wait intervall in program runs */
  count = waitCount; /* to make a initial measurement */
  ok = gprs.init();
  if (!ok){
    onTime = 255;
  };
  ok = gprs.wake();
  if (!ok){
    onTime = 255;
  };
  digitalWrite(GPRSPOWERPIN, HIGH); //switch module off
  digitalWrite(PIN_TX,LOW);
  digitalWrite(PIN_RX,LOW);
  for (i = 0; i<3; 1)
  {
    digitalWrite(ALIVEPIN, HIGH);
    delay(onTime);
    digitalWrite(ALIVEPIN, LOW);
    delay(onTime);
    i++;
  }
  
}

void loop() {
  // loop of the H-Sniffler
  boolean dataValid;
  char S_humidity[6]; // must be 1 longer than the conversion with dtostrf()
  char S_temperature[6];
  char S_voltage[6];
  char *ptr;
  float supplyVoltage; 

  DEBUG_PRINT(count);
  if (count >= waitCount) {
    count = 0;
    // start measurement
    // read supply voltage first
    DEBUG_PRINT("Read supply voltage");
    supplyVoltage = float(unsigned(analogRead(VINMEASUREPIN)/1023.0 * 500) / 100.0); 
    DEBUG_PRINT(supplyVoltage);
    // now read DHT 22
    digitalWrite(DHT5VPIN, HIGH);
    delay(1000);
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    dataValid = true;
    wdt_reset();
    DEBUG_PRINT("DHT22 read!");
    float h = dht.readHumidity();
    delay(2000);
    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
          float h = dht.readHumidity();
          // Read temperature as Celsius (the default)
          wdt_reset();
          float t = dht.readTemperature();
      if (isnan(h) || isnan(t)) {
            DEBUG_PRINT("Failed to read temperature/humidity sensor!");
            dataValid = false;
      }
    }
    digitalWrite(DHT5VPIN, LOW);
    digitalWrite(DHTPIN,LOW);
    if (dataValid == true) {
      DEBUG_PRINT("DHT data valid");
      oldHumidity = newHumidity;
      newHumidity = h;
      dtostrf(h, 5, 1, S_humidity);
      dtostrf(t, 5, 1, S_temperature);
      DEBUG_PRINT(S_humidity);
      DEBUG_PRINT(S_temperature);
      if (((newHumidity > HUMIDITYTHRESHOLD) and (oldHumidity <= HUMIDITYTHRESHOLD))or (firstMessage == true)) {
        // threshold was exceeded after beeing below threshold before
        // send SMS
        firstMessage = false;
        DEBUG_PRINT("Send Humidity SMS!");
        ptr = strstr(myHumAlarmString, "%rH");
        if (ptr != NULL) {
          strncpy(ptr-6, S_humidity, 5);
        }
        ptr = strstr(myHumAlarmString, "*C");
        if (ptr != NULL) {
          strncpy(ptr-6, S_temperature, 5);
        }
        DEBUG_PRINT(myHumAlarmString);
        sendMySMS(PHONE_NUMBER, myHumAlarmString);
      }
    }
    // check voltage
    oldVoltage = newVoltage;
    newVoltage = supplyVoltage;
    if ((newVoltage < VOLTAGETHRESHOLD) and (oldVoltage >= VOLTAGETHRESHOLD)) {
      // send alert supply gets down
      DEBUG_PRINT("Send Voltage SMS!");
      dtostrf(supplyVoltage, 5, 1, S_voltage);
      ptr = strstr(myVoltAlarmString, "V");
      if (ptr != NULL) {
        DEBUG_PRINT(S_voltage);
        strncpy(ptr-6, S_voltage, 5);
      }
      DEBUG_PRINT(myVoltAlarmString);
      sendMySMS(PHONE_NUMBER, myVoltAlarmString);
    }
  }
else {
  count++; 
}
enter_sleep();
}


