//------------ Include libraries ------------------------ 

#include <Wire.h>
#include "Arduino.h"
#include "Sodaq_RN2483.h"
#include "Sodaq_UBlox_GPS.h"

//-------------- Define variables -------------------------

#define CONSOLE_SERIAL SerialUSB
#define loraSerial Serial1
#define ARRAY_DIM(arr)  (sizeof(arr) / sizeof(arr[0]))

#define ACCEL_ADR 0b0011110
#define TEST_REG 0x0F

#define TEST_INT1
#define TEST_INT2

#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN BAT_VOLT

#define COORD_MASK 0x3FF
#define DATA_LENGTH 3
#define BAT_MAX_VALUE 65535
#define BAT_MASK 0xF 

//---------------- GPS, Battery & LoRa part -----------------------------
//ABP
const uint8_t devAddr[4] = { 0x02,0x03,0x04,0x05 };
const uint8_t appSKey[16] = { 0x05,0xCB,0x23,0x18,0x61,0x95,0x99,0x2C,0x33,0x1D,0x29,0x8A,0x28,0x11,0xC6,0xB4 };
const uint8_t nwkSKey[16] = { 0x05,0x57,0x81,0xB6,0x70,0x8D,0xE4,0x5D,0xB0,0xC3,0x1B,0x61,0x09,0x59,0x55,0x8F };

//OTAA
uint8_t DevEUI[8] = { 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
uint8_t AppEUI[8] = { 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
uint8_t AppKey[16] = { 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
uint32_t intervals[] = {1UL * 60 * 10,}; size_t interval_ix = 0;

//Coords struct
struct rawCoords { double longitude; double latitude;}; 
struct coords { uint16_t x; uint16_t y; };
struct reference { double north=43.573069; double south=43.567577; double east=1.472837; double west=1.462962; }; reference fixref;
struct coords transformCoords(struct rawCoords rc) {
    CONSOLE_SERIAL.println(String(rc.longitude));
    CONSOLE_SERIAL.println(String(rc.latitude));
    CONSOLE_SERIAL.println(String(rc.longitude - fixref.west));
    CONSOLE_SERIAL.println(String(rc.latitude - fixref.south));
    CONSOLE_SERIAL.println(String(fixref.east - fixref.west));
    CONSOLE_SERIAL.println(String(fixref.north - fixref.south));
    CONSOLE_SERIAL.println(String(((rc.longitude - fixref.west) / (fixref.east - fixref.west))*1023));
    CONSOLE_SERIAL.println(String(((rc.latitude - fixref.south) / (fixref.north - fixref.south))*1023));
  return {
     x: (((rc.longitude - fixref.west) / (fixref.east - fixref.west))*1023),
     y: (((rc.latitude - fixref.south) / (fixref.north - fixref.south))*1023)
        };
};

struct rawCoords find_fix(uint32_t delay_until);

void setupLoRaABP(){  
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
  {CONSOLE_SERIAL.println("Communication to LoRaBEE successful.");}
  else
  {CONSOLE_SERIAL.println("Communication to LoRaBEE failed!");}
}
void setupLoRaOTAA(){
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, true))
  {CONSOLE_SERIAL.println("Communication to LoRaBEE successful.");}
  else
  {CONSOLE_SERIAL.println("OTAA Setup failed!");}
}
void setupLoRa(){
 //ABP
 setupLoRaABP();
 //OTAA
 //setupLoRaOTAA();
}
//---------------- Interrupt variables ------------------

volatile bool int1_flag = false;
volatile bool int2_flag = false;
int i = 0;

//---------------- LED functions --------------------------

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void OFF() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

//------------------- Set up function ---------------------------

void setup() 
{
  pinMode(ENABLE_PIN_IO, OUTPUT); // ONE
  digitalWrite(ENABLE_PIN_IO, HIGH); // ONE
  delay(5000);
  Wire.begin();

  //Setup LoRa
  CONSOLE_SERIAL.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  //Debug output from LoRaBee
  LoRaBee.setDiag(CONSOLE_SERIAL); 
  //connect to the LoRa Network
  setupLoRa();

  // Setup interrupts
  pinMode(ACCEL_INT1, INPUT_PULLUP);
  pinMode(ACCEL_INT2, INPUT_PULLUP);
  attachInterrupt(ACCEL_INT1, ISR1, FALLING);
  attachInterrupt(ACCEL_INT2, ISR2, FALLING);

  // Setup sleep mode
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  // Reboot 
  // CTRL0
  writeReg(0x1F, 0b10000000);

  // Set to 50z all axes active
  // CTRL1
  writeReg(0x20, 0b01010111);

  // Interrupt source 1
  // Set to be Y sensitive
  // IG_SRC1
  writeReg(0x30, 0b10001000); // Axes mask
  writeReg(0x32, 0b00111111); // Threshold
  writeReg(0x33, 0b00000000); // Duration

  // Interrupt source 2
  // Set to be X sensitive
  // IG_SRC2
  writeReg(0x34, 0b10000010); // Axes mask
  writeReg(0x36, 0b00111111); // Threshold
  writeReg(0x37, 0b00000000); // Duration

  // CTRL3 (INT1)
  // INT1 sources from IG_SRC1
#ifdef TEST_INT1
  writeReg(0x22, 0b00100000);
#else
  writeReg(0x22, 0b00000000);
#endif

  // CTRL4 (INT2)
  // INT2 sources from IG_SRC2
  // Note the bit positions are different
  // in CTRL3 and CTRL4 Datasheet p.38
#ifdef TEST_INT2
  writeReg(0x23, 0b00100000);
#else
  writeReg(0x23, 0b00000000);
#endif

  // Setup LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);


  //setup GPS
  CONSOLE_SERIAL.println("Enabling GPS...");
  sodaq_gps.init(GPS_ENABLE);
  find_fix(900L* 1000);
}

//------------------------ GPS, Battery & LoRa part-------------------------

// Function to send a LoRa packet
void sendPacket(String packet) {
  switch (LoRaBee.send(1, (uint8_t*)packet.c_str(), packet.length()))
    {
    case NoError:
      CONSOLE_SERIAL.println("Successful transmission.");
      break;
    case NoResponse:
      CONSOLE_SERIAL.println("There was no response from the device.");
      setupLoRa();
      break;
    case Timeout:
      CONSOLE_SERIAL.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      CONSOLE_SERIAL.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      CONSOLE_SERIAL.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case Busy:
      CONSOLE_SERIAL.println("The device is busy. Sleeping for 10 extra seconds.");
      delay(10000);
      break;
    case NetworkFatalError:
      CONSOLE_SERIAL.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NotConnected:
      CONSOLE_SERIAL.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NoAcknowledgment:
      CONSOLE_SERIAL.println("There was no acknowledgment sent back!");
      break;
    default:
      break;
    }
}

// To get the battery level
uint16_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));
    return voltage;
}

// To get GPS coordinates
/*Find a GPS fix, but first wait a while */
struct rawCoords find_fix(uint32_t delay_until)
{  rawCoords rc_from_gps;
   uint32_t start = millis();
   uint32_t timeout = 900L* 100;
   CONSOLE_SERIAL.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
   if (sodaq_gps.scan(false, timeout)) {
        CONSOLE_SERIAL.println(String(" time to find fix: ") + (millis() - start) + String("ms"));
        uint32_t delay_gps = millis() - start;
        CONSOLE_SERIAL.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
        CONSOLE_SERIAL.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        CONSOLE_SERIAL.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
        CONSOLE_SERIAL.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
        //Get raw coords from gps
        rc_from_gps= {(sodaq_gps.getLon()),(sodaq_gps.getLat())};
        //Change here to have different rate of transmissions
        //delay(10000 - delay_gps);
   } else {
       CONSOLE_SERIAL.println("No Fix");
   }
  return rc_from_gps;
}

//----------------------- Main Program --------------------------

void loop()
{
  GREEN(); // Indicates that the program is running
  delay(100);
  
  //-------------- GPS, Battery & LoRa part --------------------
  //10 bits x, 10 bits y, 4 bit battery
  uint8_t data[DATA_LENGTH];
  uint32_t wait_ms = intervals[interval_ix];
  if (++interval_ix > ARRAY_DIM(intervals)) { interval_ix = 0; }
  int batlevel=100;

  // Interrupt routines
  if (int1_flag) {
    int1_flag = false;
    CONSOLE_SERIAL.println("INT1 Interrupt");
  }
  if (int2_flag) {
    int2_flag = false;
    CONSOLE_SERIAL.println("INT2 Interrupt");
  }

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Set sleep mode to idle


  //-------------- GPS, Battery & LoRa part ---------------------
  //Measure Battery level
  batlevel = (int(getBatteryVoltage()) / BAT_MAX_VALUE);
 
  //Get GPS coordinates
  coords c_from_transform = transformCoords(find_fix(wait_ms));
  CONSOLE_SERIAL.println(c_from_transform.x);
  CONSOLE_SERIAL.println(c_from_transform.y);
  uint32_t tosend = (c_from_transform.x & COORD_MASK) | ((c_from_transform.y & COORD_MASK) << 10) | ((batlevel & BAT_MASK) << 20);

  // Create and send the LoRa packet
  CONSOLE_SERIAL.println("toSend "+String(tosend, HEX));
   
  memcpy(data, ((uint8_t*)&tosend), DATA_LENGTH);
  
  CONSOLE_SERIAL.println("0 = "+String(data[0]));
  CONSOLE_SERIAL.println("1 = "+String(data[1]));
  CONSOLE_SERIAL.println("2 = "+String(data[2]));
  //CONSOLE_SERIAL.println(String(tosend));
  String packet = (char*)data; 
  //String packet = (String)tosend;
  CONSOLE_SERIAL.println("packet = "+packet);
  BLUE(); // Indicates the start of the transmission of a packet
  sendPacket(packet);
  OFF(); // Indicates that the packet have been sent
  delay(1000);
  CONSOLE_SERIAL.println("envoyé...");
   
  RED(); // Indicates that the deivce is entering sleep mode
  delay(100);
  __WFI(); // Go to sleep
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADR, 0x01);
  
  uint8_t val = Wire.read();
  Wire.endTransmission();

  return val;
}

uint8_t writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

void ISR1()
{
  int1_flag = true;
}

void ISR2()
{
  int2_flag = true;
}
