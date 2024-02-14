/*
  Copyright (c) 2021 Historic Electric Ltd
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.


  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.cur
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <Watchdog_t4.h>
#include <ADC.h>
#include <ADC_util.h>
#include <EEPROM.h>
#include <Filters.h>
#include <Teensy_PWM.h>
// CAN Setup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

#define NUM_TX_MAILBOXES 15
#define NUM_RX_MAILBOXES 15

CAN_message_t msg;

/* bms status values
Boot 0
Ready 1
Drive 2
Charge 3
Precharge 4
Error 5
*/

signed long loopTime = 0;

void canRX_289(const CAN_message_t &msg); // Inverter RPM, Battery and Torque
void canRX_299(const CAN_message_t &msg); // Inverter Temps
void canRX_351(const CAN_message_t &msg); // BMS Status
void canRX_001(const CAN_message_t &msg); // BMS Status *
void canRX_002(const CAN_message_t &msg); // BMS Status *
void canRX_003(const CAN_message_t &msg); // BMS Status *
void canRX_004(const CAN_message_t &msg); // BMS Status *
void canRX_005(const CAN_message_t &msg); // BMS Status *
void canRX_006(const CAN_message_t &msg); // BMS Status * 
void canRX_007(const CAN_message_t &msg); // BMS Status *
void canRX_356(const CAN_message_t &msg); // BMS HV Voltage
void canRX_377(const CAN_message_t &msg); // Outlander Charger Low voltage stats
void canRX_38A(const CAN_message_t &msg); // Outlander Charger EVSE
void canRX_389(const CAN_message_t &msg); // Outlander Charger HV stats
void canRX_398(const CAN_message_t &msg); // Outlander Heater stats
void canRX_732(const CAN_message_t &msg); // Inverter Current
void canRX_733(const CAN_message_t &msg); // Inverter Temps
void dashComms();                         // update the dash-board
void bmsComms();                          // Comms to the BMS - Set Key on

void dogFood();
void BMS_Stat(); //*
void BMS_Read(); //*
void menu();
void readPins();
void readPedal();
void inverterComms();
void EvseStart(); //*
void tempCheck();
void Speedo();
void showInfo();
void loadDefault();
void saveVarsToEEPROM();
void stateHandler();
void HeaterComms();

// Timer for BMS heartbeat
IntervalTimer BMStimer;
IntervalTimer BMSread;
// Metro Timers

Metro timer50_1 = Metro(50);     // inverter timer
Metro timer50_2 = Metro(50);     // De-bounce check
Metro timer100_1 = Metro(100);   // 2nd inverter timer
Metro timer100_2 = Metro(96);    // longer Debounce
Metro timer100_3 = Metro(110);   // Temp handler
Metro timer100_4 = Metro(100);   // Speedo handler
Metro timer500_1 = Metro(50);    // pedal debug timer
Metro timer500_2 = Metro(500);    // Charger timer
Metro timer5_1 = Metro(5);    // BMS_Stat *
Metro timer1000_3 = Metro(3000);    // BMS_Stat *
Metro timer1000_2 = Metro(1000);    // BMS_Stat *
Metro timer1000_1 = Metro(1000); // General 1s timer
Metro timer2000_1 = Metro(2000); // Serial update timer
Metro timer30s_1 = Metro(30000); // 30Sec Timer to check DC-DC State
Metro timer30_1 = Metro(30);  // Heartbeat timer 
Metro timer10_1 = Metro(10);     // Dash coms timer - needs to be fast for stepper motor
Metro timer20_1 = Metro(20);     // Dash coms timer - needs to be fast for stepper motor *

// Watchdog Setup
WDT_T4<WDT1> wdt;

void wdtCallback()
{
  Serial.println("FEED THE DOG SOON, OR RESET!");
}

// ADC setup

ADC *adc = new ADC();

// Filter setup
float filterFrequency = 5.0;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

// Define Outputs
#define OUT1 6    // Ignition is started
#define OUT2 9    // PRE Charge Contactor
#define OUT3 10   // HV Drive Contactor
#define OUT4 11   // Brake Lights
#define OUT5 12   // Heater Pump
#define OUT6 24   // FAN control
#define OUT7 25   // RPM signal
#define OUT8 28   // DC -DC Enable
#define OUT9 29   // Temp Gauge signal
#define OUT10 33  // Oil LED
#define OUT11 36  // Voltage LED
#define OUT12 37  // Reverse Lights
#define LEDpin 13 // Builtin LED

// Define Inputs
#define ISO_IN1 2  // FWD
#define ISO_IN2 3  // START
#define ISO_IN3 4  // BRAKE
#define ISO_IN4 5  // REV
#define ISO_IN5 26 // HEATER *
#define ISO_IN6 27 // MAP 3 SPORT
#define ISO_IN7 32 // IGNITION
#define ISO_IN8 21 // PP DETECT

#define POT_A 41 // POT A
#define POT_B 39 // POT B

// VCU Staus

#define boot 1
#define ready 2
#define driveNeutral 3
#define driveForward 4
#define driveReverse 5
#define charging 6
#define error 7

uint8_t VCUstatusChangeCounter = 0;    // used for hysterisys to stop jitter on VCU status
uint8_t VCUstatusChangeThreshold = 60; // n loops

uint8_t VCUstatus = 1;

// Setup Variables

uint8_t brake_pedal; // Brake lights
int flag=0; //*
int flag1=0; //*

uint8_t start = 1;
uint8_t ppDetect = 0;    // Prox pilot pin
uint8_t ignition = 1;    // ignition
uint8_t dir_FWD = 1;     // Drive Switch is set to forward
uint8_t dir_REV = 1;     // DRIVE Sitch is to Reverse
uint8_t dir_NEUTRAL = 0; // Set Neutral as the default state
uint8_t BMS_Status = 1;  // BMS Status
uint8_t Heater_pin = 1; //heater pin active
uint8_t BMS_SOC;
uint8_t inverterFunction = 0x00;
uint8_t BMS_keyOn = 0;

float BMS_avgvolt;
float BMS_avgtmp; // BMS Battery AVG Temp
float currentact; // BMS Current
float BMS_packvoltage;
float BMS_Volt[84];
float BMS_nom;
float BMS_max;
float BMS_min;

int BMS_discurrent;

unsigned long pretimer1 = 0;
unsigned long pretimer2 = 0;
unsigned long preChargeRiseTime = 0;

int chargerHVbattryVolts = 0;
int chargerACvolts = 0;
float charger12vbattryVolts = 0;
float charger12vCurrent = 0;

int Heatertemp = 0;
int Heatertemp1 = 0;
int Heatertemp2 = 0;
int chargerTemp1 = 0;
int chargerTemp2 = 0;
int chargerTemp3 = 0;
int chargerTempCH = 0;
int chargerHVcurrent = 0;
int chargercurrent = 0;
uint8_t chargerStatus;
int DCDCTemp = 0;
bool chargerHV1 = false; //charger voltage threshold 1
bool chargerHV2 = false;  //charger voltage threshold 2

int motorRPM = 0;
int motorTempPeak = 0;
int motorTemp1 = 0;
int motorTemp2 = 0;
int motorHVbatteryVolts = 0;
int motorTorque = 0;
int motorCurrent1 = 0;
int motorCurrent2 = 0;
int avgMotorTemp = 0;
int selectdir = -1; // Select direction of motor rotation
int inverterTemp1 = 0;
int inverterTemp2 = 0;
int avgInverterTemp = 0;
const int HVprecharge = 300; //precharge voltage

byte torqueHibyte = 0;
byte torqueLoByte = 0;
int torqueRequest = 0;
int targetTorque = 0;
int curentTorque = 0;
int throttlePosition = 0;
uint8_t pedalDebug = 0;
uint8_t inverterEnable = 1;
int regenTarget = 0;       // target regen torque
uint32_t regenTimer = 0;   // timer for regen delay
uint32_t regenDelay = 250; // delay before regen Starts
uint8_t regenState = 0;    // Are we requesting regen
uint32_t brakeDelay = 0;
int regenRequest = 0;

int incomingByte;
uint8_t menuLoad = 0;
uint8_t showStats = 1;
uint8_t pumpState = 0;
uint8_t fanState = 0;
// EEPROM Stored Vars

uint8_t tempGaugeMin = EEPROM.read(1); // Temp Gauge min PWM  
uint8_t tempGaugeMax = EEPROM.read(2); // Temp Gauge Max PWM
uint8_t pumpOnTemp = EEPROM.read(3);
uint8_t fanOnTemp = EEPROM.read(6);
uint8_t minTorque = EEPROM.read(8);   // Used for creeping feature to be added
uint16_t chargerstop = EEPROM.read(10) * 255 + EEPROM.read(9); // Maximal charger voltage
uint8_t torqueIncrement = EEPROM.read(12);                       // used for ramping up torque request
uint16_t maxTorque = EEPROM.read(31) * 255 + EEPROM.read(30);     // not used currently
                           
uint16_t tpslowOffset = EEPROM.read(21) * 255 + EEPROM.read(20);  // Value when foot off pedal
uint16_t tpshighOffset = EEPROM.read(23) * 255 + EEPROM.read(22); // Value when foot on pedal

uint8_t setTpsLow = 0;
uint8_t setTpsHigh = 0;

uint8_t active_map = 3; // Active Pedal map
uint8_t map3;           // Sport MAp

// Define constants
// speed = wheelsRPM * tireD * PI * 60 / 1000 Km/h
const float tireD = 0.596; //15in wheels
const float ratio = 7.065;
float calcspeed = 0;
float calcHz = 0;
float calcHzi = 0;

// Setup the peddal map arrays..

byte idx_j, idx_k; // index of tps,rpm bins
int pedal_offset;

const int num_rpm_bins = 21;
const int tpsbins[21] = {0, 3, 5, 8, 10, 13, 18, 20, 23, 25, 28, 32, 34, 40, 50, 60, 70, 80, 90, 100, 101};

const int rpmbins[num_rpm_bins] = {
    250, 500, 625, 750, 1000, 1250, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 10000};

const int pedal_map_one[21][22] = {
    // Normal
    // map 1..
    /*250*/ {0, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*500*/ {-10, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*625*/ {-20, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*750*/ {-30, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*1000*/ {-50, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*1250*/ {-70, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*1500*/ {-90, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*2000*/ {-110, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*2500*/ {-130, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*3000*/ {-150, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*3500*/ {-150, 0, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 8, 10},
    /*4000*/ {-150, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*4500*/ {-150, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*5000*/ {-160, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*5500*/ {-180, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*6000*/ {-200, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*6500*/ {-200, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*7000*/ {-225, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*7500*/ {-250, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*8000*/ {-300, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
    /*10000*/ {-300, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 8, 10},
};

const int pedal_map_two[21][22] = {
    // ECO
    // map 2..
    /*250*/ {0, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*500*/ {-10, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*625*/ {-20, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*750*/ {-30, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1000*/ {-50, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1250*/ {-70, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*1500*/ {-90, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*2000*/ {-110, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*2500*/ {-130, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*3000*/ {-150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*3500*/ {-150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*4000*/ {-150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*4500*/ {-150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*5000*/ {-160, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*5500*/ {-180, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*6000*/ {-200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*6500*/ {-200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*7000*/ {-225, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*7500*/ {-250, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*8000*/ {-300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
    /*10000*/ {-300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
};

const int pedal_map_three[21][22] = {
    // Sport
    // map 3..
    /*250*/ {0, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*500*/ {-10, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*625*/ {-20, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*750*/ {-30, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*1000*/ {-50, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*1250*/ {-70, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*1500*/ {-90, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*2000*/ {-110, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*2500*/ {-130, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*3000*/ {-150, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*3500*/ {-150, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 9, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*4000*/ {-150, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 9, 10, 10, 10, 10, 12, 12, 14, 14},
    /*4500*/ {-150, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 14, 14},
    /*5000*/ {-160, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 14},
    /*5500*/ {-180, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*6000*/ {-200, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*6500*/ {-200, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*7000*/ {-225, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*7500*/ {-250, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*8000*/ {-300, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
    /*10000*/ {-300, 0, 3, 4, 5, 5, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 12, 12, 12, 12},
};

void setup()
{

  // Setup Can

  Can1.begin();
  Can2.begin();
  Can3.begin();
  Can1.setBaudRate(250000);
  Can2.setBaudRate(500000);
  Can3.setBaudRate(500000);

  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }
  Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can2.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }
  Can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can3.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }

  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, TX, STD); // *
    Can2.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    Can3.setMB((FLEXCAN_MAILBOX)i, TX, STD);
  }

  Can1.setMBFilter(REJECT_ALL);
  Can1.enableMBInterrupts();
  Can2.setMBFilter(REJECT_ALL);
  Can2.enableMBInterrupts();
  Can3.setMBFilter(REJECT_ALL);
  Can3.enableMBInterrupts();

  Can2.onReceive(MB0, canRX_377); // Charger LV Stats
  Can2.onReceive(MB1, canRX_38A); // Charger LV Stats
  Can2.onReceive(MB2, canRX_389); // Charger HV Stats
  Can2.onReceive(MB3, canRX_398); // Heater Stats  
  Can2.onReceive(MB4, canRX_289); // Inverter RPM Battery and Torque
  Can2.onReceive(MB5, canRX_299); // Inverter Temps
  Can2.onReceive(MB6, canRX_732); // Inverter current
  Can2.onReceive(MB7, canRX_733); // Inverter Temps
  
 Can1.onReceive(MB11, canRX_001); // BMS Status
/* Can1.onReceive(MB12, canRX_002); // BMS Status
 Can1.onReceive(MB13, canRX_003); // BMS Status
 Can1.onReceive(MB14, canRX_004); // BMS Status
 Can1.onReceive(MB12, canRX_005); // BMS Status
 Can1.onReceive(MB13, canRX_006); // BMS Status
 Can1.onReceive(MB14, canRX_007); // BMS Status
*/

  Can2.setMBFilter(MB0, 0x377);
  Can2.setMBFilter(MB1, 0x38A);
  Can2.setMBFilter(MB2, 0x389);
  Can2.setMBFilter(MB3, 0x398);  
  Can2.setMBFilter(MB4, 0x289);
  Can2.setMBFilter(MB5, 0x299);
  Can2.setMBFilter(MB6, 0x732);
  Can2.setMBFilter(MB7, 0x733);
  Can1.setMBFilterRange(MB11, 0x01,0x07);

  Can1.mailboxStatus();
  Can2.mailboxStatus();
  Can3.mailboxStatus();

  // Setup ADC

  adc->adc0->setAveraging(32);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  adc->adc1->setAveraging(32);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  // Set up Outputs
  pinMode(OUT1, OUTPUT);   // Neg Cnct
  pinMode(OUT2, OUTPUT);   // Pre Cnct
  pinMode(OUT3, OUTPUT);   // Drive cnct
  pinMode(OUT4, OUTPUT);   // Brake lights
  pinMode(OUT5, OUTPUT);   // Heater Pump
  pinMode(OUT6, OUTPUT);   // Fan
  pinMode(OUT7, OUTPUT);   // No conection
  pinMode(OUT8, OUTPUT);   // DC-DC Enable
  pinMode(OUT9, OUTPUT);   // Temp Gauge
  pinMode(OUT10, OUTPUT);  // Dash Red LED
  pinMode(OUT11, OUTPUT);  // Dash Green LED
  pinMode(OUT12, OUTPUT);  // Rev Lights
  pinMode(LEDpin, OUTPUT); // Builtin LED

  // Setup Inputs
  pinMode(ISO_IN1, INPUT); // FWD
  pinMode(ISO_IN2, INPUT); // start from key
  pinMode(ISO_IN3, INPUT); // Brake
  pinMode(ISO_IN4, INPUT); // REV
  pinMode(ISO_IN5, INPUT); // Heater
  pinMode(ISO_IN6, INPUT); // Map 1 
  pinMode(ISO_IN7, INPUT); // IGN
  pinMode(ISO_IN8, INPUT); // PP Detect

  pinMode(POT_A, INPUT); // Throtle Pot A
  pinMode(POT_B, INPUT); // Throtle Pot B

  // Write initial Vals to Outputs
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  digitalWrite(OUT5, LOW);
  digitalWrite(OUT6, LOW);
  digitalWrite(OUT7, LOW);
  digitalWrite(OUT8, LOW);
  digitalWrite(OUT9, LOW);
  digitalWrite(OUT10, HIGH);
  digitalWrite(OUT11, LOW);
  digitalWrite(OUT12, LOW);
  digitalWrite(LEDpin, HIGH);

  // Watch Dog setup
  WDT_timings_t config;
  config.trigger = 2; // in seconds, 0->128
  config.timeout = 5; // in seconds, 0->128
  config.callback = wdtCallback;
  wdt.begin(config);
  BMStimer.begin(bmsComms, 10000);
  BMSread.begin(BMS_Stat, 500000);

  Serial.begin(9600);
  Serial.println("Mini-E VCU Starting Up.....");
  Serial.print("VCU Status: ");
  Serial.println(VCUstatus);
  delay(1000);
}

ADC::Sync_result result;

void loop()
{
  long loopTimestart = micros();
  dogFood(); // Reset the Watchdog

  Can1.events(); // Call CAN bus bus interupts
  Can2.events();
  Can3.events(); // Currently this only send.

//  dashComms();
  tempCheck();
 // BMS_Stat();
  stateHandler(); 
  readPins(); 

  if (pedalDebug == 1)
  {
    readPedal();
  }

  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (incomingByte == 115) // press s for menu
    {
      menuLoad = 0;
      showStats = 0;
      menu();
    }
    else
    {
      menu();
    }
  }


 // Speedo(); // speedo 

 if (Heater_pin == 0)  {
  digitalWrite(OUT5, HIGH);
}
else {
  digitalWrite(OUT5, LOW);
}


  if (timer30s_1.check()==1)  //Check the DC-DC so we only enable when needed.
  {
    if (charger12vbattryVolts > 13.6) //Greater than 12.6 volts probably menas not much load let's check
        {
          if (charger12vCurrent < .5)
          {
            digitalWrite(OUT8, LOW);
          }
        }
    else
    {
      digitalWrite(OUT8, HIGH);
    }
  }



  if ((timer2000_1.check() == 1) && showStats == 1)
  {

    showInfo();
  }
  loopTime = micros();
  loopTime = loopTime - loopTimestart;
}

//---------------end of loop -----------------------

void menu()
{
  if (menuLoad == 0)
  { // Serial Menu
    Serial.println("----------------Mini E VCU Settings Menu----------------");
    Serial.println();
    Serial.println("1 - Set Pump on Temp");
    Serial.println("2 - Set Fan on Temp");
    Serial.println("3 - Temp Gauge PWM Test");
    Serial.println("4 - Set Temp Gauge Low Value");
    Serial.println("5 - Set Temp Gauge High Value");
    Serial.println("6 - Set Pedal Lo Offset");
    Serial.println("7 - Set Pedal High Offset");
    // Serial.println("8 - Set Regen Tourqe");
    Serial.println("9 - Set Map");
    Serial.println("0 - Set Torque Increment");
    Serial.println("U - Charger Stop Voltage");
    Serial.println("F - Fan On / Off");
    Serial.println("M - Pump on / Off");
    Serial.println("P - Show Pedal debug info");
    Serial.println("X - Enable / Disable inverter");
    Serial.println("E - Enable / Disable DC-DC");  
    Serial.println("L - Load Default Values");
    Serial.println("V - VCU Status");
    Serial.println("I - Input Test");
    Serial.println("C - Contactor Test - ONLY WITH HV OFF!!");
    Serial.println("R - Reset Board");
    Serial.println("S - Show this menu");
    Serial.println("Q - Quit and Save");
    menuLoad = 1;
  }

  switch (incomingByte)
  {
  case '1':
  {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    pumpOnTemp = value;
    Serial.print("Pump on Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }
  case '2':
 {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    fanOnTemp = value;
    Serial.print("Fan on Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }

  case '3':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    //analogWrite(OUT9, value);
    Serial.print("Gauge Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  } 

  case '4':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    if (value < 0)
    {
      value = 0;
    }
    tempGaugeMin = value;
    Serial.print("Gauge Min Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }
  break;

  case '5':
  {
    int value = Serial.parseInt();
    if (value > 255)
    {
      value = 255;
    }
    if (value < 0)
    {
      value = 0;
    }
    tempGaugeMax = value;
    Serial.print("Gauge Max Set to: ");
    Serial.println(value);
    menuLoad = 0;
  } 

  break;

  case '6':
    setTpsLow = 1;
    readPedal();
    Serial.print(" ADC1 Reading: ");
    Serial.print(" Pedal Low Offset Set to: ");
    Serial.println(tpslowOffset);
    menuLoad = 0;

    break;

  case '7':
    setTpsHigh = 1;
    readPedal();
    Serial.print(" ADC1 Reading: ");
    Serial.print(" Pedal High Offset Set to: ");
    Serial.println(tpshighOffset);
    menuLoad = 0;

    break;

 /* case '8':
  {
    int value = Serial.parseInt();
    if (value > 1000)
    {
      value = 0;
    }
    if (value < 0)
    {
      value = 0;
    }
     = value;
    Serial.print("Regen Toruqe: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  } */

  case '9':
  {
    int value = Serial.parseInt();
    if (value > 4)
    {
      value = 4;
    }
    if (value < 1)
    {
      value = 1;
    }
    active_map = value;
    Serial.print("Map Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }

  case '0':
  {
    int value = Serial.parseInt();
    if (value > 100)
    {
      value = 100;
    }
    if (value < 0)
    {
      value = 0;
    }
    torqueIncrement = value;

    Serial.print("Torque Increment Set to: ");
    Serial.println(value);
    menuLoad = 0;
  }

 case 'u':
  {
    int value = Serial.parseInt();
    if (value > 420)
    {
      value = 420;
    }
    chargerstop = value;
    Serial.print("Charger Stop Voltage Set to: ");
    Serial.println(value);
    menuLoad = 0;

    break;
  }

  case 'f':
    if (fanState == 0)
    {
      fanState = 2; // 2 = Debug
      digitalWrite(OUT6, HIGH);
      Serial.print("FAN ON");
    }
    else
    {
      fanState = 0;
      digitalWrite(OUT6, LOW);
      Serial.print("FAN OFF");
    }
    break;

    break;

  case 'l':
    loadDefault();
    menuLoad = 0;

    break;

  case 'c':
    if (VCUstatus == 2)
    {
      Serial.println("NEG ON");
      digitalWrite(OUT1, HIGH);
      delay(1000);
      Serial.println("NEG OFF");
      digitalWrite(OUT1, LOW);
      delay(500);
      Serial.println("PRE ON");
      digitalWrite(OUT2, HIGH);
      delay(1000);
      Serial.println("PRE OFF");
      digitalWrite(OUT2, LOW);
      delay(500);
      Serial.println("MAIN ON");
      digitalWrite(OUT3, HIGH);
      delay(1000);
      Serial.println("MAIN OFF");
      digitalWrite(OUT3, LOW);
    }
    else
    {
      Serial.println("HV MUST BE OFF");
    }
    break;

  case 'm':
    if (pumpState == 0)
    {
      pumpState = 2; // 2 = Debug
      digitalWrite(OUT5, HIGH);
      Serial.print("PUMP ON");
    }
    else
    {
      pumpState = 0;
      digitalWrite(OUT5, LOW);
      Serial.print("PUMP OFF");
    }
    break;

  case 'p':
    if (pedalDebug == 0)
    {
      pedalDebug = 1;
      showStats = 0;
    }
    else
    {
      pedalDebug = 0;
      showStats = 1;
    }
    break;

  case 'x':
    if (inverterFunction == 0x00)
    {
      inverterFunction = 0x03;
      inverterEnable = 1;
      Serial.println("Inverter Enable");
    }
    else

    {
      inverterFunction = 0x00;
      inverterEnable = 0;
      Serial.println("Inverter Disable");
    }

    break;

  case 'e':
    if (digitalRead(OUT8)==HIGH)
    {
      digitalWrite(OUT8, LOW);
    }
    else
    {
      digitalWrite(OUT8, HIGH);
    }

    break;


  case 'v':
    Serial.print("");
    Serial.print("VCU Status: ");
    Serial.println(VCUstatus);
    menuLoad = 0;

    break;

  case 'i':
    readPins();
    Serial.println("Reading Pins...");
    Serial.print("ISO 2 - Start Signal:");
    Serial.println(start);
    Serial.print("ISO 3 - Brake Pedal:");
    Serial.println(brake_pedal);
    Serial.print("ISO 1 - Forward:");
    Serial.println(dir_FWD);
    Serial.print("ISO 4 - Reverse:");
    Serial.println(dir_REV);
    Serial.print("ISO  8 - Charge Plug Detect:");
    Serial.println(ppDetect);
    Serial.print("ISO 6 - SPORT:");
    Serial.println(map3);
    Serial.print("ISO 7 - IGITION:");
    Serial.println(ignition);
    Serial.print("Thotle POT A:");
    Serial.print(result.result_adc0);
    Serial.print(" , Thotle POT B:");
    Serial.println(result.result_adc1);
    Serial.print("Rotation:");
    Serial.println(selectdir);
    menuLoad = 0;

    break;


  case 'r':
    wdt.reset();
    break;

  case 'q':

    saveVarsToEEPROM();
    showStats = 1;
    menuLoad = 0;
    break;
  }
}

void readPins()
{

  dir_FWD = digitalRead(ISO_IN1);
  start = digitalRead(ISO_IN2);
  brake_pedal = digitalRead(ISO_IN3);
  dir_REV = digitalRead(ISO_IN4);
  Heater_pin = digitalRead(ISO_IN5);
  map3 = digitalRead(ISO_IN6);
  ignition = digitalRead(ISO_IN7);
  ppDetect = !digitalRead(ISO_IN8);

 }

void readPedal()

// Sync Analog read
// compare the result

{
  ADC::Sync_result result = adc->analogSyncRead(POT_A, POT_B);
  result.result_adc0 = (uint16_t)result.result_adc0;
  result.result_adc1 = (uint16_t)result.result_adc1;
  float comparisonResult = ((result.result_adc1 * 1.0) / (result.result_adc0 * 1.0));
  if (comparisonResult != 0)
  {
    if ((comparisonResult > 1.0) && (comparisonResult < 2.0))
    {
      lowpassFilter.input(result.result_adc1);
      throttlePosition = floor(lowpassFilter.output());

      throttlePosition = map(result.result_adc1, tpslowOffset, tpshighOffset, 0, 100);

      if (throttlePosition < 2)
      {
        throttlePosition = 0;
      }
      if (throttlePosition > 100)
      {
        throttlePosition = 100;
      }
    }

    // if ((comparisonResult < 1.0 || (comparisonResult > 2.0)))
    // {
    //   // Serial.println("--!PEDDAL MISMATCH!--");
    //   // throttlePosition = 0;
    // }
  }

  if (adc->adc0->fail_flag != ADC_ERROR::CLEAR)
  {
    Serial.print("ADC0: ");
    Serial.println(getStringADCError(adc->adc0->fail_flag));
  }
  if (adc->adc1->fail_flag != ADC_ERROR::CLEAR)
  {
    Serial.print("ADC1: ");
    Serial.println(getStringADCError(adc->adc1->fail_flag));
  }

  if ((timer500_1.check() == 1) && pedalDebug == 1)
  {
    Serial.print("ADC0 Reading: ");
    Serial.print(result.result_adc0);
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    Serial.print(" Comparison Result: ");
    Serial.print(comparisonResult);

    Serial.print(" Throtle Position: ");
    Serial.println(throttlePosition);

    Serial.print("Filtered Position: ");
    Serial.println(lowpassFilter.output());
  }

  if (setTpsLow == 1)
  {
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    tpslowOffset = result.result_adc1;
    setTpsLow = 0;
  }
  if (setTpsHigh == 1)
  {
    Serial.print(" ADC1 Reading: ");
    Serial.print(result.result_adc1);
    tpshighOffset = result.result_adc1;
    setTpsHigh = 0;
  }
}

void dogFood()
{
  wdt.feed(); // Feed the Watchdog
}

// CAN Functions

void BMS_Stat()
{
// if (timer1000_2.check() == 1) {
	
BMS_avgtmp = 0;
BMS_packvoltage = 0.00;  
memset(BMS_Volt, 0, sizeof(BMS_Volt)); 
   Serial.println("BMS_Stat");

   msg.id = 0x01;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);
//    Can2.write(msg);
//    Can3.write(msg);
    
   // delay(2); 
    msg.id = 0x02;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);

    //delay(2); 
    msg.id = 0x03;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);

    //delay(2); 
    msg.id = 0x04;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);
    
    //delay(2); 
    msg.id = 0x05;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);

    //delay(2); 
    msg.id = 0x06;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);

    //delay(2); 
    msg.id = 0x07;
    msg.len = 1;
    msg.buf[0] = 0xFF;
    Can1.write(msg);

//delay(200); 
 BMS_max = 0;
  BMS_min = 5;
  BMS_packvoltage = 0;
 for(uint8_t i=0; i<sizeof(BMS_Volt)/sizeof(int);i++) {

  BMS_max = max(BMS_Volt[i],BMS_max);
 if (BMS_Volt[i] > 0)
  {
  BMS_min = min(BMS_Volt[i],BMS_min);
BMS_packvoltage = BMS_packvoltage+BMS_Volt[i];
 
    }

}
Serial.println(BMS_packvoltage);
BMS_SOC = map(uint16_t(BMS_avgvolt*1000), 2550, 4150, 0,100); 
BMS_discurrent = 225;
if (BMS_discurrent > 0)
    {
      //Temperature based///

      if (BMS_avgtmp > 30)
      {
        BMS_discurrent = BMS_discurrent - map(BMS_avgtmp, 30, 100, 0, 225);
      }
      //Voltagee based///
      if (BMS_min < (3.5))
      {
        BMS_discurrent = BMS_discurrent - map(BMS_min, 3.2, 3.5, 225, 0);
      }

}
//}
}

void canRX_289(const CAN_message_t &msg)
{
  motorTorque = ((((msg.buf[0] * 256) + msg.buf[1]) - 10000) / 10); // Motor Torque -200 / + 200nm
  motorRPM = (msg.buf[2] * 256 + msg.buf[3] - 20000);               // 289_RrRPM,Rear RPM,220289,C*256+D-20000,-10000,10000,RPM,
  motorRPM = motorRPM * -1;  //Changing direction of RPM reading
  motorHVbatteryVolts = (msg.buf[4] * 256 + msg.buf[5]);            // Inverter HV
}

void canRX_299(const CAN_message_t &msg)
{
  // inverter Temps
  motorTempPeak = (msg.buf[0] - 40);
  inverterTemp1 = (msg.buf[1] - 40);
  inverterTemp2 = (msg.buf[4] - 40);

  avgInverterTemp = (inverterTemp1 + inverterTemp2) / 2;
}

/* void canRX_351(const CAN_message_t &msg)
{
//  BMS_discurrent = (((msg.buf[5] * 256) + msg.buf[4]) / 10);
} */

void canRX_001(const CAN_message_t &msg)
{

if (msg.buf[0] == 1)
  {
   BMS_avgvolt=max(BMS_avgvolt,((msg.buf[5] / 1000.0) * 256) + (msg.buf[6] / 1000.0)); 
     BMS_avgtmp = max(BMS_avgtmp,(msg.buf[2] + msg.buf[1]) + 8);
  }

  if (msg.buf[0] == 4)
  {
  if (msg.buf[1] < 19)

  {
    if (msg.id == 0x01)
  {
      //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1]] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 1] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 2] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }
  if (msg.id == 0x02)
  {
      //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 13] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 14] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 15] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }
  if (msg.id == 0x03)
  {
      //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 25] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 26] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 27] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }
  if (msg.id == 0x04)
  {
         //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 37] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 38] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 39] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }

      if (msg.id == 0x05)
  {
         //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 49] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 50] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 52] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }

   if (msg.id == 0x06)
  {
         //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 61] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 62] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 63] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }   

  if (msg.id == 0x07)
  {
         //BMS_avgtmp4=0;
    //BMS_packvoltage4 = 0;
    BMS_Volt[msg.buf[1] + 73] = (((msg.buf[2] / 1000.0) * 256) + (msg.buf[3] / 1000.0));
    BMS_Volt[msg.buf[1] + 74] = (((msg.buf[4] / 1000.0) * 256) + (msg.buf[5] / 1000.0));
    BMS_Volt[msg.buf[1] + 75] = (((msg.buf[6] / 1000.0) * 256) + (msg.buf[7] / 1000.0));
  }
    
  }
   }

 // Serial.println(msg.id);
if (msg.buf[0] == 0x01)
  {
   // BMS_avgtmp1=0;
   // BMS_packvoltage1 = 0;
     }

  BMS_SOC = ((msg.buf[1] * 256) + msg.buf[0]);
  //BMS_avgtmp = (((msg.buf[5] / 10.0) * 256) + (msg.buf[4] / 10.0));
  //BMS_Status = 3;

  if (BMS_Status == 5)
  {
    if (VCUstatusChangeCounter > VCUstatusChangeThreshold)

      VCUstatusChangeCounter = 0;
    VCUstatus = 7;
  }
  else
  {
    VCUstatusChangeCounter++;
  }
}

/* void canRX_356(const CAN_message_t &msg)
{

//  BMS_packvoltage = (((msg.buf[1] * 256) / 100.0) + (msg.buf[0] / 100.0));

  int amps = (msg.buf[3] * 256) + (msg.buf[2]);
  currentact = ((amps - 3000) / 10.0);
} */

void canRX_377(const CAN_message_t &msg)
{
  charger12vbattryVolts = float(((msg.buf[0] * 256) + msg.buf[1]) * 0.01);
  charger12vCurrent = float(((msg.buf[2] * 256) + msg.buf[3]) * 0.01);
  chargerTemp1 = msg.buf[4] - 40;
  chargerTemp2 = msg.buf[5] - 40;
  chargerTemp3 = msg.buf[6] - 40;
  chargerStatus = msg.buf[7];
  DCDCTemp = (chargerTemp1 + chargerTemp2 + chargerTemp3) / 3;
}

void canRX_389(const CAN_message_t &msg)
{
  chargerHVbattryVolts = msg.buf[0] * 2; // Charger HV Battery Voltage
  chargerACvolts = msg.buf[1];
  chargerHVcurrent = msg.buf[2] * 0.1;
  chargerTempCH = msg.buf[4] - 40;
}

void canRX_398(const CAN_message_t &msg) // Heater stats
{
Heatertemp1 = msg.buf[3] - 40;  
Heatertemp2 = msg.buf[4] - 40;
Heatertemp = (Heatertemp1 + Heatertemp2) / 2;
}

void canRX_38A(const CAN_message_t &msg)
{
 // chargerHVbattryVolts = msg.buf[0] * 2; // Charger HV Battery Voltage
  chargercurrent = msg.buf[3];
}

void canRX_732(const CAN_message_t &msg)
{
  // inverter Current
  motorCurrent1 = (msg.buf[0] * 256 + msg.buf[1] - 1000);
  motorCurrent2 = (msg.buf[2] * 256 + msg.buf[3] - 1000);
}

void canRX_733(const CAN_message_t &msg)
{
  // inverter Temps
  motorTemp1 = (msg.buf[0] - 40);
  motorTemp2 = (msg.buf[2] - 40);
  avgMotorTemp = (motorTemp1 + motorTemp2 + motorTempPeak) / 3;
}

void EvseStart()
{

 if (timer500_2.check() == 1)
  {
  
    msg.id = 0x286;
      msg.len = 8;
      msg.buf[0] = 0x10; //voltage 360V
      msg.buf[1] = 0x0E;
      if ((chargerHVbattryVolts > (chargerstop - 2)) && (chargerHVbattryVolts <= (chargerstop))) { // if charger is at 370V
      msg.buf[2] = 0x1E;        
      }
      else if (chargerHVbattryVolts > chargerstop)  { // if charger is at 388V
      msg.buf[2] = 0x00;        
      }
      else  { //any other case
      msg.buf[2] = 0x78;        
      }
      msg.buf[3] = 0x37;
      msg.buf[4] = 0x0;
      msg.buf[5] = 0x0;
      msg.buf[6] = 0x0A;
      msg.buf[7] = 0x0;
    Can2.write(msg);
 //  Serial.println(map(chargercurrent,13,27,6,12)*10); 
}
}

void inverterComms()
{

  if (timer50_1.check() == 1)
  {

    if (regenState == 0 && VCUstatus == 4 && motorRPM > 2000) // Increment Torque delivery
    {

      if (curentTorque < targetTorque)
      {
     //   curentTorque += torqueIncrement;
     //   torqueRequest = curentTorque;
       torqueRequest = targetTorque;
       curentTorque = torqueRequest;    
      }

      if (curentTorque >= targetTorque)
      {
        torqueRequest = targetTorque;
        curentTorque = targetTorque;
      }
    }
    else if (regenState == 0 && VCUstatus == 4 && motorRPM < 2000)
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

    if (active_map == 3 && VCUstatus == 4) // No need to increment in 'sport' mode
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

   if (active_map == 1 && VCUstatus == 4) // Regen in map1 is to be off
    {
        regenState = 0;
        regenTarget = 0;
        regenRequest = 0;
        brakeDelay = 0;
    }


    if (regenState == 1 && VCUstatus == 4)

    {
      if (regenTarget < regenRequest) // increment Regen
      {
        regenRequest -= 5; 
        torqueRequest = regenRequest;
        // Serial.println("Regen inc.");
      }
      else
        regenRequest = regenTarget;
      torqueRequest = regenRequest;
    }

    if (regenState == 2 && VCUstatus == 4)

    {
      if (regenTarget > regenRequest) // increment Regen off
      {
        regenRequest += 20; 
        torqueRequest = regenRequest;
        // Serial.println("Regen Dec.");
      }
    }

    if (torqueRequest > (2800)) // Going for 230Nm
    {
      torqueRequest = 2800;
      Serial.println("--!OVER TOURQUE!--");
    }
    if (torqueRequest < (-2800))
    {
      torqueRequest = -2800;
      Serial.println("--!UNDER TOURQUE!--");
    }

    torqueRequest = torqueRequest;
    torqueRequest += 10000;

   /* if (BMS_discurrent < currentact) // Decrese tourque if we are over current - Crude needs work..
    {
      torqueRequest -= 20;
      Serial.println("--!OVER CURRENT!--");
      if (torqueRequest < 0)
      {
        torqueRequest = 0;
      }
    } */

    if (pedalDebug == 1)
    {
      Serial.print("Offset: ");
      Serial.print(pedal_offset);
      Serial.print(" Tourque Request: ");
      Serial.println(torqueRequest - 10000);
      Serial.print("Regen Target:");
      Serial.print(regenTarget);
      Serial.print(" Regen Request: ");
      Serial.print(regenRequest);
      Serial.print(" Motor Torque ");
      Serial.println(motorTorque);
    }
    if (inverterEnable != 1)
    {
      inverterFunction = 0x00;
    }
    torqueLoByte = lowByte(torqueRequest);
    torqueHibyte = highByte(torqueRequest);
    msg.id = 0x287;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = torqueHibyte;
    msg.buf[3] = torqueLoByte;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = inverterFunction;
    msg.buf[7] = 0;
    Can2.write(msg);
    torqueRequest = 0;

  }

  if (timer100_1.check() == 1)
  {
    msg.id = 0x371;
    msg.len = 8;
    msg.buf[0] = 48;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can2.write(msg);

    msg.id = 0x286;
    msg.len = 8;
    msg.buf[0] = 0x10;
    msg.buf[1] = 0x0E;
    msg.buf[2] = 0;
    msg.buf[3] = 0x37;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0x0A;
    msg.buf[7] = 0;
    Can2.write(msg);

  }
}

void BMS_Read()
{
  if (timer10_1.check() == 1)
  {
 BMS_max = 0;
  BMS_min = 5;
  BMS_packvoltage = 300;
 for(uint8_t i=0; i<sizeof(BMS_Volt)/sizeof(int);i++){
 // Serial.print("Элемент ");
 //   Serial.print(i);
 //   Serial.print(": ");
 //   Serial.println(BMS_Volt[i]);
  BMS_max = max(BMS_Volt[i],BMS_max);
 if (BMS_Volt[i] > 0)
  {
  BMS_min = min(BMS_Volt[i],BMS_min);
// BMS_packvoltage = BMS_packvoltage+BMS_Volt[i];
  
    }
}
Serial.println(BMS_packvoltage);
BMS_SOC = map(uint16_t(BMS_avgvolt*1000), 2550, 4150, 0,100); 
BMS_discurrent = 225;
if (BMS_discurrent > 0)
    {
      //Temperature based///

      if (BMS_avgtmp > 30)
      {
        BMS_discurrent = BMS_discurrent - map(BMS_avgtmp, 30, 100, 0, 225);
      }
      //Voltagee based///
      if (BMS_min < (3.5))
      {
        BMS_discurrent = BMS_discurrent - map(BMS_min, 3.2, 3.5, 225, 0);
      }
}
}
}

void dashComms()
{

  if (timer10_1.check() == 1)
  {
    msg.id = 0x555;
    msg.len = 8;
    msg.buf[0] = highByte(motorRPM);
    msg.buf[1] = lowByte(motorRPM);
    msg.buf[2] = VCUstatus;
    msg.buf[3] = BMS_Status;
    msg.buf[4] = BMS_SOC;
    msg.buf[5] = chargerHVbattryVolts / 2;
    msg.buf[6] = DCDCTemp;
    msg.buf[7] = avgMotorTemp;
    Can3.write(msg);

    int amps = ((currentact * 10) + 3000);
    msg.id = 0x558;
    msg.len = 8;
    msg.buf[0] = lowByte(amps);
    msg.buf[1] = highByte(amps);
    msg.buf[2] = lowByte(int16_t(BMS_avgtmp * 10));
    msg.buf[3] = highByte(int16_t(BMS_avgtmp * 10));
    msg.buf[4] = lowByte(uint16_t(BMS_packvoltage) * 100);
    msg.buf[5] = highByte(uint16_t(BMS_packvoltage) * 100);
    msg.buf[6] = torqueHibyte;
    msg.buf[7] = torqueLoByte;
    Can3.write(msg);

    msg.id = 0x560;
    msg.len = 8;
    msg.buf[0] = active_map;
    msg.buf[1] = 0;
    ;
    msg.buf[2] = 0;
    ;
    msg.buf[3] = 0;  
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can3.write(msg);
  }
}

void HeaterComms()
{
  if (timer100_2.check() == 1)
{
    msg.id = 0x188;
    msg.len = 8;
    if (digitalRead(ISO_IN5) == LOW)
{
    msg.buf[0] = 0x03; // byte0 status command Heater pin is on
}
else {
    msg.buf[0] = 0x00; // Heater pin is off
}
    msg.buf[1] = 0x50; // byte1 20 to 50 works
if (Heatertemp >= 55) {
    msg.buf[2] = 0x00; //Heater off when at 55deg   
}    
else if (Heatertemp > 50) {
    msg.buf[2] = 0x32; //Heater at 1/2 when at 50deg   
}    
else {
    msg.buf[2] = 0xA2; //Heater on at full power (dec/10)
}    
    msg.buf[3] = 0x36; // byte3 30 to 40 works
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can2.write(msg);  

}    
}  

void bmsComms()
{
    msg.id = 0x285;
    msg.len = 8;
    msg.buf[0] = 0x00;
    msg.buf[1] = 0x00;
    if (BMS_Status == 3)
      {
    msg.buf[2] = 0xB6;
      }
    else if (BMS_Status == 5)
      {
    msg.buf[2] = 0x00;
      }
    else 
      {
    msg.buf[2] = 0x14;
      }      
    msg.buf[3] = 0x39;
    msg.buf[4] = 0x91;
    msg.buf[5] = 0xFE;
    msg.buf[6] = 0x0E;
    msg.buf[7] = 0x10;
    Can2.write(msg);
}

void tempCheck()
{
  if (timer100_3.check() == 1)
  {

    if (Heatertemp > pumpOnTemp) // Temp Handling
    {

      if (pumpState == 0)
      {
        digitalWrite(OUT5, HIGH);
        pumpState = 1;
      }
    }
     else if (Heatertemp < (pumpOnTemp - 5))
      {
        if (pumpState == 1)
        {
          digitalWrite(OUT5, LOW);
          pumpState = 0;
        }
      }

    if ((DCDCTemp || chargerTempCH || avgInverterTemp || avgMotorTemp) > fanOnTemp)
    {
      if (fanState == 0)
      {
        digitalWrite(OUT6, HIGH);
        fanState = 1;
      }
    }
    else if ((DCDCTemp && chargerTempCH && avgInverterTemp && avgMotorTemp) < (fanOnTemp - 5))
    {
      if (fanState == 1)
      {
        digitalWrite(OUT6, LOW);
        fanState = 0;
      }
    }

    // Send To Temp Gaauge
    uint8_t tempGauge = (DCDCTemp + avgMotorTemp + avgInverterTemp) / 3;
    tempGauge = map(tempGauge, 0, 100, tempGaugeMin, tempGaugeMax);
    //analogWrite(OUT9, tempGauge);
  }
}

void Speedo()
{
if (timer100_4.check() == 1)
  {  
calcspeed = (motorRPM / ratio) * tireD * 3.14 * 60 / 1000; // speed = wheelsRPM * tireD * PI * 60 / 1000 Km/h
calcHz = calcspeed * 1.4;
if (calcHz >= 0) {
calcHzi = calcHz;
}
else {
calcHzi = calcHz * (-1);
}
tone(OUT7, calcHzi);
//analogWriteFrequency(OUT7, calcHzi);
}
}

void showInfo()
{
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print("VCU Staus: ");
  Serial.print(VCUstatus);
  Serial.print("Active Map:");
  Serial.print(active_map);
  Serial.print("  BMS Status: ");
  Serial.print(BMS_Status);
  Serial.print(" Charger Status: ");
  Serial.print(chargerStatus, HEX);
  Serial.print(" BMS Current: ");
  Serial.print(currentact);
  Serial.print(" BMS HV: ");
  Serial.print(BMS_packvoltage);
  Serial.print(" BMS TEMP: ");
  Serial.print(BMS_avgtmp);
  Serial.print(" BMS Dischrge Limit");
  Serial.println(BMS_discurrent);
  Serial.print("SOC %: ");
  Serial.print(BMS_SOC);
  Serial.print(" DC-DC Volts: ");
  Serial.print(charger12vbattryVolts);
  Serial.print(" DC-DC Current: ");
  Serial.print(charger12vCurrent);
  Serial.print(" DC-DC Enable: ");
  Serial.print(digitalRead(OUT8));
  Serial.print(" HV charger Current: ");
  Serial.print(chargerHVcurrent);
  Serial.print(" HV Battery Voltage: ");
  Serial.print(chargerHVbattryVolts);
  Serial.print(" Charger AC volts: ");
  Serial.println(chargerACvolts);
  Serial.print("Heater Temperature: ");
  Serial.print(Heatertemp);
  Serial.print("  Heater Status: ");
  Serial.print(Heater_pin);
  Serial.print("  Charger Stop Voltage: ");
  Serial.print(chargerstop);
  Serial.print("  Charger Temp: ");
  Serial.print(chargerTempCH);
  Serial.print("  DCDC Temp: ");
  Serial.println(DCDCTemp);
  Serial.print(" Avg Inverter Temp: ");
  Serial.println(avgInverterTemp);
  Serial.print("  Torque Request: ");
  Serial.println(torqueRequest);
  Serial.print("  Torque Increment: ");
  Serial.println(torqueIncrement);
  Serial.print("RPM: ");
  Serial.print(motorRPM);
  Serial.print(" Motor Peak Temp: ");
  Serial.print(motorTempPeak);
  Serial.print(" Mtr Temp 1: ");
  Serial.print(motorTemp1);
  Serial.print(" Mtr Temp 2: ");
  Serial.print(motorTemp2);
  Serial.print("Avg motor Temp: ");
  Serial.println(avgMotorTemp);
  Serial.print("Motor HV: ");
  Serial.print(motorHVbatteryVolts);  
  Serial.print("  motor Torque: ");
  Serial.print(motorTorque);
  Serial.print("  Motor Amps1: ");
  Serial.print(motorCurrent1);
  Serial.print("  Motor amps2: ");
  Serial.println(motorCurrent2);
  Serial.print("throttle low offset: ");
  Serial.print(tpslowOffset);
  Serial.print(" throttle high offset: ");
  Serial.print(tpshighOffset);
  Serial.print("Loop Time: ");
  Serial.print(loopTime);
}

void loadDefault() // Load the defaul values
{
  tempGaugeMin = 18;  // Temp Gauge min PWM
  tempGaugeMax = 128; // Temp Gauge Max PWM
  chargerstop = 340; // stop charger
  pumpOnTemp = 30;
  fanOnTemp = 30;
  maxTorque = 2000;      // Not used curently
  minTorque = 0;        // Not used  curently
  tpslowOffset = 1700;  // Value when i just put my foot on the pedal and push a bit when reading the offset
  tpshighOffset = 4080; // Value when pedal fully pressed 
  torqueIncrement = 20; // Value to ramp tourqe by
  active_map = 3;

  Serial.println("Loaded Default Vlues");

}

void saveVarsToEEPROM() // Save Values to EEPROM
{
  Serial.println("Writing Values to EEPROM...");
  EEPROM.update(1, tempGaugeMin);
  EEPROM.update(2, tempGaugeMax);
  EEPROM.update(3, pumpOnTemp);
  EEPROM.update(6, fanOnTemp);
  EEPROM.update(8, minTorque);
  EEPROM.update(9, lowByte(chargerstop));
  EEPROM.update(10, highByte(chargerstop));
  EEPROM.update(12, torqueIncrement); 
  EEPROM.update(20, lowByte(tpslowOffset));
  EEPROM.update(21, highByte(tpslowOffset));
  EEPROM.update(22, lowByte(tpshighOffset));
  EEPROM.update(23, highByte(tpshighOffset));
  EEPROM.update(30, lowByte(maxTorque));
  EEPROM.update(31, highByte(maxTorque));

  Serial.println("Finished Writing Values to EEPROM...");
}

void stateHandler()
{
  switch (VCUstatus)
  {
  case boot:
  {

    VCUstatus = 2;

    Serial.print("VCU Status: ");
    Serial.println(VCUstatus);
    break;
  }
  case ready:
  {
	  
    if ((BMS_Status == 1) && (ignition == 1) )    
     {

  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  digitalWrite(OUT5, LOW);
  digitalWrite(OUT6, LOW);
  digitalWrite(OUT7, LOW);
  digitalWrite(OUT8, LOW);
  digitalWrite(OUT9, LOW);
  digitalWrite(OUT10, HIGH);
  digitalWrite(OUT11, LOW);
  digitalWrite(OUT12, LOW);
  digitalWrite(LEDpin, LOW);
    }

    if ((digitalRead(ISO_IN8) == HIGH)) //PP pin
    {
      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        BMS_Status = 3;
        digitalWrite(OUT1, HIGH);
        VCUstatus = charging;
        Serial.print("VCU Status: ");
        Serial.println(VCUstatus);
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }


    if ((start == 0) && (brake_pedal == 0) && (BMS_Status != 5))
    {
      if ((dir_FWD == 1) && (dir_REV == 1))
      {
        BMS_Status = 2;
        digitalWrite(OUT1, HIGH);
        VCUstatus = 3; // Neutral
        Serial.print("VCU Status: ");
        Serial.println(VCUstatus);
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }
    break;
  }

    case driveNeutral:
  {
    if (digitalRead(OUT12) == HIGH)
    {
      digitalWrite(OUT12, LOW); // Turn off the reversing lights
    }
    if (digitalRead(OUT4) == HIGH)
    {
      digitalWrite(OUT4, LOW); // Turn off the Brake lights
    }
    if (digitalRead(OUT1) == LOW) // Check if Neg Contactor is off then start the sequence
    {

      digitalWrite(OUT1, HIGH); // Negative Contactor

      Serial.println("Negative On!...");
      pretimer1 = millis();
    }
    if (pretimer1 + 500 < millis())
    {
      if ((digitalRead(OUT2) == LOW) && (digitalRead(OUT3) == LOW))
      {
        digitalWrite(OUT11, HIGH); // Voltage light during during precharge on
        digitalWrite(OUT10, HIGH); // Error light on during precharge    
        digitalWrite(OUT2, HIGH);
        Serial.println("PreCharge On!...");
        pretimer1 = millis();
        pretimer2 = millis();
      }
    }
    if (pretimer1 + 1000 < millis())
    {
       if ((motorHVbatteryVolts >= chargerHVbattryVolts - 5.0) && (digitalRead(OUT3) == LOW)) // Check that the preCharge has finished
      {
        digitalWrite(OUT11, LOW); // Voltage light during during precharge on
        digitalWrite(OUT10, LOW); // Error light on during precharge    
        digitalWrite(OUT3, HIGH); // Turn on drive Contactor
        Serial.println("Main On!...");
        pretimer1 = millis();
      }
    }

    if ((pretimer1 + 500 < millis()) && (digitalRead(OUT2) == HIGH) && (digitalRead(OUT3) == HIGH))
    {
        digitalWrite(OUT2, LOW);
        digitalWrite(OUT11, HIGH); // Voltage light during during precharge on
        digitalWrite(OUT10, LOW); // Error light on during precharge    
      Serial.println("PreCharge OFF!...");
      digitalWrite(OUT8, HIGH); // DC-DC Enable on
      Serial.println("DC-DC Enabled...");
      Serial.print("VCU Status: ");
      Serial.println(VCUstatus);
      Serial.print("Precharge Time: ");
      Serial.println(preChargeRiseTime);
      pretimer1 = 0;
    }


    if ((digitalRead(OUT3) == LOW) && (digitalRead(OUT2) == HIGH))
    {

      if (motorHVbatteryVolts >= chargerHVbattryVolts - 5.0)
      {
        if (preChargeRiseTime == 0)
        {
          preChargeRiseTime = millis() - pretimer2;
        }
      }
    }


    if ((digitalRead(OUT3) == HIGH) && (digitalRead(OUT2) == LOW) && (digitalRead(OUT1) == HIGH))
    {
      if (preChargeRiseTime < 500)
      {
        digitalWrite(OUT11, HIGH); // Voltage light during during precharge on
        digitalWrite(OUT10, LOW); // Error light on during precharge            
      }
      if (preChargeRiseTime >= 500)
      {
        digitalWrite(OUT11, LOW); // Voltage light during during precharge off
        digitalWrite(OUT10, LOW); // Error light off during precharge    
      }

      if (motorRPM < 100)
      {                          // Dont shut down inverter above 2mph
        BMS_keyOn = 0;           // BMS Key off...Inverter Disable
        inverterFunction = 0x00; // Inverter Disable

      }

      if ((dir_FWD == 0) && (dir_REV == 1) && (BMS_Status != 5))
      {
        if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
        {
          VCUstatusChangeCounter = 0;
          VCUstatus = 4; // Drive forward
        }
        else
        {
          VCUstatusChangeCounter++;
        }
      }
      if ((dir_FWD == 1) && (dir_REV == 0) && (BMS_Status != 5))
      {

        if (motorRPM < 200)
        {
          if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
          {
            VCUstatusChangeCounter = 0;
            VCUstatus = 5; //Drive reverse
          }
          else
          {
            VCUstatusChangeCounter++;
          }
        }
      }
    }
tone(OUT9, 3000);    
Speedo();  // speedo     
HeaterComms();

   if (digitalRead(ISO_IN5) == LOW)  {
  digitalWrite(OUT5, HIGH);
}
else {
  digitalWrite(OUT5, LOW);
}

    if (BMS_Status != 2) {
BMS_Status = 2;
    }

      // Check for ISO_IN7 ignition inactivity
   if ((digitalRead(ISO_IN1) == HIGH) && (digitalRead(ISO_IN4) == HIGH) && (digitalRead(ISO_IN7) == HIGH))
      {
     if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        BMS_Status = 1;
        VCUstatus = 2;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
      }

    break;
  }

  case driveForward:
  {
    if ((dir_FWD == 1) && (dir_REV == 1))
    {

      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        digitalWrite(OUT12, LOW); // Turn off Rev Lights
        VCUstatus = 3;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }

      if (fanState == 0)
      {
        digitalWrite(OUT6, HIGH);
        fanState = 1;
      }

    readPedal();
    BMS_keyOn = 1; // Key on for BMS
  digitalWrite(OUT11, LOW); // Voltage light during during precharge off
  digitalWrite(OUT10, LOW); // Error light off during precharge  
    digitalWrite(OUT12, LOW); // Turn off Rev Lights
    // Pedal Map Handler

    byte j = 0; // index of tps
    byte k = 0; // index of throttle pos
    while (throttlePosition > tpsbins[k])
    {
      k++;
    }
    while (motorRPM > rpmbins[j] && j <= num_rpm_bins - 1)
    {
      j++;
    }
    if (j > num_rpm_bins - 1)
    {
      j = num_rpm_bins - 1;
    };
    idx_k = k;
    idx_j = j;

     if ( map3 == 0)
    {
      active_map = 1;
          // digitalWrite(OUT6, HIGH); // Fan on   
    }
     else
    {
      active_map = 3;
    }

    switch (active_map)
    {
    case 1: // copy this code for other maps..
    {
      pedal_offset = pedal_map_one[idx_j][idx_k];
      break;
    }

    case 2: // copy this code for other maps..
    {
      pedal_offset = pedal_map_two[idx_j][idx_k];
      break;
    }

    case 3: // copy this code for other maps..
    {
      pedal_offset = pedal_map_three[idx_j][idx_k];
      break;
    }
    }

    if (pedal_offset > 1 && regenState != 0)
    {

      regenState = 2;
      regenTarget = 0;

      if (regenRequest >= -9) 
      {
        regenState = 0;
        regenTarget = 0;
        regenRequest = 0;
        brakeDelay = 0;
        digitalWrite(OUT4, LOW); // Brake Lights off
      }
    }
    else if (pedal_offset > 1 && regenState != 2)
    {
      inverterFunction = 0x03;                              // Enable inverter
      targetTorque = (throttlePosition * pedal_offset) * -2; // *2 for OEM because we are 200nm Max torque
      regenState = 0;
      regenTarget = 0;
      regenRequest = 0;
    }

    if (pedal_offset < 0)
    {

      inverterFunction = 0x03;
      regenState = 1;
      regenTarget = pedal_offset * -1; // previously +2 for OEM direction
      if (brake_pedal == 1 && brakeDelay == 0)
      {
        brakeDelay = millis();
      }
      if (brake_pedal == 1 && brakeDelay + 1000 < millis() && regenRequest < -9) 
      {
        digitalWrite(OUT4, HIGH);
        brakeDelay = 0;
      }
    }

    if (pedal_offset == 0)
    {
      torqueRequest = 0;
      targetTorque = 0;
    }

     if (motorRPM > 10000)  // when motor RPM is larger than 10K we remove torque
      {
        torqueRequest = 0;
      }

    if (brake_pedal == 0 && motorRPM < 30)  //if brake is pressed and motor rpm is less than 30rpm
    {
      torqueRequest = 0;       // 0 Torque if the brake is presed and we are nearly stopped
      inverterFunction = 0x00; // 0x00; // Shut off inverter
      regenState = 0;
      regenTarget = 0;
      regenRequest = 0;
      regenTimer = 0;
      brakeDelay = 0;
      digitalWrite(OUT4, LOW); // Brake Lights off
    }
Speedo();  // speedo 
inverterComms();
HeaterComms();

   if (digitalRead(ISO_IN5) == LOW)  {
  digitalWrite(OUT5, HIGH);
}
else {
  digitalWrite(OUT5, LOW);
}

 if (BMS_Status != 2) {

BMS_Status = 2;

    }

      // Check for ISO_IN7 ignition inactivity
   if ((digitalRead(ISO_IN1) == LOW) && (digitalRead(ISO_IN7) == HIGH))
      {
     if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        BMS_Status = 1;
        VCUstatus = 2;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
      }

    break;
  }

  case driveReverse:
  {
    if ((dir_FWD == 1) && (dir_REV == 1))
    {

      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = 3;
        digitalWrite(OUT4, LOW);  // Turn off Barke Lights
        digitalWrite(OUT12, LOW); // Turn off Rev Lights
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }

    if ((dir_FWD == 0) && (dir_REV == 1))
    {

      if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        VCUstatus = 4;
        digitalWrite(OUT12, LOW); // Turn off Rev Lights
      }
      else
      {
        VCUstatusChangeCounter++;
      }
    }

    readPedal();
    BMS_keyOn = 1;             // enable the inverter
    digitalWrite(OUT11, LOW); // Voltage light during during precharge off
    digitalWrite(OUT10, LOW); // Error light off during precharge  
    digitalWrite(OUT12, HIGH); // Turn on the reversing lights

      if (fanState == 0)
      {
        digitalWrite(OUT6, HIGH);
        fanState = 1;
      }

    if (throttlePosition > 5)
    {
      torqueRequest = throttlePosition * 5; // lets make the pedal less responsive *6
      inverterFunction = 0x03;
      if (motorRPM < -2000)
      {
        torqueRequest = 0;
      }
    }

    if (brake_pedal == 0)
    {
      torqueRequest = 0; // 0 Torque if the brake is presed
    }
Speedo(); // speedo 
inverterComms();
HeaterComms();

   if (digitalRead(ISO_IN5) == LOW)  {
  digitalWrite(OUT5, HIGH);
}
else {
  digitalWrite(OUT5, LOW);
}


    if (BMS_Status != 2) {
BMS_Status = 2;
    }

      // Check for ISO_IN7 ignition inactivity
    if ((digitalRead(ISO_IN4) == LOW) && (digitalRead(ISO_IN7) == HIGH))
      {
     if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        BMS_Status = 1;
        VCUstatus = 2;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
          }

    break;
  }
  case charging:
  {
    inverterFunction = 0x00;      // disable the inverter
    if (digitalRead(OUT1) == LOW) // Check if Neg Contactor is off then start the sequence
    {

      digitalWrite(OUT1, HIGH); // Negative Contactor
      Serial.println("Negative On!...");
      pretimer1 = millis();
    }
    if (pretimer1 + 500 < millis())
    {
      if ((digitalRead(OUT2) == LOW) && (digitalRead(OUT3) == LOW))
      {
        digitalWrite(OUT2, HIGH);
        Serial.println("PreCharge On!...");
        pretimer1 = millis();
        pretimer2 = millis();
      }
    }
    if (pretimer1 + 1000 < millis())
    {
          if ((chargerHVbattryVolts >= motorHVbatteryVolts - 5.0) && (digitalRead(OUT3) == LOW)) // Check that the preCharge has finished
      {
        digitalWrite(OUT3, HIGH); // Turn on drive Contactor
        Serial.println("Main On!...");
        pretimer1 = millis();
      }
    }

    if ((pretimer1 + 500 < millis()) && (digitalRead(OUT2) == HIGH) && (digitalRead(OUT3) == HIGH))
    {
      digitalWrite(OUT2, LOW);
      Serial.println("PreCharge OFF!...");
      digitalWrite(OUT8, HIGH); // DC-DC Enable on
      Serial.println("DC-DC Enabled...");
      Serial.print("VCU Status: ");
      Serial.println(VCUstatus);
      Serial.print("Precharge Time: ");
      Serial.println(preChargeRiseTime);
      pretimer1 = 0;
    }
    if ((digitalRead(OUT3) == LOW) && (digitalRead(OUT2) == HIGH))
    {

      if (chargerHVbattryVolts >= motorHVbatteryVolts - 5.0)
      {
        if (preChargeRiseTime == 0)
        {
          preChargeRiseTime = millis() - pretimer2;
        }
      }
    }

    // Check the contactor state
    if ((digitalRead(OUT3) == HIGH) && (digitalRead(OUT2) == LOW) && (digitalRead(OUT1) == HIGH))
    {
      if (BMS_Status == 3)
      {

        BMS_keyOn = 0; // Disable the inverter drive

        digitalWrite(OUT2, LOW); // We don't want any more smoke!
        if (preChargeRiseTime < 500)
        {
          digitalWrite(OUT10, HIGH);
        }
        if (preChargeRiseTime >= 500)
        {
          digitalWrite(OUT10, LOW); // Red LED Off
        }
        if (timer1000_1.check() == 1)
        {
          digitalWrite(OUT11, !digitalRead(OUT11)); // Flash the Green LED
        }
        if (charger12vbattryVolts < 12.6)
        {
          digitalWrite(OUT8, HIGH); // DC-DC Enable on (charge the aux battery as well..)
        }

        if (timer30s_1.check() == 1)
        { // No need to keep the DC-DC on if the battery is charged
          if (charger12vCurrent < .5)
          {
            digitalWrite(OUT8, LOW);
          }
          else 
          {
            digitalWrite(OUT8, HIGH);
          }
        }

if (digitalRead(ISO_IN5) == LOW)  {
  digitalWrite(OUT5, HIGH);
}
else {
  digitalWrite(OUT5, LOW);
}

if (chargerTempCH > fanOnTemp)
    {
      if (fanState == 0)
      {
        digitalWrite(OUT6, HIGH);
        fanState = 1;
      }
    }
    else if (chargerTempCH < (fanOnTemp - 5))
    {
      if (fanState == 1)
      {
        digitalWrite(OUT6, LOW);
        fanState = 0;
      }
    }

HeaterComms();
EvseStart(); //start charging if precharge is done and BMS is in correct state

      }
      else if (BMS_Status != 3)
      {
        if (timer1000_1.check() == 1)
        {
          Serial.println("Waiting for BMS!");
          BMS_keyOn = 0; // Disable the inverter drive
        }
      }
    }
	
	      // Check for ISO_IN8 PP inactivity
      if (digitalRead(ISO_IN8) == LOW) 
      {
     if (VCUstatusChangeCounter > VCUstatusChangeThreshold)
      {
        VCUstatusChangeCounter = 0;
        BMS_Status = 1;
        VCUstatus = 2;
      }
      else
      {
        VCUstatusChangeCounter++;
      }
      }
	
    break;
  }

  case error:
  {
    BMS_keyOn = 0;
    inverterFunction = 0x00;
    digitalWrite(OUT11, HIGH); // Voltage light during during precharge on
    digitalWrite(OUT10, HIGH); // Error light on during precharge    
    Serial.println("ERROR !!!");

    break;
  }
  }
}
