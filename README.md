# Mini-E-VCU
VCU Controller for rear inverter and motor from Mitsubishi Outlander PHEV.
This is for V2 Hardware with all opto isolated inputs

open inverter forum thread:
https://openinverter.org/forum/viewtopic.php?t=2167

Update - 29.08.22

Imporved precharge handling - added timer and voltage check to indicate possible welding or faliure of precharge resistor

Imporved DC-DC control 

Added filtering to pedal input

Bug fixes and general clean ups.

Update - 31.12.23

Adding JK BMS code 

Update - 05.1.24

Additional code to revert from each of defined VCU states back to "ready" state when "PP" or "ignition" signal goes off.

14.1.24
- Separated 0x285 telegram from all other functions. I use Intervaltimer command. 0x285 is a heartbeat from BMS and we only need one instance for all hardware to work. To do this i went and added a timer countdown at 10ms so 0x285 goes out regardless of other functions.
- Made function for heater control.
- Repaired error in CAN mailbox declarations, now CAN reading works good
- Remapped Charger control
- Corrected Metro timer function assignments so that only one timer is assigned to single function.
- Identified out two lines that govern direction of torque in code that work as i wanted. Providing negative attribute will reverse direction.
  "targetTorque = (throttlePosition * pedal_offset) * 2;"
  "torqueRequest = throttlePosition * -6;"
  
4.2.24
- Identified "Overcurrent" sensing code and comment it
- Setup serial to setup regen on the fly
- changed "torqueRequest" to 2000 in both directions
  
14.2.24
- Torque limiter changed to 2800 with no significant effect
- Throttle Map3 adapted to all around acceleration, throttle off regen set to *-1 
- Throttle Map1 adapted to slower acceleration and inhibit regen
  
9.3.24
- I removed other CAN lines other than CAN2 in code as other CAN lines would clutter interrupts on Teensy.

3.8.25
Changes made at 3rd August 25
Inputs and outputs are redefined.....
// Define Outputs
#define OUT1 6    // Ignition is started
#define OUT2 9    // PRE Charge Contactor
#define OUT3 10   // HV Drive Contactor
#define OUT4 11   // Power steering pump 
#define OUT5 12   // Heater Pump
#define OUT6 24   // FAN control
#define OUT7 25   // RPM signal
#define OUT8 28   // DC-DC Enable
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

Redefined 3 throttle maps.
Map1 is without regen for winter use. it is active if I press "sport" switch. 
Map3 is active first with full regen and sport function, i inverted sport switch function
Map2 I use only for low battery and error events. 
void turtle_mode(), this will only give small torque when battery is low. It will get signal from can bus function or external from BMS.

Charging control loop is simple: VCU looks to 0x389 id and in byte0 is HV voltage report compressed into single byte value. 
I use code comparator and when i am near my selected threshold value i slow charger down to 3A. Then when i reach threshold i stop charger. This means my charger is accurate to 2V difference... example to 388Vdc, 390Vdc or 392Vdc. But it is enough to be accurate. I am running this code for 2 years now and in several cars. I also read EVSE AC current and compare it to DC current.
