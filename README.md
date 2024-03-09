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
