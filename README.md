ClockIt MOD
===========

This is a software (and an optional hardware) mod to the SparkFun ClockIt Kit:
https://www.sparkfun.com/products/10930

Handware is based on ATMEGA328P-PU running and 16MHz crystal,
connected to common-anode 7-segment 4-digit display, three pusbuttons (up, down, and snooze)
and one switch for alarm on/off.

Features
--------

* 24h and 12h (am/pm) clock modes (NEW!).
* Time blinks when time is not set when clock was reset due to power loss (NEW!).
* Alarm with snooze.
* Adjust clock speed in units of +/- 0.1s per day for ultra-precise time keeping (NEW!).
* Optinal support for resistive light sensor with ability to dim or turn off display at night (NEW!).
* Fully configurable display brightness, alarm duration, snooze duration, etc (NEW!).

Usage
-----

In regular mode display shows "HH:MM" (colon blinks). Alarm dot if on if alarm is set and alram switch is on.

* Press and hold UP+DOWN for 1 second to set time. 
** There is a short beep and display shows "HH:MM" (colon does not blink). Alarm apostrophe is off.
** Press UP/DOWN to change time. Press and hold for faster change.
** Press SNOOZE to confirm time. There is a short confirmation beep, seconds are reset to zero, clock is back to regular mode.

* Press and hold SNOOZE for 1 second to set alarm time.
** There is a short beep and display shows "HH:MM" (colon does not blink). Alarm apostrophe is on.
** Press UP/DOWN to change alarm time. Press and hold for faster change.
** Press SNOOZE to confirm alarm time, there is a short confirmation beep, clock is back to regular mode.

* Press and hold UP for 1 second to display seconds.
** There is a short beep and display shows "  :MM" (colon blinks).
** Press any button to exit back to regular mode.

* Press and hold DOWN for 1 second to display light level.
** There is a short been and display shows "L  XX", where XX is a a light level from 0 to 99.
** Press any button to exit back to regular mode.

* Press and hold DOWN+SNOOZE for 1 second to configure advanced functions.
** There is a short beep and display shows "A X.X"/"A-X.X" for time adjustment value (Function 0) or
   "FX.YY", where "X" denotes a funciton number from 1 to 99, "YY" denotes a function value.
** Press UP/DOWN to select function.
** Press and hold SNOOZE for 1 second to edit function value. 
** There are two short beeps and dot start to blink.
** Press UP/DOWN to change function value.
** Press and hold SNOOZE for 1 second to confirm change, clock goes back to function selection.
** Press shooze for less than 1 second to exit back to regular mode.
