/*                                                                                                  
--------------------------------------------------------------------------------
   HARDWARE: ATMEGA328P-PU connected to common-anode 7-segment 4-digit display

                             +----------------+
                       -- 1  | RESET      PC5 | 28 -- B
                DIG1-A -- 2  | PD0        PC4 | 27 -- G
                DIG2-A -- 3  | PD1        PC3 | 26 -- A
                     D -- 4  | PD2        PC2 | 25 -- C + COL-C
                 COL-A -- 5  | PD3        PC1 | 24 -- F + APOS-C
                DIG3-A -- 6  | PD4        PC0 | 23 -- E + LIGHT SENSOR (optional)
                       -- 7  | VCC        GND | 22 --
                       -- 8  | GND       AREF | 21 --
                       -- 9  | PB6       AVCC | 20 --
                       -- 10 | PB7    SCK/PB5 | 19 --
              (DOT) DP -- 11 | PD5   MISO/PB4 | 18 --
                DIG4-A -- 12 | PD6   MOSI/PB3 | 17 -- APOS-A
                SNOOZE -- 13 | PD7        PB2 | 16 -- BUZZ2
                 ALARM -- 14 | PB0        PB1 | 15 -- BUZZ1
                             +----------------+

--------------------------------------------------------------------------------
	
    Set fuses:
    avrdude -p m328p -U lfuse:w:0xE6:m
    LFUSE: CKDIV8 CKOUT SUT1 SUT0 CKSEL3 CKSEL2 CKSEL1 CKSEL0
             1       1    1   0     0      1      1       0      -> Full Swing Crystal
    
    avrdude -p m328p -U hfuse:w:0xDF:m
    HFUSE: RSTDISBL DWEN SPIEN WDTON EESAVE BOOTSZ1  BOOTSZ0 BOOTRST
               1      1    0     1      1       1       1      1 -> min boot size (128 words)
    
    avrdude -p m328p -U efuse:w:0x01:m
    EFUSE: BODLEVEL2 BODLEVEL1 BODLEVEL0
               0         0        1	 ??????????????????

    program hex:
    avrdude -p m328p -U flash:w:clockit_24.hex

--------------------------------------------------------------------------------

    2-24-11
    by Jim Lindblom
    Layout was modified to use a new LED, used in v14 of the layout.
    New LED is common-anode, rather than common-cathode.
	
    3-4-09
    Copyright Spark Fun Electronics 2009
    Nathan Seidle
	
    A basic alarm clock that uses a 4 digit 7-segment display. Includes alarm and snooze.
    Alarm will turn back on after 9 minutes if alarm is not disengaged. 
    
    Alarm is through a piezo buzzer.
    Three input buttons (up/down/snooze)
    1 slide switch (engage/disengage alarm)
    
    Display is with PWM of segments - no current limiting resistors!
    
    Uses external 16MHz clock as time base.
*/

#ifndef __AVR_ATmega328P__
#error "Must be compiled for AVR ATmega328P"
#endif

#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <string.h>
#include "Ports.h"

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734 
#ifdef PROGMEM 
#undef PROGMEM 
#define PROGMEM __attribute__((section(".progmem.data"))) 
#endif

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Inline here means "always inline"

#define inline inline __attribute__((always_inline))

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Hardware pin mapping

// display cathodes
PortC<3> segA;
PortC<5> segB;
PortC<2> segC; // also serves as col cathode
PortD<2> segD;
PortC<0> segE; // also serves as light sensor pin
PortC<1> segF; // also servers as apos cathode
PortC<4> segG;
PortD<5> dot;

// display anodes
PortD<0> dig1;
PortD<1> dig2;
PortD<4> dig3;
PortD<6> dig4;
PortD<3> col;  
PortB<3> apos;

// buttons
PortB<5> butUp;      // PCINT5 pin
PortB<4> butDown;    // PCINT4 pin
PortD<7> butSnooze;  // PCINT23 pin
PortB<0> butAlarm;

// buzzer
PortB<1> buzz1;
PortB<2> buzz2;

// light sensor
PortC<0> lightSensor;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Basic data types

enum Mode {
	MODE_START,        // initial sequence 
	MODE_SHOW_TIME,    // Show current time (default mode)   
	MODE_SHOW_SEC,     // Show current seconds, hold UP to enter, any key to exit 
	MODE_SHOW_PARAMS,  // Show various internal parameters, hold DOWN to enter, any key to exit
	MODE_EDIT_TIME,    // Edit time, hold UP+DOWN to enter, SNOOZE to exit
	MODE_EDIT_ALARM,   // Edit alarm, hold SNOOZE to enter, SNOOZE to exit
	MODE_SHOW_CONFIG,  // Show configuration, hold DOWN+SNOOZE to enter, SNOOZE to exit
	MODE_EDIT_CONFIG,  // Edit configuration, hold SNOOZE in SHOW_CONFIG to enter, SNOOZE to exit, hold SNOOZE to confirm change
};

enum TimeSet : uint8_t { OFF, ON };

struct Time {
	TimeSet set = OFF; // ON when this time was set (it is not active until not set), 0xff for non-initialized EEPROM
	int8_t hours = 0;
	int8_t minutes = 0;
	int8_t seconds = 0;
};

typedef uint8_t Buttons;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// General constants

// Internal buttons masks
const Buttons UP = 0x01;
const Buttons DOWN = 0x02;
const Buttons SNOOZE = 0x04;

// turn on display for 10us at a single time (that is one unit of brightness)
const uint8_t DISPLAY_US = 10; 

// wakeup counters constants 
const uint8_t WAKE_ACTIVE_SEC = 100; // wake up seconds in non-default modes

// default brightness value
const uint8_t DEFAULT_BRIGNTESS = 50;

// use 3 display interrupts to change brightness +1/-1 
// so, brightness goes from 40 to 0 in 120 display interrupts ~= 2 sec
const uint8_t BRIGHTNESS_CHANGE_DISPS = 3; 

const uint32_t DAY_SEC = 24L * 60 * 60; // seconds in day

// hardware timing

// TIMER0 is configured with clk/8 prescaler and should reset every 100us (10Khz)
const uint8_t TIMER0_TOP = 199;
const uint16_t TIMER0_PER_MS = 10; // timer0 resets every 100us 

// TIMER1 is configured with clk/256 prescaler and should reset every second (1Khz)
const uint16_t TIMER1_FREQ = 62500;  

// TIMER2 is configured with 
const uint16_t DISP_PER_SEC = 61; // number of display interrupt firings per second

// button timings
const uint16_t BUTTON_DELAY_MS = 100; // ms to detect repeated button presses and to debounce
const uint16_t BUTTON_LONG_MS = 1000; // ms to for "long" press of a button to enter settings

// other
const uint16_t VOLTAGE_THRESHOLD = 230; // 1.1V ref / 4.9 min * 1024 -> when voltage below 4.9V don't wake up on light

// display chars
const char DISPLAY_MASK = 0x7f;
const char DISPLAY_DOT = 0x80;

enum DisplayItem { D_1, D_2, D_3, D_4, D_COL_APOS, D_COUNT };

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Declare global variables

// Current mode
Mode mode;
uint16_t wake_countdown; // in display interrupts (see DISP_PER_SEC) increments
uint8_t brightness_change_countdown; // in display interrupts (see DISP_PER_SEC) increments

uint8_t brightness; // the most recent display brightness
uint8_t target_brightness; // target display brightness level
bool display_off = false; // for blinking
char display[D_COUNT]; // the six characters to display

uint8_t alarm_going_countdown; // in 1s increments

volatile uint16_t delay_countdown; // in 100us increments
uint16_t siren_going_countdown; // in 100us increments
uint8_t siren_phase; // 0..5

uint8_t measure_voltage;
uint8_t light_level;

Time time;   // current clock time
Time alarm;  // current alarm time
Time snooze; // next time to alarm in snooze mode
Time edit;   // time that is being edited when mode == MODE_EDIT_TIME/ALARM

// current edited config item when mode == MODE_EDIT_CONFIG
uint8_t configItem;

// sling-shot state (editing via continuous button press)
int8_t sling_shot;
int8_t change;
Buttons previous_button;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Configuration

// Config items
const uint8_t F_ADJUST = 0;           // +/- 100ms per day
const uint8_t F_CLOCK_MODE = 1;       // 12 or 24
const uint8_t F_NIGHT_MODE = 2;       // 1 to turn off display at night
const uint8_t F_LIGHT_THRESHOLD = 3;  // threshold of light between day and night
const uint8_t F_DAY_BRIGHTNESS = 4;   // brightness level for day (also used when editing settings)
const uint8_t F_NIGHT_BRIGHTNESS = 5; // brightness level for night
const uint8_t F_ALARM_SEC = 6;        // number of seconds alarm works
const uint8_t F_SNOOZE_MIN = 7;       // number of minutes to snooze
const uint8_t F_WAKE_BUTTON_SEC = 8;  // number of seconds to be awake after button press
const uint8_t F_WAKE_LIGHT_SEC = 9;   // number of seconds to be awake after light detection (when light goes off)
const uint8_t F_COUNT = 10;           // number of config items

// Default configuration
const uint8_t PROGMEM PGM_CONFIG_DEFAULT[F_COUNT] = {
	0,  // F_ADJUST
	24, // F_CLOCK_MODE
	0,  // F_NIGHT_MODE
	0,  // F_LIGHT_THRESHOLD
	DEFAULT_BRIGNTESS, // F_DAY_BRIGHTNESS
	DEFAULT_BRIGNTESS, // F_NIGHT_BRIGHTNESS
	30, // F_ALARM_SEC
	9,  // F_SNOOZE_MIN
	5,  // F_WAKE_BUTTON_SEC
	2   // F_WAKE_LIGHT_SEC
};

// Configuration in EEPROM
struct EEConfig {
	Time time;  // initial time for future extensions, not modified
	Time alarm; // alarm time
	uint8_t nConfig; // number of config items or 0xff for fresh EEPROM
	uint8_t config[F_COUNT];
};

EEConfig EEMEM eeConfig; // configuration stored in EEPROM
uint8_t config[F_COUNT]; // current configuration in RAM (copied from EEPROM on start)

// convenient references to config items
const int8_t& ADJUST = config[F_ADJUST];
const uint8_t& CLOCK_MODE = config[F_CLOCK_MODE];
const uint8_t& NIGHT_MODE = config[F_NIGHT_MODE];
const uint8_t& LIGHT_THRESHOLD = config[F_LIGHT_THRESHOLD];
const uint8_t& DAY_BRIGHTNESS = config[F_DAY_BRIGHTNESS];
const uint8_t& NIGHT_BRIGHTNESS = config[F_NIGHT_BRIGHTNESS];
const uint8_t& ALARM_SEC = config[F_ALARM_SEC];
const uint8_t& SNOOZE_MIN = config[F_SNOOZE_MIN];
const uint8_t& WAKE_BUTTON_SEC = config[F_WAKE_BUTTON_SEC];
const uint8_t& WAKE_LIGHT_SEC = config[F_WAKE_LIGHT_SEC];

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// General functions

template<typename T> inline T max(T a, T b) {
	return a > b ? a : b;
} 

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Config functions

int8_t fixConfigValue(uint8_t configItem, int8_t value) {
	switch (configItem) {
		case F_ADJUST: // -99 to +99
			if (value < -99) return -99;
			if (value > 99) return 99;
			return value;
		case F_CLOCK_MODE: // 12 or 24
			if (value < 12) return 24;
			if (value == 12) return 12;
			if (value < 18) return 24;
			if (value < 24) return 12;
			if (value == 24) return 24;
			return 12;
		case F_NIGHT_MODE: // 0 or 1
			if (value < 0) return 1;
			if (value > 1) return 0;
			return value;
		case F_LIGHT_THRESHOLD: // 0 to 99
			if (value < 0) return 0;
			if (value > 99) return 99;
			return value;
		default: // 1 to 99
			if (value < 1) return 1;
			if (value > 99) return 99;
			return value;
	}
}

inline void readConfig() {
	// read EEPROM items
	eeprom_read_block(&time, &(eeConfig.time), sizeof(time));
	if (time.set != 1 && time.set != 0)
		time = Time(); // clear on invalid value
	eeprom_read_block(&alarm, &(eeConfig.alarm), sizeof(alarm));
	if (alarm.set != 1 && alarm.set != 0)
		alarm = Time(); // clear on invalid value
	uint8_t nConfig = eeprom_read_byte(&(eeConfig.nConfig));
	if (nConfig > 0 && nConfig <= F_COUNT) {
		eeprom_read_block(&config, &(eeConfig.config), nConfig);
		for (uint8_t i = 0; i < nConfig; i++)
			config[i] = fixConfigValue(i, config[i]); 
	} else
		nConfig = 0; // assume no stored config -- all default
	// read defaults if needed
	for (uint8_t i = nConfig; i < F_COUNT; i++)
       config[i] = pgm_read_byte_near(&PGM_CONFIG_DEFAULT[i]);
}

inline void writeConfigAlarm() {
	eeprom_write_block(&alarm, &(eeConfig.alarm), sizeof(alarm));
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Time functions

// call it with disabled interrupts to ensure atomicity
void fixTimeI(Time& t) {
	if (t.seconds >= 60) {
		t.seconds -= 60;
		t.minutes++;
	} else if (t.seconds < 0) {
	 	t.seconds += 60;
		t.minutes--;
	}
	if (t.minutes >= 60) {
		t.minutes -= 60;
		t.hours++;
	} else if (t.minutes < 0) {
		t.minutes += 60;
		t.hours--;
	}
	if (t.hours >= 24) 
		t.hours -= 24;
	else if (t.hours < 0) 
		t.hours += 24;
}

// Compares two times, both times must be set for comparison to be true
bool operator == (Time& t1, Time& t2) {
	return t1.set && t2.set && t1.hours == t2.hours && t1.minutes == t2.minutes && t1.seconds == t2.seconds;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Button functions

Buttons getButtons() {
	Buttons b = 0;
	if (!butUp)
		b |= UP;
	if (!butDown)
		b |= DOWN;
	if (!butSnooze)
		b |= SNOOZE;
	return b;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Siren based on TIMER0 OCR0A 

// call it with interrupts disabled 
inline void sirenI(uint16_t ms) {
	siren_going_countdown = TIMER0_PER_MS * ms; // convert to 100us units
	siren_phase = 0;
	sbi(TIMSK0, OCIE0A); // enable TIMER0_COMPA interrupt
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Control delays based on TIMER0 OCR0B 

inline void delayStartI(uint16_t ms) {
	delay_countdown = TIMER0_PER_MS * ms; // convert to 100us units
	OCR0B = TCNT0; // trigger TIMER0_COMPB in 100us when TCNT0 reaches the same value again
	cbi(TIFR0, OCF0B); // clear TIMER0_COMPB interrupt flag
	sbi(TIMSK0, OCIE0B); // enable interrupt on TIMER0_COMPB
}

inline void delayStopI() {
	cbi(TIMSK0, OCIE0B); // disable interrupt on TIMER0_COMPB
}

inline void delay(uint16_t ms) {
	cli();
	delayStartI(ms);
	while (delay_countdown != 0) {
		sei();
		sleep_cpu();
		cli();
	}	
	delayStopI();
	sei();
}

inline bool pressedButtons(Buttons b, uint16_t ms) {
	cli();
	delayStartI(ms);
	while (delay_countdown != 0 && getButtons() == b) {
		sei();
		sleep_cpu();
		cli();
	}
	delayStopI();
	sei();
	return delay_countdown == 0;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/* 
   Display segments layout:

          -- A --
         |       |
         F       B
         |       |
          -- G --
         |       |
         E       C
         |       |
          -- D --

*/

// Display one character for DISPLAY_US time
inline void displayItemI(DisplayItem i, char c) {
	if (c == 0)
		return; // nothing to display
		
	// Float all segment pins (turn to input mode)
	(segA | segB | segC | segD | segE | segF | segG | dot).input();

	// drive to zero 0 only on segments we want to turn on
	switch(c & DISPLAY_MASK) {
		case '0': (segA | segB | segC | segD | segE | segF).output(); break;
		case '1': (segB | segC).output(); break;
		case '2': (segA | segB | segG | segE | segD).output(); break;
		case '3': (segA | segB | segG | segC | segD).output(); break;
		case '4': (segF | segB | segG | segC).output(); break;
		case '5': (segA | segF | segG | segC | segD).output(); break;
		case '6': (segA | segF | segE | segD | segC | segG).output(); break;
		case '7': (segA | segB | segC).output(); break;
		case '8': (segA | segB | segC | segD | segE | segF | segG).output(); break;
		case '9': (segA | segB | segG | segF | segC | segD).output(); break;
		case 'A': (segA | segF | segB | segG | segE | segC).output(); break;
		case 'F': (segA | segF | segG | segE).output(); break;
		case 'L': (segF | segE | segD).output(); break;
		case '-': segG.output(); break;
		case ':': segC.output(); break; // Note: colon is only used at D_COL_APOS
	}
	// drive dot to zero if needed, too
	if (c & DISPLAY_DOT) {
		if (i == D_COL_APOS) 
			segF.output(); // dot here is apostrophe on segF
		else 
			dot.output();
	}

	// now power ON the corresponding anode (set it to one)
	switch(i) {
		case D_1: dig1.set(); break;
		case D_2: dig2.set(); break;
		case D_3: dig3.set(); break;
		case D_4: dig4.set(); break;
		case D_COL_APOS: (col | apos).set(); break;
		default: break; // should not happen, but need to avoid "not handled" warning
	}
	// wait a bit (busy wait)
	_delay_us(DISPLAY_US);
	// clear display -- set all anodes to zero
	(dig1 | dig2 | dig3 | dig4 | col | apos).clear();
	// after that can set all segments to output zero 
	(segA | segB | segC | segD | segE | segF | segG | dot).output();
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Various display modes

inline void setDisplayStartI() {
	display[D_1] = '8' | DISPLAY_DOT;
	display[D_2] = '8' | DISPLAY_DOT;
	display[D_3] = '8' | DISPLAY_DOT;
	display[D_4] = '8' | DISPLAY_DOT;
	display[D_COL_APOS] = ':' | DISPLAY_DOT;
}

void setDisplayTimeI(const Time& t) {
	int8_t h = t.hours;
	bool am = false;
	if (CLOCK_MODE == 12) {
		am = h < 12;
		if (!am)
			h -= 12;
		if (h == 0)
			h = 12;
	}
	display[D_1] = t.hours >= 10 ? '0' + h / 10 : 0;
	display[D_2] = '0' + h % 10;
	display[D_3] = '0' + t.minutes / 10;
	display[D_4] = ('0' + t.minutes % 10);
	display[D_COL_APOS] = (t.seconds & 1) == 0 ? ':' : ' ';
	if (am) 
		display[D_COL_APOS] |= DISPLAY_DOT;
	if ((mode == MODE_SHOW_TIME && butAlarm && alarm.set) || mode == MODE_EDIT_ALARM)
		display[D_4] |= DISPLAY_DOT;
}

inline void setDisplaySecI() {
	display[D_COL_APOS] = (time.seconds & 1) == 0 ? ':' : ' ';
	display[D_3] = '0' + time.seconds / 10;
	display[D_4] = '0' + time.seconds % 10;
}

inline void setDisplayLightLevelI() {
	display[D_1] = 'L';
	display[D_3] = '0' + light_level / 10;
	display[D_4] = '0' + light_level % 10;
}

inline void setDisplayAdjustI(bool editConfig) {
	int8_t adj = ADJUST;
	display[D_1] = 'A';
	if (!editConfig || ((time.seconds & 1) == 0))
		display[D_1] |= DISPLAY_DOT; // blink dot when editing config
	if (adj < 0) {
		adj = -adj;
		display[D_2] = '-';
	}
	display[D_3] = ('0' + adj / 10) | DISPLAY_DOT;
	display[D_4] = '0' + adj % 10;
}

inline void setDisplayConfigItemI(bool editConfig) {
	display[D_1] = 'F';
	display[D_2] = ('0' + configItem);
	if (!editConfig || ((time.seconds & 1) == 0))
		display[D_2] |= DISPLAY_DOT; // blink dot when editing config
	uint8_t cfg = config[configItem];
	display[D_3] = '0' + cfg / 10;
	display[D_4] = '0' + cfg % 10;
}

// Called from various ISRs and when mode variables change
// must be protected by cli/sti if called outside of ISR
void updateDisplayI() {
	display_off = false; // display is ON by default
	memset(&display, 0, sizeof(display)); // clear by default
	target_brightness = DAY_BRIGHTNESS; // day brightness by default (overridden in time display mode and start mode)
	switch (mode) {
		case MODE_START:
			setDisplayStartI();
			target_brightness = DEFAULT_BRIGNTESS;
			break;
		case MODE_SHOW_TIME: 
			setDisplayTimeI(time);
			// Blink display every second in wake mode when time is not set yet
			if (!time.set &&  (time.seconds & 1) != 0)
				display_off = true;
			target_brightness = light_level >= LIGHT_THRESHOLD ? DAY_BRIGHTNESS : NIGHT_BRIGHTNESS;
			// in night mode display is (gradually!) turned off when wake countdown reaches zero
			if (NIGHT_MODE && wake_countdown == 0)
				target_brightness = 0;
			break;
		case MODE_SHOW_SEC:
			setDisplaySecI();
			break;
		case MODE_SHOW_PARAMS:
			setDisplayLightLevelI();
			break;
		case MODE_EDIT_TIME:
			setDisplayTimeI(edit);
			break;
		case MODE_EDIT_ALARM:
			setDisplayTimeI(edit);
			break;
		case MODE_SHOW_CONFIG:
		case MODE_EDIT_CONFIG:
			bool editConfig = mode == MODE_EDIT_CONFIG;
			if (configItem == F_ADJUST)
				setDisplayAdjustI(editConfig);
			else
				setDisplayConfigItemI(editConfig);
			break;
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Wakes from sleep

// NOTE: Invoked from ISRs only when interrupts are disabled
void wakeupOnButtonI() {
	switch (mode) {
		case MODE_SHOW_TIME:
			wake_countdown = max(wake_countdown, WAKE_BUTTON_SEC * DISP_PER_SEC);
			break;
		default: // other (non-default, active) modes
			wake_countdown = max(wake_countdown, WAKE_ACTIVE_SEC * DISP_PER_SEC);
	}
}

// NOTE: Invoked from ISRs only when interrupts are disabled
void wakeupOnLightI() {
	switch (mode) {
		case MODE_SHOW_TIME:
			wake_countdown = max(wake_countdown, WAKE_LIGHT_SEC * DISP_PER_SEC);
			break;
		default: // other (non-default, active) modes
			break; // nothing -- light does not affect active mode
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Check to see if the time is equal to the alarm time and make alarm sounds
// Called every second from timer ISR
inline void checkAlarmI() {
	//Check whether the alarm slide switch is on or off and we're in normal mode (not setting anything)
	if (mode == MODE_SHOW_TIME && butAlarm) {
		if (alarm_going_countdown == 0) {
			//Check to see if the time equals the alarm time (== operator ensures that both are set)
			if (time == alarm) 
				alarm_going_countdown = ALARM_SEC; //Set it off!
			//Check to see if we need to set off the alarm again after a ~9 minute snooze
			if (time == snooze)
				alarm_going_countdown = ALARM_SEC; //Set it off!
		}                                                                                                                
		if (alarm_going_countdown > 0) {
			alarm_going_countdown--;
			wakeupOnLightI(); // wake up on alarm as if light detected
			sirenI(300); // start siren for 300ms
		}
	} else {
		alarm_going_countdown = 0;
		snooze.set = OFF; //If the alarm switch is turned off, this resets the ~9 minute additional snooze timer
	}
}

// Checks snooze button on pin change to turn off alarm
// NOTE: Invoked from ISR (interrupts off)
inline void checkSnoozeI() {
	//If the user hits snooze while alarm is going off, record time so that we can set off alarm again in SNOOZE_MIN minutes
	if (butSnooze && alarm_going_countdown > 0) {
		alarm_going_countdown = 0; //Turn off alarm
		snooze.set = ON; //But remember that we are in snooze mode, alarm needs to go off again in a few minutes
		snooze.seconds = 0;
		snooze.minutes = time.minutes + SNOOZE_MIN; //Snooze in a few minutes from now
		snooze.hours = time.hours;
		fixTimeI(snooze);
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// SOUND INTERRUPT -- called every 100us (enabled by siren_ms function)
// Generate 1/600us ~ 1666 Hz sound
ISR (TIMER0_COMPA_vect) {
	if (siren_going_countdown-- == 0) {
		// no siren
		buzz1.clear();
		buzz2.clear();
		cbi(TIMSK0, OCIE0A); // no more TIMER0_COMPA interrupt
		return;	
	}
	if (siren_phase < 3) {
		// 0...2 == 300us half wave
		buzz1.clear();
		buzz2.set();
	} else {
		// 3...5 == 300us the other half
		buzz1.set();
		buzz2.clear();
	}
	if (++siren_phase >= 6)
		siren_phase = 0;
}

// CONTROL delay interrupt -- called every 100us (enabled by delayStart function)
ISR (TIMER0_COMPB_vect) {
	if (delay_countdown != 0)
		delay_countdown--;
}

// CLOCK INTERRUPT -- called every second
ISR (TIMER1_OVF_vect) {
	time.seconds++;
	fixTimeI(time);
	checkAlarmI();
	updateDisplayI();
}

// DISPLAY INTERRUPT -- called every ~16ms
ISR (TIMER2_OVF_vect) {
	// Now move brightness into direction of target brightness
	if (++brightness_change_countdown >= BRIGHTNESS_CHANGE_DISPS) {
		if (brightness > target_brightness)
			brightness--;
		else if (brightness < target_brightness)
			brightness++;
		brightness_change_countdown = 0;
	}

	// wakeup timeout countdown
	if (wake_countdown > 0)
		wake_countdown--;
	
	// Display is around during BRIGHT_LEVEL * DISPLAY_US * 6 us
	// For BRIGHT_LEVEL = 50 that is equal to 3 ms
	// It uses BUSY WAIT to measure time (interrupts are disabled during display routine)

	// do only if display is ON
	if (!display_off) {
		// the actual display loop
		for (uint8_t k = 0; k < brightness; k++)
			for (uint8_t i = 0; i < D_COUNT; i++) {
				displayItemI((DisplayItem)i, display[i]);
				// let other interrupts (sound) be processed
				sei();
				__asm__ ( "nop" );
				cli();
			}
	}

	// measure voltage using internal reference two times (first reading will be wrong)
	measure_voltage = 2;
	cbi(PRR, PRADC); // power on ADC
	ADMUX = 0x0E; // Measure 1.1V reference
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // enable & start, prescaler = clk/128 (125KHz)
}

// ADC INTERRUPT
ISR (ADC_vect) {
	if (measure_voltage) {
		// was measuring voltage
		if (--measure_voltage > 0) {
			// measure it again
			ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // enable & start, prescaler = clk/128 (125KHz)
			return;	
		}
		// check this measurement
		if (ADC < VOLTAGE_THRESHOLD) {
			// now start measuring ambient light level
			lightSensor.input(); // configure light sensor pin as INPUT
			lightSensor.set(); // enable pull up on light sensor pin
			ADMUX = lightSensor.pin(); // use AREF, measure LIGHT_SENSOR pin
			ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // enable & start, prescaler = clk/128 (125KHz)
			return; // wait for next interrupt
		} else {
			light_level = 0; // don't measure any light, work as if no light when power is low
		}
	} else {
		// was measuring light
		light_level = ((uint32_t)(1023 - ADC) * 100) >> 10;
		if (light_level >= LIGHT_THRESHOLD) {
			wakeupOnLightI();
		}
		// reconfigure back
		lightSensor.clear(); // disable pull up on light sense pin
		lightSensor.output(); // configure light sense pin as OUTPUT
	}
	// done with measurements
	sbi(PRR, PRADC); // power off ADC
}

// SNOOZE BUTTON INTERRUPT -- called on snooze button press                                                          R
ISR (PCINT2_vect) {
	checkSnoozeI();
	wakeupOnButtonI();
	updateDisplayI();
}

// UP/DOWN BUTTONS INTERRUPT -- called on up or down button press
ISR (PCINT0_vect) {
	wakeupOnButtonI();
	updateDisplayI();
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Control logic

void processRamp(Buttons b, int8_t maxChange) {
	//Ramp minutes faster if we are holding the button
	if (previous_button == b) {
		sling_shot++;
	} else {
		sling_shot = 0;
		change = 1;
	}
	previous_button = b;
	if (sling_shot > 5) {
		change++;
		if (change > maxChange) {
			change = maxChange;
		}
		sling_shot = 0;
	}
}

void resetRamp() {
	sling_shot = 0;
	change = 1;
	previous_button = 0;
}

void waitButtonsRelease() {
	while (getButtons() != 0)
	sleep_cpu();
	delay(BUTTON_DELAY_MS);
}

void shortBeep() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		sirenI(100);
	}
}

void startMode(Mode set_mode) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		mode = set_mode;
		updateDisplayI();
	}
}

void restoreShowTimeMode() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		mode = MODE_SHOW_TIME;
		wake_countdown = 0; // reset wake countdown to zero when exiting  mode (it was big)
		wakeupOnButtonI(); // set wake_countdown for MODE_SHOW_TIME
		updateDisplayI();
	}
}

int8_t getButtonDir(Buttons b) {
	switch (b) {
		case UP: return 1;
		case DOWN: return -1;
		default: return 0;
	}
}

void modeEditTimeOrAlarm(Time& t, Mode set_mode) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		edit = t;
		edit.seconds = 0; // reset seconds to zero (!), also shows colon while editing
	}
	shortBeep();
	startMode(set_mode);
	waitButtonsRelease();
	resetRamp();
	while (true) {
		if (wake_countdown == 0) {
			// no action for too long -- give short siren and quit edit mode
			restoreShowTimeMode();
			shortBeep();
			return;
		}
		Buttons b = getButtons();
		switch (b) {
			case SNOOZE: //All done!
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					t = edit;
					t.set = ON; // time is now set
				}
				shortBeep();
				restoreShowTimeMode();
				waitButtonsRelease();
				return; // done setting -- quit loop
			case UP:
			case DOWN:
				if (pressedButtons(b, BUTTON_DELAY_MS)) {
					processRamp(b, 30);
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
						edit.minutes += change * getButtonDir(b);
						fixTimeI(edit);
						updateDisplayI();
					}
					break;
				}
				resetRamp();
				break;
			default:
				resetRamp(); // when no button pressed or combo pressed
		}
	}
}

inline void modeShowSec() {
	shortBeep();
	startMode(MODE_SHOW_SEC);
	waitButtonsRelease();
	while (true) {
		if (wake_countdown == 0 || getButtons() != 0) {
			// no action for too long or button pressed
			shortBeep();
			restoreShowTimeMode();
			waitButtonsRelease();
			return;
		}
	}
}

inline void modeShowParams() {
	shortBeep();
	startMode(MODE_SHOW_PARAMS);
	waitButtonsRelease();
	while (true) {
		if (wake_countdown == 0 || getButtons() != 0) {
			// no action for too long or button pressed
			shortBeep();
			restoreShowTimeMode();
			waitButtonsRelease();
			return;
		}
	}
}

inline bool modeEditConfig() {
	startMode(MODE_EDIT_CONFIG);
	shortBeep();
	waitButtonsRelease();
	resetRamp();
	uint8_t savedValue = config[configItem];
	while (true) {
		if (wake_countdown == 0) {
			// no action for too long -- give short siren and quit mode
			shortBeep();
			restoreShowTimeMode();
			config[configItem] = savedValue;
			return false; // bail out
		}
		Buttons b = getButtons();
		switch (b) {
			case SNOOZE:
				shortBeep();
				if (pressedButtons(SNOOZE, BUTTON_LONG_MS)) {
					// long press -- update item
					shortBeep();
				} else {
					// short press -- restore old value & quit
					config[configItem] = savedValue;
				}
				startMode(MODE_SHOW_CONFIG);
				waitButtonsRelease();
				return true; // done setting -- quit loop
			case UP:
			case DOWN:			
				if (pressedButtons(b, BUTTON_DELAY_MS)) {
					processRamp(b, 5);
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
						int8_t prevValue = config[configItem];
						config[configItem] = fixConfigValue(configItem, prevValue + change * getButtonDir(b));
						updateDisplayI();
					}
				}
				break;					
			default:
				resetRamp(); // when no button pressed or combo pressed
		}
	}
}

inline void modeShowConfig() {
	startMode(MODE_SHOW_CONFIG);
	shortBeep();
	waitButtonsRelease();
	while (true) {
		if (wake_countdown == 0) {
			// no action for too long -- give short siren and quit mode
			restoreShowTimeMode();
			return;
		}
		Buttons b = getButtons();
		switch (b) {
			case SNOOZE: 
				shortBeep();
				if (pressedButtons(SNOOZE, BUTTON_LONG_MS)) {
					// long press -- edit item
					if (!modeEditConfig())
						return; // bail out
				} else {
					// short press -- quit
					restoreShowTimeMode();
					waitButtonsRelease();
					return; // done setting -- quit loop
				}
			case UP:
			case DOWN:
				if (pressedButtons(b, BUTTON_DELAY_MS))
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
						configItem = (configItem + F_COUNT + getButtonDir(b)) % F_COUNT;
						updateDisplayI();
					}
				break;
		}
	}
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Time adjustment formula

void adjustFreqI() {
	/*
	  Let f be Timer1 true frequency (16MHz clock / 256 prescler ~= 62500 Hz)
	  Let T0 be the Timer1 assumed frequency (T0 == TIMER1_FREQ == 62500)
  	    day0 = DAY_SEC * T0 / f in seconds --- measured day length (86 400 seconds per day)
	    adjust = DAY_SEC * (T0 / f - 1) in seconds
	      adjust > 0 if it takes more than a day (runs behind and needs speed up)
		  adjust < 0 if it takes less than a day (runs faster and needs slow down)
	  Therefore:
	    adjust / DAY_SEC = T0 / f - 1
	    adjust / DAY_SEC + 1 = T0 / f
		f = T0 / (adjust / DAY_SEC + 1)
		f = DAY_SEC * T0 / (adjust + DAY_SEC)
	  f - is a desired TOP value, 
	*/
	// compute top value
	uint16_t f = uint64_t(DAY_SEC) * TIMER1_FREQ / (ADJUST + DAY_SEC); 
	OCR1A = f - 1; // Set the top value at frequency - 1
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Main 

int main(void) {
	// Read configuration
	readConfig();

	// All segments, digits and buzzer output at ground (0) by default
	// That minimizes power consumption when display is not used 
	(segA | segB | segC | segD | segE | segF | segG | dot | dig1 | dig2 | dig3 | dig4 | col | apos | buzz1 | buzz2).output();

	// Pull up all buttons
	(butUp | butDown | butAlarm | butSnooze).set();

	// Init Timer0 for sound generation and logic control
	// Use CTC mode (clear counter on reaching OCR0A), set Prescaler to clk/8 (1click = 0.5us)
	TCCR0B = _BV(CS01); 
	TCCR0A = _BV(WGM01); 
	OCR0A = TIMER0_TOP; // will clear every 100us

	// Init Timer1 for second counting
	// Use fast PWM mode with TOP at OCR1A (double buffered), set prescaler to clk/256
	TCCR1A |= _BV(WGM11) | _BV(WGM10); 
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
	TIMSK1 = _BV(TOIE1); // Enable overflow interrupts
	adjustFreqI(); // compute and set OCR1A taking ajustment into account
	
	// Init Timer2 for updating the display via interrupts
	// TCNT2 should overflow every 16.384 ms (256 * 64us)
	// Set prescaler to clk/1024 : 1 click = 64us
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
	TIMSK2 = _BV(TOIE2);

	// Enable Pin Change Interrupt #0 for up and down buttons and #2 for snooze button
	PCICR = _BV(PCIE2) | _BV(PCIE0);
	PCMSK0 = _BV(PCINT4) | _BV(PCINT5);
	PCMSK2 = _BV(PCINT23);

	// Configure power reduction (disable unused hardware)
	PRR = _BV(PRTWI) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC);

	// Initially wakeup as if button was pressed
	wakeupOnButtonI(); 
	sirenI(1000); //Make some noise at power up for 1 sec
	sleep_enable();
	
	// initial display (will increase brightness from zero to default)
	mode = MODE_START;
	updateDisplayI();
	
	// Init done-- enable interrupts
	sei(); 
	
	delay(2000); // show start sequence
		
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		mode = MODE_SHOW_TIME;
		updateDisplayI();
	}

	// Main loop
	while (1) { // Continuously check if we need to set the time or snooze
		// sleep until interrupted
		sleep_cpu();
		// check buttons
		Buttons b = getButtons();
		if (b == 0)
			continue; // no buttons pressed
		if (!pressedButtons(b, BUTTON_LONG_MS))
			continue; // not held long enough
		switch (b) {
			case UP:
				modeShowSec();
				break;
			case DOWN:
				modeShowParams();
				break;
			case DOWN | SNOOZE:
				modeShowConfig();
				break;
			case UP | DOWN:
				modeEditTimeOrAlarm(time, MODE_EDIT_TIME);
				break;
			case SNOOZE:
				modeEditTimeOrAlarm(alarm, MODE_EDIT_ALARM);
				break;		
		}
	}
	return 0;
}
