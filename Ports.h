/*
  Utilitity header file for type-safe maniputation of data ports on 8-bit AVR MCUs (ATTiny and ATMega series)
  Use to define variables that correspond to connected perepherials. For example:

	PortB<3> button1;
	PortC<4> led1;
	PortC<5> led2;

  Then the following methods are available:
	button1.input(); // configure as input
	led1.output();   // configure as output
	led1.set();      // write 1 in output or pullup in input
	led2.clear();    // write 0 in output or no pullup in input
	led2.flip();     // flip bit's state
	led2 = expr;     // clear or set based on expression
	if (button1) ... // use in expressions to read value
	
  The advanced feature is ability to do the same operation on multiple pins by combing them with | operator.
  For example:

	(led1 | l2ed2).clear(); // clear both leds

  Note, that each single set or clear operation compiles to 1 word of code ("sti"/2"cli"),
  while the corresponding multi-operation compiles to 3 words of code ("in", "ori"/"andi", "out"),
  so it makes when manipulating 3 or more pins at the same time.

  Ports A to D are currently supported. Easily extended to more if needed.
*/

#ifndef PORTS_H_
#define PORTS_H_

#include <avr/io.h>

// ----------------------------------------------------------------------------------
// Inline here means "always inline"

#define inline inline __attribute__((always_inline))

// ----------------------------------------------------------------------------------
// Utility methods to manipulate arbitrary AVR ports

inline void sbi(volatile uint8_t& port, uint8_t pin) {
	port |= _BV(pin);
}

inline void cbi(volatile uint8_t& port, uint8_t pin) {
	port &= ~_BV(pin);
}

// ----------------------------------------------------------------------------------
// Utility marcos that are used to define Ports and PortX struts

#define PORTS_SET(REG,X)    if ((_m >> (X ## _INDEX * 8)) & 0xff) REG##X |=  (_m >> (X ## _INDEX * 8));
#define PORTS_CLEAR(REG,X)  if ((_m >> (X ## _INDEX * 8)) & 0xff) REG##X &= ~(_m >> (X ## _INDEX * 8));
#define PORTS_ASSIGN(REG,X) if ((_m >> (X ## _INDEX * 8)) & 0xff) REG##X  =  (_m >> (X ## _INDEX * 8));
#define PORTS_QUERY(REG,X)  if ((_m >> (X ## _INDEX * 8)) & REG##X) return true; 

#ifdef PORTA
#define PORTS_COUNT_A       1
#define PORTS_INDEX_A       A_INDEX,
#define PORTS_SET_A(REG)    PORTS_SET(REG,A)    
#define PORTS_CLEAR_A(REG)  PORTS_CLEAR(REG,A)
#define PORTS_ASSIGN_A(REG) PORTS_ASSIGN(REG,A)
#define PORTS_QUERY_A(REG)  PORTS_QUERY(REG,A)
#else
#define PORTS_COUNT_A       0
#define PORTS_INDEX_A
#define PORTS_SET_A(REG)
#define PORTS_CLEAR_A(REG)
#define PORTS_ASSIGN_A(REG)
#define PORTS_QUERY_A(REG)
#endif

#ifdef PORTB
#define PORTS_COUNT_B       1
#define PORTS_INDEX_B       B_INDEX,
#define PORTS_SET_B(REG)    PORTS_SET(REG,B)    
#define PORTS_CLEAR_B(REG)  PORTS_CLEAR(REG,B)
#define PORTS_ASSIGN_B(REG) PORTS_ASSIGN(REG,B)
#define PORTS_QUERY_B(REG)  PORTS_QUERY(REG,B)
#else
#define PORTS_COUNT_B       0
#define PORTS_INDEX_B
#define PORTS_SET_B(REG)
#define PORTS_CLEAR_B(REG)
#define PORTS_ASSIGN_B(REG)
#define PORTS_QUERY_B(REG)
#endif

#ifdef PORTC
#define PORTS_COUNT_C       1
#define PORTS_INDEX_C       C_INDEX,
#define PORTS_SET_C(REG)    PORTS_SET(REG,C)    
#define PORTS_CLEAR_C(REG)  PORTS_CLEAR(REG,C)
#define PORTS_ASSIGN_C(REG) PORTS_ASSIGN(REG,C)
#define PORTS_QUERY_C(REG)  PORTS_QUERY(REG,C)
#else
#define PORTS_COUNT_C       0
#define PORTS_INDEX_C
#define PORTS_SET_C(REG)
#define PORTS_CLEAR_C(REG)
#define PORTS_ASSIGN_C(REG)
#define PORTS_QUERY_C(REG)
#endif

#ifdef PORTD
#define PORTS_COUNT_D       1
#define PORTS_INDEX_D       D_INDEX,
#define PORTS_SET_D(REG)    PORTS_SET(REG,D)    
#define PORTS_CLEAR_D(REG)  PORTS_CLEAR(REG,D)
#define PORTS_ASSIGN_D(REG) PORTS_ASSIGN(REG,D)
#define PORTS_QUERY_D(REG)  PORTS_QUERY(REG,D)
#else
#define PORTS_COUNT_D       0
#define PORTS_INDEX_D
#define PORTS_SET_D(REG)
#define PORTS_CLEAR_D(REG)
#define PORTS_ASSIGN_D(REG)
#define PORTS_QUERY_D(REG)
#endif

// This macro gives a total number of port in our MCU
#define PORTS_COUNT (PORTS_COUNT_A + PORTS_COUNT_B + PORTS_COUNT_C + PORTS_COUNT_D)

// ----------------------------------------------------------------------------------
// Ports structure is used when multiple pins are combines with | operator

class Ports {
public:
	// Enumerate all ports
	enum Index {
		PORTS_INDEX_A // expands to A_INDEX, if needed
		PORTS_INDEX_B // expands to B_INDEX, if needed
		PORTS_INDEX_C // expands to C_INDEX, if needed
		PORTS_INDEX_D // expands to D_INDEX, if needed
		LAST_INDEX // last enum for the number of ports we have, also eats trailing comma
	};
	
#if PORTS_COUNT <= 1
	typedef uint8_t mask_t;
#elif PORTS_COUNT <= 2
	typedef uint16_t mask_t;
#else 
	typedef uint32_t mask_t;
#endif

private:
	mask_t _m;

public:
	inline Ports(mask_t m) : _m(m) {}
	inline Ports(const Ports& ports) : _m(ports._m) {}

	inline void input() {
		PORTS_CLEAR_A(DDR);
		PORTS_CLEAR_B(DDR);
		PORTS_CLEAR_C(DDR);
		PORTS_CLEAR_D(DDR);
	}

	inline void output() {
		PORTS_SET_A(DDR);
		PORTS_SET_B(DDR);
		PORTS_SET_C(DDR);
		PORTS_SET_D(DDR);
	}

	inline void set() {
		PORTS_SET_A(PORT);
		PORTS_SET_B(PORT);
		PORTS_SET_C(PORT);
		PORTS_SET_D(PORT);
	}

	inline void clear() {
		PORTS_CLEAR_A(PORT);
		PORTS_CLEAR_B(PORT);
		PORTS_CLEAR_C(PORT);
		PORTS_CLEAR_D(PORT);
	}

	inline void flip() {
		PORTS_ASSIGN_A(PIN);
		PORTS_ASSIGN_B(PIN);
		PORTS_ASSIGN_C(PIN);
		PORTS_ASSIGN_D(PIN);
	}

	inline bool operator = (bool x) {
		if (x) set(); else clear();
		return x;
	}
	
	inline operator bool() {
		PORTS_QUERY_A(PIN);
		PORTS_QUERY_B(PIN);
		PORTS_QUERY_C(PIN);
		PORTS_QUERY_D(PIN);
		return false;
	}

	inline mask_t mask() { return _m; }
	inline Ports operator | (Ports ports) { return _m | ports._m; }
};

// ----------------------------------------------------------------------------------
// Macro that defines individual PortX structs

#define PORT_STRUCT(X) \
template<uint8_t _pin> struct Port##X { \
	inline void input()  { DDR##X  &= ~_BV(_pin); } \
	inline void output() { DDR##X  |=  _BV(_pin); } \
	inline void set()    { PORT##X |=  _BV(_pin); } \
	inline void clear()  { PORT##X &= ~_BV(_pin); } \
	inline void flip()   { PIN##X   =  _BV(_pin); } \
	inline bool operator = (bool x) { if(x) set(); else clear(); return x; } \
	inline operator bool() { return (PIN##X & _BV(_pin)) != 0; } \
	inline uint8_t pin() { return _pin; } \
	inline Ports::mask_t mask() { return ((Ports::mask_t)_BV(_pin)) << (Ports::X ## _INDEX * 8); } \
	inline Ports operator | (Ports ports) { return mask() | ports.mask(); } \
	inline operator Ports() { return mask(); } \
};

#ifdef PORTA
PORT_STRUCT(A)
#endif

#ifdef PORTB
PORT_STRUCT(B)
#endif

#ifdef PORTC
PORT_STRUCT(C)
#endif

#ifdef PORTD
PORT_STRUCT(D)
#endif

// ----------------------------------------------------------------------------------
// Undef all temp macros

#undef PORTS_SET
#undef PORTS_CLEAR
#undef PORTS_ASSIGN
#undef PORTS_QUERY

#undef PORTS_COUNT_A
#undef PORTS_INDEX_A
#undef PORTS_SET_A
#undef PORTS_CLEAR_A
#undef PORTS_ASSIGN_A
#undef PORTS_QUERY_A

#undef PORTS_COUNT_B
#undef PORTS_INDEX_B
#undef PORTS_SET_B
#undef PORTS_CLEAR_B
#undef PORTS_ASSIGN_B
#undef PORTS_QUERY_B

#undef PORTS_COUNT_C
#undef PORTS_INDEX_C
#undef PORTS_SET_C
#undef PORTS_CLEAR_C
#undef PORTS_ASSIGN_C
#undef PORTS_QUERY_C

#undef PORTS_COUNT_D
#undef PORTS_INDEX_D
#undef PORTS_SET_D
#undef PORTS_CLEAR_D
#undef PORTS_ASSIGN_D
#undef PORTS_QUERY_D

#undef PORTS_COUNT

#undef PORT_STRUCT

#undef inline

#endif
