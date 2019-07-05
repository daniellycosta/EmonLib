#ifndef EmonLib_h
#define EmonLib_h

#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#include <inttypes.h>
//#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega88P__) || defined (__AVR_ATmega328P__)
#include <avr/io.h>
#include <avr/interrupt.h>

//--------------------------------------------------------------------------------------
// Variables and functions prototypes - For prevening type errors
//--------------------------------------------------------------------------------------

#define inPinV Emon_inPinV
#define VCAL Emon_VCAL
#define PHASECAL Emon_PHASECAL

#define inPinI Emon_inPinI
#define ICAL Emon_ICAL

#define voltage EmonVoltage
#define current EmonCurrent
#define voltageTX EmonVoltageTX
#define currentTX EmonCurrentTX
#define calcVI EmonCalcVI
#define calcIrms EmonCalcIrms
#define serialprint EmonSerialprint
#define readVcc EmonReadVcc

//--------------------------------------------------------------------------------------
// Public variables and functions declaration
//--------------------------------------------------------------------------------------

extern volatile double realPower,apparentPower,powerFactor,Vrms,Irms;

extern void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
extern void current(unsigned int _inPinI, double _ICAL);

extern void voltageTX(double _VCAL, double _PHASECAL);
extern void currentTX(unsigned int _channel, double _ICAL);

extern void calcVI(unsigned int crossings, unsigned int timeout);
extern double calcIrms(unsigned int NUMBER_OF_SAMPLES);
extern void serialprint();
//#endif

#endif