/*
In your code insert

#define DEBUG
#include "DebugUtils.h"

Then simply use dprintv("message"); to print your debugging message. You can also use dprint and dprintln.
Or if you don't want the debug messages to appear simply comment out the DEBUG definition as follows

//#define DEBUG
#include "DebugUtils.h"

And you won't need to comment out any DEBUG_PRINT calls.
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG
#include <Arduino.h>
#define dprintv(str)    \
    Serial.print(millis());     \
    Serial.print(": ");    \
    Serial.print(__PRETTY_FUNCTION__); \
    Serial.print(' ');      \
    Serial.print(__FILE__);     \
    Serial.print(':');      \
    Serial.print(__LINE__);     \
    Serial.print(' ');      \
    Serial.println(str);
#define dprintln(str)     \
	Serial.println(str);     
#define dprint(str)     \
	Serial.print(str);
#else
#define dprintv(str)
#define dprint(str)
#define dprintln(str)
#endif

#endif

