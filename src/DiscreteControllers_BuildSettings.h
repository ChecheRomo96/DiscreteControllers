#ifndef DISCRETE_CONTROLLERS_BUILD_SETTINGS_H
#define DISCRETE_CONTROLLERS_BUILD_SETTINGS_H
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CPVector Version

        #ifndef DISCRETE_CONTROLLERS_VERSION
            #define DISCRETE_CONTROLLERS_VERSION "0.0.1"
        #endif
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Arduino IDE

        #if defined(ARDUINO)
            #include <Arduino.h>
        #endif
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // AVR 
 
        #if defined(__avr__)
            #include <avr/pgmspace.h>
            
            #ifndef PROGMEM_MACRO
                #define PROGMEM_MACRO PROGMEM
            #endif
        #else
            #ifndef PROGMEM_MACRO
                #define PROGMEM_MACRO
            #endif
        #endif
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PSoC Creator
    
        #if defined(PSOC_CREATOR)
            //#define CPVECTOR_VERSION "0.0.1"
        #endif
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Desktop C++
    
        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__APPLE__) || defined(linux)
            #include <vector>
        #endif
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif//DISCRETE_CONTROLLERS_BUILD_SETTINGS_H
