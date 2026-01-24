//
//
#ifndef MAIN_H
#define MAIN_H

#pragma once

#include <Arduino.h>
#include <inttypes.h>
#include <util/delay.h>
struct Signal 
{
   byte throttle;
   byte pitch;
   byte roll;
   byte yaw;
   byte aux1;
   byte aux2;
   
};

#define SLAVE_PIN 4

#define PPM_PIN      4
#define PULSE_LENGTH 200    // µs
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000


#define BATTX        118
#define BATTY        2
#define BATTH        44
#define BATTB        7

#define FLYBATTX        102
#define FLYBATTY        2
#define FLYBATTH        44
#define FLYBATTB        12


// defines for array pos
#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3

#define PFEIL_UP     0
#define PFEIL_DOWN   1
#define PFEIL_LEFT   2
#define PFEIL_RIGHT  3

#define BUZZPIN         7    // BUZZER PCB nano 5: 5
#define  BUZZINTERVALL 0xF0


#define NUM_SERVOS  4

#define  ANZAHLMODELLE        5
#define  KANALSETTINGBREITE   4
#define  MODELSETTINGBREITE   32 // nur Kanalsettings. Anschliessend MixingSettings
#define  EEPROM_MODELSETTINGBREITE 64 //Kanalsettings und MixingSettings

#define  STATUS_OFFSET     0 
#define  LEVEL_OFFSET      1 //0x20 // 32
#define  EXPO_OFFSET       2 //0x30 // 48
#define  FUNKTION_OFFSET   3 // 96


#define DEVICE_OFFSET      0x70 // 122
#define AUSGANG_OFFSET     0x80 // 128

#define SAVE_STATUS      0
#define SAVE_LEVEL      1
#define SAVE_MIX        2
#define SAVE_EXPO       3
#define SAVE_FUNKTION   4
#define SAVE_DEVICE     5
#define SAVE_AUSGANG    6

// Joystick

#define OLED 1

#define ANZ_SCREEN         3
#define TASTE_OK           1
#define AKTION_OK           2
#define UPDATE_OK           3
#define T5_WAIT              4 // Warten auf 3 Impulse
#define MENU_ON            5

/*
#define JOYSTICKTASTE1 70
#define JOYSTICKTASTE2 121
#define JOYSTICKTASTE3 192
#define JOYSTICKTASTE4 246
#define JOYSTICKTASTE5 308
#define JOYSTICKTASTE6 368
#define JOYSTICKTASTE7 426
#define JOYSTICKTASTE8 510
#define JOYSTICKTASTE9 580
*/
/*
// External Ref 3v3
// Joystick-Tastatur Board2

#define JOYSTICKTASTE1  51
#define JOYSTICKTASTE2  88
#define JOYSTICKTASTE3  140
#define JOYSTICKTASTE4  180
#define JOYSTICKTASTE5  225
#define JOYSTICKTASTE6  269
#define JOYSTICKTASTE7  312
#define JOYSTICKTASTE8  375
#define JOYSTICKTASTE9  478
*/

// External Ref 3v3
// Joystick-Tastatur Board6
#define JOYSTICKTASTE1  51
#define JOYSTICKTASTE2  88
#define JOYSTICKTASTE3  140
#define JOYSTICKTASTE4  179
#define JOYSTICKTASTE5  225
#define JOYSTICKTASTE6  269
#define JOYSTICKTASTE7  312
#define JOYSTICKTASTE8  374
#define JOYSTICKTASTE9  478


/*
// 3*3-Tastatur
#define JOYSTICKTASTE1  109
#define JOYSTICKTASTE2  155
#define JOYSTICKTASTE3  200
#define JOYSTICKTASTE4  246
#define JOYSTICKTASTE5  287
#define JOYSTICKTASTE6  342
#define JOYSTICKTASTE7  398
#define JOYSTICKTASTE8  433
#define JOYSTICKTASTE9  502
*/



#define TAB0    2
#define TAB1    4
#define TAB3    8


#define TAB4    80
#define TAB5    100

// balken
#define VBX    64
#define VBY    12
#define HBX    6
#define HBY    54




#endif


