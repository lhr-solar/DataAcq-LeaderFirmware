#ifndef __SLCAN_H
#define __SLCAN_H

#include <stdint.h>

typedef struct {
    const char* name;
    const char* can_id;   // ASCII version of CAN ID
    uint8_t length;       // in bytes
    uint8_t index_used;   // 1 if index used, 0 otherwise
} CANSignal;

extern CANSignal can_signals[] = {
    { "CONTROL_MODE",                     "0x580",  1, 0 },
    { "Motor Precharge Enable",           "0x582",  1, 0 },
    { "IO_STATE",                         "0x581",  3, 0 },
    { "Controls_Fault",                   "0x583",  1, 0 },
    { "Motor Controller Safe",            "0x584",  1, 0 },
    { "Motor Controller Identification",  "0x240",  8, 0 },
    { "Motor Status",                     "0x241",  8, 0 },
    { "Motor Controller Bus",             "0x242",  8, 0 },
    { "Velocity",                         "0x243",  8, 0 },
    { "Motor Controller Phase Current",   "0x244",  8, 0 },
    { "Motor Voltage Vector",             "0x245",  8, 0 },
    { "Motor Current Vector",             "0x246",  8, 0 },
    { "Motor BackEMF",                    "0x247",  8, 0 },
    { "Low Voltage Rail Measurement",     "0x248",  8, 0 },
    { "DSP Voltage Rail Measurement",     "0x249",  8, 0 },
    { "Reserved",                         "0x24A",  8, 0 },
    { "Motor Temperature",                "0x24B",  8, 0 },
    { "DSP Board Temperature",            "0x24C",  8, 0 },
    { "Reserved",                         "0x24D",  8, 0 },
    { "Odometer / Bus Amp Hours",         "0x24E",  8, 0 },
    { "Slip Speed Measurement",           "0x257",  8, 0 }
};

#define CAN_SIGNAL_COUNT (sizeof(can_signals) / sizeof(CANSignal))


#endif /* __SLCAN_H */