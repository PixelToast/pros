#ifndef INC_EMULATOR_H
#define INC_EMULATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

#define __IO volatile

typedef struct {
  // Control register
  __IO uint32_t CR;
  // Args
  __IO uint32_t A1;
  __IO uint32_t A2;
  __IO uint32_t A3;
  // Results
  __IO uint32_t R1;
  __IO uint32_t R2;
  __IO uint32_t R3;
  __IO uint32_t R4;
} EMULATOR_TypeDef;

#define EMULATOR ((EMULATOR_TypeDef*)0x40021400)

inline int emuCall(uint16_t mod, uint16_t func, uint32_t a1, uint32_t a2, uint32_t a3) {
  _enterCritical();
  EMULATOR->A1 = a1;
  EMULATOR->A2 = a2;
  EMULATOR->A3 = a3;
  EMULATOR->CR = func | (mod << 16);
  int out = EMULATOR->R1;
  _exitCritical();
  return out;
}

// Redundant EMUCONCAT to evaluate EMUMODNAME
#define EMUCONCAT2(x,y) Emu ## x ## _ ## y
#define EMUCONCAT(x,y) EMUCONCAT2(x,y)
#define EMUFNAME(x) EMUCONCAT(EMUMODNAME,x)

#define EMUDEFINEFUNC(id, fname) inline int EMUFNAME(fname)() { return emuCall(EMUMODID, id, 0, 0, 0); }
#define EMUDEFINEFUNC1(id, fname, a1type, a1) inline int EMUFNAME(fname)(a1type a1) { return emuCall(EMUMODID, id, (int)a1, 0, 0); }
#define EMUDEFINEFUNC2(id, fname, a1type, a1, a2type, a2) inline int EMUFNAME(fname)(a1type a1, a2type a2) { return emuCall(EMUMODID, id, (int)a1, (int)a2, 0); }
#define EMUDEFINEFUNC3(id, fname, a1type, a1, a2type, a2, a3type, a3) inline int EMUFNAME(fname)(a1type a1, a2type a2, a3type a3) { return emuCall(EMUMODID, id, (int)a1, (int)a2, (int)a3); }

#define EMUMODNAME Serial
#define EMUMODID 0
EMUDEFINEFUNC3(0, init, int, port, int, baud, int, flags);
EMUDEFINEFUNC1(1, shutdown, int, port);
EMUDEFINEFUNC2(2, putc, int, port, int, c);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME FS
#define EMUMODID 1
EMUDEFINEFUNC(0, programOn);
EMUDEFINEFUNC(1, programOff);
EMUDEFINEFUNC1(2, erasePage, int, page);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME I2C
#define EMUMODID 2
EMUDEFINEFUNC3(0, startRead, int, addr, void*, data, int, count);
EMUDEFINEFUNC3(1, startWrite, int, addr, void*, data, int, count);
EMUDEFINEFUNC1(2, setAddr, int, addr);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME GPIO
#define EMUMODID 3
EMUDEFINEFUNC1(0, ADCInit, uint32_t, data);
EMUDEFINEFUNC2(1, SetDir, int, port, int, mode);
EMUDEFINEFUNC1(2, GetInput, int, port);
EMUDEFINEFUNC1(3, GetOutput, int, port);
EMUDEFINEFUNC2(4, SetOutput, int, port, int, value);
EMUDEFINEFUNC2(5, SetInterrupt, int, port, int, edges);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME Motor
#define EMUMODID 4
EMUDEFINEFUNC1(0, get, int, channel);
EMUDEFINEFUNC2(1, set, int, channel, int, value);
EMUDEFINEFUNC(2, stop);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME Comp
#define EMUMODID 5
EMUDEFINEFUNC(0, init);
EMUDEFINEFUNC(1, enableStandalone);
EMUDEFINEFUNC1(2, setName, char*, name);
EMUDEFINEFUNC1(3, getStatus, void*, buff);
#undef EMUMODNAME
#undef EMUMODID

#define EMUMODNAME System
#define EMUMODID 6
EMUDEFINEFUNC(0, exit);
EMUDEFINEFUNC(1, break);
#undef EMUMODNAME
#undef EMUMODID

#ifdef __cplusplus
}
#endif

#endif
