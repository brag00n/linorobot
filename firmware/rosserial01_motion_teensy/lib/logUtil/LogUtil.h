#ifndef LOGUTIL_H
#define LOGUTIL_H

#include <Arduino.h>
//#define SERIAL_TEST

const bool DEBUG1=true;
const int DEBUG_Level=2 ;

char *ftoa(char *a, double f, int precision);
void StreamPrint_progmem(int pType,int pLevel,Print &out,PGM_P format,...);

#define logError(format, ...) StreamPrint_progmem(1,0,Serial,PSTR(format),##__VA_ARGS__)
#define logDebug(level,format, ...) StreamPrint_progmem(2,level,Serial,PSTR(format),##__VA_ARGS__)
// E.g: logDebug(3,"Init_Steer: servo7=%d\n",angle7);

#define Serialprint(level,format, ...) StreamPrint_progmem(1,level,Serial,PSTR(format),##__VA_ARGS__)
#define Streamprint(level,stream,format, ...) StreamPrint_progmem(1,level,stream,PSTR(format),##__VA_ARGS__)

#define StreamPrint(pType,ptr) out.print(ptr);

#endif