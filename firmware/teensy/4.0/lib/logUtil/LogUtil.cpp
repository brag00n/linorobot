#include "LogUtil.h"

// = START Logger ================
// Logger
// ===============================
//Debug

//http://www.utopiamechanicus.com/article/low-memory-serial-print/
void StreamPrint_progmem(int pType,int pLevel,Print &out,PGM_P format,...)
{
  if (DEBUG1==false || pLevel>DEBUG_Level) return;

  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[256], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
  
  // null terminate - leave last char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString)-2 ]='\0';
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
  va_list args;
  
  formatString[ strlen(formatString) ]='\n';
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0';
  
#ifdef SERIAL_TEST
  StreamPrint(pType,ptr); 
  //out.print(ptr);
#else
  // If error
  /*if (pType==1){
     nh.logfatal(ptr);
  }else {
     nh.loginfo(ptr);
  } */
  
#endif
}

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

// = END Logger ================