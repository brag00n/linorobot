/*
    _   ___ ___  _   _ ___ _  _  ___  _    ___   ___ 
   /_\ | _ \   \| | | |_ _| \| |/ _ \| |  / _ \ / __|
  / _ \|   / |) | |_| || || .` | (_) | |_| (_) | (_ |
 /_/ \_\_|_\___/ \___/|___|_|\_|\___/|____\___/ \___|
                                                     
  Log library for Arduino
  version 1.0.3
  https://github.com/thijse/Arduino-Log

Licensed under the MIT License <http://opensource.org/licenses/MIT>.

Permission is hereby  granted, free of charge, to any  person obtaining a copy
of this software and associated  documentation files (the "Software"), to deal
in the Software  without restriction, including without  limitation the rights
to  use, copy,  modify, merge,  publish, distribute,  sublicense, and/or  sell
copies  of  the Software,  and  to  permit persons  to  whom  the Software  is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE  IS PROVIDED "AS  IS", WITHOUT WARRANTY  OF ANY KIND,  EXPRESS OR
IMPLIED,  INCLUDING BUT  NOT  LIMITED TO  THE  WARRANTIES OF  MERCHANTABILITY,
FITNESS FOR  A PARTICULAR PURPOSE AND  NONINFRINGEMENT. IN NO EVENT  SHALL THE
AUTHORS  OR COPYRIGHT  HOLDERS  BE  LIABLE FOR  ANY  CLAIM,  DAMAGES OR  OTHER
LIABILITY, WHETHER IN AN ACTION OF  CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE  OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ArduinoLog.h"

/*
// = START Logger ================
// Logger
// ===============================
//Debug
const boolean DEBUG1=true;
const int DEBUG_Level=2 ;

//http://www.utopiamechanicus.com/article/low-memory-serial-print/
void StreamPrint_progmem(int pType,int pLevel,Print &out,PGM_P format,...)
{
  if (DEBUG1==false || pLevel>DEBUG_Level) return;

  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[128], *ptr;
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
  out.print(ptr);
#else
  // If error
  if (pType==1){
     nh.logfatal(ptr);
  }else {
     nh.loginfo(ptr);
  }
  
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

#define logError(format, ...) StreamPrint_progmem(1,0,Serial,PSTR(format),##__VA_ARGS__)
#define logDebug(level,format, ...) StreamPrint_progmem(2,level,Serial,PSTR(format),##__VA_ARGS__)
// E.g: logDebug(3,"Init_Steer: servo7=%d\n",b);

#define Serialprint(level,format, ...) StreamPrint_progmem(1,level,Serial,PSTR(format),##__VA_ARGS__)
#define Streamprint(level,stream,format, ...) StreamPrint_progmem(1,level,stream,PSTR(format),##__VA_ARGS__)

// = END Logger ================
*/

void Logging::begin(int level, Print* logOutput, bool showLevel)
{
#ifndef DISABLE_LOGGING
	setLevel(level);
	setShowLevel(showLevel);
	_logOutput = logOutput;
#endif
}

void Logging::setLevel(int level)
{
#ifndef DISABLE_LOGGING
	_level = constrain(level, LOG_LEVEL_PLOT, LOG_LEVEL_VERBOSE);
#endif
}

int Logging::getLevel() const
{
#ifndef DISABLE_LOGGING
	return _level;
#else
	return 0;
#endif
}

void Logging::setShowLevel(bool showLevel)
{
#ifndef DISABLE_LOGGING
	_showLevel = showLevel;
#endif
}

bool Logging::getShowLevel() const
{
#ifndef DISABLE_LOGGING
	return _showLevel;
#else
	return false;
#endif
}

void Logging::setPrefix(printfunction f)
{
#ifndef DISABLE_LOGGING
	_prefix = f;
#endif
}

void Logging::setSuffix(printfunction f)
{
#ifndef DISABLE_LOGGING
	_suffix = f;
#endif
}

void Logging::print(const __FlashStringHelper *format, va_list args)
{
#ifndef DISABLE_LOGGING	  	
	PGM_P p = reinterpret_cast<PGM_P>(format);
	char c = pgm_read_byte(p++);
	for(;c != 0; c = pgm_read_byte(p++))
	{
		if (c == '%')
		{
			c = pgm_read_byte(p++);
			printFormat(c, &args);
		}
		else
		{
			_logOutput->print(c);
		}
	}
#endif
} 
  
void Logging::print1(PGM_P format,...) {
#ifndef DISABLE_LOGGING
  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[1024], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem
  
  // null terminate - leave last char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString)-2 ]='\0';
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...
  va_list args;
  
  //formatString[ strlen(formatString) ]='\n';
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0';
  
  _logOutput->print(ptr);
#endif
}


void Logging::print(const char *format, va_list args) {
#ifndef DISABLE_LOGGING  	
	for (; *format != 0; ++format)
	{
		if (*format == '%')
		{
			++format;
			printFormat(*format, &args);
		}
		else
		{
			_logOutput->print(*format);
		}
	}
#endif
}

void Logging::printFormat(const char format, va_list *args) {
#ifndef DISABLE_LOGGING
	if (format == '%')
	{
		_logOutput->print(format);
	}
	else if (format == 's')
	{
		register char *s = (char *)va_arg(*args, int);
		_logOutput->print(s);
	}
	else if (format == 'S')
	{
		register __FlashStringHelper *s = (__FlashStringHelper *)va_arg(*args, int);
		_logOutput->print(s);
	}
	else if (format == 'd' || format == 'i')
	{
		_logOutput->print(va_arg(*args, int), DEC);
	}
	else if (format == 'D' || format == 'F')
	{
		_logOutput->print(va_arg(*args, double));
	}
	else if (format == 'x')
	{
		_logOutput->print(va_arg(*args, int), HEX);
	}
	else if (format == 'X')
	{
		_logOutput->print("0x");
		_logOutput->print(va_arg(*args, int), HEX);
	}
	else if (format == 'b')
	{
		_logOutput->print(va_arg(*args, int), BIN);
	}
	else if (format == 'B')
	{
		_logOutput->print("0b");
		_logOutput->print(va_arg(*args, int), BIN);
	}
	else if (format == 'l')
	{
		_logOutput->print(va_arg(*args, long), DEC);
	}
	else if (format == 'u')
	{
		_logOutput->print(va_arg(*args, unsigned long), DEC);
	}
	else if (format == 'c')
	{
		_logOutput->print((char) va_arg(*args, int));
	}
	else if(format == 't')
	{
		if (va_arg(*args, int) == 1)
		{
			_logOutput->print("T");
		}
		else
		{
			_logOutput->print("F");
		}
	}
	else if (format == 'T')
	{
		if (va_arg(*args, int) == 1)
		{
			_logOutput->print(F("true"));
		}
		else
		{
			_logOutput->print(F("false"));
		}
	}
#endif
}


char *Logging::ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 char formatString[1024], *ptr;
 strncpy_P( formatString, ret, sizeof(formatString) ); // copy in from program mem
 return ret;
}

char *Logging::ftoa(char *a, double f){
  return ftoa(a, f, 5);
}

char *Logging::ftoa(double f){
  char buffer[1024];
  return ftoa(buffer, f);
  //String test_string = String(f, 2);
  //test_string.toCharArray(_ftoaBuffer, 100);
  //return _ftoaBuffer;
}

char *Logging::ftoa(double f, int precision){
  //char buffer[1024];
  return ftoa(_ftoaBuffer, f,precision);
}
 
Logging Log = Logging();
