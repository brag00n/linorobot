#ifdef USE_ENCODER_HC89
#include "Encoder_HC-89.h"
#else
#include "Encoder_default.h"
#endif

#include "ArduinoLog.h"

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

Encoder_internal_state_t * Encoder::interruptArgs[];


