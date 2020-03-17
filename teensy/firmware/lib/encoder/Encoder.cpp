#ifdef USE_ENCODER_HC-89
#include "Encoder_HC-89.h"
#else
#include "Encoder.h"
#endif

// Yes, all the code is in the header file, to provide the user
// configure options with #define (before they include it), and
// to facilitate some crafty optimizations!

Encoder_internal_state_t * Encoder::interruptArgs[];


