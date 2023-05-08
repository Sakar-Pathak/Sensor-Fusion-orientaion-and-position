#include "Arduino.h"

#define PRBS_LENGTH 255

extern byte prbs[PRBS_LENGTH];
extern int prbs_index_motor;
extern int prbs_index_vanes;


void prbs_generate();
