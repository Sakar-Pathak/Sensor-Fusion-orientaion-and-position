#include "prbs.h"

int prbs_index_motor;
int prbs_index_vanes;
byte prbs[PRBS_LENGTH];

void prbs_generate()
{
      // Generate PRBS sequence for motor
  unsigned int lfsr = 0xACE1;
  for (int i = 0; i < PRBS_LENGTH; i++) {
    prbs[i] = (lfsr >> 15) & 0x01; // Select a different bit each iteration
    lfsr = (lfsr << 1) | prbs[i];
  }

}