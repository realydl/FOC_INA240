#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

#include "foc_utils.h" 
#include "main.h"

/******************************************************************************/
void InlineCurrentSense(float _shunt_resistor, float _gain);
//void InlineCurrentSense_Init(void);
PhaseCurrent_s getSelectionPhaseCurrents(uint8_t sector);
PhaseCurrent_s getPhaseCurrents(void);
/******************************************************************************/


#endif
