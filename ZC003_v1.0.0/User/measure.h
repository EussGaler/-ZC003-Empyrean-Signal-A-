#ifndef __MEASURE_H
#define __MEASURE_H

#include "ti_msp_dl_config.h"

void StartMeasure(void);
void StopMeasure(void);
void StopMeasure2(void);
uint32_t Result_Calculate(uint32_t *IndexDiff, uint16_t len);
int compare(const void *a, const void *b);
void Measure_Init(void);

#endif
