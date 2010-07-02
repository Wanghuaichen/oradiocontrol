/*
 * OCR_rx.h
 *
 *  Created on: 02.07.2010
 *      Author: josef
 */

#include "ORC.h"

typedef struct t_EEData
{
  uint8_t       step;
  uint16_t      id;     //sync
}__attribute__((packed)) EEData;
