/* mjerenje_temperature.h */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include <inttypes.h>


#define SAMPLE_PERIOD        (3000)   // milliseconds, PERIOD MJERENJA!
#define ULAZNI_PIN       (GPIO_NUM_15) //Ulazni PIN cipa
#define MAX_DEVICES          (5) 
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT) //Rezolucija(broj bita odvojen za predstavljanje temperature!)

#endif
