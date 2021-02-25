/* parametri.h */

/*Parametres, needed for sensors readings!*/

#ifndef _BOARD_H_
#define _BOARD_H_

int PERIOD_TEMP = 5000;
int PERIOD_PH = 120000;
int PERIOD_PROTOK = 5000;

int PRAG_PROTOKA = 2;
int PH_D_P = 5;
int PH_G_P = 12;
int TEMP_D_P = 10;
int TEMP_G_P = 30;

uint8_t VENTIL_ON_OFF = 0;
bool IZVRSITI_KALIBRACIJU = false;
#endif
