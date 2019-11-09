#ifndef I2C_H_
#define I2C_H_

uint16_t humidityThreshold= 0;
uint32_t temperatureThreshold = 303.15;//30* C

extern uint16_t Update_Humidity();
extern uint16_t Update_Temperature();
extern uint16_t Update_Temperature_From_Last_Reading();

#endif
