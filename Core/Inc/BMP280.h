#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#define BMP280_ADDRESS 0x76
#define PRESSURE_REG 0xF6

void BMP280_Pressure (double *pressure);

#endif /* INC_BMP280_H_ */
