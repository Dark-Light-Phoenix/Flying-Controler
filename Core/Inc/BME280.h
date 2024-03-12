#ifndef INC_BME280_H_
#define INC_BME280_H_

void BME280_ReadPressure (void);
void BME280_ReadTemperature (void);

#define BME280_ADDRESS 0x76 // Address of BME280
#define PRESS_MSB_REG 0xF7 // Address of temperature bit information
#define BME280_TEMPERATURE_MSB_REG 0xFA // Address of the most significant bit of information
#define BME280_TEMPERATURE_LSB_REG 0xFB // Address of less significant bit of information
#define BME280_TEMPERATURE_XLSB_REG 0xFC // Address of least significant bit

#endif /* INC_BME280_H_ */
