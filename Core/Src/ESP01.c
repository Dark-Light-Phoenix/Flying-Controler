#include "ESP01.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"

extern USART_HandleTypeDef husart1;
extern float arr_accel_x, arr_accel_y, arr_accel_z, arr_gyro_x, arr_gyro_y, arr_gyro_z, Kalman_accel_x, Kalman_accel_y, Kalman_accel_z, Kalman_gyro_x, Kalman_gyro_y, Kalman_gyro_z, StartHeight, DeltaHeight;

void USART_Transmit (uint8_t *data, uint16_t length)
{
	HAL_USART_Transmit (&husart1, data, length, HAL_MAX_DELAY);
}

void Transmit_Data(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z, float *StartHeight, float *DeltaHeight)
{
    char buffer[512];
    sprintf(buffer, "arr_accel_X:%f,arr_accel_Y:%f,arr_accel_Z:%f,arr_gyro_X:%f,arr_gyro_Y:%f,arr_gyro_Z:%f,Kalman_accel_X:%f,Kalman_accel_Y:%f,Kalman_accel_Z:%f,Kalman_gyro_X:%f,Kalman_gyro_Y:%f,Kalman_gyro_Z:%f,StartHeight:%f,DeltaHeight:%f", *arr_accel_x, *arr_accel_y, *arr_accel_z, *arr_gyro_x, *arr_gyro_y, *arr_gyro_z, *Kalman_accel_x, *Kalman_accel_y, *Kalman_accel_z, *Kalman_gyro_x, *Kalman_gyro_y, *Kalman_gyro_z, *StartHeight, *DeltaHeight);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));
}

void ATCommand (char* cmd)
{
	HAL_USART_Transmit (&husart1, (uint8_t *) cmd, strlen (cmd), HAL_MAX_DELAY);
}

void WEBServer (void)
{
	ATCommand ("AT+CIPMUX=1\r\n");
	HAL_Delay (1000);

	ATCommand ("AT+CIPSERVER=1,80\r\n");
	HAL_Delay (1000);
}

char rxBuffer[BUFFER];

void ClientRequest (void)
{
	int length = HAL_USART_Receive (&husart1, (uint8_t *) rxBuffer, BUFFER, HAL_MAX_DELAY);

	if (length > 0)
	{
		if (strstr (rxBuffer, "GET"))
		{
			char httpResponse [] = "HTTP/1.1 200 OK\r\nContent-Type: text\html\r\n\r\n"
								   "<!DOCTYPE HTML>\r\n<html>\r\n>"
								   "<head><title> Flyight Controler </title></head>\r\n"
								   "<body><h5> YOU ARE HERE</h5>\r\n"
								   "<p id = 'data'>Loading...</p>\r\n"
								   "<script>\r\n"
								   "setInterval (function() {\r\n"
								   "	var xhttp = new XMLHttpRequest(); \r\n"
								   "	xhhtp.onreadystatechange = function (); \r\n"
								   "		if (this.readyState == 4 && this.status = 200) {\r\n"
								   "			document.getElementById ('data').innerHTML = this.responseText;\r\n"
								   "			}\r\n"
								   "		};\r\n"
								   "	xhttp.open ('GET', '/data', true);\r\n"
								   "	xhttp.send ();\r\n"
								   "}, 1000);\r\n"
								   "</script>\r\n"
								   "</body>\r\n"
								   "</html>\r\n";

		ATCommand ("AT+CIPSEND=0");
		char Str [10];
		sprintf (Str, "%d", strlen (httpResponse));
		ATCommand (Str);
		ATCommand ("\r\n");
		HAL_Delay (1000);
		ATCommand (httpResponse);
		HAL_Delay (1000);

		ATCommand ("AT+CIPCLOSE=0\r\n");
		} else if (strstr (rxBuffer, "GET /data"))
		{
			char dataResponse [64];
			sprintf (dataResponse, "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n%lu", HAL_GetTick());
			ATCommand ("AT+CIPSEND0=0,");
			char Str[10];
			sprintf (Str, "%d", strlen(dataResponse));
			ATCommand (Str);
			ATCommand ("\r\n");
			HAL_Delay (1000);
			ATCommand (dataResponse);
			HAL_Delay (1000);

			ATCommand ("AT+CIPCLOSE=0\r\n");
		}
	}}
