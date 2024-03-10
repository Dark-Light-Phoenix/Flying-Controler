#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

typedef struct // Kalman filter structure for accelerometer
{
    float x;
    float P;
} KalmanFilterAccel;

typedef struct // Kalman filter structure for gyroscope
{
    float x;
    float P;
} KalmanFilterGyro;

void KalmanFilterUpdateAccel(KalmanFilterAccel *filter, float z, float R, float H);
void KalmanFilterUpdateGyro(KalmanFilterGyro *filter, float z, float R, float H);
void initKalmanFilterAccel(KalmanFilterAccel *filter);
void initKalmanFilterGyro(KalmanFilterGyro *filter);

#endif /* INC_FUNCTIONS_H_ */
