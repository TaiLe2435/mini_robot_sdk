#include <Arduino.h>

void transform(float accX, float accY, float accZ, float matrix[3][3], String acc );

void transpose(float matrix[3][3]);

void euler(int sDdot[3][1], int sdot[3][1], int s0[3][1]);

void simpsons();

void trapezoidal(int s1[3][1], int s0[3][1], int ic[3][1]);

unsigned long CalculateDeltaTime();

void initIMU();

float* poseEstimation(); 