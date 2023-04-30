#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <math.h>
#include <stdint.h>
#include "lib.h"
// #include <stdio.h>

// #define print_vec(ex, size) printf(#ex);printf(" = ");for (int i = 0; i < size; i++) printf ("%0.8f ", ex[i]); printf("\n");

#define RAD_CONV 57.2957
#define WINDOW_SIZE 10
#define DECLINATION -1.21f; //degrees in bangalore
#define COLLISION_THRESHOLD 250.0f

static int counter = 0;
static float data[WINDOW_SIZE] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

float dot(const float* v, const float* w, uint8_t size){
    float res = 0;
    for (int i = 0; i < size; i++){
        res += v[i]*w[i];
    }
    return res;
}

float norm (const float* v, uint8_t size){
    double res =  (double) (dot(v,v, size));
    res = sqrt(res);
    return res;
}

void unitVector(const float* v, float* res, uint8_t size){
    float norm_v = norm(v, size);
    for (int i = 0; i < size; i++){
        res[i] = v[i];
        res[i] /= norm_v;
    }
}

void print_float(float data, int dec_places){

    int64_t k = 1;
    for (int i = 0; i < dec_places; i++)
        k *= 10;
    int num, dec;
      num = (int) (data);
      dec = (int) ((data-num)*k); 
      dec = dec > 0 ? dec : -dec;
      printf("%d.%d ",num,dec);
}

void quat2euler(const float* quat, float* angles){
    angles[2] = atan2(2*quat[3]*quat[2] + 2*quat[0]*quat[1], pow(quat[3],2) 
                + pow(quat[0],2) - pow(quat[1],2) - pow(quat[2],2));

    angles[1] = atan2(2*quat[3]*quat[1] - 2*quat[0]*quat[2], 
                sqrt(pow((2*quat[3]*quat[0] + 2*quat[1]*quat[2]),2) 
                + pow((pow(quat[3],2) - pow(quat[0],2) - pow(quat[1],2) + pow(quat[2],2)),2)));

    angles[0] = atan2(2*quat[3]*quat[0] + 2*quat[1]*quat[2], 
                pow(quat[3],2) - pow(quat[0],2) - pow(quat[1],2) + pow(quat[2],2));
}

float movAvg(float heading){
    
    if (counter < WINDOW_SIZE){
        data[counter] = heading;
        counter++;
        return heading;
    } else {
        for (int i = 1; i < WINDOW_SIZE; i++){
            data[i-1] = data[i];
        }
        data[WINDOW_SIZE-1] = heading;
        float sum = 0.0;
        for (int i = 0; i < WINDOW_SIZE; i++){
            sum +=data[i];
        }
        sum /= WINDOW_SIZE;
        counter++;
        return sum;
    }
}

float yawToHeading(float yaw){
    float heading = yaw + DECLINATION;
    if (heading < 0.0){
        heading += 360.0;
    }
    return heading;
}

float estimate_heading(const float* accData, const float* magData, float* angles) {
    // Signs choosen so that, when axis is down, the value is + 1g
  float accl_x = -accData[0];
  float accl_y = accData[1];
  float accl_z = accData[2];

  // Signs should be choosen so that, when the axis is down, the value is + positive.
  // But that doesn't seem to work ?...
  float magn_x = magData[0];
  float magn_y = -magData[1];
  float magn_z = -magData[2];  
  
  // Freescale solution
  angles[0] = atan2(accl_y, accl_z);
  angles[1] = atan(-accl_x / (accl_y * sin(angles[0]) + accl_z * cos(angles[0])));
  
  float magn_fy_fs = magn_z * sin(angles[0]) - magn_y*cos(angles[0]);
  float magn_fx_fs = magn_x * cos(angles[1]) + magn_y * sin(angles[1]) * sin(angles[0]) + magn_z * sin(angles[1]) * cos(angles[0]);
  
  angles[2] = atan2(magn_fy_fs, magn_fx_fs);
  
  angles[0] = angles[0] * RAD_CONV;
  angles[1] = angles[1] * RAD_CONV;
  angles[2] = angles[2] * RAD_CONV;
  
  return yawToHeading(angles[2]);
}

void estimate_angles(const float* accData, const float* magData, float* angles) {
    /**
     * This is based on FAMC see ahrs
    */
    // Normalize the measurements
    float acc_norm[3], mag_norm[3];
    if (!(norm(accData, 3) > 0)  || !(norm(magData, 3) > 0))
        return;
    unitVector(accData, acc_norm, 3);
    unitVector(magData, mag_norm, 3);

    // Dynamic magnetometer reference vector
    float m_D = dot(acc_norm, mag_norm, 3);
    float m_N = sqrt(1.0 - pow(m_D,2));

    // Parameters
    float B[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (int row = 0; row < 3; row++){
        B[row][0] = (m_N*mag_norm[row])*0.5;
        B[row][2] = (m_D*mag_norm[row] + acc_norm[row])*0.5;
    }
    float tau = B[0][2] + B[2][0];

    float alpha[3] = {0,0,0};
    float Y[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    alpha[0] = B[0][0] - B[2][2] - 1;
    Y[0][0] = -1/alpha[0];
    Y[0][1] = B[1][0]/alpha[0];
    Y[0][2] = tau/alpha[0];

    alpha[1] = pow(B[1][0],2)/alpha[0] - B[0][0] - B[2][2] - 1;
    Y[1][0] = (-B[1][0]/alpha[0])/alpha[1];
    Y[1][1] = -1/alpha[1];
    Y[1][2] = (B[1][2]+B[1][0]*Y[0][2])/alpha[1];

    alpha[2] = alpha[0] - 2 + pow(tau,2)/alpha[0] + pow(Y[1][2], 2)*alpha[1];
    Y[2][0] = (tau+B[1][0]*Y[1][2]/alpha[0])/alpha[2];
    Y[2][1] = Y[1][2]/alpha[2];
    Y[2][2] = 1/alpha[2];

    // Quaternion Elements
    float quat_norm[4], quat[4] = {-1.0, 0.0, 0.0, 0.0};
    quat[1] = B[1][2]*(Y[0][0] + Y[0][1]*(Y[1][2]*Y[2][0] + Y[1][0]) + Y[0][2]*Y[2][0]) - (B[0][2]-B[2][0])*(Y[1][2]*Y[2][0] + Y[1][0]) - Y[2][0]*B[1][0];
    quat[2] = B[1][2]*(          Y[0][1]*(Y[1][2]*Y[2][1] + Y[1][1]) + Y[0][2]*Y[2][1]) - (B[0][2]-B[2][0])*(Y[1][2]*Y[2][1] + Y[1][1]) - Y[2][1]*B[1][0];
    quat[3] = B[1][2]*(          Y[0][1]* Y[1][2]*Y[2][2]            + Y[0][2]*Y[2][2]) - (B[0][2]-B[2][0])*(Y[1][2]*Y[2][2])           - Y[2][2]*B[1][0];

    unitVector(quat, quat_norm, 4);
    quat2euler(quat, angles);
    // print_vec(quat_norm,4)
    // print_vec(angles,3)
}

/**
 * Collision detection algorithm is based on the paper 
 * Buoy Collision Detection : https://ieeexplore.ieee.org/document/6338483
 * Algorithm goes like this 
 * 1. Remove dc components
 *      A = Ax + Ay + Az
 *      a10 = (1+xt)/2
 *      a11 = -a10
 *      b11 = xt
 *      xt = exp(-2*pi*fc) // wher fc = cut-off frequency typical value = 0.5135
 *      y1(n) = a10.x1(n) + a11.x1(n-1) + b1.y1(n-1)
 * 2. compute the factor
 *      x2(n) = y1(n)^2
 *      y2(n) = a20.(x2(n) + x2(n-1) + x2(n-2)) + b21.y2(n-1)
 *      a20 = (1-xt).g // g is signal gain
 *      b21 = xt
 * 3. values used here 
 *         a10 = 0.4844
 *         a11 = -a10
 *         b11 = 0.9375
 *         b21 = 0.7812
 *         a20 = 0.2188     
*/
int isCollision(const float* accData){
    static float x1n_1, x2n_1, x2n_2, y1n_1, y2n_1; 

    // constants
    float a10 = 0.4844, b11 = 0.9375;
    float a11 = -a10;
    float a20 = 0.2188, b21 = 0.7812;
    
    // remove dc components
    float x1n = accData[0] + accData[1] + accData[2];
    float y1n = a10*x1n + a11*x1n_1 + b11*y1n_1;
    float x2n = y1n*y1n;

    // averaging and low pass filter
    float y2n = a20*(x2n + x2n_1 + x2n_2) + b21*y2n_1; 

    // store previous value    
    x1n_1 = x1n;
    y1n_1 = y1n;
    x2n_1 = x2n;
    x2n_2 = x2n_1;
    y2n_1 = y2n;

    int result = y2n > 100 ? 1 : 0;

    return result;

}


#endif // ESTIMATOR_H_ 