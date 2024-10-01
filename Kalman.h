/*
 * Kalman.h
 *
 *  Created on: Sep 25, 2024
 *      Author: jctam
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdio.h>
#include <stdint.h>

#define FIXED_POINT_FACTOR 1024 // Scaling factor (2^10) for fixed-point arithmetic
#define BufferSize 13

//extern FixedKalmanFilter_t *Fxkf;
//extern Median_t *Median;

// Define a structure for the Kalman filter state
typedef struct {
    int32_t x_estimate;       // The estimate (scaled by 65536)
    int32_t p_error_cov;      // The error covariance (scaled by 65536)
    int32_t Q_process_var;    // Process variance (scaled by 65536)
    int32_t R_meas_var;       // Measurement variance (scaled by 65536)
    int32_t kalman_gain;      // Kalman gain (scaled by 65536)
} FixedKalmanFilter_t;



typedef struct {
    int32_t x_estimate;   // Estimated state (x̂)
    float p_error_cov;  // Error covariance (P)
    float Q_process_var; // Process variance (Q)
    float R_meas_var;    // Measurement variance (R)
    float kalman_gain; // Kalman gain (K)
} FloatKalmanFilter_t;


typedef struct {
	uint8_t isfull; // counter that indicates thats full when its = buffersize
	int32_t MedianBuffer[BufferSize];
}Median_t;


// Fixed-point multiply: a * b / FIXED_POINT_FACTOR
static inline uint32_t fixed_point_mul(uint32_t a, uint32_t b) {
    return (a * b) / FIXED_POINT_FACTOR;
}

// Fixed-point divide: a * FIXED_POINT_FACTOR / b
static inline uint32_t fixed_point_div(uint32_t a, uint32_t b) {
    return (a * FIXED_POINT_FACTOR) / b;
}
void Fixedkalman_init(FixedKalmanFilter_t *kf, int32_t initial_estimate, int32_t initial_error_cov, int32_t process_var, int32_t meas_var);
int32_t Fixedkalman_update(FixedKalmanFilter_t *kf, int32_t measurement);
void Median_init(Median_t *Median);
void Add_to_buffer(Median_t *Median, int32_t Val_to_add);
int32_t Get_median_value(Median_t *Median,int32_t Val_to_add);
int32_t Get_deviation(Median_t *Median, int32_t val);
void Floatkalman_init(FloatKalmanFilter_t *kf, uint32_t initial_estimate, uint32_t initial_error_cov, float process_var, float meas_var);
int32_t Floatkalman_update(FloatKalmanFilter_t *kf, int32_t measurement);
int32_t Fixed_Mean_kalmanFilter(Median_t *Median,FixedKalmanFilter_t *Fxkf, int32_t measurement);



#endif /* INC_KALMAN_H_ */
