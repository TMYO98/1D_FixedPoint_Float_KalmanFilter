/*
 * Kalman.c
 *
 *  Created on: Sep 25, 2024
 *      Author: jctam
 */
#include <stdio.h>
#include <stdint.h>
#include <kalman.h>
#include <math.h>

void Fixedkalman_init(FixedKalmanFilter_t *kf, int32_t initial_estimate, int32_t initial_error_cov, int32_t process_var, int32_t meas_var) {
    // make sure that you scale the initial estimates and variances by 65536 before introducing them as an argument
//    kf = (FixedKalmanFilter_t *)malloc(sizeof(FixedKalmanFilter_t));
	kf->x_estimate = initial_estimate ;
    kf->p_error_cov = initial_error_cov;
    kf->Q_process_var = process_var;  // Scale 0.003
    kf->R_meas_var = meas_var;        // Scale 0.05
}


int32_t Fixedkalman_update(FixedKalmanFilter_t *kf, int32_t measurement) {
    // p_error_cov += Q_process_var
    kf->p_error_cov += kf->Q_process_var;

    // kalman_gain = p_error_cov / (p_error_cov + R_meas_var)
    kf->kalman_gain = (kf->p_error_cov * 65536) / (kf->p_error_cov + kf->R_meas_var);

    // x_estimate = x_estimate + kalman_gain * (measurement - x_estimate)
    kf->x_estimate += (kf->kalman_gain * (measurement - kf->x_estimate)) / 65536;

    // p_error_cov = (1 - kalman_gain) * p_error_cov
    kf->p_error_cov = ((65536 - kf->kalman_gain) * kf->p_error_cov) / 65536;


    return kf->x_estimate;
}


void Median_init(Median_t *Median){
//	Median = (Median_t *)malloc(sizeof(Median_t));
	for (uint8_t i =0; i<BufferSize;i++){
		Median->MedianBuffer[i] = 0;
	}
	Median->isfull = 0;
}
void Add_to_buffer(Median_t *Median, int32_t Val_to_add){
    // Shift elements to the left
    for(uint8_t i = 0; i < BufferSize-1; i++){
        Median->MedianBuffer[i] = Median->MedianBuffer[i+1];
    }
    // Add the new value at the end
    Median->MedianBuffer[BufferSize-1] = Val_to_add;
}

int32_t Get_median_value(Median_t *Median,int32_t Val_to_add){
    int32_t Medianval = 0;
    if (Median->isfull == BufferSize) {
        for (uint8_t i = 0; i < BufferSize; i++) {
            Medianval += Median->MedianBuffer[i];
        }
        Add_to_buffer(Median, Val_to_add);
        return (int32_t)(Medianval / BufferSize); // Safely return the average
    }
    Median->isfull += 1;
    Add_to_buffer(Median, Val_to_add);
    return Val_to_add;
}
int32_t Get_deviation(Median_t *Median, int32_t val){
	int32_t mean = Get_median_value(Median, val);
	int32_t variance = 0;
	for (uint8_t i = 0; i < BufferSize; i++) {
		variance += (Median->MedianBuffer[i]-mean) * (Median->MedianBuffer[i]-mean);
	}
	variance = variance/(BufferSize-1);
	return sqrt(variance)*1;
}

void Floatkalman_init(FloatKalmanFilter_t *kf, uint32_t initial_estimate, uint32_t initial_error_cov, float process_var, float meas_var) {
    kf->x_estimate = initial_estimate ;
    kf->p_error_cov = initial_error_cov ;
    kf->Q_process_var = process_var ;
    kf->R_meas_var = meas_var ;
}

int32_t Floatkalman_update(FloatKalmanFilter_t *kf, int32_t measurement){
	kf->p_error_cov += kf->Q_process_var;
	kf->kalman_gain = kf->p_error_cov/(kf->p_error_cov + kf->R_meas_var);
	kf->x_estimate = kf->x_estimate + kf->kalman_gain * (measurement-kf->x_estimate);
	kf->p_error_cov = ((1-kf->kalman_gain) * kf->p_error_cov);
	return kf->x_estimate;
}


int32_t Fixed_Mean_kalmanFilter(Median_t *Median,FixedKalmanFilter_t *Fxkf, int32_t measurement){
//	Add_to_buffer(Median, measurement);
	int32_t mean = Get_median_value(Median,measurement );
	int32_t absmean = abs(mean);
	int32_t absmeasure = abs(measurement);
	if(absmeasure>absmean*1.5 ||absmeasure<absmean*.5 ){
		measurement = Median->MedianBuffer[BufferSize-2];
//		measurement = mean;

//		Median->MedianBuffer[BufferSize-1] = measurement;
	}
	return Fixedkalman_update(Fxkf, measurement);
}

void Filters_init(Median_t *Median,FixedKalmanFilter_t *Fxkf,int32_t initial_estimate, int32_t initial_error_cov, int32_t process_var, int32_t meas_var){
	Fixedkalman_init(Fxkf, initial_estimate, initial_error_cov, process_var, meas_var);
	Median_init(Median);

}
