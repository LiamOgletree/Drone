/*
 * kalmanfilter.h
 *
 *  Created on: Sep 14, 2024
 *      Author: liamt
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include "main.h"

typedef struct {
    float P;    // State variance
    float Q;    // Process variance
    float R;    // Measurement variance
    float x;    // Estimate
} KALMAN_FILTER;

float run_filter(KALMAN_FILTER * const filter,  // Persistent Kalman Filter
                 float z,                       // Measurement
                 float dx,                      // Expected motion
                 float dt);                     // Time elapsed

#endif /* INC_KALMANFILTER_H_ */
