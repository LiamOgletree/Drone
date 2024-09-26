/*
 * kalmanfilter.c
 *
 *  Created on: Sep 14, 2024
 *      Author: liamt
 */

#include "kalmanfilter.h"

float run_filter(KALMAN_FILTER * const filter,  // Kalman Filter
                 float z,                       // Measurement
                 float dx,                      // Expected motion
                 float dt) {                    // Time elapsed
    // Predict
    filter->x += (dx * dt);
    filter->P += filter->Q;
    // Update
    float K = filter->P / (filter->P + filter->R);
    filter->x += K * (z - filter->x);
    filter->P *= (1 - K);
    // Return filtered measurement
    return filter->x;
}
