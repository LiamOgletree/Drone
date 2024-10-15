/*
 * kalmanfilter.h
 *
 *  Created on: Sep 16, 2024
 *      Author: liamt
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include "main.h"
#include "arm_math.h"

#define dim_x  (9)  // dimension of state machine
#define dim_za (3)  // dimension of accelerometer measurement
#define dim_zp (1)  // dimension of pressure measurement
#define dim_zg (3)  // dimension of GPS measurement

typedef struct {
    float pos_x;
    float vel_x;
    float acc_x;
    float pos_y;
    float vel_y;
    float acc_y;
    float pos_z;
    float vel_z;
    float acc_z;
} StateMachine_t;

typedef struct StateMachine {
    // fundamental matrices
    float x[dim_x][1];           // state mean
    float F[dim_x][dim_x];       // state covariance
    float P[dim_x][dim_x];       // state transition function
    float Q[dim_x][dim_x];       // process covariance
    float Ha[dim_za][dim_x];     // measurement function - accelerometer
    float Hp[dim_zp][dim_x];     // measurement function - pressure
    float Hg[dim_zg][dim_x];     // measurement function - GPS
    float Ra[dim_za][dim_za];    // measurement covariance - accelerometer
    float Rp[dim_zp][dim_zp];    // measurement covariance - pressure
    float Rg[dim_zg][dim_zg];    // measurement covariance - GPS
    float ya[dim_za][1];         // residual - accelerometer
    float yp[dim_zp][1];         // residual - pressure
    float yg[dim_zg][1];         // residual - GPS
    float Ka[dim_x][dim_za];     // kalman gain - accelerometer
    float Kp[dim_x][dim_zp];     // kalman gain - pressure
    float Kg[dim_x][dim_zg];     // kalman gain - GPS
    float I[dim_x][dim_x];       // identity matrix

    // intermediate products
    float Ft[dim_x][dim_x];      // F transpose
    float FP[dim_x][dim_x];      // F * P
    float FPFt[dim_x][dim_x];    // F * P * F transpose
    float Hxa[dim_za][1];        // H * x - accelerometer
    float Hxp[dim_zp][1];        // H * x - pressure
    float Hxg[dim_zg][1];        // H * x - GPS
    float Hta[dim_x][dim_za];    // H transpose - accelerometer
    float Htp[dim_x][dim_zp];    // H transpose - pressure
    float Htg[dim_x][dim_zg];    // H transpose - GPS
    float PHta[dim_x][dim_za];   // P * H transpose - accelerometer
    float PHtp[dim_x][dim_zp];   // P * H transpose - pressure
    float PHtg[dim_x][dim_zg];   // P * H transpose - GPS
    float HPHta[dim_za][dim_za]; // H * P * H transpose - accelerometer
    float HPHtp[dim_zp][dim_zp]; // H * P * H transpose - pressure
    float HPHtg[dim_zg][dim_zg]; // H * P * H transpose - GPS
    float Sa[dim_za][dim_za];    // S = H * P * H tranpose + R - accelerometer
    float Sp[dim_zp][dim_zp];    // S = H * P * H tranpose + R - pressure
    float Sg[dim_zg][dim_zg];    // S = H * P * H tranpose + R - GPS
    float Sia[dim_za][dim_za];   // S^(-1) - accelerometer
    float Sip[dim_zp][dim_zp];   // S^(-1) - pressure
    float Sig[dim_zg][dim_zg];   // S^(-1) - GPS
    float Ky[dim_x][1];          // K * y
    float KH[dim_x][dim_x];      // K * H
    float IKH[dim_x][dim_x];     // I - K * H
    float IKHP[dim_x][dim_x];    // (I - K * H) * P

    // matrix representation required by CMSIS DSP package
    arm_matrix_instance_f32 x_instance;
    arm_matrix_instance_f32 F_instance;
    arm_matrix_instance_f32 P_instance;
    arm_matrix_instance_f32 Q_instance;
    arm_matrix_instance_f32 Ha_instance;
    arm_matrix_instance_f32 Hp_instance;
    arm_matrix_instance_f32 Hg_instance;
    arm_matrix_instance_f32 Ra_instance;
    arm_matrix_instance_f32 Rp_instance;
    arm_matrix_instance_f32 Rg_instance;
    arm_matrix_instance_f32 ya_instance;
    arm_matrix_instance_f32 yp_instance;
    arm_matrix_instance_f32 yg_instance;
    arm_matrix_instance_f32 Ka_instance;
    arm_matrix_instance_f32 Kp_instance;
    arm_matrix_instance_f32 Kg_instance;
    arm_matrix_instance_f32 I_instance;
    arm_matrix_instance_f32 Ft_instance;
    arm_matrix_instance_f32 FP_instance;
    arm_matrix_instance_f32 FPFt_instance;
    arm_matrix_instance_f32 Hxa_instance;
    arm_matrix_instance_f32 Hxp_instance;
    arm_matrix_instance_f32 Hxg_instance;
    arm_matrix_instance_f32 Hta_instance;
    arm_matrix_instance_f32 Htp_instance;
    arm_matrix_instance_f32 Htg_instance;
    arm_matrix_instance_f32 PHta_instance;
    arm_matrix_instance_f32 PHtp_instance;
    arm_matrix_instance_f32 PHtg_instance;
    arm_matrix_instance_f32 HPHta_instance;
    arm_matrix_instance_f32 HPHtp_instance;
    arm_matrix_instance_f32 HPHtg_instance;
    arm_matrix_instance_f32 Sa_instance;
    arm_matrix_instance_f32 Sp_instance;
    arm_matrix_instance_f32 Sg_instance;
    arm_matrix_instance_f32 Sia_instance;
    arm_matrix_instance_f32 Sip_instance;
    arm_matrix_instance_f32 Sig_instance;
    arm_matrix_instance_f32 Ky_instance;
    arm_matrix_instance_f32 KH_instance;
    arm_matrix_instance_f32 IKH_instance;
    arm_matrix_instance_f32 IKHP_instance;
} StateMachine;

typedef enum {
    SM_ACCELEROMETER,
    SM_PRESSURE,
    SM_GPS
} SM_UPDATE_TYPE;

typedef enum {
    SM_SUCCESS,
    SM_FAILURE
} SM_STATUS;

void SM_ctor(StateMachine * const sm);
void SM_set_timestep(StateMachine * const sm,
                     float const dt);
void SM_predict(StateMachine * const sm);
SM_STATUS SM_update(StateMachine * const sm,
                    arm_matrix_instance_f32 const z,
                    SM_UPDATE_TYPE const sm_update_type);

#endif /* INC_STATEMACHINE_H_ */
