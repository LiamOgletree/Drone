/*
 * kalmfilter.c
 *
 *  Created on: Sep 16, 2024
 *      Author: liamt
 */

#include "kalmanfilter.h"

void filter_ctor(KALMAN_FILTER * const filter)
{
    // Initialize fundamental matrices for matrix computations.
    arm_mat_init_f32(&filter->x_instance, dim_x, 1, filter->x[0]);
    arm_mat_init_f32(&filter->F_instance, dim_x, dim_x, filter->F[0]);
    arm_mat_init_f32(&filter->P_instance, dim_x, dim_x, filter->P[0]);
    arm_mat_init_f32(&filter->Q_instance, dim_x, dim_x, filter->Q[0]);
    arm_mat_init_f32(&filter->Ha_instance, dim_za, dim_x, filter->Ha[0]);
    arm_mat_init_f32(&filter->Hp_instance, dim_zp, dim_x, filter->Hp[0]);
    arm_mat_init_f32(&filter->Hg_instance, dim_zg, dim_x, filter->Hg[0]);
    arm_mat_init_f32(&filter->Ra_instance, dim_za, dim_za, filter->Ra[0]);
    arm_mat_init_f32(&filter->Rp_instance, dim_zp, dim_zp, filter->Rp[0]);
    arm_mat_init_f32(&filter->Rg_instance, dim_zg, dim_zg, filter->Rg[0]);
    arm_mat_init_f32(&filter->ya_instance, dim_za, 1, filter->ya[0]);
    arm_mat_init_f32(&filter->yp_instance, dim_zp, 1, filter->yp[0]);
    arm_mat_init_f32(&filter->yg_instance, dim_zg, 1, filter->yg[0]);
    arm_mat_init_f32(&filter->Ka_instance, dim_x, dim_za, filter->Ka[0]);
    arm_mat_init_f32(&filter->Kp_instance, dim_x, dim_zp, filter->Kp[0]);
    arm_mat_init_f32(&filter->Kg_instance, dim_x, dim_zg, filter->Kg[0]);
    arm_mat_init_f32(&filter->I_instance, dim_x, dim_x, filter->I[0]);

    // Initialize matrices to store intermediate products.
    arm_mat_init_f32(&filter->Ft_instance, dim_x, dim_x, filter->Ft[0]);
    arm_mat_init_f32(&filter->FP_instance, dim_x, dim_x, filter->FP[0]);
    arm_mat_init_f32(&filter->FPFt_instance, dim_x, dim_x, filter->FPFt[0]);
    arm_mat_init_f32(&filter->Hxa_instance, dim_za, 1, filter->Hxa[0]);
    arm_mat_init_f32(&filter->Hxp_instance, dim_zp, 1, filter->Hxp[0]);
    arm_mat_init_f32(&filter->Hxg_instance, dim_zg, 1, filter->Hxg[0]);
    arm_mat_init_f32(&filter->Hta_instance, dim_x, dim_za, filter->Hta[0]);
    arm_mat_init_f32(&filter->Htp_instance, dim_x, dim_zp, filter->Htp[0]);
    arm_mat_init_f32(&filter->Htg_instance, dim_x, dim_zg, filter->Htg[0]);
    arm_mat_init_f32(&filter->PHta_instance, dim_x, dim_za, filter->PHta[0]);
    arm_mat_init_f32(&filter->PHtp_instance, dim_x, dim_zp, filter->PHtp[0]);
    arm_mat_init_f32(&filter->PHtg_instance, dim_x, dim_zg, filter->PHtg[0]);
    arm_mat_init_f32(&filter->HPHta_instance, dim_za, dim_za, filter->HPHta[0]);
    arm_mat_init_f32(&filter->HPHtp_instance, dim_zp, dim_zp, filter->HPHtp[0]);
    arm_mat_init_f32(&filter->HPHtg_instance, dim_zg, dim_zg, filter->HPHtg[0]);
    arm_mat_init_f32(&filter->Sa_instance, dim_za, dim_za, filter->Sa[0]);
    arm_mat_init_f32(&filter->Sp_instance, dim_zp, dim_zp, filter->Sp[0]);
    arm_mat_init_f32(&filter->Sg_instance, dim_zg, dim_zg, filter->Sg[0]);
    arm_mat_init_f32(&filter->Sia_instance, dim_za, dim_za, filter->Sia[0]);
    arm_mat_init_f32(&filter->Sip_instance, dim_zp, dim_zp, filter->Sip[0]);
    arm_mat_init_f32(&filter->Sig_instance, dim_zg, dim_zg, filter->Sig[0]);
    arm_mat_init_f32(&filter->Ky_instance, dim_x, 1, filter->Ky[0]);
    arm_mat_init_f32(&filter->KH_instance, dim_x, dim_x, filter->KH[0]);
    arm_mat_init_f32(&filter->IKH_instance, dim_x, dim_x, filter->IKH[0]);
    arm_mat_init_f32(&filter->IKHP_instance, dim_x, dim_x, filter->IKHP[0]);
}

void filter_set_timestep(KALMAN_FILTER * const filter,
                         float const dt)
{
    // Update process and noise matrices to reflect timestep for
    //  most recent sensor update.
    filter->Q[2][2] = dt;
    filter->Q[5][5] = dt;
    filter->Q[8][8] = dt;
    filter->F[0][1] = dt;
    filter->F[0][2] = 0.5 * dt * dt;
    filter->F[1][2] = dt;
    filter->F[3][4] = dt;
    filter->F[3][5] = 0.5 * dt * dt;
    filter->F[4][5] = dt;
    filter->F[6][7] = dt;
    filter->F[6][8] = 0.5 * dt * dt;
    filter->F[7][8] = dt;
}

void filter_predict(KALMAN_FILTER * const filter)
{
    // For clarity, create temporary shorthand variables for
    //  matrix computations.
    arm_matrix_instance_f32 * x    = &filter->x_instance;
    arm_matrix_instance_f32 * F    = &filter->F_instance;
    arm_matrix_instance_f32 * Q    = &filter->Q_instance;
    arm_matrix_instance_f32 * P    = &filter->P_instance;
    arm_matrix_instance_f32 * Ft   = &filter->Ft_instance;
    arm_matrix_instance_f32 * FP   = &filter->FP_instance;
    arm_matrix_instance_f32 * FPFt = &filter->FPFt_instance;

    // Compute: x = Fx
    arm_mat_mult_f32(F, x, x);

    // Compute: P = FPFt + Q
    arm_mat_trans_f32(F, Ft);
    arm_mat_mult_f32(F, P, FP);
    arm_mat_mult_f32(FP, Ft, FPFt);
    arm_mat_add_f32(FPFt, Q, P);
}

void filter_update(KALMAN_FILTER * const filter,
                   arm_matrix_instance_f32 const z,
                   FILTER_TYPE const filter_type)
{
    // For clarity, create temporary shorthand variables for
    //  matrix computations. Dynamically assign a number of
    //  matrices based on whether the sensor update is an
    //  accelerometer, pressure, or GPS reading.
    arm_matrix_instance_f32 * x    = &filter->x_instance;
    arm_matrix_instance_f32 * P    = &filter->P_instance;
    arm_matrix_instance_f32 * Ky   = &filter->Ky_instance;
    arm_matrix_instance_f32 * KH   = &filter->KH_instance;
    arm_matrix_instance_f32 * I    = &filter->I_instance;
    arm_matrix_instance_f32 * IKH  = &filter->IKH_instance;
    arm_matrix_instance_f32 * IKHP = &filter->IKHP_instance;
    arm_matrix_instance_f32 * H;
    arm_matrix_instance_f32 * R;
    arm_matrix_instance_f32 * y;
    arm_matrix_instance_f32 * K;
    arm_matrix_instance_f32 * Hx;
    arm_matrix_instance_f32 * Ht;
    arm_matrix_instance_f32 * PHt;
    arm_matrix_instance_f32 * HPHt;
    arm_matrix_instance_f32 * S;
    arm_matrix_instance_f32 * Si;

    // Dynamically assign matrices based on the sensor type.
    switch(filter_type) {
    case FILTER_ACCELEROMETER:
        H = &filter->Ha_instance;
        R = &filter->Ra_instance;
        y = &filter->ya_instance;
        K = &filter->Ka_instance;
        Hx = &filter->Hxa_instance;
        Ht = &filter->Hta_instance;
        PHt = &filter->PHta_instance;
        HPHt = &filter->HPHta_instance;
        S = &filter->Sa_instance;
        Si = &filter->Sia_instance;
        break;
    case FILTER_PRESSURE:
        H = &filter->Hp_instance;
        R = &filter->Rp_instance;
        y = &filter->yp_instance;
        K = &filter->Kp_instance;
        Hx = &filter->Hxp_instance;
        Ht = &filter->Htp_instance;
        PHt = &filter->PHtp_instance;
        HPHt = &filter->HPHtp_instance;
        S = &filter->Sp_instance;
        Si = &filter->Sip_instance;
        break;
    case FILTER_GPS:
        H = &filter->Hg_instance;
        R = &filter->Rg_instance;
        y = &filter->yg_instance;
        K = &filter->Kg_instance;
        Hx = &filter->Hxg_instance;
        Ht = &filter->Htg_instance;
        PHt = &filter->PHtg_instance;
        HPHt = &filter->HPHtg_instance;
        S = &filter->Sg_instance;
        Si = &filter->Sig_instance;
        break;
    default:
        // do nothing for now
        break;
    }

    // Compute: y = z - Hx
    arm_mat_mult_f32(H, x, Hx);
    arm_mat_sub_f32(&z, Hx, y);

    // Compute: K = PHt(HPHt + R)^(-1) = PHtS^(-1)
    arm_mat_trans_f32(H, Ht);
    arm_mat_mult_f32(P, Ht, PHt);
    arm_mat_mult_f32(H, PHt, HPHt);
    arm_mat_add_f32(HPHt, R, S);
    arm_mat_inverse_f32(S, Si);
    arm_mat_mult_f32(PHt, Si, K);

    // Compute: x = x + Ky
    arm_mat_mult_f32(K, y, Ky);
    arm_mat_add_f32(x, Ky, x);

    // Compute: P = (I - KH)P
    arm_mat_mult_f32(K, H, KH);
    arm_mat_sub_f32(I, KH, IKH);
    arm_mat_mult_f32(IKH, P, IKHP);
    arm_mat_mult_f32(IKHP, I, P);
}
