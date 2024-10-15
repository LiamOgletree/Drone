/*
 * kalmfilter.c
 *
 *  Created on: Sep 16, 2024
 *      Author: liamt
 */

#include <StateMachine.h>

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline void set_initial_conditions(StateMachine * const sm)
{
    *sm = (StateMachine){
        .x =  {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}},
        .F =  {{1, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 1, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 1, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 1, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 1, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 1, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 1, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 1, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 1}},
        .P =  {{0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0}},
        .Q =  {{0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0}},
        .Ha = {{0, 0, 0.00980665, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0.00980665, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0, 0.00980665}},
        .Hp = {{0, 0, 0, 1, 0, 0, 0, 0, 0}},
        .Hg = {{1, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 0, 1, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 1, 0, 0}},
        .Ra = {{0.014709975, 0, 0},
               {0, 0.014709975, 0},
               {0, 0, 0.014709975}},
        .Rp = {{1}},
        .Rg = {{1, 0, 0},
               {0, 1, 0},
               {0, 0, 1}}
    };
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void SM_ctor(StateMachine * const sm)
{
    // Set initial conditions for state machine.
    set_initial_conditions(sm);

    // Initialize fundamental matrices for matrix computations.
    arm_mat_init_f32(&sm->x_instance, dim_x, 1, sm->x[0]);
    arm_mat_init_f32(&sm->F_instance, dim_x, dim_x, sm->F[0]);
    arm_mat_init_f32(&sm->P_instance, dim_x, dim_x, sm->P[0]);
    arm_mat_init_f32(&sm->Q_instance, dim_x, dim_x, sm->Q[0]);
    arm_mat_init_f32(&sm->Ha_instance, dim_za, dim_x, sm->Ha[0]);
    arm_mat_init_f32(&sm->Hp_instance, dim_zp, dim_x, sm->Hp[0]);
    arm_mat_init_f32(&sm->Hg_instance, dim_zg, dim_x, sm->Hg[0]);
    arm_mat_init_f32(&sm->Ra_instance, dim_za, dim_za, sm->Ra[0]);
    arm_mat_init_f32(&sm->Rp_instance, dim_zp, dim_zp, sm->Rp[0]);
    arm_mat_init_f32(&sm->Rg_instance, dim_zg, dim_zg, sm->Rg[0]);
    arm_mat_init_f32(&sm->ya_instance, dim_za, 1, sm->ya[0]);
    arm_mat_init_f32(&sm->yp_instance, dim_zp, 1, sm->yp[0]);
    arm_mat_init_f32(&sm->yg_instance, dim_zg, 1, sm->yg[0]);
    arm_mat_init_f32(&sm->Ka_instance, dim_x, dim_za, sm->Ka[0]);
    arm_mat_init_f32(&sm->Kp_instance, dim_x, dim_zp, sm->Kp[0]);
    arm_mat_init_f32(&sm->Kg_instance, dim_x, dim_zg, sm->Kg[0]);
    arm_mat_init_f32(&sm->I_instance, dim_x, dim_x, sm->I[0]);

    // Initialize matrices to store intermediate products.
    arm_mat_init_f32(&sm->Ft_instance, dim_x, dim_x, sm->Ft[0]);
    arm_mat_init_f32(&sm->FP_instance, dim_x, dim_x, sm->FP[0]);
    arm_mat_init_f32(&sm->FPFt_instance, dim_x, dim_x, sm->FPFt[0]);
    arm_mat_init_f32(&sm->Hxa_instance, dim_za, 1, sm->Hxa[0]);
    arm_mat_init_f32(&sm->Hxp_instance, dim_zp, 1, sm->Hxp[0]);
    arm_mat_init_f32(&sm->Hxg_instance, dim_zg, 1, sm->Hxg[0]);
    arm_mat_init_f32(&sm->Hta_instance, dim_x, dim_za, sm->Hta[0]);
    arm_mat_init_f32(&sm->Htp_instance, dim_x, dim_zp, sm->Htp[0]);
    arm_mat_init_f32(&sm->Htg_instance, dim_x, dim_zg, sm->Htg[0]);
    arm_mat_init_f32(&sm->PHta_instance, dim_x, dim_za, sm->PHta[0]);
    arm_mat_init_f32(&sm->PHtp_instance, dim_x, dim_zp, sm->PHtp[0]);
    arm_mat_init_f32(&sm->PHtg_instance, dim_x, dim_zg, sm->PHtg[0]);
    arm_mat_init_f32(&sm->HPHta_instance, dim_za, dim_za, sm->HPHta[0]);
    arm_mat_init_f32(&sm->HPHtp_instance, dim_zp, dim_zp, sm->HPHtp[0]);
    arm_mat_init_f32(&sm->HPHtg_instance, dim_zg, dim_zg, sm->HPHtg[0]);
    arm_mat_init_f32(&sm->Sa_instance, dim_za, dim_za, sm->Sa[0]);
    arm_mat_init_f32(&sm->Sp_instance, dim_zp, dim_zp, sm->Sp[0]);
    arm_mat_init_f32(&sm->Sg_instance, dim_zg, dim_zg, sm->Sg[0]);
    arm_mat_init_f32(&sm->Sia_instance, dim_za, dim_za, sm->Sia[0]);
    arm_mat_init_f32(&sm->Sip_instance, dim_zp, dim_zp, sm->Sip[0]);
    arm_mat_init_f32(&sm->Sig_instance, dim_zg, dim_zg, sm->Sig[0]);
    arm_mat_init_f32(&sm->Ky_instance, dim_x, 1, sm->Ky[0]);
    arm_mat_init_f32(&sm->KH_instance, dim_x, dim_x, sm->KH[0]);
    arm_mat_init_f32(&sm->IKH_instance, dim_x, dim_x, sm->IKH[0]);
    arm_mat_init_f32(&sm->IKHP_instance, dim_x, dim_x, sm->IKHP[0]);
}

void SM_set_timestep(StateMachine * const sm,
                      float const dt)
{
    // Update process and noise matrices to reflect time step for
    //  most recent sensor update.
    sm->Q[2][2] = dt;
    sm->Q[5][5] = dt;
    sm->Q[8][8] = dt;
    sm->F[0][1] = dt;
    sm->F[0][2] = 0.5 * dt * dt;
    sm->F[1][2] = dt;
    sm->F[3][4] = dt;
    sm->F[3][5] = 0.5 * dt * dt;
    sm->F[4][5] = dt;
    sm->F[6][7] = dt;
    sm->F[6][8] = 0.5 * dt * dt;
    sm->F[7][8] = dt;
}

void SM_predict(StateMachine * const sm)
{
    // For clarity, create temporary shorthand variables for
    //  matrix computations.
    arm_matrix_instance_f32 * x    = &sm->x_instance;
    arm_matrix_instance_f32 * F    = &sm->F_instance;
    arm_matrix_instance_f32 * Q    = &sm->Q_instance;
    arm_matrix_instance_f32 * P    = &sm->P_instance;
    arm_matrix_instance_f32 * Ft   = &sm->Ft_instance;
    arm_matrix_instance_f32 * FP   = &sm->FP_instance;
    arm_matrix_instance_f32 * FPFt = &sm->FPFt_instance;

    // Compute: x = Fx
    arm_mat_mult_f32(F, x, x);

    // Compute: P = FPFt + Q
    arm_mat_trans_f32(F, Ft);
    arm_mat_mult_f32(F, P, FP);
    arm_mat_mult_f32(FP, Ft, FPFt);
    arm_mat_add_f32(FPFt, Q, P);
}

SM_STATUS SM_update(StateMachine * const sm,
                    arm_matrix_instance_f32 const z,
                    SM_UPDATE_TYPE const sm_update_type)
{
    // For clarity, create temporary shorthand variables for
    //  matrix computations. Dynamically assign a number of
    //  matrices based on whether the sensor update is an
    //  accelerometer, pressure, or GPS reading.
    arm_matrix_instance_f32 * x    = &sm->x_instance;
    arm_matrix_instance_f32 * P    = &sm->P_instance;
    arm_matrix_instance_f32 * Ky   = &sm->Ky_instance;
    arm_matrix_instance_f32 * KH   = &sm->KH_instance;
    arm_matrix_instance_f32 * I    = &sm->I_instance;
    arm_matrix_instance_f32 * IKH  = &sm->IKH_instance;
    arm_matrix_instance_f32 * IKHP = &sm->IKHP_instance;
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

    switch(sm_update_type) {
    // If update is due to an accelerometer measurement:
    case SM_ACCELEROMETER:
        H = &sm->Ha_instance;
        R = &sm->Ra_instance;
        y = &sm->ya_instance;
        K = &sm->Ka_instance;
        Hx = &sm->Hxa_instance;
        Ht = &sm->Hta_instance;
        PHt = &sm->PHta_instance;
        HPHt = &sm->HPHta_instance;
        S = &sm->Sa_instance;
        Si = &sm->Sia_instance;
        break;
    // If update is due to a pressure measurement:
    case SM_PRESSURE:
        H = &sm->Hp_instance;
        R = &sm->Rp_instance;
        y = &sm->yp_instance;
        K = &sm->Kp_instance;
        Hx = &sm->Hxp_instance;
        Ht = &sm->Htp_instance;
        PHt = &sm->PHtp_instance;
        HPHt = &sm->HPHtp_instance;
        S = &sm->Sp_instance;
        Si = &sm->Sip_instance;
        break;
    // If update is due to a GPS measurement:
    case SM_GPS:
        H = &sm->Hg_instance;
        R = &sm->Rg_instance;
        y = &sm->yg_instance;
        K = &sm->Kg_instance;
        Hx = &sm->Hxg_instance;
        Ht = &sm->Htg_instance;
        PHt = &sm->PHtg_instance;
        HPHt = &sm->HPHtg_instance;
        S = &sm->Sg_instance;
        Si = &sm->Sig_instance;
        break;
    default:
        return SM_FAILURE;
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

    return SM_SUCCESS;
}
