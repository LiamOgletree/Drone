/*
 * taskStateMachine.c
 *
 *  Created on: Oct 3, 2024
 *      Author: liamt
 */

#include "StateMachine.h"
#include "taskShared.h"
#include "taskStateMachine.h"
#include <stdbool.h>

// TODO:
//  1. Implement pressure, magnetometer, gyroscope, and GPS updates.

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartStateMachine(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    // Initialize and perform basic state machine setup. NOTE: The state
    //  machine is implemented using a Kalman Filter.
    StateMachine stateMachine;
    SM_ctor(&stateMachine);

    // Storage for temporary variables needed on each iteration.
    RingBuffer_t update;
    RingBuffer_t sm_update;
    RB_STATUS status;
    uint32_t last_update;
    uint32_t dt;
    StateMachine_t state;
    arm_matrix_instance_f32 z_instance;
    SM_UPDATE_TYPE update_type;
    float z[3][1];
    bool first_iteration = true;

    // Update the state machine indefinitely.
    for(;;) {
        // Wait for a producer task to read from a sensor, place the data
        //  in the State Machine ring buffer, and post a semaphore.
        osSemaphoreAcquire(*args.sm_semaphore, osWaitForever);

        // Retrieve an update from the State Machine ring buffer.
        osMutexAcquire(*args.sm_mutex, osWaitForever);
        status = RingBuffer_dequeue(args.sm_ringbuffer, &update);
        osMutexRelease(*args.sm_mutex);

        // If ring buffer dequeue returns a failure, report the error
        //  via the UART ring buffer and then suspend the State Machine task.
        if(status != RB_SUCCESS) {
            task_report_error(args, "STATE MACHINE DEQUEUE ERROR");
            vTaskSuspend(NULL);
        }

        switch(update.type) {
        // If receiving a pressure measurement:
        case RB_PRESSURE:
            break;
        // If receiving a magnetometer measurement:
        case RB_MAGNETOMETER:
            break;
        // If receiving a gyroscope measurement:
        case RB_GYROSCOPE:
            break;
        // If receiving an accelerometer measurement:
        case RB_ACCELEROMETER:
            // Set matrix values and initialize ARM math structure.
            z[0][0] = update.lsm6dso32.X;
            z[1][0] = update.lsm6dso32.Y;
            z[2][0] = update.lsm6dso32.Z;
            arm_mat_init_f32(&z_instance, 3, 1, &z[0][0]);
            // Compute time step.
            dt = update.lsm6dso32.timestamp > last_update
               ? update.lsm6dso32.timestamp - last_update
               : 0xFFFFFFFF - last_update + update.lsm6dso32.timestamp + 1;
            // Set update flag.
            update_type = SM_ACCELEROMETER;
            break;
        // Otherwise, notify of a State Machine error and suspend task.
        default:
            task_report_error(args, "STATE MACHINE UPDATE TYPE ERROR");
            vTaskSuspend(NULL);
        }

        /******** TEMPORARY ********/
        if(update.type != RB_ACCELEROMETER) continue;
        /******** TEMPORARY ********/

        // On the first iteration, we do not have a meaningful last update
        //  time. We set dt=0 to initialize the state to the first reading,
        //  and we set the last update time to the first reading time.
        //  NOTE: Unsigned integer overflow is well-defined.
        if(first_iteration) {
            last_update += dt;
            dt = 0;
            first_iteration = false;
        }

        // Update state machine.
        SM_set_timestep(&stateMachine, dt / (float)TIM5_CLK_SPEED);
        SM_predict(&stateMachine);
        if(SM_update(&stateMachine, z_instance, update_type) != SM_SUCCESS) {
            task_report_error(args, "STATE MACHINE UPDATE ERROR");
            vTaskSuspend(NULL);
        }

        // Store the newest update time, only if we successfully update the
        //  state machine. NOTE: Unsigned integer overflow is well-defined.
        last_update += dt;

        // Update state machine object to pass to UART ring buffer.
        state = (StateMachine_t){
            .pos_x = stateMachine.x[0][0],
            .vel_x = stateMachine.x[1][0],
            .acc_x = stateMachine.x[2][0],
            .pos_y = stateMachine.x[3][0],
            .vel_y = stateMachine.x[4][0],
            .acc_y = stateMachine.x[5][0],
            .pos_z = stateMachine.x[6][0],
            .vel_z = stateMachine.x[7][0],
            .acc_z = stateMachine.x[8][0]
        };

        // Send update to UART ring buffer.
        sm_update = (RingBuffer_t){.type = RB_STATEMACHINE, .state = state};
        task_send_update_log(args, sm_update);
    }
}
