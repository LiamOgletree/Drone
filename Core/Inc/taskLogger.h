/*
 * taskUART.h
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#ifndef INC_TASKLOGGER_H_
#define INC_TASKLOGGER_H_

typedef enum {
    LOGGER_SUCCESS,
    LOGGER_FAILURE
} LOGGER_STATUS;

void StartLogger(void *argument);

#endif /* INC_TASKLOGGER_H_ */
