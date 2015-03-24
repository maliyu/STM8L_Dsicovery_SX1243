/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : task.h
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#ifndef _TASK_H_
#define _TASK_H_

#define FIRMWARE_VERSION        "3.4.2"

typedef struct sTaskInstance
{
  void *p_device1;
  void *p_data;
  void *p_dataLen;
}tTaskInstance;

tTaskInstance* task_init(void);
void task_exec(tTaskInstance *task);

#endif /* _TASK_H_ */
