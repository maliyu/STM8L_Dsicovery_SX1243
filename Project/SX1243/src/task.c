/******************************************************************************
 * Project        : STM8L_Discovery_SX1278_SFM1L
 * File           : task.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#if defined(STM8S003)
#include "stm8s.h"
#elif defined(STM8L15X_MD)
#include <string.h>
#include "stm8l15x_flash.h"
#include "stm8l15x.h"
#endif
#include "sx1243.h"
#include "task.h"
#include "board.h"

