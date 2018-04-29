/***************************************************************************
 **
 **
 **    Master inlude file
 **
 **    Used with ARM IAR C/C++ Compiler
 **
 **    (c) Copyright IAR Systems 2005
 **
 **    $Revision: 1.2 $
 **
 ***************************************************************************/

#ifndef __INCLUDES_H
#define __INCLUDES_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <inarm.h>
#include <iolpc2142.h>
#include "modules/LPC_Vic.h"

#include "../cc1101_lib/cc1101_reg.h"
#include "../cc1101_lib/cc1101_drv.h"
#include "../cc1101_lib/cc1101_bsp.h"
#include "../sensors/accel_drv.h"
#include "../sensors/hyro_drv.h"
#include "algoritm_beg.h"

#endif
