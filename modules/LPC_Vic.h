#ifndef __LPC_VIC_H
#define __LPC_VIC_H

/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2003
 *
 *    File name   : LPC_Vic.h
 *    Description :
 *
 *    History :
 *    1. Data: July 8th, 2004
 *       Author: Wilson Liu
 *       Description: Create the basic function
 *
 *    2. Data: August 4th, 2004
 *       Author: Shawn Zhang
 *       Description: Clean up the functions. Support nonvector interrupt at first.
 *
 *    3. Data        : Oct 11, 2004
 *       Author      : Stanimir Bonev
 *       Description : Modify some function and interface
 *
 *    $Revision: 1.2 $
 **************************************************************************/
#include "includes.h"

#define IRQ_FLAG 0x80

// Interrupt protection type
typedef enum {
    UserandPrivilegedMode = 0,
    PrivilegedMode
} LPC_Vic_ProtectionMode_t;

typedef enum {
    VIC_Slot0 = 0, // high priority
    VIC_Slot1,
    VIC_Slot2,
    VIC_Slot3,
    VIC_Slot4,
    VIC_Slot5,
    VIC_Slot6,
    VIC_Slot7,
    VIC_Slot8,
    VIC_Slot9,
    VIC_Slot10,
    VIC_Slot11,
    VIC_Slot12,
    VIC_Slot13,
    VIC_Slot14,
    VIC_Slot15
} LPC_VicIrqSlots_t;

/* Declare API functions */
void VIC_SetProtectionMode(LPC_Vic_ProtectionMode_t ProtectionType);
LPC_Vic_ProtectionMode_t VIC_GetProtectionMode(void);

void VIC_Init(void);

void VIC_EnableInt(unsigned int IntType);
void VIC_DisableInt(unsigned int IntType);

unsigned int VIC_GetIRQStatus(void);
unsigned int VIC_GetFIQStatus(void);

void VIC_EnableNonVectoredIRQ(void (*pIRQSub)());
void VIC_DisableNonVectoredIRQ(void);

void VIC_SetVectoredIRQ(void (*pIRQSub)(), LPC_VicIrqSlots_t VicIrqSlot, unsigned int VicIntSouce);

/*************************************************************************
 * Function Name: restore_IRQ
 * Parameters: unsigned long IFlag
 * Return: void
 * Description: Restore I flag state
 *
 *************************************************************************/
#pragma inline
__arm void restore_IRQ(unsigned long IFlag)
{
    unsigned long tmp;
    tmp = __get_CPSR();
    __set_CPSR(tmp & (IFlag | ~IRQ_FLAG));
}

/*************************************************************************
 * Function Name: disable_IRQ
 * Parameters:
 * Return: unsigned long
 * Description: Disable IRQ and return previous state state of flgas I
 *
 *************************************************************************/
#pragma inline
__arm unsigned long disable_IRQ(void)
{
    unsigned long tmp;
    tmp = __get_CPSR();
    __set_CPSR(tmp | IRQ_FLAG);
    return tmp & IRQ_FLAG;
}

/*************************************************************************
 * Function Name: restore_interrupts
 * Parameters: unsigned long IFlag
 * Return: void
 * Description: Restore F,I flag state
 *
 *************************************************************************/
#pragma inline
__arm void restore_interrupts(unsigned long IFlag)
{
    __set_CPSR(IFlag);
}

/*************************************************************************
 * Function Name: disable_interrupts
 * Parameters:
 * Return: unsigned long
 * Description: Disable interrupts and return previous state state of flgas I
 *
 *************************************************************************/
#pragma inline
__arm unsigned long disable_interrupts(void)
{
    unsigned long tmp;
    tmp = __get_CPSR();
    __disable_interrupt();
    return tmp;
}
#endif // __LPC_VIC_H
