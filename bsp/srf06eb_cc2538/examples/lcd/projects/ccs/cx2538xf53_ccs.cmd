/******************************************************************************
 *
 *
 * cx2538xf53_ccs.cmd - Example CCS linker configuration file for CC2538SF53, CC2538NF53 and CM2538SF53
 *
 *
 *****************************************************************************/

--retain=g_pfnVectors

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */
#define APP_BASE                0x00200000
#define FLASH_CCA_BASE          0x0027FFD4
#define RAM_BASE                0x20000000
#define RAM_NON_RETENTION_BASE  RAM_BASE
#define RAM_RETENTION_BASE      0x20004000

/* System memory map */

MEMORY
{
    /* Application stored in and executes from internal flash */
    /* Flash Size 512 KB */
    FLASH (RX) : origin = APP_BASE, length = 0x0007FFD4
    /* Customer Configuration Area and Bootloader Backdoor configuration in flash, 12 bytes*/
    FLASH_CCA (RX) : origin = FLASH_CCA_BASE, length = 12
    /* Application uses internal RAM for data */
    /* RAM Size 32 KB */
    /* 16 KB of RAM is non-retention */
    SRAM_NON_RETENTION (RWX) : origin = RAM_NON_RETENTION_BASE, length = 0x00004000
    /* 16 KB of RAM is retention. The stack, and all variables that need     */
    /* retention through PM2/3 must be in SRAM_RETENTION */
    SRAM_RETENTION (RWX) : origin = RAM_RETENTION_BASE, length = 0x00004000
}

/* Section allocation in memory */

SECTIONS
{
    .intvecs    :   > APP_BASE
    .text       :   > FLASH
    .const      :   > FLASH
    .cinit      :   > FLASH
    .pinit      :   > FLASH
    .init_array :   > FLASH
    .flashcca   :   > FLASH_CCA

    .vtable     :   > RAM_RETENTION_BASE
    .data       :   > SRAM_RETENTION
    .bss        :   > SRAM_RETENTION
    .sysmem     :   > SRAM_RETENTION
    .stack      :   > SRAM_RETENTION
    .nonretenvar:   > SRAM_NON_RETENTION
}

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;
