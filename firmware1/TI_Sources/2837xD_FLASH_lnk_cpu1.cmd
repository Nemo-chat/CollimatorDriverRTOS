
MEMORY
{
   /* BEGIN is used for the "boot to Flash" bootloader mode */

   BEGIN            : origin = 0x080000, length = 0x000002
   BOOT_RSVD        : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x000123, length = 0x0002DD
   RAMM1            : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */

   RAMLSxDxGSx      : origin = 0x008000, length = 0x008000     /* Start at RAMLS0, end at RAMGS3 */
   RAMGSx           : origin = 0x010000, length = 0x006FFF     /* Start at RAMGS4, end at RAMGS10 */

   CPU2TOCPU1RAM    : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM    : origin = 0x03FC00, length = 0x000400
   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800

   /* Flash sectors */
   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
   FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASHJ           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
   FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASHN           : origin = 0x0BE000, length = 0x001FF0	/* on-chip Flash */

   RESET            : origin = 0x3FFFC0, length = 0x000002
}


SECTIONS
{
   codestart             : > BEGIN
   .text                 : >> FLASHB | FLASHC | FLASHD | FLASHE
   .cinit                : > FLASHA
   .switch               : > FLASHA
   .const                : > FLASHF
   .reset                : > RESET, TYPE = DSECT /* not used */
   .stack                : > RAMLSxDxGSx
   .freertosStaticStack  : >> RAMM1 | RAMM0 | RAMLSxDxGSx
   .freertosHeap         : > RAMLSxDxGSx

#if defined(__TI_EABI__)
   .init_array      : > FLASHA
   .bss             : > RAMGSx
   .bss:output      : > RAMGSx
   .data            : > RAMGSx
   .sysmem          : > RAMLSxDxGSx
#else
   .pinit           : > FLASHA
   .ebss            : > RAMGSx
   .econst          : > FLASHF
   .esysmem         : > RAMLSxDxGSx
#endif

   ramm0                : > RAMM0, type=NOINIT
   ramm1                : > RAMM1, type=NOINIT
   MSGRAM_CPU1_TO_CPU2  : > CPU1TOCPU2RAM, type=NOINIT
   MSGRAM_CPU2_TO_CPU1  : > CPU2TOCPU1RAM, type=NOINIT

#ifdef __TI_COMPILER_VERSION__
   #if __TI_COMPILER_VERSION__ >= 15009000
      #if defined(__TI_EABI__)
         .TI.ramfunc : {} LOAD = FLASHD,
                             RUN = RAMM0,
                             LOAD_START(RamfuncsLoadStart),
                             LOAD_SIZE(RamfuncsLoadSize),
                             LOAD_END(RamfuncsLoadEnd),
                             RUN_START(RamfuncsRunStart),
                             RUN_SIZE(RamfuncsRunSize),
                             RUN_END(RamfuncsRunEnd)
      #else
         .TI.ramfunc : {} LOAD = FLASHD,
                             RUN = RAMM0,
                             LOAD_START(_RamfuncsLoadStart),
                             LOAD_SIZE(_RamfuncsLoadSize),
                             LOAD_END(_RamfuncsLoadEnd),
                             RUN_START(_RamfuncsRunStart),
                             RUN_SIZE(_RamfuncsRunSize),
                             RUN_END(_RamfuncsRunEnd)
      #endif
   #else
      ramfuncs      : LOAD = FLASHD,
                      RUN = RAMM0,
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_SIZE(_RamfuncsLoadSize),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      RUN_SIZE(_RamfuncsRunSize),
                      RUN_END(_RamfuncsRunEnd)
   #endif
#endif
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
