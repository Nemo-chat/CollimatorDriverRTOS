
MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to Flash" bootloader mode */
   BEGIN            : origin = 0x080000, length = 0x000002
   RESET            : origin = 0x3FFFC0, length = 0x000002

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

   /* Program RAM for ramfunc execution */
   RAMLS0           : origin = 0x008000, length = 0x000800

PAGE 1 :
   BOOT_RSVD        : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x000123, length = 0x0002DD
   RAMM1            : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */

   /* Data RAM (LS1..D1) */
   RAMLSxDx         : origin = 0x008800, length = 0x003800
   RAMGSx           : origin = 0x017000, length = 0x004FFF     /* Start at RAMGS11, end at RAMGS15 */

   CPU2TOCPU1RAM    : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM    : origin = 0x03FC00, length = 0x000400
   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
}

SECTIONS
{
   codestart             : > BEGIN, PAGE = 0
   .text                 : >> FLASHB | FLASHC | FLASHD | FLASHE, PAGE = 0
   .cinit                : > FLASHA, PAGE = 0
   .switch               : > FLASHA, PAGE = 0
   .const                : > FLASHF, PAGE = 0
   .reset                : > RESET, PAGE = 0, TYPE = DSECT /* not used */

   .stack                : > RAMLSxDx, PAGE = 1
   .freertosStaticStack  : >> RAMM1 | RAMM0, PAGE = 1
   .freertosHeap         : > RAMLSxDx, PAGE = 1

#if defined(__TI_EABI__)
   .init_array      : > FLASHA, PAGE = 0
   .bss             : > RAMGSx, PAGE = 1
   .bss:output      : > RAMGSx, PAGE = 1
   .data            : > RAMLSxDx, PAGE = 1
   .sysmem          : > RAMGSx, PAGE = 1
#else
   .pinit           : > FLASHA, PAGE = 0
   .ebss            : > RAMGSx, PAGE = 1
   .econst          : > FLASHF, PAGE = 0
   .esysmem         : > RAMLSxDx, PAGE = 1
#endif

   ramm0                : > RAMM0, PAGE = 1, type=NOINIT
   ramm1                : > RAMM1, PAGE = 1, type=NOINIT
   MSGRAM_CPU1_TO_CPU2  : > CPU1TOCPU2RAM, PAGE = 1, type=NOINIT
   MSGRAM_CPU2_TO_CPU1  : > CPU2TOCPU1RAM, PAGE = 1, type=NOINIT

   /* Keep this unconditional to satisfy compiler-generated .TI.ramfunc section. */
   .TI.ramfunc : {} LOAD = FLASHD,
                       RUN = RAMLS0,
                       LOAD_START(RamfuncsLoadStart),
                       LOAD_SIZE(RamfuncsLoadSize),
                       LOAD_END(RamfuncsLoadEnd),
                       RUN_START(RamfuncsRunStart),
                       RUN_SIZE(RamfuncsRunSize),
                       RUN_END(RamfuncsRunEnd),
                       PAGE = 0
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
