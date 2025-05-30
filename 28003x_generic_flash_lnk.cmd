MEMORY
{
   BEGIN            : origin = 0x00088000, length = 0x00000002
   BOOT_RSVD        : origin = 0x00000002, length = 0x00000126

   RAMM0            : origin = 0x00000128, length = 0x000002D8
   RAMM1            : origin = 0x00000400, length = 0x000003F8
   // RAMM1_RSVD       : origin = 0x000007F8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMLS0           : origin = 0x00008000, length = 0x00000800
   RAMLS1           : origin = 0x00008800, length = 0x00000800
   RAMLS2           : origin = 0x00009000, length = 0x00000800
   RAMLS3           : origin = 0x00009800, length = 0x00000800
   RAMLS4           : origin = 0x0000A000, length = 0x00000800
   RAMLS5_6_7       : origin = 0x0000A800, length = 0x00001800
   //RAMLS6           : origin = 0x0000B000, length = 0x00000800
   //RAMLS7           : origin = 0x0000B800, length = 0x00000800

   RAMGS0           : origin = 0x0000C000, length = 0x00001000
   RAMGS1           : origin = 0x0000D000, length = 0x00001000
   RAMGS2           : origin = 0x0000E000, length = 0x00001000
   RAMGS3           : origin = 0x0000F000, length = 0x00000FF8
   // RAMGS3_RSVD      : origin = 0x0000FFF8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   BOOTROM          : origin = 0x003F8000, length = 0x00007FC0
   SECURE_ROM       : origin = 0x003F2000, length = 0x00006000

   RESET            : origin = 0x003FFFC0, length = 0x00000002

   /* Flash sectors */
   /* BANK 0 */
   //FLASH_BANK0_SEC0  : origin = 0x080002, length = 0x000FFE
   //FLASH_BANK0_SEC1  : origin = 0x081000, length = 0x001000
   //FLASH_BANK0_SEC2  : origin = 0x082000, length = 0x001000
   //FLASH_BANK0_SEC3  : origin = 0x083000, length = 0x001000
   //FLASH_BANK0_SEC4  : origin = 0x084000, length = 0x001000
   //FLASH_BANK0_SEC5  : origin = 0x085000, length = 0x001000
   //FLASH_BANK0_SEC6  : origin = 0x086000, length = 0x001000
   //FLASH_BANK0_SEC7  : origin = 0x087000, length = 0x001000
   FLASH_BANK0_SEC8  : origin = 0x088002, length = 0x000FFE
   FLASH_BANK0_SEC9  : origin = 0x089000, length = 0x001000
   FLASH_BANK0_SEC10 : origin = 0x08A000, length = 0x001000
   FLASH_BANK0_SEC11 : origin = 0x08B000, length = 0x001000
   FLASH_BANK0_SEC12 : origin = 0x08C000, length = 0x001000
   FLASH_BANK0_SEC13_15 : origin = 0x08D000, length = 0x003000
   //FLASH_BANK0_SEC14 : origin = 0x08E000, length = 0x001000
   //FLASH_BANK0_SEC15 : origin = 0x08F000, length = 0x001000

   /* BANK 1 */
   FLASH_BANK1_SEC0_7  : origin = 0x090000, length = 0x008000
   //FLASH_BANK1_SEC1  : origin = 0x091000, length = 0x001000
   //FLASH_BANK1_SEC2  : origin = 0x092000, length = 0x001000
   //FLASH_BANK1_SEC3  : origin = 0x093000, length = 0x001000
   //FLASH_BANK1_SEC4  : origin = 0x094000, length = 0x001000
   //FLASH_BANK1_SEC5  : origin = 0x095000, length = 0x001000
   //FLASH_BANK1_SEC6  : origin = 0x096000, length = 0x001000
   //FLASH_BANK1_SEC7  : origin = 0x097000, length = 0x001000
   //FLASH_BANK1_SEC8  : origin = 0x098000, length = 0x001000
   //FLASH_BANK1_SEC9  : origin = 0x099000, length = 0x001000
   //FLASH_BANK1_SEC10 : origin = 0x09A000, length = 0x001000
   //FLASH_BANK1_SEC11 : origin = 0x09B000, length = 0x001000
   //FLASH_BANK1_SEC12 : origin = 0x09C000, length = 0x001000
   //FLASH_BANK1_SEC13 : origin = 0x09D000, length = 0x001000
   //FLASH_BANK1_SEC14 : origin = 0x09E000, length = 0x001000
   //FLASH_BANK1_SEC15 : origin = 0x09F000, length = 0x001000

  /* BANK 2 */
   //FLASH_BANK2_SEC0  : origin = 0x0A0000, length = 0x001000
   //FLASH_BANK2_SEC1  : origin = 0x0A1000, length = 0x001000
   //FLASH_BANK2_SEC2  : origin = 0x0A2000, length = 0x001000
   //FLASH_BANK2_SEC3  : origin = 0x0A3000, length = 0x001000
   //FLASH_BANK2_SEC4  : origin = 0x0A4000, length = 0x001000
   //FLASH_BANK2_SEC5  : origin = 0x0A5000, length = 0x001000
   //FLASH_BANK2_SEC6  : origin = 0x0A6000, length = 0x001000
   //FLASH_BANK2_SEC7  : origin = 0x0A7000, length = 0x001000
   //FLASH_BANK2_SEC8  : origin = 0x0A8000, length = 0x001000
   //FLASH_BANK2_SEC9  : origin = 0x0A9000, length = 0x001000
   //FLASH_BANK2_SEC10 : origin = 0x0AA000, length = 0x001000
   //FLASH_BANK2_SEC11 : origin = 0x0AB000, length = 0x001000
   //FLASH_BANK2_SEC12 : origin = 0x0AC000, length = 0x001000
   //FLASH_BANK2_SEC13 : origin = 0x0AD000, length = 0x001000
   //FLASH_BANK2_SEC14 : origin = 0x0AE000, length = 0x001000
   //FLASH_BANK2_SEC15 : origin = 0x0AF000, length = 0x000FF0

// FLASH_BANK0_SEC15_RSVD     : origin = 0x0AFFF0, length = 0x000010  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

}


SECTIONS
{
   codestart        : > BEGIN, ALIGN(8)
   .text            : >> FLASH_BANK0_SEC13_15 | FLASH_BANK1_SEC0_7 /*| FLASH_BANK1_SEC10 | FLASH_BANK1_SEC11 | FLASH_BANK1_SEC12 | FLASH_BANK1_SEC13 | FLASH_BANK1_SEC14 | FLASH_BANK1_SEC15 | FLASH_BANK2_SEC0 | FLASH_BANK2_SEC1 | FLASH_BANK2_SEC2 | FLASH_BANK2_SEC3 | FLASH_BANK2_SEC4 | FLASH_BANK2_SEC5 | FLASH_BANK2_SEC6 |FLASH_BANK2_SEC7 | FLASH_BANK2_SEC8*/,   ALIGN(8)
   .cinit           : > FLASH_BANK0_SEC9 | FLASH_BANK0_SEC12 ,  ALIGN(8)
   .switch          : > FLASH_BANK0_SEC9,  ALIGN(8)
   .reset           : > RESET,                  TYPE = DSECT /* not used, */

   .stack           : > RAMM1

#if defined(__TI_EABI__)
   .init_array      : > FLASH_BANK0_SEC9,  ALIGN(8)
   .bss             : > RAMLS5_6_7
   .bss:output      : > RAMLS3
   .bss:cio         : > RAMLS0
   .data            : > RAMLS5_6_7
   .sysmem          : > RAMLS5_6_7
   .const           : > FLASH_BANK0_SEC11,  ALIGN(8)
#else
   .pinit           : > FLASH_BANK0_SEC9,  ALIGN(8)
   .ebss            : > RAMLS5_6_7
   .esysmem         : > RAMLS5_6_7
   .cio             : > RAMLS0
   .econst          : > FLASH_BANK0_SEC11,  ALIGN(8)
#endif

    ramgs0 : > RAMGS0
    ramgs1 : > RAMGS0

    /*  Allocate IQ math areas: */
   IQmath           : > FLASH_BANK0_SEC9, ALIGN(8)
   IQmathTables     : > FLASH_BANK0_SEC10, ALIGN(8)

   .TI.ramfunc      : LOAD = FLASH_BANK0_SEC9,
                      RUN = RAMLS0,
                      LOAD_START(RamfuncsLoadStart),
                      LOAD_SIZE(RamfuncsLoadSize),
                      LOAD_END(RamfuncsLoadEnd),
                      RUN_START(RamfuncsRunStart),
                      RUN_SIZE(RamfuncsRunSize),
                      RUN_END(RamfuncsRunEnd),
                      ALIGN(8)

}
