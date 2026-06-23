/* Custom load region for the FlexSPI XIP *library* example.
 *
 * Unlike `xip.x` (which only relocates code), this fragment places a whole
 * mini-library -- both its code (`.xip_text`) and its read-only data
 * (`.xip_rodata`, e.g. lookup tables) -- into the external-flash `XIP` window so
 * the library lives *entirely* in external flash:
 *
 * * Run address (VMA) is in the `XIP` window (0x8000_0000+), so intra-library
 *   calls, literal-pool loads, and absolute references to the tables resolve to
 *   where the code actually executes and where the tables actually live.
 * * Load address (LMA) is in internal `FLASH` via `AT > FLASH`, so probe-rs
 *   programs the bytes into internal flash as part of the normal image.
 *
 * At runtime the application copies [__sxip_load, __sxip_load + (__exip - __sxip))
 * into external flash and branches to __sxip; probe-rs never touches external
 * flash. Code and rodata are kept contiguous so a single copy relocates both.
 *
 * INSERT AFTER .text augments cortex-m-rt's link.x rather than replacing it.
 */
SECTIONS
{
  .xip_lib :
  {
    . = ALIGN(8);
    __sxip = .;
    KEEP(*(.xip_text .xip_text.*))
    . = ALIGN(4);
    KEEP(*(.xip_rodata .xip_rodata.*))
    . = ALIGN(8);
    __exip = .;
  } > XIP AT > FLASH

  __sxip_load = LOADADDR(.xip_lib);
} INSERT AFTER .text;
