/* Custom load region for FlexSPI XIP code.
 *
 * `.xip_text` is linked at its run address in the `XIP` window (VMA =
 * 0x8000_0000+) so that intra-section calls, literal-pool loads, and absolute
 * references resolve to where the code actually executes. Its load address
 * (LMA) is placed in internal `FLASH` via `AT > FLASH`, so probe-rs programs the
 * bytes into internal flash as part of the normal image. At runtime the
 * application copies [__sxip_load, __sxip_load + (__exip - __sxip)) into external
 * flash and branches to __sxip.
 *
 * INSERT AFTER .text augments cortex-m-rt's link.x rather than replacing it.
 */
SECTIONS
{
  .xip_text :
  {
    . = ALIGN(8);
    __sxip = .;
    KEEP(*(.xip_text .xip_text.*))
    . = ALIGN(8);
    __exip = .;
  } > XIP AT > FLASH

  __sxip_load = LOADADDR(.xip_text);
} INSERT AFTER .text;
