MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 2M
    RAM   : ORIGIN = 0x20000000, LENGTH = 640K

    /* FlexSPI external-flash XIP window (non-secure alias). Code placed in the
       .xip_text section is *linked* here (VMA) so its calls, literal pools, and
       absolute references resolve to the run address, but it is *loaded* into
       internal FLASH (LMA, via `AT > FLASH` in xip.x). The application copies it
       from the LMA into external flash at runtime and executes from this window,
       so probe-rs only ever programs internal FLASH. */
    XIP   : ORIGIN = 0x80000000, LENGTH = 16M
}
