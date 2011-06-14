#include <avr/io.h>

  .global flashProgram
  .section .bloader, "ax"

flashProgram:
  ldi   ZH, 0
  ldi   ZL, 0
Write_page:
Wrloop:
  rcall Get_data
  mov   r0, r1
  rcall Get_data
  ldi   r20, (1<<SELFPRGEN)
  rcall Do_spm
  adiw  ZL, 2

  mov   r24, ZL
  andi  r24, SPM_PAGESIZE-1            ; Wenn alle Bits 0
  brne  Wrloop
  ;
  sbiw  ZL, 1
  ; Page Erase
  ldi   r20, (1<<PGERS) | (1<<SELFPRGEN)
  rcall Do_spm
  ; execute Page Write
  ldi   r20, (1<<PGWRT) | (1<<SELFPRGEN)
  rcall Do_spm
  adiw  ZL, 1
  mov   r24, ZH
  subi  r24, (63 * SPM_PAGESIZE) / 256
  brne  Write_page
Return:
  rjmp  __vector_default
Do_spm:
; check for previous SPM complete
Wait_spm:
  in    r16, _SFR_IO_ADDR(SPMCSR)
  sbrc  r16, SELFPRGEN
  rjmp  Wait_spm
; SPM timed sequence
  out   _SFR_IO_ADDR(SPMCSR), r20
  spm
  ret
Get_data:
  lds   r1, UCSR0A
  sbrs  r1, 7
  rjmp  .-8
  lds   r1, UDR0
  sts   UDR0, ZL
  ret


