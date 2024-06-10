.section ".flashdata","a"

.balign 4
.global Jingle
Jingle:
.incbin "S16LE2_44100.raw"
.balignw 64,0x7f00
.global _sizeof_Jingle
.set _sizeof_Jingle, . - Jingle   /* Defines the size of data */
