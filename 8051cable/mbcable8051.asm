;;; --- MBcable8051.asm ---
;;; Firmware for 'Matt's Serial Multiboot Cable' for GBA.
;;; (c) Matt Evans 3rd July 2002
;;; www.axio.ms   or   www.cs.man.ac.uk/~evansm7/projects/GBA/
	
;;; This program is designed for use with PAULMON2 (from www.pjrc.com) as a prototyping
;;; environment, though should be trivial to alter to run directly.  This code runs on
;;; the Dallas High-Speed 8051 microcontrollers, for instance the DS89C420.
;;; Use the as31 assembler to assemble this code! (Also available from www.pjrc.com)
;;; 
;;; You will need an 8051 derivative with some form of memory, internal flash, external
;;; rom, whatever.  Use paulmon2 for serial i/o routines (or borrow cin1 below for standalone,
;;; and do some setup of your microcontroller to get the correct baud rate/clock frequency
;;; etc.)  'locat' below changes where this compiles to.  When using paulmon2 it will start
;;; up the moment the system boots -- change the header ID byte to 'command' for testing.
;;;
;;; My cable looks like this:
;;;
;;;      GBA	                            DS89C420                                  PC
;;;
;;;         Vcc --+---+---+---+           ___ Vcc(5V)
;;;               |   |   |   |            |
;;;               <   <   <   <       +--------------------------+
;;;       4k7 res >   >   >   >       |                          |     +--------+
;;;               <   <   <   <       |                      Rx0 |<----| MAX232 |<---   PC
;;;               |   |   |   |       |                      Tx0 |---->|        |--->  RS232
;;;	SC <------O---|---|---|-------| P1.0    Dallas           |     +--------+
;;;               |   |   |   |       |         DS89C420         |
;;;	SD <------|---O---|---|-------| P1.1                     |    
;;;               |   |   |   |       |                          |
;;;     SO -------|---|---O---|------>| P1.2  (alt RX1)          |               
;;;               |   |   |   |       |                          |  +-------+
;;;     SI <------|---|---|---O-------| P1.3  (alt TX1)          |  |       |    
;;;               |   |   |   |       |                       X1 |--+  |#|  |  11.0592MHz
;;;               |   |   |   |       |                       X2 |--+--|#|--+  
;;;              ,-' ,-' ,-' ,-'      |     Gnd                  |  |  |#|  |
;;;  3.3V zener  /_\ /_\ /_\ /_\      +--------------------------+ ===     ===
;;;               |   |   |   |              |                      |       |
;;;               +---+---+---+--------------+----------------------+-------+
;;;                                        __|__
;;;                                         ---
;;;                                          -
;;; I recommend the DS89C420s because they include 16K internal program flash memory,
;;; so are completely self-contained.  Bear in mind the above diagram is for guidance,
;;; you may need to add a capacitor for reset, etc.  (And an LED wired to Vcc and P1.7 if
;;; you want a useless blinking light (good for debug!)
;;; 
	
;;; DISCLAIMER
;;;   Use this diagram and this code AT YOUR OWN RISK.  I shall NOT be held responsible
;;;   for any broken hardware, fried Gameboy Advances, bent serial port pins, soldering
;;;   burns etc. etc.  IT IS YOUR PROBLEM.  Electronics knowledge is required to build
;;;   this thing.  It works fine for me :P

;;; This code is (c) 2002 Matt Evans (matt@axio.ms), and is released under the GPL.
;;; Distribute this code freely, do not sell it, hoard changes, etc.  It may only
;;; be distributed with this text attached, as part of the whole Multiboot Cable set of
;;; files as downloaded from my website.
	
;;; ----------------------------------------------------------------------------------------
;;; ----------------------------------------------------------------------------------------	
;;; ----------------------------------------------------------------------------------------
		
	;; 96 clocks = 115200bps. to test, 96*2 clocks is 57600

	;; 8 = 115k2, 16=57k6
	;;  19 does odd nos.. 32 34 36 38 etc...?
.equ	ClocksPerHalf, 8; ;(2*96/12)	; 96 system clocks is 8 timer clocks
	;; 8 is correct- results in roughly 8.5uS per
	;; bit as seen on the scope.
	
.equ	TimHHalf, ((65536-ClocksPerHalf) >> 8)
.equ	TimLHalf, ((65536-ClocksPerHalf) & 0xff)
		
.equ	SCON1,0xc0
.flag	RI1,SCON1.0
.flag	TI1,SCON1.1
.equ	SBUF1,0xc1

	;; Internal RAM variables:
.equ	IntPosn,0x60
.equ	XferL,0x20
.equ	XferH,0x28
	;; .equ	ToGo,IntPosn+2		
.equ	BitInByte,IntPosn+3
.equ	SByte,IntPosn+4
	;; .equ	Dev,IntPosn+5
	;; .equ	Fun,IntPosn+6
	
.flag	done,0x20.0	; Int has sampled enough

;;; Signals driven to the Gameboy advance.  Note that this chip is the MASTER
;;; as far as the link is concerned, and the GBA is the slave:
;;; (note these labels are from THIS end, i.e. 'SI' is connected
;;; to (in my cable) the brown wire, which is the GBA's SO.)
.flag	SC,P1.0			; To the GBA's SC
.flag	SD,P1.1			; To SD
.flag	SI,P1.2			; To GBA's SO (uart Rx also)
.flag	SO,P1.3			; To GBA's SI (uart Tx also)
	
.flag	LED,P1.7		; Diagnostic flashy LED ;)

;;; ----------- PAULMON2 DEFINES ---------------
	
.equ	locat, 0x2000		;  0x0400 for internal ram
.equ	paulmon2, 0x0800	;location where paulmon2 is at (usually 0000)
.equ    phex1, 0x2E+paulmon2
.equ    cout, 0x30+paulmon2		;send acc to uart
.equ	cin, 0x32+paulmon2
.equ    phex, 0x34+paulmon2		;print acc in hex
.equ    phex16, 0x36+paulmon2		;print dptr in hex
.equ    pstr, 0x38+paulmon2		;print string @dptr
.equ    ghex, 0x3A+paulmon2		;get two-digit hex (acc)
.equ    ghex16, 0x3C+paulmon2		;get four-digit hex (dptr)
.equ	upper, 0x40+paulmon2		;convert acc to uppercase
.equ	newline, 0x48+paulmon2
.equ	pcstr, 0x45+paulmon2
.equ	pint, 0x50+paulmon2
.equ	smart_wr, 0x56+paulmon2
.equ	cin_filter, 0x62+paulmon2
.equ	asc2hex, 0x65+paulmon2

;;; ---------------------------------------------
	
        ;; A paulmon program header, to run at startup.
.org    locat

.db     0xA5,0xE5,0xE0,0xA5     ;signiture bytes
.db     253,255,0,0             ;id (35=prog, 249=init, 253=startup, 254=cmd)
.db     0,0,0,0                 ;prompt code vector
.db     0,0,0,0                 ;reserved
.db     0,0,0,0                 ;reserved
.db     0,0,0,0                 ;reserved
.db     0,0,0,0                 ;user defined
.db     255,255,255,255         ;length and checksum (255=unused)
.db     "GBAcable",0
.org    locat+64                ;executable code begins here


start:
	acall	setuptimer

	mov	DPTR,#banner
	acall	pString
	
	setb	SC
	setb	SD
	setb	SO

	;; SO connected to GBA Sin! 
	
	;; To test bit time sample frequency with a scope on P1.6:
	;; (e.g. debugging software timing loops on different MCU)
;lp:	cpl	P1.6		; 
; 	acall	wait1bit
;	sjmp	lp
	
;;; ------------------------------------------------------------------------
;;;				Main loop
	
	;; I want to escape on character 255:
	;; To transmit '255' send '255' then '0'
	;; Otherwise drops into command mode.
	;; '255,255' does nothing & remains in command mode... send three
	;; to ensure in command mode waiting for another byte of command,
	;; when first probing for the cable with host software.
	;; See getch defs for more deetails on commands.
	
xferloop:
  	acall	getch	
 	mov	XferL,A

 	acall	getch
 	mov	XferH,A

	clr	LED
	;; led on
 	acall	tx_halfword	; Send the 2 bytes to the GBA

 	acall	rx_halfword	; Wait for, and receive two bytes from GBA
	;; led off
	setb	LED
	
 	mov	A,XferL
	lcall	cout
 	mov	A,XferH
 	lcall	cout		; Send bytes back to host.

	sjmp	xferloop
	
	;; Get 2 bytes from serial port.  Then,
	;; tx_halfword to GBA:	this does the bits with software
	;; timing loop.  Set timer to run (NO INTERRUPTS) as outputting
	;; the start bit.  Then output successive bits as spinning on
	;; timer_not_zero_yet.
	
	;; Then, must RX something from GBA:
	;; spin on start bit.
	;; when arrived, count bit times and sample the centre of each bit.
	;; TX both bytes and loop.
	

;;; getch:	Gets a byte from the 8051 serial port 0, checks to see
;;;		if it's the escape char (255), and performs commands.
getch:				
	lcall	cin

	;; If not 255 (escape char) just continue
	cjne	A,#255,getch_end

	;; We got escape char!

	;; Need second char:
getch_getcmd:
	lcall	cin

	;; Command:

	cjne	A,#0,getch_not0
	;; Was '0' - escaped '255' so return '255' as char
	mov	A,#255
	ret
	
getch_not0:
	cjne	A,#1,getch_not1
	;; Was '1': this is a 'probe' string.  return banner string
	mov	DPTR,#banner
	acall	pString
	sjmp	getch_getcmd	; another command now
	
getch_not1:
	cjne	A,#2, getch_not2
	;; Was '2': this is 'go into multilink mode':
	sjmp	xferloop	; Start multilink afresh
	
	
getch_not2:
	cjne	A,#3, getch_not3
	;; was '3': this is be 'go to UART mode'
	;; for 2nd stage bootloader

	sjmp	uarts_are_go	; never return

getch_not3:
	;; Anything else (particularly 255) goes back to
	;; command input.
	sjmp	getch_getcmd
	
getch_end:
	ret
	

;;;		 *************** UART MODE ************
;;;	This turns the cable into a completely transparent (aside from latency ;)
;;;	passthrough cable.  Once the first bootloader has been loaded, the main
;;;	multiboot code is loaded with a plain serial non-encrypted protocol (see 2ndloader)

;;;	This can just be faked by setting the out bit to whatever the in bit is, and is
;;;	an exercise for the reader ("Make a microcontroller look like a bit of wire"...)
uarts_are_go:
	setb	SD
	setb	SC
	;; Now SO and SI are data in/out to the 8051's second UART (not present on basic 8051!)
	;; Present on the DS89C420!
	;; pcon 80
	setb	0xd8.7		; SMOD1_0 - double rate, same as Ser0
	mov	SCON1, #0x52	; SCON1: Mode 1, 

	;; With a spot of luck, ser port 1 is set up here.......

uart_passthrough:
	;; check for bytes in from GBA to PC
	jnb	RI1, uart_norx
	;; Received a byte from GBA - send to PC!
	clr	RI1
	mov	A, SBUF1
	lcall	cout

uart_norx:
	;; check for bytes in from PC to GBA
	jnb	RI, uart_passthrough	; round again 

	;; Byte waiting from PC
	clr	RI
	mov	A, SBUF
	acall	cout1

	sjmp	uart_passthrough

	;; -----------------------------------------

cout1:	jnb	TI1, cout1
	clr	TI1		;clr ti before the mov to sbuf!
	mov	sbuf1, a
	ret

	
	;; ************************ Software UART stuff ***********

	;; Transmit a 16-bit word to the GBA (held in XferL and XferH)
tx_halfword:

	;; OK, before trying to send must wait for both SD
	;; and SC to go high (it's my turn to start a transaction)

	setb	SO
	setb	SC
	setb	SD
	nop

	;; Bits in XferL and XferH
			
tx_waitstart:	
	jnb	SC,tx_waitstart
	jnb	SD,tx_waitstart

	;; SC and SD are high now
	
	; setb	SO		; SO =1; master (me) transmits
	
	acall	resettimer

	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit	; 8 bit times

		
	;;  Now set start bit (SD low) AND SC low at same time:

	;; so sd sc
	;; SC SD SI SO
;	mov	P1,#0x78 | 0x01	;  SO high, 1001  sd low sc low

	mov	P1,#0x74 | 0x8	; SO high, sd low sc low, 1100

	
	acall	wait1bit	; wait till next

	mov	C,XferL.0	; First data bit
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.1
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.2
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.3
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.4
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.5
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.6
	mov	SD,C
	acall	wait1bit

	mov	C,XferL.7
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.0
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.1
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.2
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.3
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.4
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.5
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.6
	mov	SD,C
	acall	wait1bit

	mov	C,XferH.7
	mov	SD,C
	acall	wait1bit

	;; Stop bit:
	setb	SD
	acall	wait1bit

	;; here SD=1, SO=1, SC=0
	ret

	;; Spin waiting for the GBA to drive the data bit low, as a start bit, then
	;; read 16 bits of data from the GBA. Store them in XferL and XferH.
rx_halfword:
	setb	SD		; (stays high)
	clr	SC		; (stays low)
	clr	SO		; request reception from GBA
	
	;; wait for Start bit

rx_startbit:
 	jb	SD,rx_startbit

	acall	resettimer
	nop
	nop			; see where this gets us
	;; middle of bit?
	
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.0,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.1,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.2,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.3,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.4,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.5,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.6,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferL.7,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.0,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.1,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.2,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.3,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.4,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.5,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.6,C
	acall	wait1bit

	mov	C,SD
	mov	P1.6,C
	mov	XferH.7,C
	acall	wait1bit

	;; stop bit here
	acall	wait1bit
	
rx_end:	
	setb	SO		; request done
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit
	acall	wait1bit	; other GBAs... hope only 1 connected :P
	
	setb	SC		; transaction over!
	
	ret



wait1bit:	
	;; spin looking at timer until timer overflows (then reset & return!)
	jnb	TF0,wait1bit	; not overflowed yet
	mov	TL0,#TimLHalf
	mov	TH0,#TimHHalf	; Set up timer to roll over again soon
	clr	TF0
	ret
	

	;; ------------------------------- timer routines ---------------------------

resettimer:
	clr	TF0
	mov	TL0,#TimLHalf
	mov	TH0,#TimHHalf	
	setb	TR0
	ret
	
	
setuptimer:

	mov	0x8e,#0x01	; CKCON = default (12 clks/timer tick)
	mov	0x96,#0xD7	; CKMOD = default (timers from clk/12)
	;;  But, T1MH = 1 (so TM1 is at 1 clock per tick (same as paulmon))
	;; i.e. baud stays 115k2
	
	clr	TR0
	mov	A,TMOD
	anl	A,#0xf0
	orl	A,#0x01		; Timer0 is in mode 1 (16-bit), internal stuff
	clr	TF0
	mov	TMOD,A
	mov	TL0,#TimLHalf
	mov	TH0,#TimHHalf	; 8 clocks from overflow...

	setb	PT0		; 'high' priority level for T0
	mov	IE,#2		;  T0 int enabled
	clr	EA		; No interrupts
	setb	TR0		;  Timer running

	mov	BitInByte,#8	; pretend we've already done some. shifts result
	mov	SByte,#0

	ret
	
	
timer_int:
	mov	C,SD		;  Read immediately! 3-9 cycles from timer ovf.
	mov	TL0,#TimLHalf
	mov	TH0,#TimHHalf	; Set up timer to roll over again soon?

	mov	A,SByte
	rrc	A		; earliest bit at bottom, thanks...
	mov	SByte,A

	cpl	P1.4


	djnz	BitInByte, timer_int_stillgoing
	;; Ok, so here we've done all the bits.
	
	mov	XferL,SByte
	
	clr	TR0		; Stop!
	setb	done		; Signal finished
	
timer_int_stillgoing:	
	reti


;;; ---------- Misc routines ----------

;;; redo print string:	Use movx instead as movc doesn't
;;; work accessing internal SRAM on the DS89c420 :(
	
pString:
	push	acc
pstr1:
 	clr	A		; instead of 
 	movc	a,@dptr+a	; these two,
	;; 	 movx	a,@dptr	; use this <- if you're running from external combined
				; program/data memory.
	inc	dptr
	jz	pstr2
	mov	c, acc.7
	anl	a, #0x7F
	lcall	cout
	jc	pstr2
	sjmp	pstr1
pstr2:	pop	acc
	ret

banner:	.db "[GBA:MBA RS232 cable v0.1, 15/5/2002 me]",0
