; StartingPoint_Spr17.asm
; This program includes a basic movement API that allows the
; user to specify a desired heading and speed, and the API will
; attempt to control the robot in an appropriate way.
; Also includes several math subroutines.

; This code uses the timer interrupt for the control code.
ORG 0                  ; Jump table is located in mem 0-4
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)

	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety

WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

	LOAD	Increment_Speed
	OUT		SSEG1
	LOAD	Increment_Angle
	OUT		SSEG2

;***************************************************************
;* Main code
;***************************************************************
Main:
    LOADI 150
    STORE Increment_Speed
    LOADI 90
    STORE Increment_Angle
    IN      IR_HI                                           ; get the high word
    OUT     SSEG1						; display the high word
    IN      IR_LO                                           ; get the low word
    OUT     SSEG2						; display the low word
    STORE	IR_Current_Val			                ;Else, store new value and start down tree
    Call    Reset_IR					;Reset IR to not read same value twice
    LOAD	IR_Current_Val
    SUB     Play_IR
    JZERO Loop
    JUMP Main
Loop:
	IN      IR_HI                                           ; get the high word
	OUT     SSEG1						; display the high word
	IN      IR_LO                                           ; get the low word
	OUT     SSEG2						; display the low word
	STORE	IR_Current_Val			                ;Else, store new value and start down tree
	Call    Reset_IR					;Reset IR to not read same value twice
	LOAD	IR_Current_Val
    SUB     Stop_IR
    JZERO   Die
    Call    Move_Forward
    Call    Turn_Left
    JUMP    Loop



	SUB		IR_Power				;Check if power button (E-Stop)
	JZERO	Die

	SUB		IR_1
	LOAD    ONE
	JUMP    Goto_Spot

	SUB		IR_Play					;Check if it is pause button (Stop motion)
	JZERO	Pause_Motion

	SUB		IR_5
	LOAD    FIVE
	JUMP    Goto_Spot

	SUB		IR_9
	LOAD    NINE
	JUMP    Goto_Spot

	SUB		IR_Enter
	JUMP    Parallel

	SUB		IR_VolUp				;Increase the increment in motion and angle
	JZERO	Increase_Increment

	SUB		IR_RW					;Do stuff to turn left
	JZERO	Turn_Left

	SUB		IR_3
	LOAD    THREE
	JUMP    Goto_Spot

	SUB		IR_7
	LOAD    SEVEN
	JUMP    Goto_Spot

	SUB		IR_Pause				;Do stuff to back up
	JZERO	Move_Backward

	SUB		IR_2
	LOAD    TWO
	JUMP    Goto_Spot

	SUB		IR_6
	LOAD    SIX
	JUMP    Goto_Spot

	SUB		IR_0					;Do stuff to go forward
	JZERO	Move_Forward

	SUB		IR_VolDwn				;Decrease the increment in motion and angle
	JZERO	Decrease_Increment

	SUB		IR_FF					;Do stuff to turn right
	JZERO	Turn_Right

	SUB		IR_4
	LOAD    FOUR
	JUMP    Goto_Spot

	SUB		IR_8
	LOAD    EIGHT
	JUMP    Goto_Spot

	SUB		IR_TV_VCR
	JUMP    Perpendicular

	JUMP	Main					        ;Match not found, return to begining

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	CLI    &B1111      ; disable all interrupts
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2       ; "dEAd" on the LEDs
Forever:
	JUMP   Forever     ; Do this forever.
	DEAD:  DW &HDEAD   ; Example of a "local" variable


; Timer ISR.  Currently just calls the movement control code.
; You could, however, do additional tasks here if desired.
CTimer_ISR:
	CALL   ControlMovement
	RETI   ; return from ISR


; Control code.  If called repeatedly, this code will attempt
; to control the robot to face the angle specified in DTheta
; and match the speed specified in DVel
DTheta:    DW 0
DVel:      DW 0
ControlMovement:
	LOADI  50          ; used later to get a +/- constant
	STORE  MaxVal
	CALL   GetThetaErr ; get the heading error
	; A simple way to get a decent velocity value
	; for turning is to multiply the angular error by 4
	; and add ~50.
	SHIFT  2
	STORE  CMAErr      ; hold temporarily
	SHIFT  3           ; multiply by another 4
	CALL   CapValue    ; get a +/- max of 50
	ADD    CMAErr
	STORE  CMAErr


	; For this basic control method, simply take the
	; desired forward velocity and add a differential
	; velocity for each wheel when turning is needed.
	LOADI  510
	STORE  MaxVal
	LOAD   DVel
	CALL   CapValue    ; ensure velocity is valid
	STORE  DVel        ; overwrite any invalid input
	ADD    CMAErr
	CALL   CapValue    ; ensure velocity is valid
	OUT    RVELCMD
	LOAD   CMAErr
	CALL   Neg         ; left wheel gets negative differential
	ADD    DVel
	CALL   CapValue
	OUT    LVELCMD

	RETURN
	CMAErr: DW 0       ; holds angle error velocity

; Returns the current angular error wrapped to +/-180
GetThetaErr:
	; convenient way to get angle error in +/-180 range is
	; ((error + 180) % 360 ) - 180
	IN     THETA
	SUB    DTheta      ; actual - desired angle
	CALL   Neg         ; desired - actual angle
	ADDI   180
	CALL   Mod360
	ADDI   -180
	RETURN

; caps a value to +/-MaxVal
CapValue:
	SUB     MaxVal
	JPOS    CapVelHigh
	ADD     MaxVal
	ADD     MaxVal
	JNEG    CapVelLow
	SUB     MaxVal
	RETURN
CapVelHigh:
	LOAD    MaxVal
	RETURN
CapVelLow:
	LOAD    MaxVal
	CALL    Neg
	RETURN
	MaxVal: DW 510

;***************************************************************
;* Subroutines
;***************************************************************
Move_Forward:							;Manually move bot forward, by increment
	LOADI	0
	ADD		Increment_Speed
	STORE	DVel
	Call	Wait2
	LOADI	0
	STORE	DVel
	JUMP	Main

Move_Backward:							;Manually move bot back my increment
	LOADI	0
	SUB		Increment_Speed
	STORE	DVel
	Call	Wait2
	LOADI	0
	STORE	DVel
	JUMP	Main

Turn_Left:							;Manually turn bot to left by increment
	IN    	THETA
	ADD		Increment_Angle
	STORE 	DTheta
	JUMP 	Main

Turn_Right:							;Manually turn bot to right by increment
	IN    	THETA
	SUB		Increment_Angle
	STORE 	DTheta
	JUMP	Main

Increase_Increment:						;Increase linear and angular increment for manual adjustments
	LOAD	Increment_Speed
	JZERO	Fix_Increment
	JNEG	Fix_Increment
	ADDI	50
	STORE	Increment_Speed
	OUT		SSEG1
	LOAD	Increment_Angle
	JZERO	Fix_Increment
	JNEG	Fix_Increment
	ADDI	0
	STORE	Increment_Angle
	OUT		SSEG2
	JUMP	Main

Decrease_Increment:						;Decrease linear and angular increment for manual adjustments
	LOAD	Increment_Speed
	JZERO	Fix_Increment
	JNEG	Fix_Increment
	ADDI	-10
	STORE	Increment_Speed
	OUT		SSEG1
	LOAD	Increment_Angle
	JZERO	Fix_Increment
	JNEG	Fix_Increment
	ADDI	-5
	STORE	Increment_Angle
	OUT		SSEG2
	JUMP	Main

Fix_Increment:							;Return Increments to Positive, non-zero, values
	LOAD	Ten
	STORE 	Increment_Speed
	LOAD	Five
	STORE	Increment_Angle
	JUMP Main

Pause_Motion:							;Pause motion from motors
	LOAD	Zero
	STORE	DVel
	IN		THETA
	STORE	DTHETA
	JUMP	Main

Reset_IR:							;Return IR value to zero (Function Call)
	LOAD	Zero
	OUT     IR_HI
	OUT     IR_LO
	RETURN

;*****************************NOT TESTED**************************************************

Goto_Spot:
	STORE   Spot						;Save the target spot
	CALL	Goto_Auto_Init_Pos				;Initial position in front of parking spot 9
	LOAD	NINE
	SUB     Spot						;Calculate the spot offset relative to spot 9
	STORE   m16sA						;Multiply spot offset with the spot width
	LOAD    Spot_Width
	STORE   m16sB
	CALL    Mult16s
	LOAD 	mres16sL
	;Goto_Forward method to move by mres16sL""", assuming gotoforward exists and goes forward by the amount
	;In place 90 degrees turn to the right"""
	;Goto_Forward method to move by Auto_Perp_Distance"""
	JUMP    Perpendicular

Goto_Auto_Init_Pos:						;Not defined, needs to measure the arena"""
	RETURN							;Goes to an initial position in front of spot 9
								;Facing towards the further wall, not spots

Perpendicular:
        ;Goto_Forward method to move by Perpendicular_Distance"""
	JUMP Main

Parallel:
	;Required moves for parallel parking"""
	JUMP Main

Goto_Forward:
	;Logic to go forward by the specified amount"""
	RETURN

;*****************************NOT TESTED**************************************************

;*************************
;* Predefined Subroutines
;*************************

;*******************************************************************************
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod360:
	; easy modulo: subtract 360 until negative then add 360 until not negative
	JNEG   M360N
	ADDI   -360
	JUMP   Mod360
M360N:
	ADDI   360
	JNEG   M360N
	RETURN

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Neg: 2's complement negation
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
Neg:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	CALL   Neg          ; Negatge the number
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	CALL   Neg          ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry,
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	CALL   Neg
	STORE  dres16sQ
	RETURN
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs"6’eì(¢ô°ı„®Ÿ êÈa‘â±o>xSay¬\/wÈÖı(è}Æ¡d“½eU¢F2¾hxŸÿs¢,Õ:)v¬o5äçe¼\5‘|£¡m(m¼›™ãäËÔ¥­ñ/±qğBdÙNÓü‘Û’ -!¡pL„*væ1üX*ì¤Æ\dÎÈMÕ~ÌEh@eòıÜÿıU%ŠtZ6Ÿö
ÆAØ(PZü0ò:áz)2ã²Ü´áÙÖÖ7_W&~‹dÑÖÒN„_)®bÁsKİXWòñ«Š^	4¿ƒĞûer¶ĞV£+lØw²ñ	Îm¤]†x6$:ÌğjÄÙÎ÷àí-„„üŞX*úrYwl¿òKEDN†—Çg;ZÃÃÏxÕXí¬Û<2çÍ	<ª 62Œxr]„w¯ŞÎŠ2âWşÌ‹œh«Øns ÕzNt¢uL5†4'Ï	˜Êc¾Êã}f9é=¬5Ç »;¥Š#g°dC¾ª|¾©éá‡|ÌçÆ±ıÃ\”ÄO÷Ñ–çÑ?íC@×X·Ù`õ4ã¦o¤Kh0Şz6‡ ×Ú
ş7!Š>q+8ÃßœîkÈßÜ±Áõ  ZçÑè‡Şöl$2
Ì¡os%Ş½ÆR#ã¯°IY<–‘&œ/^>›¼¹ï›îÃçòƒ`ÓË–­œ§DøíÇìú°ºv‹P•VâWPEXá¨.©²ŸæN¡]Â'csR ì7¤Ä7à£¬¶î¹Î7ä³ ÿ=ã 	Ln©V‰‡™º­d­Ú—·¾Ãñ±ZO
äçÚQìÇ õ²ÀÍ^gÂ™ÕT5Ò¬÷mË¾ÛÓ†)rş°>$+õq&Ñ®˜u
1zª%À`FöÄ¤Qq§r¥ı¶ÄÓ’môvç¼5gì)S4–:J3 œ áÕ¸×3à¢Y¿ÚÛ‘Pø³ıiPbd^Éµ	(È§›2™FªrüG4ÚÜœšêgoM*¿%T€ER+LXó#½š@HÎ›*/âx(0Wˆ†"¾SHÏ’OÖ;¶vPÛ˜KDX9ašÆƒ0Ô"oü¤‹I¢'_P6ÿ»MßïKØeÏã,íÚ$rü¦÷ ’Ô@kDUÍ²ÆÖU,\ÑE{óR‘T6	¦ˆÇ7xÏRµØÌ•#Ãl<>n¹±¾"4Pİ«ÚzÀ²ä¾ˆ1ŒZÀùÚö!w”æÌ˜x¼?Á›f±©É†!u´L§ğjUm$´¹?+"FÑë1œ!(º^à8®•o[í)ÜÚ¡Ó¸P—ÑŸ…,Ò™ù8şablÔœ®òµ-@ì:™ô<¡'ô£Ûiwùë "İ5‘—Ä›
j:ëNÆK­-‚µî“HNÁwà·¡Cô®PHÎ·jº‹a²¶Z^çï¤¹—œ }1A%¯86îf3’_¹¾¡÷²°ôİ€lä¥ªèqırEşoŠ%Ã#¦ƒú‘Z¤¶&á´­´•ÈYÕ±±&©º¨¿{Kx‘%€iÃİ© ´¢È*ÛªúYêvÄ	Ô4ÕKèLí¤çşFód„ä-»­VÈú8(2*ıÇŸ‹Ó¤ŸF/H–@]¶ûv‰ÛŞ¨BÔ¡YHpıtJY†Å9@uÈ_2]9fRÓmÃ%ıMëşŒó€ÃÏÃiz9]ò/^&4÷™¼=J^#£ğ"Ø0Ÿôí(ùÉ¡Z\æşx¯Ø†Ã3Ñc
“Çî›šq%ËT‡)>­/«RÑÀNg4‡FÎƒÓ›5œÇúQêSÄ„¿%Şq}WşÊ’o•6ÍğøG~ôŠ]gØç9JAsĞVÏfí^ûN¶°º$/şLhZP€Õ!M>–~œ¥hò@ÜĞÄTĞğaæwKAqš£€\«±•V½Äë/yğ‰w¯6Û‚´ŞäÁIÑ’ø^ˆpƒ68YR7Ù ß»œjfcâ U±¶Õ@bIM\«…|!µY…1ˆŞJ1í%ú#j	mQ£ÖÊŸH¾Òî&zİşx+.¢)ˆy¶á&zb{–¹ƒ :‡Ş‚w\Ó&±‹ø³€Ñ©H*mô=§ÁâUğNBÿÊû8.?ÅøùãMªàrÀ¾fZÚİju•5Ë Dúˆ¼Íå:7ª ğ"¯>à¤µÛ€Ì	#êS`¨p„[ÆØĞ*şLT
³•<óĞÁáÕ°Eî!8odªÛÚÉŸ‰²e_;5dk'ym½İ4ÂÏè*À ¨qş»èì¿ŠGj"ö÷¥hş¨)À&=XP±»×ÛœÅœ¥‚™Æ³ÁéAéÃå¨rØ‘<á ˆ÷Ë¥3¸cª¢ı!h};ıxIGŠ#Ø2øäãNş.£Ï=vVŸÓ=†jı\1Jn\Ä'Òµ|g×¢Q`F£úıqº2;–‘“ iOâÎ‚¦#ªæ®(İ`6$)qµ¤‘ïmÈSf‘—½|@#¡x]Ãiõ]ôå-uƒt\ğ/¢×Lˆ³×ô+ÂòÑØ¦yK%dÒ•Bœ_øúı7Ğ4œgÈq‚‘eØ&ïƒ­$jZÿ²—AÂú€cCéÅ7E"ÑñÇêV·uPy$~Çõ0(rßäËQmAİÑ}¾¼ùãY^‚á‚p^=GLI¸\›£RŞÍŞÆ7é_ñ’ ,X[F‘şæ4é‚<§#ßùæq}|Owi­ª³9\·my$Ê³•Ùº@w|W÷&?I£‘Â À6ó’Qˆ”–¸Åiß9n¶Bòe¿5ïá·}:7’ÜÍ¹u]DAóøjxl‡^9eZ,üÑy„P÷‹ğsfÍ-ƒMëç·`R’~8!›®löd•†cr@Û1÷ia¦ƒ»|6~¯Ë9ë"üDqçXÈÖüxQÉóĞ4go7ÆúC’Ú©m6¤>Ş.…{ãòÊÍ‡yrô‘×ãGoÖ¸\Q¶çÒO©¥zwSxÜnç‰Ézì•TŞ·åfF»ó=ßrXòÜ¯hş?7Ğˆ¨ö,`‘è6¬ñ)¶Ë}µpv~œ®ôÒˆ¼>iqå7B§#ÏKU¿¦ì·jÊú «'Ø&’‰‹o|ÏĞ3÷üÏ®T$ÒJ=À:`1¿şLt@%‘¿@,©sFÊœj—Èçà“Ã¤çˆìâ×ezá ÛA&qmí¿ò’i˜®`¥cÂ—x	›×,»ë×[Öûôñ§38&é"ƒ`ıY¤Ôñ)Ÿ²İqşçİ%En ˆ!İ.d“8íPc'ï)‰-)Ã?å³sì}ù…5·/P¦€)°ä¨¸wV¶·®Œ³À/Ó÷C*3½k·gfTÓºšë¼§õ°°ûÌÉ1ñ˜ ö¿ä¼ò3¨€™tw›Ÿº âÆ’}M´4{âòOQÂKm9*v}´Ç¡“\¯Åë]f.vº¼—&>ÑÎ™yˆôl@œœ#{ÉŸØ˜ë·µPÉó˜EºEZô–v¯i*„Â¶Sş¼@ïâA[Å>û
#nÜåœY¦<éÏÒÀ÷v
2œiW|HUµU†e!B‚$¬hİ*ûeD¨ÍDqw†œ+ıë[¤bcR3Èzˆ‘­åòª%eÁã İŠ´$Uâ{<à×¾Ø¿¬S}dĞšô<­­°µEVtõüuÆ¿& /Õò~vÜ°ôEççxvÆgì$~% ¯ÿ\ºŠ­§’ôÑx
¬ñÍwÖ'ËIµânµÿ¥ÏÂ½2gœp7†oïéøëUì=o8L\˜{õ±½nÜ×eŞRÈ–™¶³‹W-ÅßHğ‘ø\Ê87Q¼q)U¥Æ€C×u¹G–ŠL! Î®”BjA4âw}Æ‡j§å½Í³2îèu3-^òûz"LÁ[[ğ¼Ö9 èû½P´ÃŒ¤éí€w:¼ğpÛÌÕO†q,òúv¡_°“}A(ÉÚo´ò‚	ò>{ø‡·²wÀÈ+)"¹´aZ6g©>n6ØEÄó$A…X·ù† rûÜ¨ÄÏê(ntvüàã|%\õŞ"ªÁÓ’á¼¶ƒõ×Wj6[üö{&u /[Q¸•6wÔUÇÁDÑ­ÅuísRš"rTl>“"©œ¿ëdG€W‘ı9Û}mœ‘Òœõ€à-#PÔ¿én“J‡8Ù&àlı0KûO±ıI*¦‰‚…fy	ıwÕ’ 6æ16—êõò‘¦[‡E;¦É±¹JÎ»¶) ¹¶ Jšö7I¾OâZBå¯\Ê«ª][ª°3®dD ü<¥ÈŸãÚTQ]lpm!«Hw"¸_ß˜Ã|œçˆ@– ıyŒ§wsÚÙ·—CÏ¬÷/Ùf*Š=†ZyÕ¼à‡`¢_„,dŒq£¯BSŠÌÖÖÓ¸Üÿwí1.øA%İ|ZpŞ†HÁKÃ9`Z¢;SÌn6›/‡Ùu£“0¾¸9—ì¾V>§ıÛñcø2ãŸˆÑİ«+	3	µv¿nmˆª“(´åó|d"\@Y£óÏêŒèR¢Øó‚şÛ{œªICTø›/©İEH&nßt<îÈ¢Z9}êg³åÔš‡8ğUòKônì]iPÏ ßA˜`û&y^3`BfcT£vQqYµ—w¥ŸÓb›µEİH¿ÅÀM=î®<S 4O:†¸înç±Âå~ÚOpC×iJ?TŠÂãïNr7+Ñı/›²<T<Ñ£8€ İ°%µQ¹İŞêÁ}îe ûu;Õ9†	N¥±á•êñz	¹1€$î!-ÖMÃÊOÔ¾<Tv<N´Òù.ÓÉ”¸Jy¥7!QâPz,ñje£ß–Ü:–Èm%3Ş[’ óœg\úè¶LÿƒÖãÓÚ}«f«±”F|})Õ¢F9lç8}¦0Li\j4-F¹<ìívÂXó·ÙvC‰Æç„3Ë$ûÕ|›+Â^SÌíºÃÜ²ôŠ80]IŒ*a‘ı‡C6ªbµ9 wa	×"[Ù¼ó~æ!lª-ŞbğøHIg6=KW«Ïx3dhg„.”w–×/™-gW[—*dHJ¨©ÕE,_MpÎÆım¸øx^+ÀºÕX_{v®,CØ94C.İ¯S¬œeVr"íf¼!_Às*)yp=˜µ¯Ÿõl¸t;“òÿ˜`èÀíxºCjSÌ², ¡$ñÄiìr(hDÑ´sù@Ê¦hÌ;oŠñ¤ãĞŞêkó– 8ŞÎhİ
|"æ›Yİ?è²­Æ‹È·"n®^ÎŒ$ÿ›8üo]îÙfÙX8X3$ …s¨Ö_ÿ¢­}:­úé„	²
Ò^hù0 ²â“ëO2ß8rBÑ-•qAK`\¾éH
pL;ä{°ÓıÊ·ØfÊ37¯WoÄñ±Ú#»}<hiÄgó@r#ı½™í¯&5/[~Mñïâ·üû¹¨®Ï‡/`â{Ö5|²OU/´Æ7gù-£ZºÆş
*<ÄöÂÒ6z·qpş\wD|r,gÔÉ’	Ÿ#IB«¼*ëkJ=NÖúuQõ9ÀìülÇ|§«íéÌ¡‚%	ªø,­-[='·Ì&]h(UÿÖUÿ_œH¼$·l7”£r–»ÄX7z¶'líÁ:¯¨èíî•Aùx£Àq’Á}ÆÜûÖØ©ä®Î%òK
º"Ä÷E]ÿ9´âÁR¦4€åÛŞ†Qq[`Ê§¤º6%]s„bx?)¼-0,ÁP×ÿ­u¶fDİzº]‰P[,tÎˆkVK@3Œg,l¹‰üf‰¢ÌMQc†àä(%M·Ÿ¾êVİÏÒ0X;kG—Çw®†*1+ËÂ—ÃA,&â9Â: ÷R`f`ñéwOÃÛÇÒÔˆ˜gøĞDÂFT’'& ·0AX¿¯qïŠT;yÓi²5€EYëy[=o/—f%Ôü×ùj¯¼ñÆ$qêaœïtÊ_ŞT¢–|Pµò¹_yA`®çMzœQêL]×Ÿåëk—mb‰Ş-À@·Å?šÆŒTq½	+‚·gÊ¬.Ağzå=+0ÎªITYï{€ôÿùÛ*(£ÇõGt!½ğ\è/ÛK#OQãå¤~®¦éw(¤ ºän´a3Z¦°“zÿ¸™t¾hªUÇ’•a`:S©…›Åâ×\ßÇ{:„ ç‹0ûP;€õş9©B,¦DÔ"9½§%V¹ûÅè°buÁIfCÁ©yg`F”çI¥»¥‹P\eßÀ‡„Cn@,¬ÀCßiõ çˆ—Õ!’h	¼eRùÍ;A^>A&jé!‚¯ã«ç[Õpnódgƒ"¥I‚Âö@G²AÓĞ€Ÿö ’r üÀ’¾lçÒJÑx¨‘¯
uG^EûI¤íwå›ø°šQ–ìAóÏİË†ıŞ…r˜=ÍğHàå²Ù*VÒ@®Ç•x•ØÀ[§x.Ø*èO ¯v®€ëòkDbRÉú(Ó½‹Ì$îTá„çUH^‘Jµ!ëzE4×a—aş]ò á| [¡\ß*/`³‘..ƒ}´¤ğz‘8›a Üò(#Î"á“to\g§‚ê#Š0œÑKÈÕœÕ"
¹Ï¶ph$n~)Y@¸Ë†İPw1ËÍ‚‘0Ø°‘ªò—nxßˆWB36ùaZç€hãÅ··ú‹µŞiQ¯¯ËiVM41$‘
jhóq6ËnB+YòdV‘¨ó¾$×ÜdÕ[Fò±rWøùËÉ‡í½ßsE¬ÅŸvŒ{QC>™Ãª·¡æ3û]a¦©ÒT|ü.|º5ĞŞLD²&¼8”C*v‘QÇ1è4Dı
}5Æfÿª÷ĞfÔ—å*(Ñï{™ÃR×·GKštßÙ$ùİ’ÖŸ[©a1ö™ GM011İ,]œ­xH/ 4„Ú²ÎÇ	= İ±É™‡Ëì'ù¹.;Y«à£Ó góz¸Ş¯²nwk¹á$«öMyâ<uôH”¤‚lN²Ì‘¨2dÄƒP'd7§™|ğÒƒa3²vØ Å6dm¹ ¹şT–õ¥i±šÔE›™FÃ‘óÚÉá¸â|¥EZ—ÅDü8J%õï·¸m1¨ “JãÚWÿl¬ä40&Â¼ò#ğ°„?*½_¡0˜O°¬ í6ŞÖê	 ŞA[‘2Á&Š)öW…êëH¦Ñ9©H¦-ô™’X–ŠGäğ ®SA¢ÏAàèºquj³ò9/ƒ²¢Çqè½ıpBV©D… 6°è(ß%ªCj¥>×|Uì–¸Ë¬u[<İÄŸñ $ÂELP‡çá¹P©Ê?P7‰h¦ç]QïÀnq%56g ĞH7bù&‰‡YPx”j.ÊïyŠ§$¥mïRwü]Çóã	é4‹Uük¶:o:ÖÑá\p”	ÚV6e“W›.±ôİß¡@ñe3UpR6-^“Ô0‡_ƒáª¨B )‚$6G‰Aßª8YŒ…qÖF½{%ï´'²˜ºtMk„ÑëÉğ9·ÑñÃ©oäÎÖ¼í¹ ‡G”@*Ä?-Ñ?Z… —Ö=i|H	Q®J’jJ*F
¼%o£®é8}ˆrßD½tŠ»NSÃ¢j×§Åbui1"a™ÁğF@<#ªÿ<{8^;øÒ¯nº"N‘áˆt.C‹ ÈõêÙ¹µÎÚ©=]°ÑŒjÕ×KfÅ‘›OÚ}/c®l"b&Òë@É Ù9óXSÆà1ÓckY2½5U"L|†µÆH~V³H´i6å·4
òJFícjû@†Škí¢²gÈA¸M¡ääÅQ¸¬ùü×KX›áO¥m­¡ìcO"Q
İ‰«Š`g¸ë]’Ä"}=gA×†Â^Êlólªyşd“8zE#’Nl¼¦²'-¥ãÄKµ\ŸRrïN²O—.–}‰‚ö>ª	•´²Ç
d=Œ¸¼V‘x\t²ì7Kñ!G•5AïY¯k5ãAMÂ,8¯Øú® 4DJQî+âcY03Ë{Ävl®^B÷ŞA³éPºY
á"¦0c‚‡Êxµ3kçÁ~«ò¯,xŒÎL!–<”1ƒ¤‘÷{b«&Ç X¸aÏcÄh}K)
¦hß)x¶lÃ†hg·_=œ¡ĞŞ°£pÉ!ÙïP¸}ŒYÈyñ:ÁW\?~$óÎÑi‚FÔş.ÿQ²°dÍ':¦4ÃÓÁÍH×µ*QÓ«‡{áJ@Še&Ow¯‰4Ë™ìü3ü*œO³ˆŒŠö˜‘±“byñtYÛÍ7Ø ïF„øªÜMuÎDçvèşËÚé#Í˜)î<( ãôm	MœŞ $ÙÍ°ËYéVÄ†½P@¾4ºÔ}øƒÁÜ2ÂËM-¢t ÷Û?/8Wø8÷!»oXır“GøNîÿ{òŸ	#p0÷MNlì@Sgq§È`;bSÜr‚¨fCe¢ù}O}A°«v¹ã¤6)&ÇÜìóÔ¿­^u—’ö£m4&9g¿ƒr\êñ -5šLÒ†fğÔ¾ì
çŒÇ ûâùë$kã•†ó¤ ¥¬f¹•zd³1N?é?å@~Œ"œ¿Xbô~v/ ½Tb4­ uOvu|SI»vë5#?û–å0¹øáÅ"}Ywÿåf¯â#nk—"¯DaÂ¥E]Ywæœ€Cêİ‡Ÿ]c~j?ª™*ÈÍÑ±µ"ï¹çîå-…<Â	IW‚ Å³Wƒ;D$AûÑ’–Räs»Ò0O'cŸÇ
‹gòaä]QÒÄ+ &×şµàİJš{ ÆL‹œĞÛŞšÓc—]¿„ßsUˆ&ÅD¨ÓŠ±‘˜™À}À´QCe»æ@àºñmgªƒKP/wÃª?QÚNÈ –`è» <.™BR7›Üß¢„bjÂ­îz\M¸b÷ª_vM6Ö#VšæÄƒº‡)½»ÃìÚ91#¾ŸŒo™L3ßW
¼×î• 2åb7<¾±ÛáqÁ_Xj«­S}‰*uxÏ0A™?ØÄÖØJ[ˆYİiÍŠÔû€¦AàbB@¯ZYü,Ax.íÆ±ù¿˜¸b'Qša·fb²å<lòT¿R”Rı°Å]ö»»D‰m®‡sÖíoqªu)ùÙçÈ4b%”öèÔã¸§5¡ğ¾Bú•2¢z4‹–Ç
òT5©ÔÇÏÊõbÚ 8ğÌ­ĞU~ô‚U/ãKYŞÇE`ãø­P€v»
œÇœèÓ©ÃØEÖ¬ÓĞÚ}UFéÔÚˆÆsä‚Ğ)¾b[0pÔÈş—¥²¹$c"Qø}°Aâ£š¡dÆõ­g$ì#1c}áó[¼Óÿ‹á­Ñ«Sµ´ëäµ˜Å[rùÉ7ºäRõ‹Yüíëı¨cô;±!D”a;X€B˜±ÌáÆ(íÊ€Ö}_¶ğÕX©§¸>Q…’´0&ßå6V—étxqğËáM™³Š†¿®7W33æç6]SH#ô:¤H–.&¯%€hºB‰ÆĞdV€(aÙ¨C vEúY¡¢Ô83Y6=PKù’1Z3¡ÔD^#°v5ÖG:Å™®ºú¿wü´J¬‹zY|EiÜõíhwZ°ó åt?¿¥iş/¨¥ì/ŒÆ¬)q¥¤Uö´€)³V/läY°Ç”ü~½Ø/ì•ñ¶Ä7gnD@Úê¤À%ì®Öÿ	)çËB^€{Ô]…5>7’d§É¨¶ÍE‰Ö!KœæÆÈIªŸGJ†Yùë{¨Æ1I"x·‹ˆôwÜ®VbíøÛ:]' «§[ëOYÅ`ıÅMÑ¥¼Ÿö£ÚIÍèˆvy|À«ü ,õ}æÃ3°)_?í£R@š>\°
İ
Û?kGOş©Rİ(tPÃûI.t"êo‘ØgúH´0útCWä3Ş*:uQhÒÑK­Ì\~¤5NT^mîfÀ¿/L¼M‰£e*ôèB€É$ÄŒœQÏá·N-òæ`Œ,£—'GÂ''Úİ=Öcİ´	–¿}ND(Û·Ùõ¦R‚[ãy ]0öO7Ú}®ß´‘±C?ü&?ä}wİ^Nš‡Ô@Š÷ßz°x:Òñ;úÚ´éÌø5ºuÇ!¶¥ñè­&0H'½g<=ÈÓûpâ'i«E<şÑ‡@Ú%‚ÿ‰P”Î=`mÉé
#ª`Ú ‹în[W‹‘8n#ŠOÄØhsšÅÖ°`‡5N÷„ˆ™:Mw:—‘³Èy˜{2‡@}WíTû(Öv¿÷ÔlùÒ«¿Aq(Øã¥ë÷KC/C¹ÒÃØ¢+iïÏ•]9ó´›$Eïw|u	Q"5|†»9ñ½ÓÎV¿£Mr’ã2Â¡â~ey¶Ó™uÏPwÁ¬"ßé±ú@?e–İòN3yüzïïHÃÍ-ânÁ±mõû•Ôœä]75uK4n‹!‘8ñï­k§ÔÛÖ¬Ï–YÀ³óIµ2=6QÓİ4U¥î"[`OŒ	h¼¶¾ˆ	g{aŠ²Ö"Îš¸7“U=ï@!¦2i¶‰µß_oEÄ˜Ş=i¶äµ,Ÿû¨:#9à;õµÈ€/'.
6Ÿi‰ïWSh—'Û‡]ó?òÓ2|¡¬áUi^5€Î`t2ÔG™ÂŸÛ[÷}&"q3ZR{•ŠN$¡àø¿ö¾á›Šˆ{~:ÚĞ9fÁlzÈªeãÍyÅ‡XZŒe*ıŒ‰kÂÊÜA¢£ò4ƒd,^ª’{6f$ºÓf+b©R6‚€vÔb7ï›ë,;¹nºíUÒæ‹)ÕÂÓö±Ÿ5øíA[¢££c7ŠpH³qà—Yíš5X#8Pªâ/‹›`ŞSõì'ïõ¥;¨Ï5´Å&×ÖõFàu0#Éµ);È˜úÙ¹.›{-‰! îÖ´õ¨5TTMóöó