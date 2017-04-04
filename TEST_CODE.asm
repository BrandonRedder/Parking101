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
	; take abs"�6�e�(����������a��o>xSay�\/w�ց�(�}ơd��eU�F2�hx��s�,�:)v�o5��e�\5�|���m(m������ԥ��/�q��Bd�N����� -!�pL�*v�1�X�*��\d��M�~���Eh�@e�����U%�tZ6��
�A�(PZ�0�:�z)2��������֞7_W&~�d���N�_)�b�sK�XW��^	4����er��V�+l�w��	�Ύm�]�x6$:��j������-����X*�rYwl��K�EDN���g;Z���x�X��<2��	<�� 62�xr]�w����2�W�̋�h���ns��zNt�uL5�4'�	��c���}f9�=�5� �;��#g�dC��|�����|��Ʊ��\��O�і��?�C@�X��`�4�o�Kh0�z6� ��
�7!�>q+8�ߜ�k��ܱ��� Z�����l$2
̡os%޽�R#㯰IY<��&�/^>���������`�˖����D�������v�P�V�WPEX�.�����N�]�'cs�R �7��7�����7� �=� 	Ln�V�����d�ڗ����ZO
���Q�� ����^g�T5Ҭ�m˾���)r��>$+�q&Ѯ�u
1z�%�`F���Qq�r����Ӓm��v�5g�)S4�:J3�� �ո�3�Y��ۑP����iPbd^ɵ	(ȧ�2�F�r�G4�ܜ��goM*�%T�ER+LX�#��@HΛ*/�x(0W��"�SHϒO�;�vPۘKDX9a�ƃ0�"o���I�'_P6��M��K��e��,��$r���� ��@kDUͲ��U,\�E{�R�T6	���7x�R��̕#�l<>n���"4Pݫ�z��侈1�Z����!w��̘x�?��f����!u�L���jUm$��?+"F��1�!(�^�8��o[�)�ڡӸP�џ�,��ҙ�8�abl����-@�:��<�'���iw�� "�5��ě
j:�N�K�-����HN�w��C��P�Hηj��a��Z^�魯���}1A%�86�f3�_�������݀l䥪�q�rE�o�%�#����Z��&ᴭ���Y���&����{Kx�%��i�ݩ����*۪�Y�v�	�4�K��L����F�d��-��V��8(2*�ǟ�Ӥ�F/H�@]��v��ިBԡYHp�tJY��9@u��_2]9fR�m�%�M�������iz9]�/��^&4���=J^#��"�0���(�ɡZ\���x�؆�3�c
��q%�T�)>�/�R��Ng4�F��ӛ5�ǎ��Q�SĄ�%ޞq}W�ʒo�6͍��G~�]g��9JAs�V��f�^�N����$/�L�hZP��!M>�~��h�@���T��a�wKAq���\���V���/y��w�6ۂ����Iђ�^�p�68�YR7٠߻�jfc� U���@bIM\��|�!��Y�1��J1�%��#j	mQ��ʟH���&z��x+.�)�y��&zb{��� :�ނw\�&�����ѩH*�m�=���U�NB���8.?����M��r��fZ��ju�5� D�����:7���"�>���ې��	#�S`�p�[���*�LT
��<����հE��!8od���ɟ��e_;5dk'ym��4���*���q���쿊Gj"���h��)��&=XP����ۜ�����Ƴ��A���rؑ<� ��˥3�c���!h};�xIG�#�2���N�.��=vV�Ӎ=�j�\1Jn\�'ҵ|gעQ`F���q�2;���� iO�΂�#��(�`6$)q����m�Sf����|@#�x]�i�]��-�u��t\�/��L����+��ئyK%dҎ�B�_����7�4�g�q��e�&$jZ���A���cC��7E"����V�u��Py$~��0(r���QmA��}����Y^��p^=�GLI��\��R����7�_�,X[F���4�<�#���q}|Owi���9\�my$ʳ�ٺ@w|W�&?I��� �6���Q������i�9n�B�e�5��}:7��͹u]DA��jxl�^9eZ,���y�P���sf�-�M���`R�~8!��l��d��cr@�1��ia���|6~��9��"�D��q�X���xQ���4go7��C�کm6�>�.�{������yr����Goָ\Q���O��zwSx�n���z�T޷�fF���=�rX�ܯh�?7Ј��,`��6��)��}�pv~���҈�>iq�7B�#�KU���j�� �'�&���o|��3���ϮT$�J=�:`1��Lt@%��@,�sFʜj�����ä����ez��A&�qm��i��`�cx�	��,���[����38&�"�`�Y��ԍ�)���q���%En �!�.d�8�Pc'�)��-)�?�s�}��5�/��P���)�䨸wV������/��C*3�k�gfTӺ���������1� �����3���tw�����ƒ}M�4{��OQ�Km9*v}�ǡ�\���]f.v���&>�Ιy��l@��#{ɟؘ뷵P��E�EZ��v�i*�¶S��@��A[�>�
#n��Y�<������v
2�iW|HU�U�e!B�$�h�*�eD��Dqw��+��[�bcR3��z�����%e��� ݊�$U�{<�׾ؿ�S}dК�<�����EVt��uƿ& /��~vܰ�E��xv�g�$~% ��\�������x
���w��'�I��n����½2g�p7�o����U�=o8L\�{���n��e�RȖ����W-��H��\�87Q�q)U�ƀC�u�G��L! ή�BjA4�w}Ƈj��ͳ2��u3-^��z"L�[[��9 ���P�Í�����w:��p���O�q,��v�_��}A(��o��	�>{����w��+)"��aZ6g�>n6�E��$A�X��� r�ܨ���(ntv���|%\��"��Ӓ��ἶ���Wj6[���{&u /[Q���6w�U��Dѭ�u�sR�"rTl>�"����dG��W��9�}m��Ҝ���-#PԿ�n�J�8�&�l�0K�O��I*����fy	�w���6�16������[�E;����Jλ���)��� J��7I�O�ZB��\���][���3�dD �<�ȟ��TQ]lpm!�Hw"�_ߘ�|���@�� �y��ws�ٷ�CϬ�/�f*�=�Zyռ��`��_�,�d�q��BS����Ӹ��w�1.�A%�|ZpކH�K�9`Z�;S�n6�/��u���0��9��V>�����c�2㟈�ݫ+	3	�v�nm���(���|d"\@Y�����R������{��ICT��/��EH&n��t<��ȢZ9}�g��Ԛ�8�U�K�n�]iP� �A�`��&y^3`BfcT�vQqY��w����b��E��H���M=��<S 4O:���n���~�Op�C�iJ?T���Nr7+��/��<T<ѣ8� ݰ%�Q��ޝ��}�e �u;�9�	N�����z	�1�$�!-�M��OԾ<T�v�<N���.�ɔ�Jy�7!�Q�Pz,�je�ߖ�:��m%3�[����g\��L������}�f���F|})բF9l�8}�0Li\j4-F�<��vX��vC���3�$��|�+�^S���ܲ��80]I�*a���C6�b�9�wa	�"[ټ�~�!�l�-�b��HIg6=KW��x3dhg�.�w��/��-gW[�*dHJ���E,_Mp���m��x^+���X_{�v�,C�94C.ݯS��eVr"�f�!_�s*)yp=�����l�t;����`���x�CjS̲, �$��i�r(hDѴs��@ʦh�;o������k� 8��h�
|"�Y�?貭Ƌȷ"n�^Ό$��8�o]��f�X8X3$��s��_���}:���	�
�^h�0 ����O2�8rB��-�qAK`\��H
pL;�{���ʷ�f�37�Wo���#�}<hi�g�@r#����&5/[~M��������χ/`�{�5|�OU/��7g�-��Z���
*<����6z�qp�\wD|r,g���	�#IB��*�kJ=N��uQ�9���l�|�������%	��,�-[='��&]h(U��U�_�H�$�l7��r���X7z�'l��:�����A�x��q��}����ة��%�K
�"���E]�9����R�4���ކQq[`ʧ��6%]s�bx?)�-0,�P���u�fD�z�]�P[,tΈkVK@3�g,l���f����MQc���(%M�����V���0X;kG��w���*1+��A,&�9�: �R`f`��wO����Ԉ�g��D�FT�'& �0AX��q�T;y�i�5�EY�y[=o/�f%����j�����$q�a��t�_�T��|P���_yA`��Mz�Q�L]ן��k�mb��-�@��?�ƌTq�	+��gʬ.A�z�=+0ΪIT�Y�{�����*(���Gt!��\�/�K#OQ��~���w(� ��n�a3Z���z���t�h�Uǒ�a`:S������\��{:���0�P;���9�B,��D�"9��%V����bu�IfC��yg`F��I����P\e����Cn@,��C�i� ���!�h	�eR�͎;A^>A&j�!�����[�pn�dg�"�I��@G�A�Ѐ����r�����l��J�x���
uG^E��I��w�����Q��A�������ޅr�=��H���*V�@�Ǖx���[�x.�*�O �v����kDbR��(ӽ���$�T��UH�^�J�!�zE4�a�a�]� �| [�\�*/`��..�}���z�8��a ��(#�"�to\g���#�0��K����"
�϶ph$n~)Y@��ˆ�Pw1�͂�0ذ����nx߈WB36�a�Z�h�ŷ�����iQ���iVM41$�
jh�q6�nB+Y�dV���$��d�[F�rW���ɇ��ߍsE�şv�{QC>�ê���3�]a���T|��.|�5��LD�&�8��C*v�Q�1�4D�
}5�f����fԗ�*(��{��R��GK�t��$�ݒ֟[�a1����GM011�,]��xH/ 4�ڲ��	= ݱə���'���.;Y��� g�z��ޯ��nwk��$��My�<u�H���l�N�̑��2dăP'd7��|�҃a3�v� �6dm� ���T����i���E��F�������|�EZ��D�8J%�ﷸm1� �J��W��l��40&¼�#�?*�_�0�O�� �6���	 �A[�2�&�)�W���H��9�H�-���X��G�� �SA��A��quj��9/�����q��pBV�D� 6��(�%�Cj�>�|U언��u[<�ğ� $�ELP���P��?P7�h���]Q��nq%56g��H7b�&��YPx�j.��y��$�m��Rw�]���	�4�U��k�:o:���\p�	�V6e��W��.�����@�e3UpR6-^��0�_�᪍�B )�$6G�Aߪ8Y��q�F���{%�'���tMk�����9����éo��ּힹ��G�@*�?-�?Z� ��=i|H	Q�J���jJ*F
�%o���8}��r�D�t��NSâjק�bui1"a���F@<#��<{8^;���n�"N��t.C�����������=]�ьj��Kf����O�}/c�l"b&��@�� �9�XS��1�ckY2�5U"L|���H~V�H�i6��4
�JF�cj�@��k�g�A��M����Q������KX��O�m���cO"Q
݉��`g��]��"}=gA׆�^�l�l�y�d�8zE#�Nl���'-���K�\�Rr�N�O�.�}����>�	����
d=���V�x\t��7K�!G�5A�Y��k5�AM�,8�����4DJQ�+�cY03�{�vl�^B��A��P�Y
�"�0c���x�3k��~���,x��L!�<�1����{b�&� X�a�c��h}K)
�h�)x�lÝ�hg�_=���ް�p�!�P�}�Y�y�:�W\?~$���i�F��.�Q��d�':�4����H׵*Qӫ�{�J@��e&Ow��4˙��3�*��O���������by�tY��7� �F����Mu�D�v�����#͘)�<( ����m	M�ޠ$�Ͱ�Y�VĆ�P@�4��}����2��M-�t ��?/8W�8�!�oX�r�G�N��{�	#p0�MNl�@Sgq��`;bS�r��fCe��}O}A��v��6)&����Կ��^u�����m4&9g��r\�� -5�L҆f����
�� ����$k���� ��f��zd�1N?�?�@~�"��Xb�~v/ �Tb4� uOvu|SI�v�5#?���0����"}Yw��f��#nk�"�Da¥E]Yw最C�݇�]c~j?��*Ȑ�ѱ�"�����-�<�	IW� ųW�;D$A�ђ�R�s��0O'c��
�g�a�]Q��+�&�����J�{ �L����ޚ�c�]���sU�&�D��������}��QCe��@��mg��KP/wê?�Q�NȠ�`� <.�BR7��ߢ�bj­�z\M�b��_vM6�#V��ă��)�����91#���o�L3�W
��� 2�b7<����q�_Xj��S}�*ux�0A�?����J[�Y�i͊�����A�bB@�ZY�,Ax.�������b'Q�a�fb��<l�T�R���R���]���D�m���s��oq�u)����4b%����㸧5��B��2��z4���
�T5������bڠ8�̭�U~�U/�KY��E`���P��v�
�ǜ�ө��E֬���}�UF������s䝂�)�b[0p�������$c"Q�}�A⣚�d���g$�#1c}��[�������S�������[r��7��R���Y�����c�;�!D�a;X�B�����(�ʀ�}�_���X����>Q���0&��6V��txq���M������7W33��6]SH#�:��H�.&�%�h�B���dV�(a٨C vE�Y���83Y�6=PK��1Z3��D^#�v5�G:�������w���J��zY|Ei���hwZ����t?��i�/����/�Ɛ�)q��U���)�V/l�Y�ǔ�~��/���7gnD@����%���	)��B^�{�]�5>7�d�ɨ��E��!K�����I��GJ�Y��{��1I"x����wܝ�Vb���:]' ��[�OY�`��Mѥ�����I��vy|���� ,�}��3�)_?�R@�>\�
�
�?kGO��R�(�tP��I.t"�o��g�H�0�tCW�3�*:uQh��K��\~�5NT^m�f��/L�M���e*���B��$Č�Q��N-��`�,��'G�''��=�cݴ	��}ND(۷����R�[�y�]0�O7�}�����C?�&?�}w�^N���@���z��x:��;�ڴ�����5�u�!����&0H'�g<=ȏ��p�'i�E<�ч@�%���P��=`m��
#�`ڠ��n[W��8n#�O��hs��ְ`�5N����:Mw:����y�{2�@}W�T�(�v���l�ҫ�Aq(����KC/C���آ+i�ϕ]9�$E�w|u	Q"5|��9���V��Mr��2¡�~ey�әu�Pw��"�鞱�@?e���N3y�z��H��-�n��m���Ԝ�]75uK�4n�!�8��k���֬ϖY���I�2=6Q��4U���"[`O�	h����	g{a���"Κ�7�U=�@!�2i����_oEĘ�=i��,���:#9�;����/'.
6��i��WSh�'ۇ]�?��2|���Ui^5��`t2�G����[�}&"q3ZR{��N$��������ᛊ�{~:���9f�lzȪe��yŇXZ�e�*���k���A���4�d,^���{6f$��f+b�R6��v�b7��,;�n��U�����)������5��A[���c7�p�H�q��Y�5X#8P��/���`�S��'���;��5��&���F�u0#��);Ș�ٹ.�{-�!���ִ��5TTM���