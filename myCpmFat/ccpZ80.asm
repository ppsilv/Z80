;	
;	ENHANCED CP/M CONSOLE COMMAND PROCESSOR (CCP) for CP/M REV. 2.2
;	
;	Origianl CCP disassembled by ????
;	Original CCP disassembled further by RLC
;	Original CCP commented by RLC
;	Modified and generalized by John Thomas (6/20/81)
;	Macros expanded and condtional for terminals which
;	use	form feeds to clear the screen added by
;	Bo	McCormick (6/27/81)
;
;	Converted to Z80-style mnemonics and slightly modified
;	for Z80 Playground by John Squires, January 2021
;	
;	ASSEMBLING THIS CCP FOR CP/M 2.2 *****
;	You	must be using a Z-80 processor to run this
;	program. You do not need MAC or any macro library.
;	If	you add further modifications to the program
;	the	total size of the program must not exceed
;	2K	in order to fit under the BDOS. (Unless you change the CCP_START location!)
;	Code must be added to use the Clear Screen command with
;	your terminal, if it is not VT52 compatible.
;	Also, there is a provision for a boot-up command. Place
;	the	command to be executed on cold and warm starts at 
;	location CBUFF.
;	
;	NON	-STANDARD FEATURES *****
;	The	non-standard features incorporated into this CCP are:
;	A.	The Command-Search Hierarchy, as follows --
;	    1.	Scan for a CCP-resident command and execute it if found	
;	    2.	If not CCP-resident, look for a .COM file on disk
;	    3.	If the .COM file is not found in the current user area and the current user area is not USER 0,
;	    	USER 0 is selected and scanned for the file
;		4.	If the .COM file is not found on the current logged-in disk drive, drive A: is selected
;			and	scanned for the file
;	B.	The DIR Command no longer prints the current drive spec at	the beginning of each line
;	C.	The TYPE Command pages its output
;	D.	A LIST Command now exists which is like TYPE but does not page and sends its output to the LST: device
;	E.	A CLS (Clear Screen) Command now exists which clears the screen of the terminal
;	F.	The user number is printed as part of the command prompt;
;		the	prompt is now du>, such as A0> and A15>
;	G.	Z80-code is used throughout to reduce the size of the CCP
;		and	give room to implement the additional functions
;	H.	The input line buffer has been reduced in size to 100 bytes
;	I.	The ERA Command displays the names of the files it is to erase	
;	J.	The DIR Command has an additional special form of "DIR @"
;		which	displays all files (both non-system and system),
;		while	"DIR" displays just the non-system files
;	K.	The Directory Display no longer displays the disk name at
;		the	beginning of each line and it now includes a '.' between
;		the	file name and file type (FILENAME.TYP)
;	L.	The SUBMIT File Facility now expects the $$$.SUB file to be
;		on	the currently logged-in disk (as opposed to always A:)
;	M.	The Command Line Prompt is now '$' if the command comes from
;		a	$$$.SUB file and '>' if the command comes from the user;
;		also, the '>' is not printed until all preprocessing is completed
;	N.	The TYPE and LIST Commands mask the MSB of each byte, so that
;		files created by editors such as EDIT80 are "printable"
;	
FALSE:	EQU	0
TRUE:	EQU	NOT FALSE

	include "cfg_locations.asm"

CCPLOC:	EQU	CCP_START	; START OF CCP IN MEMORY
	; REPLACE WITH VALUE
	; FOR YOUR SYSTEM
	
NLINES:	EQU	24			; NUMBER OF LINES ON CRT SCREEN
H19:	EQU	FALSE		; USING HEATH H19/H89 TERMINAL
HAZE:	EQU	FALSE		; USING HAZELTINE 1500 TERMINAL
FFTERM:	EQU	TRUE		; USING TERMINAL THAT RESPONDS TO 0CH
	; OR ANY CHARACTER IF YOU CHANGE THE
	; VALUE OF FF BELOW
CR:		EQU	0DH
LF:		EQU	0AH
TAB:	EQU	09H
FF:		EQU	0CH		;<==== CHANGE THIS BYTE IF TERMINAL
ESC:	EQU	1BH		;      NEEDS A DIFFERENT CHARACTER
					;      TO CLEAR THE SCREEN
	
WBOOT:	EQU	0000H		; CP/M WARM BOOT ADDRESS
UDFLAG:	EQU	0004H		; USER NUMBER IS IN HIGH NYBBLE, DISK IN LOW
BDOS:	EQU	0005H		; BDOS FUNCTION CALL ENTRY PT
TBUFF:	EQU	0080H		; DEFAULT DISK I/O BUFFER
TFCB:	EQU	005CH		; DEFAULT FCB BUFFER
TPA:	EQU	0100H		; BASE OF TPA
	
	
	ORG	CCPLOC		; START OF CCP IN MEMEORY IN YOUR SYSTEM
	
ENTRY:	
	JP	CCP
	JP	CCP1
	
;	INPUT	COMMAND LINE AND DEFAULT COMMAND
BUFLEN	EQU	100		; MAXIMUM BUFFER LENGTH
MBUFF:	
	DEFB	BUFLEN		; MAXIMUM BUFFER LENGTH
CBUFF:	
	DEFB	3		;<== NUMBER OF VALID CHARS IN COMMAND LINE
CIBUFF:	
	DEFB	'DIR '		;<== DEFAULT (COLD BOOT) COMMAND
	DEFB	'    '
	DEFB	'    '
	DEFB	'    '
CIBUF:	
	DEFS	85		; TOTAL IS 100 BYTES
	DEFS	20		; STACK AREA
STACK EQU	$		; TOP OF STACK
	
CIBPTR:	
	DEFW	CIBUFF		;POINTER TO CMD INPUT BUFF
CIPTR:	
	DEFW	CIBUF		;CURRENT PNTR
	
;	
;	I/O UTILITIES
;	
	
;	OUTPUT <SP>
SPACER:	
	LD	A,' '		; FALL THRU TO CONOUT
	
;	OUTPUT CHAR IN REG A TO CONSOLE AND DON'T CHANGE BC
CONOUT:	
	PUSH	BC
	PUSH	HL
	LD	C,02H
OUTPUT:	
	LD	E,A
	CALL	BDOS
	POP	HL
	POP	BC
	RET	
	
;	CALL	BDOS AND SAVE BC
BDOSB:	
	PUSH	BC
	CALL	BDOS
	POP	BC
	RET	
	
;	OUTPUT CHAR IN REG A TO LIST DEVICE
LSTOUT:	
	PUSH	BC
	PUSH	HL
	LD	C,05H
	DEFB	18H
	DEFB	OUTPUT-$-1 AND 0FFH
	
;	OUTPUT <CRLF>
CRLF:	
	LD	A,CR
	CALL	CONOUT
	LD	A,LF
	DEFB	18H
	DEFB	CONOUT-$-1 AND 0FFH
	
;	PRINT	STRING (ENDING IN 0) PTED TO BY RET ADR; START WITH <CRLF>
PRINT:	
	EX	(SP),HL		; GET PTR TO STRING
	PUSH	AF		; SAVE FLAGS
	CALL	CRLF		; NEW LINE
	CALL	PRIN1
	POP	AF		; GET FLAGS
	EX	(SP),HL		; RESTORE HL AND RET ADR
	RET	
	
;	PRINT	STRING (ENDING IN 0) PTED TO BY HL
PRIN1:	
	LD	A,(HL)		; GET NEXT BYTE
	INC	HL		; PT TO NEXT BYTE
	OR	A		; DONE IF 0
	RET	Z
	CALL	CONOUT		; PRINT CHAR
	DEFB	18H
	DEFB	PRIN1-$-1 AND 0FFH
	
;	
;	BDOS	FUNCTION ROUTINES
;	
	
RESET:	
	LD	C,0DH
	JP	BDOS
;
LOGIN:
	LD	E,A
	LD	C,0EH
	JP	BDOS
;
OPENF:	
	XOR	A
	LD	(FCBCR),A
	LD	DE,FCBDN	; FALL THRU TO OPEN
;
OPEN:	
	LD	C,0FH		; FALL THRU TO GRBDOS
;
GRBDOS:	
	CALL	BDOS
	INC	A		; SET ZERO FLAG FOR ERROR RETURN
	RET	
;
CLOSE:	
	LD	C,10H
	DEFB	18H
	DEFB	GRBDOS-$-1 AND 0FFH
;
SEARF:	
	LD	DE,FCBDN	; SPECIFY FCB
SEAR1:	
	LD	C,11H
	DEFB	18H
	DEFB	GRBDOS-$-1 AND 0FFH
;
SEARN:	
	LD	C,12H
	DEFB	18H
	DEFB	GRBDOS-$-1 AND 0FFH
;
DELETE:	
	LD	C,13H
	JP	BDOS
;
READF:	
	LD	DE,FCBDN	; FALL THRU TO READ
;
READ:	
	LD	C,14H		; FALL THRU TO GOBDOS
;
GOBDOS:	
	CALL	BDOSB		; PRESERVE B
	OR	A
	RET	
;
WRITE:	
	LD	C,15H
	DEFB	18H
	DEFB	GOBDOS-$-1 AND 0FFH
;
CREATE:	
	LD	C,16H
	DEFB	18H
	DEFB	GRBDOS-$-1 AND 0FFH
;
GETUSR:	
	LD	E,0FFH		;GET CURRENT USER NUMBER
SETUSR:	
	LD	C,20H		;SET USER NUMBER TO VALUE IN E (GET IF E=FFH)
	JP	BDOS
	
;	
;	END	OF BDOS FUNCTIONS
;	
	
;	
;	CCP	UTILITIES
;	
	
;	DEFL	USER/DISK FLAG TO CURRENT USER AND DEFAULT DISK
SETUD:	
	CALL	GETUSR		; GET NUMBER OF CURRENT USER
	ADD	A,A		; PLACE IT IN HIGH NYBBLE
	ADD	A,A
	ADD	A,A
	ADD	A,A
	LD	HL,TDRIVE	; MASK IN DEFAULT DRIVE NUMBER (LOW NYBBLE)
	OR	(HL)		; MASK IN
	LD	(UDFLAG),A	; SET USER/DISK NUMBER
	RET	
	
;	DEFL	USER/DISK FLAG TO USER 0 AND DEFAULT DISK
SETU0D:	
	LD	A,(TDRIVE)	; SET USER 0/DEFAULT DISK
	LD	(UDFLAG),A	; SET USER/DISK NUMBER
	RET	
	
;	CONVE	RT CHAR IN A TO UPPER CASE
UCASE:	
	CP	61H		; LOWER-CASE A
	RET	C
	CP	7BH		; GREATER THAN LOWER-CASE Z?
	RET	NC
	AND	5FH		; CAPITALIZE
	RET	
	
;	INPUT	NEXT COMMAND TO CCP
REDBUF:	
	LD	A,(RNGSUB)	; SUBMIT FILE CURRENTLY IN EXECUTION?
	OR	A		; 0=NO
	DEFB	28H
	DEFB	RB1-$-1 AND 0FFH; GET LINE FROM CONSOLE IF NOT
	LD	DE,SUBFCB	; OPEN $$$.SUB
	CALL	OPEN
	DEFB	28H
	DEFB	RB1-$-1 AND 0FFH; ERASE $$$.SUB IF END OF FILE AND GET CMND
	LD	A,(SUBFRC)	; GET VALUE OF LAST RECORD IN FILE
	DEC	A		; PT TO NEXT TO LAST RECORD
	LD	(SUBFCR),A	; SAVE NEW VALUE OF LAST RECORD IN $$$.SUB
	LD	DE,SUBFCB	; READ LAST RECORD OF SUBMIT FILE
	CALL	READ
	DEFB	20H
	DEFB	RB1-$-1 AND 0FFH; ABORT $$$.SUB IF ERROR IN READING LAST REC
	LD	DE,CBUFF	; COPY LAST RECORD (NEXT SUBMIT CMND) TO CBUFF
	LD	HL,TBUFF	;   FROM TBUFF
	LD	B,BUFLEN	; NUMBER OF BYTES
	CALL	MOVEHD
	LD	HL,SUBFS2	; PT TO S2 OF $$$.SUB FCB
	LD	(HL),0		; SET S2 TO ZERO
	INC	HL		; PT TO RECORD COUNT
	DEC	(HL)		; DECREMENT RECORD COUNT OF $$$.SUB
	LD	DE,SUBFCB	; CLOSE $$$.SUB
	CALL	CLOSE
	DEFB	28H
	DEFB	RB1-$-1 AND 0FFH; ABORT $$$.SUB IF ERROR
	LD	A,'$'		; PRINT SUBMIT PROMPT
	CALL	CONOUT
	LD	HL,CIBUFF	; PRINT COMMAND LINE FROM $$$.SUB
	CALL	PRIN1
	CALL	BREAK		; CHECK FOR ABORT (ANY CHAR)
	DEFB	28H
	DEFB	CNVBUF-$-1 AND 0FFH; IF <NULL> (NO ABORT), CAP COMMAND AND RUN
	CALL	SUBKIL		; KILL $$$.SUB IF ABORT
	JP	RESTRT		; RESTART CCP
	
;	INPUT	COMMAND LINE FROM USER CONSOLE
RB1:	
	CALL	SUBKIL		; ERASE $$$.SUB IF PRESENT
	CALL	SETUD		; SET USER AND DISK
	LD	A,'>'		; PRINT PROMPT
	CALL	CONOUT
	LD	C,0AH		; READ COMMAND LINE FROM USER
	LD	DE,MBUFF
	CALL	BDOS
	CALL	SETU0D		; SET CURRENT DISK NUMBER IN LOWER PARAMS
	
;	CAPIT	ALIZE STRING (ENDING IN 0) IN CBUFF
CNVBUF:	
	LD	HL,CBUFF	; PT TO USER'S COMMAND
	LD	B,(HL)		; CHAR COUNT IN B
CB1:	
	INC	HL		; PT TO 1ST VALID CHAR
	LD	A,B		; DONE WHEN <NULL> ENCOUNTERED
	OR	A
	DEFB	28H
	DEFB	CB2-$-1 AND 0FFH
	LD	A,(HL)		; CAPITALIZE COMMAND CHAR
	CALL	UCASE
	LD	(HL),A
	DEC	B		; CONTINUE UNTIL END OF COMMAND LINE
	DEFB	18H
	DEFB	CB1-$-1 AND 0FFH
CB2:	
	LD	(HL),A		; STORE ENDING <NULL>
	LD	HL,CIBUFF	; SET COMMAND LINE PTR TO 1ST CHAR
	LD	(CIBPTR),HL
	RET	
	
;	CHECK	FOR ANY CHAR FROM USER CONSOLE; RET W/ZERO SET IF NONE
BREAK:	
	PUSH	DE		; SAVE DE
	LD	E,0FFH		; GET STATUS
	LD	C,6		; DIRECT CONSOLE I/O
	CALL	BDOSB
	POP	DE
	AND	7FH		; MASK MSB AND SET ZERO FLAG
	RET	
	
;	RETUR	N NUMBER OF CURRENT DISK IN A
GETDRV:	
	LD	C,19H
	JP	BDOS
	
;	DEFL	80H AS DMA ADDRESS
DEFDMA:	
	LD	DE,TBUFF	; 80H=TBUFF
DMASET:	
	LD	C,1AH
	JP	BDOS
	
;	CHECK	FOR SUBMIT FILE IN EXECUTION AND ABORT IT IF SO
SUBKIL:	
	LD	HL,RNGSUB	; CHECK FOR SUBMIT FILE IN EXECUTION
	LD	A,(HL)
	OR	A		; 0=NO
	RET	Z
	LD	(HL),0		; ABORT SUBMIT FILE
	LD	DE,SUBFCB	; DELETE $$$.SUB
	JP	DELETE
	
;	INVAL	ID COMMAND -- PRINT IT
ERROR:	
	CALL	CRLF		; NEW LINE
	LD	HL,(CIPTR)	; PT TO BEGINNING OF COMMAND LINE
ERR2:	
	LD	A,(HL)		; GET CHAR
	CP	' '		; SIMPLE '?' IF <SP>
	DEFB	28H
	DEFB	ERR1-$-1 AND 0FFH
	OR	A		; SIMPLE '?' IF <NULL>
	DEFB	28H
	DEFB	ERR1-$-1 AND 0FFH
	PUSH	HL		; SAVE PTR TO ERROR COMMAND CHAR
	CALL	CONOUT		; PRINT COMMAND CHAR
	POP	HL		; GET PTR
	INC	HL		; PT TO NEXT
	DEFB	18H
	DEFB	ERR2-$-1 AND 0FFH; CONTINUE
ERR1:	
	LD	A,'?'		; PRINT '?'
	CALL	CONOUT
	CALL	SUBKIL		; TERMINATE ACTIVE $$$.SUB IF ANY
	JP	RESTRT		; RESTART CCP
	
;	CHECK	TO SEE IF DE PTS TO DELIMITER; IF SO, RET W/ZERO FLAG SET
SDELM:	
	LD	A,(DE)
	OR	A		; 0=DELIMITER
	RET	Z
	CP	' '		; ERROR IF < <SP>
	JP	C,ERROR
	RET	Z		; <SP>=DELIMITER
	CP	'='		; '='=DELIMITER
	RET	Z
	CP	5FH		; UNDERSCORE=DELIMITER
	RET	Z
	CP	'.'		; '.'=DELIMITER
	RET	Z
	CP	':'		; ':'=DELIMITER
	RET	Z
	CP	';'		; ';'=DELIMITER
	RET	Z
	CP	'<'		; '<'=DELIMITER
	RET	Z
	CP	'>'		; '>'=DELIMITER
	RET	
	
;	SKIP	STRING PTED TO BY DE (STRING ENDS IN 0) UNTIL END OF STRING
;	OR	NON-BLANK ENCOUNTERED (BEGINNING OF TOKEN)
SBLANK:	
	LD	A,(DE)
	OR	A
	RET	Z
	CP	' '
	RET	NZ
	INC	DE
	DEFB	18H
	DEFB	SBLANK-$-1 AND 0FFH
	
;	ADD	A,A TO HL (HL=HL+A)
ADDAH:	
	ADD	A,L
	LD	L,A
	RET	NC
	INC	H
	RET	
	
;	EXTRA	CT TOKEN FROM COMMAND LINE AND PLACE IT INTO FCBDN; FORMAT FCBDN
;	IF	TOKEN RESEMBLES FILE NAME AND TYPE (FILENAME.TYP);
;	ON	INPUT, CIBPTR PTS TO CHAR AT WHICH TO START SCAN
;	ON	OUTPUT, CIBPTR PTS TO CHAR AT WHICH TO CONTINUE AND ZERO FLAG IS SET
;	IF	'?' IS IN TOKEN
SCANER:	
	LD	A,0		; START AT DRIVE SPECIFICATION BYTE
SCAN1:	
	LD	HL,FCBDN	; POINT TO FCBDN
	CALL	ADDAH		; OFFSET INTO FCB
	PUSH	HL
	PUSH	HL
	XOR	A		; SET TEMPORARY DRIVE NUMBER TO DEFAULT
	LD	(TEMPDR),A
	LD	HL,(CIBPTR)	; GET PTR TO NEXT CHAR IN COMMAND LINE
	EX	DE,HL		; PTR IN DE
	CALL	SBLANK		; SKIP TO NON-BLANK OR END OF LINE
	EX	DE,HL
	LD	(CIPTR),HL	; SET PTR TO NON-BLANK OR END OF LINE
	EX	DE,HL		; DE PTS TO NEXT NON-BLANK OR END OF LINE CHAR
	POP	HL		; GET PTR TO NEXT BYTE IN FCBDN
	LD	A,(DE)		; END OF LINE?
	OR	A		; 0=YES
	DEFB	28H
	DEFB	SCAN2-$-1 AND 0FFH
	SBC	A,'A'-1		; CONVERT POSSIBLE DRIVE SPEC TO NUMBER
	LD	B,A		; STORE NUMBER (A:=0, B:=1, ETC) IN B
	INC	DE		; PT TO NEXT CHAR
	LD	A,(DE)		; SEE IF IT IS A COLON (:)
	CP	':'
	DEFB	28H
	DEFB	SCAN3-$-1 AND 0FFH; YES^  WE HAVE A DRIVE SPEC
	DEC	DE		; NO^  BACK UP PTR TO FIRST NON-BLANK CHAR
SCAN2:
	LD	A,(TDRIVE)	; SET 1ST BYTE OF FCBDN AS DEFAULT DRIVE
	LD	(HL),A
	DEFB	18H
	DEFB	SCAN4-$-1 AND 0FFH
SCAN3:	
	LD	A,B		; WE HAVE A DRIVE SPEC^
	LD	(TEMPDR),A	; SET TEMPORARY DRIVE
	LD	(HL),B		; SET 1ST BYTE OF FCBDN AS SPECIFIED DRIVE
	INC	DE		; PT TO BYTE AFTER ':'
	
;	EXTRA	CT FILENAME FROM POSSIBLE FILENAME.TYP
SCAN4:	
	LD	B,08H		; MAX OF 8 CHARS IN FILE NAME
SCAN5:	
	CALL	SDELM		; DONE IF DELIMITER ENCOUNTERED - <SP> FILL
	DEFB	28H
	DEFB	SCAN9-$-1 AND 0FFH
	INC	HL		; PT TO NEXT BYTE IN FCBDN
	CP	'*'		; IS (DE) A WILD CARD?
	DEFB	20H
	DEFB	SCAN6-$-1 AND 0FFH; CONTINUE IF NOT
	LD	(HL),'?'	; PLACE '?' IN FCBDN AND DON'T ADVANCE DE IF SO
	DEFB	18H
	DEFB	SCAN7-$-1 AND 0FFH
SCAN6:	
	LD	(HL),A		; STORE FILENAME CHAR IN FCBDN
	INC	DE		; PT TO NEXT CHAR IN COMMAND LINE
SCAN7:	
	DEFB	10H
	DEFB	SCAN5-$-1 AND 0FFH; DECREMENT CHAR COUNT UNTIL 8 ELAPSED
SCAN8:	
	CALL	SDELM		; 8 CHARS OR MORE - SKIP UNTIL DELIMITER
	DEFB	28H
	DEFB	SCAN10-$-1 AND 0FFH; ZERO FLAG SET IF DELIMITER FOUND
	INC	DE		; PT TO NEXT CHAR IN COMMAND LINE
	DEFB	18H
	DEFB	SCAN8-$-1 AND 0FFH
SCAN9:	
	INC	HL		; PT TO NEXT BYTE IN FCBDN
	LD	(HL),' '	; FILL FILENAME PART WITH <SP>
	DEFB	10H
	DEFB	SCAN9-$-1 AND 0FFH
	
;	EXTRA	CT FILE TYPE FROM POSSIBLE FILENAME.TYP
SCAN10:	
	LD	B,03H		; PREPARE TO EXTRACT TYPE
	CP	'.'		; IF (DE) DELIMITER IS A '.', WE HAVE A TYPE
	DEFB	20H
	DEFB	SCAN15-$-1 AND 0FFH; FILL FILE TYPE BYTES WITH <SP>
	INC	DE		; PT TO CHAR IN COMMAND LINE AFTER '.'
SCAN11:	
	CALL	SDELM		; CHECK FOR DELIMITER
	DEFB	28H
	DEFB	SCAN15-$-1 AND 0FFH; FILL REST OF TYPE IF IT IS A DELIMITER
	INC	HL		; PT TO NEXT BYTE IN FCBDN
	CP	'*'		; WILD?
	DEFB	20H
	DEFB	SCAN12-$-1 AND 0FFH; STORE CHAR IF NOT WILD
	LD	(HL),'?'	; STORE '?' AND DON'T ADVANCE COMMAND LINE PTR
	DEFB	18H
	DEFB	SCAN13-$-1 AND 0FFH
SCAN12:	
	LD	(HL),A		; STORE CHAR IN FCBDN
	INC	DE		; PT TO NEXT CHAR IN COMMAND LINE
SCAN13:	
	DEFB	10H
	DEFB	SCAN11-$-1 AND 0FFH; COUNT DOWN CHARS IN FILE TYPE (3 MAX)
SCAN14:	
	CALL	SDELM		; SKIP REST OF CHARS AFTER 3-CHAR TYPE TO
	DEFB	28H
	DEFB	SCAN16-$-1 AND 0FFH;   DELIMITER
	INC	DE
	DEFB	18H
	DEFB	SCAN14-$-1 AND 0FFH
SCAN15:	
	INC	HL		; FILL IN REST OF TYP WITH <SP>
	LD	(HL),' '
	DEFB	10H
	DEFB	SCAN15-$-1 AND 0FFH
	
;	FILL	IN EX, S1, S2, AND RC WITH ZEROES
SCAN16:	
	LD	B,4		; 4 BYTES
SCAN17:	
	INC	HL		; PT TO NEXT BYTE IN FCBDN
	LD	(HL),0
	DEFB	10H
	DEFB	SCAN17-$-1 AND 0FFH
	
;	SCAN	COMPLETE -- DE PTS TO DELIMITER BYTE AFTER TOKEN
	EX	DE,HL		; STORE PTR TO NEXT BYTE IN COMMAND LINE
	LD	(CIBPTR),HL
	
;	DEFL	ZERO FLAG TO INDICATE PRESENCE OF '?' IN FILENAME.TYP
	POP	HL		; GET PTR TO FCBDN IN HL
	LD	BC,11		; SCAN FOR '?' IN FILENAME.TYP (C=11 BYTES)
SCAN18:	
	INC	HL		; PT TO NEXT BYTE IN FCBDN
	LD	A,(HL)
	CP	'?'
	DEFB	20H
	DEFB	SCAN19-$-1 AND 0FFH
	INC	B		; B<>0 TO INDICATE '?' ENCOUNTERED
SCAN19:	
	DEC	C		; COUNT DOWN
	DEFB	20H
	DEFB	SCAN18-$-1 AND 0FFH
	LD	A,B		; A=B=NUMBER OF '?' IN FILENAME.TYP
	OR	A		; SET ZERO FLAG TO INDICATE ANY '?'
	RET	
	
;	
;	CCP	BUILT-IN COMMAND TABLE AND COMMAND PROCESSOR
;	
NCMNDS:	EQU	8		; NUMBER OF CCP COMMANDS
NCHARS:	EQU	4		; NUMBER OF CHARS/COMMAND
;	CCP	COMMAND NAME TABLE
CMDTBL:	
	DEFB	'DIR '
	DEFB	'ERA '
	DEFB	'LIST'
	DEFB	'TYPE'
	DEFB	'SAVE'
	DEFB	'REN '
	DEFB	'USER'
	DEFB	'CLS '
;	CCP	COMMAND ADDRESS TABLE
REQTBL:	
	DEFW	DIR
	DEFW	ERA
	DEFW	LIST
	DEFW	TYPE
	DEFW	SAVE
	DEFW	REN
	DEFW	USER
	DEFW	CLS
	DEFW	COM		;MUST BE A COM FILE
	
;	CMDTB	L (COMMAND TABLE) SCANNER
;	ON	RETURN, A=TABLE ENTRY # (0-5) OR 6 IF NOT FOUND (COM FILE)
CMDSER:	
	LD	HL,CMDTBL	; PT TO COMMAND TABLE
	LD	C,0		; SET COMMAND COUNTER
CMS1:	
	LD	A,C		; CHECK FOR DONE
	CP	NCMNDS		; NUMBER OF COMMANDS
	RET	NC
	LD	DE,FCBFN	; PT TO STORED COMMAND NAME
	LD	B,NCHARS	; NUMBER OF CHARS/COMMAND (8 MAX)
CMS2:	
	LD	A,(DE)		; COMPARE AGAINST TABLE ENTRY
	CP	(HL)
	DEFB	20H
	DEFB	CMS3-$-1 AND 0FFH; NO MATCH
	INC	DE		; PT TO NEXT CHAR
	INC	HL
	DEFB	10H
	DEFB	CMS2-$-1 AND 0FFH; COUNT DOWN
	LD	A,(DE)		; NEXT CHAR IN INPUT COMMAND MUST BE <SP>
	CP	' '
	DEFB	20H
	DEFB	CMS4-$-1 AND 0FFH
	LD	A,C		; TABLE ENTRY NUMBER IN A
	RET	
CMS3:	
	INC	HL		; SKIP TO NEXT COMMAND TABLE ENTRY
	DEFB	10H
	DEFB	CMS3-$-1 AND 0FFH
CMS4:	
	INC	C		; INCREMENT TABLE ENTRY NUMBER
	DEFB	18H
	DEFB	CMS1-$-1 AND 0FFH
	
;	
;	CCP	STARTING POINTS
;	
	
;	START	CCP AND DON'T PROCESS DEFAULT COMMAND STORED
CCP1:	
	XOR	A		; SET NO DEFAULT COMMAND
	LD	(CBUFF),A
	
;	START	CCP AND POSSIBLY PROCESS DEFAULT COMMAND
CCP:	
	LD	SP,STACK	; RESET STACK
	PUSH	BC
	LD	A,C		; C=USER/DISK NUMBER (SEE LOC 4)
	RRA			; EXTRACT USER NUMBER
	RRA	
	RRA	
	RRA	
	AND	0FH
	LD	E,A		; SET USER NUMBER
	CALL	SETUSR
	CALL	RESET		; RESET DISK SYSTEM
	POP	BC
	LD	A,C		; C=USER/DISK NUMBER (SEE LOC 4)
	AND	0FH		; EXTRACT DEFAULT DISK DRIVE
	LD	(TDRIVE),A	; SET IT
	CALL	LOGIN		; LOG IN DEFAULT DISK
	LD	DE,SUBFCB	; CHECK FOR $$$.SUB ON CURRENT DISK
	CALL	SEAR1
	CPL			; 0FFH IS RETURNED IF NO $$$.SUB, SO COMPLEMENT
	LD	(RNGSUB),A	; SET FLAG (0=NO $$$.SUB)
	LD	A,(CBUFF)	; EXECUTE DEFAULT COMMAND?
	OR	A		; 0=NO
	DEFB	20H
	DEFB	RS1-$-1 AND 0FFH
	
;	PROMP	T USER AND INPUT COMMAND LINE FROM HIM
RESTRT:
	LD	SP,STACK	; RESET STACK
	
;	PRINT	PROMPT (DU>)
	CALL	CRLF		; PRINT PROMPT
	CALL	GETDRV		; CURRENT DRIVE IS PART OF PROMPT
	ADD	A,'A'		; CONVERT TO ASCII A-P
	CALL	CONOUT
	CALL	GETUSR		; GET USER NUMBER
	CP	10		; USER < 10?
	DEFB	38H
	DEFB	RS00-$-1 AND 0FFH
	SUB	10		; SUBTRACT 10 FROM IT
	PUSH	AF		; SAVE IT
	LD	A,'1'		; OUTPUT 10'S DIGIT
	CALL	CONOUT
	POP	AF
RS00:	
	ADD	A,'0'		; OUTPUT 1'S DIGIT (CONVERT TO ASCII)
	CALL	CONOUT
	
;	READ	INPUT LINE FROM USER OR $$$.SUB
	CALL	REDBUF		; INPUT COMMAND LINE FROM USER (OR $$$.SUB)
	
;	PROCE	SS INPUT LINE
RS1:	
	LD	DE,TBUFF	; PT TO INPUT COMMAND LINE (IN TBUFF)
	CALL	DMASET		; SET TBUFF TO DMA ADDRESS
	CALL	GETDRV		; GET DEFAULT DRIVE NUMBER
	LD	(TDRIVE),A	; SET IT
	CALL	SCANER		; PARSE COMMAND NAME FROM COMMAND LINE
	CALL	NZ,ERROR	; ERROR IF COMMAND NAME CONTAINS A '?'
	LD	A,(TEMPDR)	; IS COMMAND OF FORM 'D:COMMAND'?
	OR	A		; NZ=YES
	JP	NZ,COM		; PROCESS AS COM FILE IMMEDIATELY
	CALL	CMDSER		; SCAN FOR CCP-RESIDENT COMMAND
	LD	HL,REQTBL	; EXECUTE COMMAND (CCP-RESIDENT OR COM)
	LD	E,A		; COMPUTE OFFSET INTO ADDRESS TABLE
	LD	D,0
	ADD	HL,DE
	ADD	HL,DE
	LD	A,(HL)		; GET ADDRESS IN HL
	INC	HL
	LD	H,(HL)		; ADDRESS HIGH
	LD	L,A		; ADDRESS LOW
	JP	(HL)		; EXECUTE CCP ROUTINE
	
;	
;	ERROR	MESSAGES
;	
PRNNF:	
	CALL	PRINT		; NO FILE MESSAGE
	DEFB	'No Files',0
	RET	
	
;	
;	MORE	CCP UTILITIES
;	
	
;	EXTRA	CT NUMBER FROM COMMAND LINE
NUMBER:	
	CALL	SCANER		; PARSE NUMBER AND PLACE IN FCBFN
	LD	A,(TEMPDR)	; TOKEN BEGIN WITH DRIVE SPEC (D:)?
	OR	A		; ERROR IF SO
	JP	NZ,ERROR
	LD	HL,FCBFN	; PT TO TOKEN FOR CONVERSION
	LD	BC,11		; B=ACCUMULATED VALUE, C=CHAR COUNT
NUM1:	
	LD	A,(HL)		; GET CHAR
	CP	' '		; DONE IF <SP>
	DEFB	28H
	DEFB	NUM2-$-1 AND 0FFH
	INC	HL		; PT TO NEXT CHAR
	SUB	'0'		; CONVERT TO BINARY (ASCII 0-9 TO BINARY)
	CP	10		; ERROR IF >= 10
	JP	NC,ERROR
	LD	D,A		; DIGIT IN D
	LD	A,B		; GET ACCUMULATED VALUE
	AND	0E0H		; CHECK FOR RANGE ERROR (>255)
	JP	NZ,ERROR
	LD	A,B		; NEW VALUE = OLD VALUE * 10
	RLCA	
	RLCA	
	RLCA	
	ADD	A,B		; CHECK FOR RANGE ERROR
	JP	C,ERROR
	ADD	A,B		; CHECK FOR RANGE ERROR
	JP	C,ERROR
	ADD	A,D		; NEW VALUE = OLD VALUE * 10 + DIGIT
	JP	C,ERROR	; CHECK FOR RANGE ERROR
	LD	B,A		; SET NEW VALUE
	DEC	C		; COUNT DOWN
	DEFB	20H
	DEFB	NUM1-$-1 AND 0FFH
	RET	
	
;	REST	OF TOKEN BUFFER MUST BE <SP>
NUM2:	
	LD	A,(HL)		; CHECK FOR <SP>
	CP	' '
	JP	NZ,ERROR
	INC	HL		; PT TO NEXT
	DEC	C		; COUNT DOWN CHARS
	DEFB	20H
	DEFB	NUM2-$-1 AND 0FFH
	LD	A,B		; GET ACCUMULATED VALUE
	RET	
	
;	MOVE	3 BYTES FROM HL TO DE
MOVHD3:	
	LD	B,3		; MOVE 3 CHARS
MOVEHD:	
	LD	A,(HL)		; GET IT
	LD	(DE),A		; PUT IT
	INC	HL		; PT TO NEXT
	INC	DE
	DEFB	10H
	DEFB	MOVEHD-$-1 AND 0FFH
	RET	
	
;	PT	TO DIRECTORY ENTRY IN TBUFF WHOSE OFFSET IS SPECIFIED BY A AND C
DIRPTR:	
	LD	HL,TBUFF	; PT TO TEMP BUFFER
	ADD	A,C		; PT TO 1ST BYTE OF DIR ENTRY
	CALL	ADDAH		; PT TO DESIRED BYTE IN DIR ENTRY
	LD	A,(HL)		; GET DESIRED BYTE
	RET	
	
;	CHECK	FOR SPECIFIED DRIVE AND LOG IT IN IF NOT DEFAULT
SLOGIN:
	XOR	A		; SET FCBDN FOR DEFAULT DRIVE
	LD	(FCBDN),A
	CALL	COMLOG		; CHECK DRIVE
	RET	Z
	JP	LOGIN		; DO LOGIN OTHERWISE
	
;	CHECK	FOR SPECIFIED DRIVE AND LOG IN DEFAULT DRIVE IF SPECIFIED<>DEFAULT
DLOGIN:	
	CALL	COMLOG		; CHECK DRIVE
	RET	Z		; ABORT IF SAME
	LD	A,(TDRIVE)	; LOG IN DEFAULT DRIVE
	JP	LOGIN
	
;	ROUTI	NE COMMON TO BOTH LOGIN ROUTINES; ON EXIT, Z SET MEANS ABORT
COMLOG:
	LD	A,(TEMPDR)	; DRIVE SPECIFIED?
	OR	A		; 0=NO
	RET	Z
	DEC	A		; COMPARE IT AGAINST DEFAULT
	LD	HL,TDRIVE
	CP	(HL)
	RET			; ABORT IF SAME
	
;	
;	CCP	DIRECTORY DISPLAY FUNCTION (DIR)
;	
DIR:	
	LD	A,80H		; SET SYSTEM BIT EXAMINATION
	PUSH	AF
	CALL	SCANER		; EXTRACT POSSIBLE D:FILENAME.TYP TOKEN
	CALL	SLOGIN		; LOG IN DRIVE IF NECESSARY
	LD	HL,FCBFN	; MAKE FCB WILD (ALL '?') IF NO FILENAME.TYP
	LD	A,(HL)		; GET FIRST CHAR OF FILENAME.TYP
	CP	' '		; IF <SP>, ALL WILD
	DEFB	28H
	DEFB	DIR0-$-1 AND 0FFH
	CP	'@'		; SYSTEM FILES?
	DEFB	20H
	DEFB	DIR2-$-1 AND 0FFH
	INC	HL		; JUST '@'?  <SP> MUST FOLLOW
	LD	A,(HL)
	DEC	HL		; BACK UP
	CP	' '		; JUST '@' IF <SP> FOLLOWS
	DEFB	20H
	DEFB	DIR2-$-1 AND 0FFH
	POP	AF		; GET FLAG
	XOR	A		; SET NO SYSTEM BIT EXAMINATION
	PUSH	AF
DIR0:	
	LD	B,11		; NUMBER OF CHARS IN FN & FT
DIR1:	
	LD	(HL),'?'	; STORE '?'
	INC	HL
	DEFB	10H
	DEFB	DIR1-$-1 AND 0FFH
DIR2:	
	POP	AF		; GET FLAG
	CALL	DIRPR		; PRINT DIRECTORY
	JP	RSTCCP		; RESTART CCP
	
;	DIREC	TORY PRINT ROUTINE; ON ENTRY, MSB OF A IS 1 (80H) IF SYSTEM FILES EXCL
DIRPR:	
	LD	D,A		; STORE SYSTEM FLAG IN D
	LD	E,0		; SET COLUMN COUNTER TO ZERO
	PUSH	DE		; SAVE COLUMN COUNTER (E) AND SYSTEM FLAG (D)
	CALL	SEARF		; SEARCH FOR SPECIFIED FILE (FIRST OCCURRANCE)
	CALL	Z,PRNNF	; PRINT NO FILE MSG; REG A NOT CHANGED
	
;	ENTRY	SELECTION LOOP	; ON ENTRY, A=OFFSET FROM SEARF OR SEARN
DIR3:	
	DEFB	28H
	DEFB	DIR11-$-1 AND 0FFH; DONE IF ZERO FLAG SET
	DEC	A		; ADJUST TO RETURNED VALUE
	RRCA			; CONVERT NUMBER TO OFFSET INTO TBUFF
	RRCA	
	RRCA	
	AND	60H
	LD	C,A		; OFFSET INTO TBUFF IN C (C=OFFSET TO ENTRY)
	LD	A,10		; ADD 10 TO PT TO SYSTEM FILE ATTRIBUTE BIT
	CALL	DIRPTR
	POP	DE		; GET SYSTEM BIT MASK FROM D
	PUSH	DE
	AND	D		; MASK FOR SYSTEM BIT
	DEFB	20H
	DEFB	DIR10-$-1 AND 0FFH; SKIP ENTRY IF BIT IS SET
	POP	DE		; GET ENTRY COUNT (=<CR> COUNTER)
	LD	A,E		; ADD 1 TO IT
	INC	E
	PUSH	DE		; SAVE IT
	AND	03H		; OUTPUT <CRLF> IF 4 ENTRIES PRINTED IN LINE
	PUSH	AF
	DEFB	20H
	DEFB	DIR4-$-1 AND 0FFH
	CALL	CRLF		; NEW LINE
	DEFB	18H
	DEFB	DIR5-$-1 AND 0FFH
DIR4:	
	CALL	SPACER		; PRINT <SP>:<SP> BETWEEN ENTRIES
	LD	A,':'
	CALL	CONOUT
	CALL	SPACER
DIR5:	
	LD	B,01H		; PT TO 1ST BYTE OF FILE NAME
DIR6:	
	LD	A,B		; A=OFFSET
	CALL	DIRPTR		; HL NOW PTS TO 1ST BYTE OF FILE NAME
	AND	7FH		; MASK OUT MSB
	CP	' '		; NO FILE NAME?
	DEFB	20H
	DEFB	DIR8-$-1 AND 0FFH; PRINT FILE NAME IF PRESENT
	POP	AF
	PUSH	AF
	CP	03H
	DEFB	20H
	DEFB	DIR7-$-1 AND 0FFH
	LD	A,09H		; PT TO 1ST BYTE OF FILE TYPE
	CALL	DIRPTR		; HL NOW PTS TO 1ST BYTE OF FILE TYPE
	AND	7FH		; MASK OUT MSB
	CP	' '		; NO FILE TYPE?
	DEFB	28H
	DEFB	DIR9-$-1 AND 0FFH; CONTINUE IF SO
DIR7:	
	LD	A,' '		; OUTPUT <SP>
DIR8:	
	CALL	CONOUT		; PRINT CHAR
	INC	B		; INCR CHAR COUNT
	LD	A,B
	CP	12		; END OF FILENAME.TYP?
	DEFB	30H
	DEFB	DIR9-$-1 AND 0FFH; CONTINUE IF SO
	CP	09H		; END IF FILENAME ONLY?
	DEFB	20H
	DEFB	DIR6-$-1 AND 0FFH; PRINT TYP IF SO
	LD	A,'.'		; PRINT DOT BETWEEN FILE NAME AND TYPE
	CALL	CONOUT
	DEFB	18H
	DEFB	DIR6-$-1 AND 0FFH
DIR9:	
	POP	AF
DIR10:	
	CALL	BREAK		; CHECK FOR ABORT
	DEFB	20H
	DEFB	DIR11-$-1 AND 0FFH
	CALL	SEARN		; SEARCH FOR NEXT FILE
	DEFB	18H
	DEFB	DIR3-$-1 AND 0FFH; CONTINUE
DIR11:	
	POP	DE		; RESTORE STACK
	RET	
	
;	
;	CCP	FILE ERASE FUNCTION (ERA)
;	
ERA:	
	CALL	SCANER		; PARSE FILE SPECIFICATION
	CP	0BH		; ALL WILD (ALL FILES = 11 '?')?
	DEFB	20H
	DEFB	ERA1-$-1 AND 0FFH; IF NOT, THEN DO ERASES
	CALL	PRINT
	DEFB	'All (Y/N)?',0
	CALL	REDBUF		; GET REPLY
	LD	HL,CBUFF	; CHECK FOR <CR>
	DEC	(HL)
	JP	NZ,RESTRT	; RESTART CCP IF JUST <CR>
	INC	HL		; PT TO RESPONSE BYTE
	LD	A,(HL)		; GET IT
	CP	'Y'		; YES?
	JP	NZ,RESTRT	; RESTART CCP IF NOT
	INC	HL		; PT TO CHAR AFTER 'Y'
	LD	(CIBPTR),HL	; SET PTR TO IT
ERA1:	
	CALL	SLOGIN		; LOG IN SELECTED DISK IF ANY
	LD	A,80H		; SKIP SYSTEM FILES (EXAMINE SYSTEM BIT)
	CALL	DIRPR		; PRINT DIRECTORY OF ERASED FILES
	LD	DE,FCBDN	; DELETE FILE SPECIFIED
	CALL	DELETE
	JP	RSTCCP		; REENTER CCP
	
;	
;	CCP	LIST FUNCTION (LIST)
;	
LIST:	
	LD	A,0FFH		; TURN ON PRINTER FLAG
	DEFB	18H
	DEFB	TYPE0-$-1 AND 0FFH
	
;	
;	CCP	TYPE FUNCTION (TYPE)
;	
TYPE:	
	XOR	A		; TURN OFF PRINTER FLAG
	
;	ENTRY	POINT FOR CCP LIST FUNCTION (LIST)
TYPE0:	
	LD	(PRFLG),A	; SET FLAG
	CALL	SCANER		; EXTRACT FILENAME.TYP TOKEN
	JP	NZ,ERROR	; ERROR IF ANY QUESTION MARKS
	CALL	SLOGIN		; LOG IN SELECTED DISK IF ANY
	CALL	OPENF		; OPEN SELECTED FILE
	JP	Z,TYPE4	; ABORT IF ERROR
	CALL	CRLF		; NEW LINE
	CALL	PAGSET		; SET LINE COUNT
	LD	HL,CHRCNT	; SET CHAR POSITION/COUNT
	LD	(HL),0FFH	; EMPTY LINE
	LD	B,0		; SET TAB CHAR COUNTER
TYPE1:	
	LD	HL,CHRCNT	; PT TO CHAR POSITION/COUNT
	LD	A,(HL)		; END OF BUFFER?
	CP	80H
	DEFB	38H
	DEFB	TYPE2-$-1 AND 0FFH
	PUSH	HL		; READ NEXT BLOCK
	CALL	READF
	POP	HL
	DEFB	20H
	DEFB	TYPE3-$-1 AND 0FFH; ERROR?
	XOR	A		; RESET COUNT
	LD	(HL),A
TYPE2:	
	INC	(HL)		; INCREMENT CHAR COUNT
	LD	HL,TBUFF	; PT TO BUFFER
	CALL	ADDAH		; COMPUTE ADDRESS OF NEXT CHAR FROM OFFSET
	LD	A,(HL)		; GET NEXT CHAR
	AND	7FH		; MASK OUT MSB
	CP	1AH		; END OF FILE (^Z)?
	JP	Z,RSTCCP	; RESTART CCP IF SO
	PUSH	AF		; SAVE CHAR
	LD	A,(PRFLG)	; TYPE OR LIST?
	OR	A		; 0=TYPE
	DEFB	28H
	DEFB	TYPE2T-$-1 AND 0FFH
	
;	OUTPU	T CHAR TO LST: DEVICE WITH TABULATION
	POP	AF		; GET CHAR
	CP	CR		; RESET TAB COUNT?
	DEFB	28H
	DEFB	TABRST-$-1 AND 0FFH
	CP	LF		; RESET TAB COUNT?
	DEFB	28H
	DEFB	TABRST-$-1 AND 0FFH
	CP	TAB		; TAB?
	DEFB	28H
	DEFB	LTAB-$-1 AND 0FFH
	CALL	LSTOUT		; LIST CHAR
	INC	B		; INCREMENT CHAR COUNT
	DEFB	18H
	DEFB	TYPE2L-$-1 AND 0FFH
TABRST:	
	CALL	LSTOUT		; OUTPUT <CR>
	LD	B,0		; RESET TAB COUNTER
	DEFB	18H
	DEFB	TYPE2L-$-1 AND 0FFH
LTAB:	
	LD	A,' '		; <SP>
	CALL	LSTOUT
	INC	B		; INCR POS COUNT
	LD	A,B
	AND	7
	DEFB	20H
	DEFB	LTAB-$-1 AND 0FFH
	DEFB	18H
	DEFB	TYPE2L-$-1 AND 0FFH
	
;	OUTPU	T CHAR TO CON: WITH TABULATION
TYPE2T:	
	POP	AF		; GET CHAR
	PUSH	AF		; SAVE CHAR
	CALL	CONOUT		; TYPE CHAR
	POP	AF
	CP	LF		; PAGE ON <LF>
	CALL	Z,PAGER	; COUNT LINES AND PAGE
	
;	CONTI	NUE PROCESSING
TYPE2L:	
	CALL	BREAK		; CHECK FOR ABORT
	DEFB	28H
	DEFB	TYPE1-$-1 AND 0FFH; CONTINUE IF NO CHAR
	CP	'C'-'@'		; ^C?
	JP	Z,RSTCCP	; RESTART IF SO
	DEFB	18H
	DEFB	TYPE1-$-1 AND 0FFH
TYPE3:	
	DEC	A		; NO ERROR?
	JP	Z,RSTCCP	; RESTART CCP
	CALL	PRINT		; PRINT READ ERROR MSG
	DEFB	'Read Error',0
TYPE4:	
	CALL	DLOGIN		; LOG IN DEFAULT DRIVE
	JP	ERROR
	
;	
;	PAGIN	G ROUTINES
;	PAGER	COUNTS DOWN LINES AND PAUSES FOR INPUT (DIRECT) IF COUNT EXPIRES
;	PAGSE	T SETS LINES/PAGE COUNT
;	
PAGER:	
	LD	A,(PAGCNT)	; COUNT DOWN
	DEC	A
	LD	(PAGCNT),A
	RET	NZ
	PUSH	HL		; SAVE HL
PAGER1:	
	LD	C,6		; DIRECT CONSOLE I/O
	LD	E,0FFH		; INPUT
	CALL	BDOSB
	OR	A		; CHAR READY?
	DEFB	28H
	DEFB	PAGER1-$-1 AND 0FFH; WAIT FOR CHAR
	CP	'C'-'@'		; ^C
	JP	Z,RSTCCP	; RESTART CCP
	POP	HL		; RESTORE HL
PAGSET:	
	LD	A,NLINES-2	; GET LINE COUNT
	LD	(PAGCNT),A
	RET	
	
;	
;	CCP	SAVE FUNCTION (SAVE)
;	
SAVE:	
	CALL	NUMBER		; EXTRACT NUMBER FROM COMMAND LINE
	PUSH	AF		; SAVE IT
	CALL	SCANER		; EXTRACT FILENAME.TYPE
	JP	NZ,ERROR	; MUST BE NO '?' IN IT
	CALL	SLOGIN		; LOG IN SELECTED DISK
	LD	DE,FCBDN	; DELETE FILE IN CASE IT ALREADY EXISTS
	PUSH	DE
	CALL	DELETE
	POP	DE
	CALL	CREATE		; MAKE NEW FILE
	DEFB	28H
	DEFB	SAVE3-$-1 AND 0FFH; ERROR?
	XOR	A		; SET RECORD COUNT FIELD OF NEW FILE'S FCB
	LD	(FCBCR),A
	POP	AF		; GET PAGE COUNT
	LD	L,A		; HL=PAGE COUNT
	LD	H,0
	ADD	HL,HL		; DOUBLE IT FOR HL=SECTOR (128 BYTES) COUNT
	LD	DE,TPA		; PT TO START OF SAVE AREA (TPA)
SAVE1:	
	LD	A,H		; DONE WITH SAVE?
	OR	L		; HL=0 IF SO
	DEFB	28H
	DEFB	SAVE2-$-1 AND 0FFH
	DEC	HL		; COUNT DOWN ON SECTORS
	PUSH	HL		; SAVE PTR TO BLOCK TO SAVE
	LD	HL,128		; 128 BYTES PER SECTOR
	ADD	HL,DE		; PT TO NEXT SECTOR
	PUSH	HL		; SAVE ON STACK
	CALL	DMASET		; SET DMA ADDRESS FOR WRITE (ADDRESS IN DE)
	LD	DE,FCBDN	; WRITE SECTOR
	CALL	WRITE
	POP	DE		; GET PTR TO NEXT SECTOR IN DE
	POP	HL		; GET SECTOR COUNT
	DEFB	20H
	DEFB	SAVE3-$-1 AND 0FFH; WRITE ERROR?
	DEFB	18H
	DEFB	SAVE1-$-1 AND 0FFH; CONTINUE
SAVE2:	
	LD	DE,FCBDN	; CLOSE SAVED FILE
	CALL	CLOSE
	INC	A		; ERROR?
	DEFB	20H
	DEFB	SAVE4-$-1 AND 0FFH
SAVE3:	
	CALL	PRINT
	DEFB	'No Space',0
SAVE4:	
	CALL	DEFDMA		; SET DMA TO 0080
	JP	RSTCCP		; RESTART CCP
	
;	
;	CCP	RENAME FILE FUNCTION (REN)
;	
REN:	
	CALL	SCANER		; EXTRACT FILE NAME
	JP	NZ,ERROR	; ERROR IF ANY '?' IN IT
	LD	A,(TEMPDR)	; SAVE CURRENT DEFAULT DISK
	PUSH	AF
	CALL	SLOGIN		; LOG IN SELECTED DISK
	CALL	SEARF		; LOOK FOR SPECIFIED FILE
	DEFB	28H
	DEFB	REN0-$-1 AND 0FFH; CONTINUE IF NOT FOUND
	CALL	PRINT
	DEFB	'File Exists',0
	JP	RENRET
REN0:	
	LD	HL,FCBDN	; SAVE NEW FILE NAME
	LD	DE,FCBDM
	LD	B,16		; 16 BYTES
	CALL	MOVEHD
	LD	HL,(CIBPTR)	; GET PTR TO NEXT CHAR IN COMMAND LINE
	EX	DE,HL		; ... IN DE
	CALL	SBLANK		; SKIP TO NON-BLANK
	CP	'='		; '=' OR UNDERSCORE OK
	DEFB	28H
	DEFB	REN1-$-1 AND 0FFH
	CP	5FH
	DEFB	20H
	DEFB	REN4-$-1 AND 0FFH
REN1:	
	EX	DE,HL		; PT TO CHAR AFTER '=' OR UNDERSCORE IN HL
	INC	HL
	LD	(CIBPTR),HL	; SAVE PTR TO OLD FILE NAME
	CALL	SCANER		; EXTRACT FILENAME.TYP TOKEN
	DEFB	20H
	DEFB	REN4-$-1 AND 0FFH; ERROR IF ANY '?'
	POP	AF		; GET OLD DEFAULT DRIVE
	LD	B,A		; SAVE IT
	LD	HL,TEMPDR	; COMPARE IT AGAINST CURRENT DEFAULT DRIVE
	LD	A,(HL)		; MATCH?
	OR	A
	DEFB	28H
	DEFB	REN2-$-1 AND 0FFH
	CP	B		; CHECK FOR DRIVE ERROR
	LD	(HL),B
	DEFB	20H
	DEFB	REN4-$-1 AND 0FFH
REN2:	
	LD	(HL),B
	XOR	A
	LD	(FCBDN),A	; SET DEFAULT DRIVE
	LD	DE,FCBDN	; RENAME FILE
	LD	C,17H		; BDOS RENAME FCT
	CALL	BDOS
	INC	A		; ERROR? -- FILE NOT FOUND IF SO
	DEFB	20H
	DEFB	RENRET-$-1 AND 0FFH
REN3:	
	CALL	PRNNF		; PRINT NO FILE MSG
RENRET:	
	JP	RSTCCP		; RESTART CCP
REN4:	
	CALL	DLOGIN		; LOG IN DEFAULT DRIVE
	JP	ERROR
	
;	
;	CCP	SET USER NUMBER FUNCTION
;	
MAXUSR:	EQU	15		; MAXIMUM USER AREA ACCESSABLE
USER:	
	CALL	NUMBER		; EXTRACT USER NUMBER FROM COMMAND LINE
	CP	MAXUSR+1	; ERROR IF >= MAXUSR
	JP	NC,ERROR
	LD	E,A		; PLACE USER NUMBER IN E
	LD	A,(FCBFN)	; CHECK FOR PARSE ERROR
	CP	' '		; <SP>=ERROR
	JP	Z,ERROR
	CALL	SETUSR		; SET SPECIFIED USER
	JP	RCCPNL		; RESTART CCP (NO DEFAULT LOGIN)
	
;	
;	CLEAR	SCREEN ROUTINE FOR CRT TERMINAL
;	
CLS:	
	LD	A, 27		; Chars to clear the screen
	CALL	CONOUT
	LD	A, '['		
	CALL	CONOUT
	LD	A, '2'		
	CALL	CONOUT
	LD	A, 'J'		
	CALL	CONOUT
	LD	A, 27		; And cursor home
	CALL	CONOUT
	LD	A, '['		
	CALL	CONOUT
	LD	A, 'H'		
	CALL	CONOUT
	JP	RCCPNL		; RESTART CCP (NO DEFAULT LOGIN)
	
;	
;	NOT	CCP-RESIDENT COMMAND -- PROCESS AS TRANSCIENT
;	
COM:	
	CALL	GETUSR		; GET CURRENT USER NUMBER
	LD	(TMPUSR),A	; SAVE IT FOR LATER
	LD	(TSELUSR),A	; TEMP USER TO SELECT
	LD	A,(FCBFN)	; ANY COMMAND?
	CP	' '		; ' ' MEANS COMMAND WAS 'D:' TO SWITCH
	DEFB	20H
	DEFB	COM1-$-1 AND 0FFH; NOT <SP>, SO MUST BE TRANSCIENT OR ERROR
	LD	A,(TEMPDR)	; LOOK FOR DRIVE SPEC
	OR	A		; IF ZERO, JUST BLANK
	JP	Z,RCCPNL
	DEC	A		; ADJUST FOR LOG IN
	LD	(TDRIVE),A	; SET DEFAULT DRIVE
	CALL	SETU0D		; SET DRIVE WITH USER 0
	CALL	LOGIN		; LOG IN DRIVE
	JP	RCCPNL		; RESTART CCP
COM1:	
	LD	A,(FCBFT)	; CHECK FOR ERROR IN FCB
	CP	' '		; ERROR IF SO
	JP	NZ,ERROR
;	
;	COMA	IS A REENTRY POINT FOR A NON-STANDARD CP/M MODIFICATION
;	THIS	IS THE RETURN POINT FOR WHEN THE .COM FILE IS NOT FOUND THE
;	FIRST	TIME, DRIVE A: IS SELECTED FOR A SECOND STTEMPT
;	
COMA:	
	CALL	SLOGIN		; LOG IN SPECIFIED DRIVE IF ANY
	LD	HL,COMMSG	; PLACE 'COM' IN FCB
	LD	DE,FCBFT	; PT TO FILE TYPE
	CALL	MOVHD3		; MOVE 3 CHARS
	CALL	OPENF		; OPEN COMMAND.COM FILE
	DEFB	20H
	DEFB	COMA1-$-1 AND 0FFH; ERROR?
	
;	ERROR	ROUTINE TO SELECT USER 0 IF ALL ELSE FAILS
	LD	A,(TSELUSR)	; GET USER FLAG
	OR	A		; SET FLAGS
	DEFB	28H
	DEFB	COMA0-$-1 AND 0FFH; TRY DISK A: IF ALREADY USER 0
	XOR	A		; SELECT USER 0
	LD	E,A
	LD	(TSELUSR),A	; RESET TEMPORARY USER NUMBER
	CALL	SETUSR
	DEFB	18H
	DEFB	COMA-$-1 AND 0FFH; TRY AGAIN
	
;	ERROR	ROUTINE TO SELECT DRIVE A: IF DEFAULT WAS ORIGINALLY SELECTED
COMA0:	
	LD	HL,TEMPDR	; GET DRIVE FROM CURRENT COMMAND
	XOR	A			; A=0
	or	(HL)
	JP	NZ,COM8		; ERROR IF ALREADY DISK A:
	LD	(HL),1		; SELECT DRIVE a:
	DEFB	18H
	DEFB	COMA-$-1 AND 0FFH
	
;	FILE	FOUND -- PROCEED WITH LOAD
COMA1:	
	LD	HL,TPA		; SET START ADDRESS OF MEMORY LOAD
COM2:	
	PUSH	HL		; SAVE ADDRESS OF NEXT SECTOR
	EX	DE,HL		; ... IN DE
	CALL	DMASET		; SET DMA ADDRESS FOR LOAD
	LD	DE,FCBDN	; READ NEXT SECTOR
	CALL	READ
	DEFB	20H
	DEFB	COM3-$-1 AND 0FFH; READ ERROR OR EOF?
	POP	HL		; GET ADDRESS OF NEXT SECTOR
	LD	DE,128		; MOVE 128 BYTES PER SECTOR
	ADD	HL,DE		; PT TO NEXT SECTOR IN HL
	LD	DE,ENTRY-128	; ARE WE GOING TO WRITE OVER CCP?
	LD	A,L		; COMPARE ADDRESS OF NEXT SECTOR (HL)
	SUB	E		;   TO START OF CCP (DE)
	LD	A,H
	SBC	A,D
	DEFB	30H
	DEFB	PRNLE-$-1 AND 0FFH; ERROR IF SAME
	DEFB	18H
	DEFB	COM2-$-1 AND 0FFH; OTHERWISE CONTINUE
	
;	LOAD	ERROR
PRNLE:	
	CALL	PRINT
	DEFB	'Bad Load',0
	JP	RSTCCP
	
COM3:	
	POP	HL		; LOAD COMPLETE!
	DEC	A
	DEFB	20H
	DEFB	PRNLE-$-1 AND 0FFH
	CALL	RESETUSR	; RESET CURRENT USER NUMBER
	;   USER MUST BE SET BEFORE LOGIN IS DONE
	CALL	DLOGIN		; LOG IN DEFAULT DRIVE
	CALL	SCANER		; SEARCH COMMAND LINE FOR NEXT TOKEN
	LD	HL,TEMPDR	; SAVE PTR TO DRIVE SPEC
	PUSH	HL
	LD	A,(HL)		; SET DRIVE SPEC
	LD	(FCBDN),A
	LD	A,10H		; OFFSET FOR 2ND FILE SPEC
	CALL	SCAN1		; SCAN FOR IT AND LOAD IT INTO FCBDN+16
	POP	HL		; SET UP DRIVE SPECS
	LD	A,(HL)
	LD	(FCBDM),A
	XOR	A
	LD	(FCBCR),A
	LD	DE,TFCB		; COPY TO DEFAULT FCB
	LD	HL,FCBDN	; FROM FCBDN
	LD	B,33		; SET UP DEFAULT FCB
	CALL	MOVEHD
	LD	HL,CIBUFF
COM4:	
	LD	A,(HL)		; SKIP TO END OF 2ND FILE NAME
	OR	A		; END OF LINE?
	DEFB	28H
	DEFB	COM5-$-1 AND 0FFH
	CP	' '		; END OF TOKEN?
	DEFB	28H
	DEFB	COM5-$-1 AND 0FFH
	INC	HL
	DEFB	18H
	DEFB	COM4-$-1 AND 0FFH
	
;	LOAD	COMMAND LINE INTO TBUFF
COM5:	
	LD	B,0		; SET CHAR COUNT
	LD	DE,TBUFF+1	; PT TO CHAR POS
COM6:	
	LD	A,(HL)		; COPY COMMAND LINE TO TBUFF
	LD	(DE),A
	OR	A		; DONE IF ZERO
	DEFB	28H
	DEFB	COM7-$-1 AND 0FFH
	INC	B		; INCR CHAR COUNT
	INC	HL		; PT TO NEXT
	INC	DE
	DEFB	18H
	DEFB	COM6-$-1 AND 0FFH
	
;	RUN	LOADED TRANSCIENT PROGRAM
COM7:	
	LD	A,B		; SAVE CHAR COUNT
	LD	(TBUFF),A
	CALL	CRLF		; NEW LINE
	CALL	DEFDMA		; SET DMA TO 0080
	CALL	SETUD		; SET USER/DISK
	CALL	TPA		; RUN TRANSCIENT
	CALL	SETU0D		; SET USER 0/DISK
	CALL	LOGIN		; LOGIN DISK
	JP	RESTRT		; RESTART CCP
	
;	TRANS	CIENT LOAD ERROR
COM8:	
	CALL	RESETUSR	; RESET CURRENT USER NUMBER
	;   RESET MUST BE DONE BEFORE LOGIN
	CALL	DLOGIN		; LOG IN DEFAULT DISK
	JP	ERROR
	
;	RESET	SELECTED USER NUMBER IF CHANGED
RESETUSR:	
	LD	A,(TMPUSR)	; GET OLD USER NUMBER
	LD	E,A		; PLACE IN E
	JP	SETUSR		; RESET
	
;	FILE	TYPE FOR COMMAND
COMMSG:	
	DEFB	'COM'
	
;	ENTRY	POINT FOR RESTARTING CCP AND LOGGING IN DEFAULT DRIVE
RSTCCP:	
	CALL	DLOGIN		; LOG IN DEFAULT DRIVE
;	ENTRY	POINT FOR RESTARTING CCP WITHOUT LOGGING IN DEFAULT DRIVE
RCCPNL:	
	CALL	SCANER		; EXTRACT NEXT TOKEN FROM COMMAND LINE
	LD	A,(FCBFN)	; GET FIRST CHAR OF TOKEN
	SUB	' '		; ANY CHAR?
	LD	HL,TEMPDR
	OR	(HL)
	JP	NZ,ERROR
	JP	RESTRT
	
	
RNGSUB:	
	DEFB	0		;0=$$$.SUB NOT PRESENT, ELSE $$$.SUB PRESENT
	
;	
;	FILE	CONTROL BLOCK (FCB), ONE
;	
SUBFCB:	
	DEFB	0		;DISK NAME
	DEFB	'$$$'		;FILE NAME
	DEFB	'     '
	DEFB	'SUB'		;FILE TYPE
	DEFB	0		;EXTENT NUMBER
	DEFB	0		;S1
SUBFS2:	
	DEFS	1		;S2
SUBFRC:	
	DEFS	1		;RECORD COUNT
	DEFS	16		;DISK GROUP MAP
SUBFCR:	
	DEFS	1		;CURRENT RECORD NUMBER
	
;	
;	FILE	CONTROL BLOCK
;	
FCBDN:	
	DEFS	1		;DISK NAME
FCBFN:	
	DEFS	8		;FILE NAME
FCBFT:	
	DEFS	3		;FILE TYPE
	DEFS	1		;EXTENT NUMBER
	DEFS	2		;S1 AND S2
	DEFS	1		;RECORD COUNT
FCBDM:	
	DEFS	16		;DISK GROUP MAP
FCBCR:	
	DEFS	1		;CURRENT RECORD NUMBER
	
;	OTHER	BUFFERS
PRFLG:	
	DEFB	0		;PRINTER FLAG (0=NO, 0FFH=YES)
PAGCNT:	
	DEFB	NLINES-2	;LINES LEFT ON PAGE
IORESL:	
	DEFB	0		;I/O RESULTS
	DEFB	'TDRIVE:'
TDRIVE:	
	DEFB	0		;TEMP DRIVE NUMBER
TEMPDR:	
	DEFB	0
CHRCNT:	
	DEFB	0		;CHAR COUNT FOR TYPE
TMPUSR:	
	DEFB	0		;TEMPORARY USER NUMBER FOR COM
TSELUSR:	
	DEFB	0		;TEMPORARY SELECTED USER NUMBER
	
	END	
OK:	
	DEFB	28H
