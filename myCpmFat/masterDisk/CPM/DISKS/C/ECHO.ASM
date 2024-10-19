	extrn	.begin,.chl,.swt
	extrn	csave,cret,.move
	PUBLIC main_
main_:	lxi d,.2
	call csave
	LXI H,1
	XCHG
	LXI H,2-.2
	DAD SP
	MOV M,E
	INX H
	MOV M,D
	JMP .4
.3:
	LXI H,2-.2
	DAD SP
	PUSH H
	MOV A,M
	INX H
	MOV H,M
	MOV L,A
	INX H
	XCHG
	POP H
	MOV M,E
	INX H
	MOV M,D
.4:
	LXI H,2-.2
	DAD SP
	MOV E,M
	INX H
	MOV D,M
	LXI H,8-.2
	DAD SP
	MOV A,M
	INX H
	MOV H,M
	MOV L,A
	CALL .lt
	JZ .5
	LXI H,2-.2
	DAD SP
	MOV E,M
	INX H
	MOV D,M
	XCHG
	INX H
	XCHG
	LXI H,8-.2
	DAD SP
	MOV A,M
	INX H
	MOV H,M
	MOV L,A
	CALL .eq
	JZ .6
	LXI H,.1+5
	JMP .7
.6:
	LXI H,.1+7
.7:
	PUSH H
	LXI H,4-.2
	DAD SP
	MOV E,M
	INX H
	MOV D,M
	XCHG
	DAD H
	XCHG
	LXI H,12-.2
	DAD SP
	MOV A,M
	INX H
	MOV H,M
	MOV L,A
	DAD D
	MOV E,M
	INX H
	MOV D,M
	PUSH D
	LXI H,.1+0
	PUSH H
	CALL printf_
	POP D
	POP D
	POP D
	JMP .3
.5:
	LXI H,0
	RET
.2 EQU -2
.1:
	DB 37,115,37,115,0,10,0,32,0
	extrn	printf_
	extrn	.eq
	extrn	.lt
	END
