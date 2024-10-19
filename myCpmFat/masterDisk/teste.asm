BDOS    equ     0005h
CONOUT  equ     02h
DIRIO   equ     06h
        org     0100h
; Output a '?' via direct I/O
        ld      c,DIRIO ; BDOS "Direct Console I/O""
        ld      e,'?'   ; The character to display
        call    BDOS    ; Call BDOS
; In a loop, keep reading via direct I/O and displaying via
; direct I/O until we read a space.
LOOP:   ld      c,DIRIO ; print a space
        ld      e,' '
        call    BDOS
        ld      c,DIRIO
        ld      e,0FDh  ; a blocking character read
        call    BDOS
        call    ASCII   ; display the ASCII result
        cp      ' '     ; did we get a space?
        jr      nz,LOOP ; nope, so try again
; yep, we got a space
; Output a '.' via direct I/O
        ld      c,DIRIO
        ld      e,'.'
        call    BDOS
        ret
ASCII:  push    af      ; save input value from clobbering
        srl     a       ; Move the top nibble into the bottom nibble
        srl     a
        srl     a
        srl     a
        call    DIGIT   ; Display the top nibble
        pop     af
        push    af
        and     0Fh
        call    DIGIT   ; Display the bottom nibble
        pop     af
        ret
; Display the byte in A as a pair of hex digits
DIGIT:  cp      0Ah     ; Is it 0-9 or A-F?
        jr      nc,LETTER
        add     '0'     ; Is a digit; Convert it to an ASCII value
        jr      DIGIT2
LETTER: add     'A'-10  ; Is a letter; Convert it to an ASCII value
DIGIT2: ld      c,CONOUT
        ld      e,a
        call    BDOS
        ret
