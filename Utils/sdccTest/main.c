/*
    SDCC - compilar para bare metal Z80
    sdcc -mz80 --no-std-crt0 --vc --code-loc 16384 main.c
*/

int main()
{
    int a;

    for ( a = 0; a < 10; a++ )
    {
    }

    return 0;
}

