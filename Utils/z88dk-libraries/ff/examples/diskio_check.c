/*----------------------------------------------------------------------/
/ Low level disk I/O module function checker                            /
/-----------------------------------------------------------------------/
/ WARNING: The data on the target drive will be lost!
*/

// zcc +yaz180 -subtype=app -clib=sdcc_iy -SO3 -v -m --list --max-allocs-per-node100000 diskio_check.c -o diskio_check -create-app

// zcc +rc2014 -subtype=hbios -clib=sdcc_iy -SO3 -v -m --list --max-allocs-per-node100000 diskio_check.c -o diskchk -create-app

// zcc +rc2014 -subtype=hbios -clib=sdcc_iy -SO3 -v -m --list -llib/hbios/diskio_hbios --max-allocs-per-node100000 diskio_check.c -o diskchk -create-app

// zcc +scz180 -subtype=hbios -clib=sdcc_iy -SO3 -v -m --list -llib/scz180/diskio_sd --max-allocs-per-node100000 diskio_check.c -o diskchk -create-app

// zcc +scz180 -subtype=hbios -clib=sdcc_iy -SO3 -v -m --list -llib/hbios/diskio_hbios --max-allocs-per-node100000 diskio_check.c -o diskchk -create-app

// zcc +hbios -clib=sdcc_iy -SO3 -v -m --list -llib/hbios/diskio_hbios --max-allocs-per-node100000 diskio_check.c -o diskchk -create-app

#include <stdio.h>
#include <string.h>

#if __YAZ180
#include <lib/yaz180/ff.h>      /* Declarations of FatFs API */
#include <arch/yaz180/diskio.h> /* Declarations of diskio & IDE functions */
#elif __RC2014
#include <lib/rc2014/ff.h>      /* Declarations of FatFs API */
#include <arch/rc2014/diskio.h> /* Declarations of diskio & IDE functions */
//#elif __RC2014
//#include <lib/hbios/ff.h>           /* Declarations of FatFs API */
//#include <lib/hbios/diskio_hbios.h> /* Declarations of diskio functions */
//#include <arch/hbios.h>             /* Declarations of HBIOS functions */
#elif __SCZ180
#include <lib/scz180/ff.h>          /* Declarations of FatFs API */
#include <lib/scz180/diskio_sd.h>   /* Declarations of diskio functions */
#include <arch/scz180.h>            /* Declarations of SD functions */
//#elif __SCZ180
//#include <lib/hbios/ff.h>           /* Declarations of FatFs API */
//#include <lib/hbios/diskio_hbios.h> /* Declarations of diskio functions */
//#include <arch/hbios.h>             /* Declarations of HBIOS functions */
#elif __HBIOS
#include <lib/hbios/ff.h>           /* Declarations of FatFs API */
#include <lib/hbios/diskio_hbios.h> /* Declarations of diskio functions */
#include <arch/hbios.h>             /* Declarations of HBIOS functions */
#else
#warning "no diskio.h functions available"
#endif

#include "ffconf.h"             /* Declaration of sector size */

DWORD buffer[1024];             /* 4096 byte working buffer */

static
DWORD pn (		/* Pseudo random number generator */
    DWORD pns	/* 0:Initialise, !0:Read */
)
{
    static DWORD lfsr;
    UINT n;


    if (pns) {
        lfsr = pns;
        for (n = 0; n < 32; n++) pn(0);
    }
    if (lfsr & 1) {
        lfsr >>= 1;
        lfsr ^= 0x80200003;
    } else {
        lfsr >>= 1;
    }
    return lfsr;
}


int test_diskio (
    BYTE pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
    UINT ncyc,      /* Number of test cycles */
    DWORD* buff,    /* Pointer to the working buffer */
    UINT sz_buff    /* Size of the working buffer in unit of byte */
)
{
    UINT n, cc, ns;
    DWORD sz_drv, lba, lba2,  pns = 1;
    WORD sz_eblk, sz_sect;
    BYTE *pbuff = (BYTE*)buff;
    DSTATUS ds;
    DRESULT dr;

    printf("\r\ntest_diskio(%u, %u, 0x%04X, 0x%04X)\n", pdrv, ncyc, (UINT)buff, sz_buff);

    if (sz_buff < FF_MAX_SS + 4) {
        printf("Insufficient work area to run program.\n");
        return 1;
    }

    for (cc = 1; cc <= ncyc; cc++) {
        printf("**** Test cycle %u of %u start ****\n", cc, ncyc);

        /* Initialization */
        printf(" disk_initalize(%u)", pdrv);
        ds = disk_initialize(pdrv);
        if (ds & STA_NOINIT) {
            printf(" - failed.\n");
            return 2;
        } else {
            printf(" - ok.\n");
        }

        /* Get drive serial */
        printf("**** Get drive serial number ****\n");
        printf(" disk_ioctl(%u, ATA_GET_SN, 0x%04X)", pdrv, pbuff);
        dr = disk_ioctl(pdrv, ATA_GET_SN, pbuff);

        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
        }

        printf("Serial number of the drive %u is %s.\n", pdrv, pbuff);        

        /* Get drive size */
        printf("**** Get drive size ****\n");
        printf(" disk_ioctl(%u, GET_SECTOR_COUNT, 0x%04X)", pdrv, (DWORD)&sz_drv);
        sz_drv = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);

        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
            return 3;
        }
 
        printf("Number of sectors on the drive %u is %lu.\n", pdrv, sz_drv);

        if (sz_drv < 128) {
            printf("Failed: Insufficient drive size to test.\n");
            return 4;
        }

        /* Get sector size */
        printf("**** Get sector size ****\n");
        printf(" disk_ioctl(%u, GET_SECTOR_SIZE, 0x%X)", pdrv, (WORD)&sz_sect);
        sz_sect = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
            return 5;
        }
        printf(" Size of sector is %u bytes.\n", sz_sect);

        /* Get erase block size */
        printf("**** Get block size ****\n");
        printf(" disk_ioctl(%u, GET_BLOCK_SIZE, 0x%X)", pdrv, (WORD)&sz_eblk);
        sz_eblk = 0;
        dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
        }
        if (dr == RES_OK || sz_eblk >= 2) {
            printf(" Size of the erase block is %u sectors.\n", sz_eblk);
        } else {
            printf(" Size of the erase block is unknown.\n");
        }

        /* Single sector write test */
        printf("**** Single sector write test ****\n");
        lba = 0;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n] = (BYTE)pn(0);
        printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
        dr = disk_write(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
            return 6;
        }
        printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
            return 7;
        }
        memset(pbuff, 0, sz_sect);
        printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
        dr = disk_read(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed %u.\n", dr);
            return 8;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE)pn(0); n++) ;
        if (n == sz_sect) {
            printf(" Data matched.\n");
        } else {
            printf("Failed: Read data differs from the data written.\n");
            return 10;
        }
        pns++;

        /* Multiple sector write test */
        printf("**** Multiple sector write test ****\n");
        lba = 1; ns = sz_buff / sz_sect;
        if (ns > 4) ns = 4;
        for (n = 0, pn(pns); n < (UINT)(sz_sect * ns); n++) pbuff[n] = (BYTE)pn(0);
        printf(" disk_write(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
        dr = disk_write(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 11;
        }
        printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 12;
        }
        memset(pbuff, 0, sz_sect * ns);
        printf(" disk_read(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
        dr = disk_read(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 13;
        }
        for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++) ;
        if (n == (UINT)(sz_sect * ns)) {
            printf(" Data matched.\n");
        } else {
            printf("Failed: Read data differs from the data written.\n");
            return 14;
        }
        pns++;

        /* Single sector write test (misaligned memory address) */
        printf("**** Single sector write test (misaligned address) ****\n");
        lba = 5;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n+3] = (BYTE)pn(0);
        printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+3), lba);
        dr = disk_write(pdrv, pbuff+3, lba, 1);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 15;
        }
        printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 16;
        }
        memset(pbuff+5, 0, sz_sect);
        printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+5), lba);
        dr = disk_read(pdrv, pbuff+5, lba, 1);
        if (dr == RES_OK) {
            printf(" - ok.\n");
        } else {
            printf(" - failed.\n");
            return 17;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (BYTE)pn(0); n++) ;
        if (n == sz_sect) {
            printf(" Data matched.\n");
        } else {
            printf("Failed: Read data differs from the data written.\n");
            return 18;
        }
        pns++;

        /* 4GB barrier test */
        printf("**** 4GB barrier test ****\n");
        if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
            lba = 6; lba2 = lba + 0x80000000 / (sz_sect / 2);
            for (n = 0, pn(pns); n < (UINT)(sz_sect * 2); n++) pbuff[n] = (BYTE)pn(0);
            printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
            dr = disk_write(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
                printf(" - ok.\n");
            } else {
                printf(" - failed.\n");
                return 19;
            }
            printf(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+sz_sect), lba2);
            dr = disk_write(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
                printf(" - ok.\n");
            } else {
                printf(" - failed.\n");
                return 20;
            }
            printf(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
            dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
            if (dr == RES_OK) {
            printf(" - ok.\n");
            } else {
                printf(" - failed.\n");
                return 21;
            }
            memset(pbuff, 0, sz_sect * 2);
            printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
            dr = disk_read(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
                printf(" - ok.\n");
            } else {
                printf(" - failed.\n");
                return 22;
            }
            printf(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+sz_sect), lba2);
            dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
                printf(" - ok.\n");
            } else {
                printf(" - failed.\n");
                return 23;
            }
            for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++) ;
            if (n == (UINT)(sz_sect * 2)) {
                printf(" Data matched.\n");
            } else {
                printf("Failed: Read data differs from the data written.\n");
                return 24;
            }
        } else {
            printf(" Test skipped.\n");
        }
        pns++;

        printf("**** Test cycle %u of %u completed ****\n\n", cc, ncyc);
    }

    return 0;
}

int main (void)
{
    int rc;

    /* Check function/compatibility of the physical drive #0 */
    rc = test_diskio(0, 3, buffer, sizeof( buffer) );
    if (rc) {
        printf("Sorry the function/compatibility test failed. (rc=%d)\nFatFs will not work on this disk driver.\n", rc);
    } else {
        printf("Congratulations! The disk driver works well.\n");
    }

    return rc;
}

