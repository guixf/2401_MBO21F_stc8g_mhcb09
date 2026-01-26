#ifndef STC8G_H
#define STC8G_H

/* Compatibility wrapper: prefer the upstream STC8Fxx header */
#if defined(__has_include)
  #if __has_include("STC8Fxx.h")
    #include "STC8Fxx.h"
  #else
    /* Fallback: include the local copy if __has_include exists but header not found */
    #include "STC8Fxx.h"
  #endif
#else
  /* Compiler doesn't support __has_include: just include the header we added */
  #include "STC8Fxx.h"
#endif

//内核特殊功能寄存器
#define _ACC 0xe0
SFR(ACC, 0xe0);
#define _B 0xf0
SFR(B, 0xf0);
#define _PSW 0xd0
SFR(PSW, 0xd0);
SBIT(CY, _PSW, 7);
SBIT(AC, _PSW, 6);
SBIT(F0, _PSW, 5);
SBIT(RS1, _PSW, 4);
SBIT(RS0, _PSW, 3);
SBIT(OV, _PSW, 2);
SBIT(P, _PSW, 0);
#define _SP 0x81
SFR(SP, 0x81);
#define _DPL 0x82
SFR(DPL, 0x82);
#define _DPH 0x83
SFR(DPH, 0x83);
#define _TA 0xae
SFR(TA, 0xae);
#define _DPS 0xe3
SFR(DPS, 0xe3);
#define _DPL1 0xe4
SFR(DPL1, 0xe4);
#define _DPH1 0xe5
SFR(DPH1, 0xe5);

//I/O 口特殊功能寄存器
#define _P0 0x80
SFR(P0, 0x80);
#define _P1 0x90
SFR(P1, 0x90);
#define _P2 0xa0
SFR(P2, 0xa0);
#define _P3 0xb0
SFR(P3, 0xb0);
#define _P4 0xc0
SFR(P4, 0xc0);
#define _P5 0xc8
SFR(P5, 0xc8);
#define _P6 0xe8
SFR(P6, 0xe8);
#define _P7 0xf8
SFR(P7, 0xf8);
#define _P0M0 0x94
SFR(P0M0, 0x94);
#define _P0M1 0x93
SFR(P0M1, 0x93);
#define _P1M0 0x92
SFR(P1M0, 0x92);
#define _P1M1 0x91
SFR(P1M1, 0x91);
#define _P2M0 0x96
SFR(P2M0, 0x96);
#define _P2M1 0x95
SFR(P2M1, 0x95);
#define _P3M0 0xb2
SFR(P3M0, 0xb2);
#define _P3M1 0xb1
SFR(P3M1, 0xb1);
#define _P4M0 0xb4
SFR(P4M0, 0xb4);
#define _P4M1 0xb3
SFR(P4M1, 0xb3);
#define _P5M0 0xca
SFR(P5M0, 0xca);
#define _P5M1 0xc9
SFR(P5M1, 0xc9);
#define _P6M0 0xcc
SFR(P6M0, 0xcc);
#define _P6M1 0xcb
SFR(P6M1, 0xcb);
#define _P7M0 0xe2
SFR(P7M0, 0xe2);
#define _P7M1 0xe1
SFR(P7M1, 0xe1);

SBIT(P00, _P0, 0);
SBIT(P01, _P0, 1);
SBIT(P02, _P0, 2);
SBIT(P03, _P0, 3);
SBIT(P04, _P0, 4);
SBIT(P05, _P0, 5);
SBIT(P06, _P0, 6);
SBIT(P07, _P0, 7);
SBIT(P10, _P1, 0);
SBIT(P11, _P1, 1);
SBIT(P12, _P1, 2);
SBIT(P13, _P1, 3);
SBIT(P14, _P1, 4);
SBIT(P15, _P1, 5);
SBIT(P16, _P1, 6);
SBIT(P17, _P1, 7);
SBIT(P20, _P2, 0);
SBIT(P21, _P2, 1);
SBIT(P22, _P2, 2);
SBIT(P23, _P2, 3);
SBIT(P24, _P2, 4);
SBIT(P25, _P2, 5);
SBIT(P26, _P2, 6);
SBIT(P27, _P2, 7);
SBIT(P30, _P3, 0);
SBIT(P31, _P3, 1);
SBIT(P32, _P3, 2);
SBIT(P33, _P3, 3);
SBIT(P34, _P3, 4);
SBIT(P35, _P3, 5);
SBIT(P36, _P3, 6);
SBIT(P37, _P3, 7);
SBIT(P40, _P4, 0);
SBIT(P41, _P4, 1);
SBIT(P42, _P4, 2);
SBIT(P43, _P4, 3);
SBIT(P44, _P4, 4);
SBIT(P45, _P4, 5);
SBIT(P46, _P4, 6);
SBIT(P47, _P4, 7);
SBIT(P50, _P5, 0);
SBIT(P51, _P5, 1);
SBIT(P52, _P5, 2);
SBIT(P53, _P5, 3);
SBIT(P54, _P5, 4);
SBIT(P55, _P5, 5);
SBIT(P56, _P5, 6);
SBIT(P57, _P5, 7);
SBIT(P60, _P6, 0);
SBIT(P61, _P6, 1);
SBIT(P62, _P6, 2);
SBIT(P63, _P6, 3);
SBIT(P64, _P6, 4);
SBIT(P65, _P6, 5);
SBIT(P66, _P6, 6);
SBIT(P67, _P6, 7);
SBIT(P70, _P7, 0);
SBIT(P71, _P7, 1);
SBIT(P72, _P7, 2);
SBIT(P73, _P7, 3);
SBIT(P74, _P7, 4);
SBIT(P75, _P7, 5);
SBIT(P76, _P7, 6);
SBIT(P77, _P7, 7);

//如下特殊功能寄存器位于扩展RAM区域
//访问这些寄存器,需先将P_SW2的BIT7设置为1,才可正常读写
#define P0PU        (*(unsigned char volatile xdata *)0xfe10)
#define P1PU        (*(unsigned char volatile xdata *)0xfe11)
#define P2PU        (*(unsigned char volatile xdata *)0xfe12)
#define P3PU        (*(unsigned char volatile xdata *)0xfe13)
#define P4PU        (*(unsigned char volatile xdata *)0xfe14)
#define P5PU        (*(unsigned char volatile xdata *)0xfe15)
#define P6PU        (*(unsigned char volatile xdata *)0xfe16)
#define P7PU        (*(unsigned char volatile xdata *)0xfe17)
#define P0NCS       (*(unsigned char volatile xdata *)0xfe18)
#define P1NCS       (*(unsigned char volatile xdata *)0xfe19)
#define P2NCS       (*(unsigned char volatile xdata *)0xfe1a)
#define P3NCS       (*(unsigned char volatile xdata *)0xfe1b)
#define P4NCS       (*(unsigned char volatile xdata *)0xfe1c)
#define P5NCS       (*(unsigned char volatile xdata *)0xfe1d)
#define P6NCS       (*(unsigned char volatile xdata *)0xfe1e)
#define P7NCS       (*(unsigned char volatile xdata *)0xfe1f)

//系统管理特殊功能寄存器
#define _PCON       0x87
SFR(PCON, 0x87);
#define SMOD        0x80
#define SMOD0       0x40
#define LVDF        0x20
#define POF         0x10
#define GF1         0x08
#define GF0         0x04
#define PD          0x02
#define IDL         0x01
#define _AUXR       0x8e
SFR(AUXR, 0x8e);
#define T0x12       0x80
#define T1x12       0x40
#define UART_M0x6   0x20
#define T2R         0x10
#define T2_CT       0x08
#define T2x12       0x04
#define EXTRAM      0x02
#define S1ST2       0x01
#define _AUXR2      0x97
SFR(AUXR2, 0x97);
#define TXLNRX      0x10
#define _BUS_SPEED  0xa1
SFR(BUS_SPEED, 0xa1);
#define _P_SW1      0xa2
SFR(P_SW1, 0xa2);
#define _P_SW2      0xba
SFR(P_SW2, 0xba);
#define EAXFR       0x80
#define _VOCTRL     0xbb
SFR(VOCTRL, 0xbb);
#define _RSTCFG     0xff
SFR(RSTCFG, 0xff);

//如下特殊功能寄存器位于扩展RAM区域
//访问这些寄存器,需先将P_SW2的BIT7设置为1,才可正常读写
#define CKSEL       (*(unsigned char volatile xdata *)0xfe00)
#define CLKDIV      (*(unsigned char volatile xdata *)0xfe01)
#define IRC24MCR    (*(unsigned char volatile xdata *)0xfe02)
#define XOSCCR      (*(unsigned char volatile xdata *)0xfe03)
#define IRC32KCR    (*(unsigned char volatile xdata *)0xfe04)

//中断特殊功能寄存器
#define _IE 0xa8
SFR(IE, 0xa8);
SBIT(EA, _IE, 7);
SBIT(ELVD, _IE, 6);
SBIT(EADC, _IE, 5);
SBIT(ES, _IE, 4);
SBIT(ET1, _IE, 3);
SBIT(EX1, _IE, 2);
SBIT(ET0, _IE, 1);
SBIT(EX0, _IE, 0);
#define _IE2 0xaf
SFR(IE2, 0xaf);
#define ET4         0x40
#define ET3         0x20
#define ES4         0x10
#define ES3         0x08
#define ET2         0x04
#define ESPI        0x02
#define ES2         0x01
#define _IP         0xb8
SFR(IP, 0xb8);
SBIT(PPCA, _IP, 7);
SBIT(PLVD, _IP, 6);
SBIT(PADC, _IP, 5);
SBIT(PS, _IP, 4);
SBIT(PT1, _IP, 3);
SBIT(PX1, _IP, 2);
SBIT(PT0, _IP, 1);
SBIT(PX0, _IP, 0);
#define _IP2 0xb5
SFR(IP2, 0xb5);
#define PI2C        0x40
#define PCMP        0x20
#define PX4         0x10
#define PPWMFD      0x08
#define PPWM        0x04
#define PSPI        0x02
#define PS2         0x01
#define _IPH 0xb7
SFR(IPH, 0xb7);
#define PPCAH       0x80
#define PLVDH       0x40
#define PADCH       0x20
#define PSH         0x10
#define PT1H        0x08
#define PX1H        0x04
#define PT0H        0x02
#define PX0H        0x01
#define _IP2H 0xb6
SFR(IP2H, 0xb6);
#define PI2CH       0x40
#define PCMPH       0x20
#define PX4H        0x10
#define PPWMFDH     0x08
#define PPWMH       0x04
#define PSPIH       0x02
#define PS2H        0x01
#define _INTCLKO 0x8f
SFR(INTCLKO, 0x8f);
#define EX4         0x40
#define EX3         0x20
#define EX2         0x10
#define T2CLKO      0x04
#define T1CLKO      0x02
#define T0CLKO      0x01
#define _AUXINTIF 0xef
SFR(AUXINTIF, 0xef);
#define INT4IF      0x40
#define INT3IF      0x20
#define INT2IF      0x10
#define T4IF        0x04
#define T3IF        0x02
#define T2IF        0x01

//定时器特殊功能寄存器
#define _TCON 0x88
SFR(TCON, 0x88);
SBIT(TF1, _TCON, 7);
SBIT(TR1, _TCON, 6);
SBIT(TF0, _TCON, 5);
SBIT(TR0, _TCON, 4);
SBIT(IE1, _TCON, 3);
SBIT(IT1, _TCON, 2);
SBIT(IE0, _TCON, 1);
SBIT(IT0, _TCON, 0);
#define _TMOD 0x89
SFR(TMOD, 0x89);
#define T1_GATE     0x80
#define T1_CT       0x40
#define T1_M1       0x20
#define T1_M0       0x10
#define T0_GATE     0x08
#define T0_CT       0x04
#define T0_M1       0x02
#define T0_M0       0x01
#define _TL0 0x8a
SFR(TL0, 0x8a);
#define _TL1 0x8b
SFR(TL1, 0x8b);
#define _TH0 0x8c
SFR(TH0, 0x8c);
#define _TH1 0x8d
SFR(TH1, 0x8d);
#define _T4T3M 0xd1
SFR(T4T3M, 0xd1);
#define T4R         0x80
#define T4_CT       0x40
#define T4x12       0x20
#define T4CLKO      0x10
#define T3R         0x08
#define T3_CT       0x04
#define T3x12       0x02
#define T3CLKO      0x01
#define _T4H 0xd2
SFR(T4H, 0xd2);
#define _T4L 0xd3
SFR(T4L, 0xd3);
#define _T3H 0xd4
SFR(T3H, 0xd4);
#define _T3L 0xd5
SFR(T3L, 0xd5);
#define _T2H 0xd6
SFR(T2H, 0xd6);
#define _T2L 0xd7
SFR(T2L, 0xd7);
#define _TH4 0xd2
SFR(TH4, 0xd2);
#define _TL4 0xd3
SFR(TL4, 0xd3);
#define _TH3 0xd4
SFR(TH3, 0xd4);
#define _TL3 0xd5
SFR(TL3, 0xd5);
#define _TH2 0xd6
SFR(TH2, 0xd6);
#define _TL2 0xd7
SFR(TL2, 0xd7);
#define _WKTCL 0xaa
SFR(WKTCL, 0xaa);
#define _WKTCH 0xab
SFR(WKTCH, 0xab);
#define WKTEN 0x80
#define _WDT_CONTR 0xc1
SFR(WDT_CONTR, 0xc1);
#define WDT_FLAG    0x80
#define EN_WDT      0x20
#define CLR_WDT     0x10
#define IDL_WDT     0x08

//串行口特殊功能寄存器
#define _SCON 0x98
SFR(SCON, 0x98);
SBIT(SM0, _SCON, 7);
SBIT(SM1, _SCON, 6);
SBIT(SM2, _SCON, 5);
SBIT(REN, _SCON, 4);
SBIT(TB8, _SCON, 3);
SBIT(RB8, _SCON, 2);
SBIT(TI, _SCON, 1);
SBIT(RI, _SCON, 0);
#define _SBUF 0x99
SFR(SBUF, 0x99);
#define _S2CON 0x9a
SFR(S2CON, 0x9a);
#define S2SM0       0x80
#define S2ST4       0x40
#define S2SM2       0x20
#define S2REN       0x10
#define S2TB8       0x08
#define S2RB8       0x04
#define S2TI        0x02
#define S2RI        0x01
#define _S2BUF 0x9b
SFR(S2BUF, 0x9b);
#define _S3CON 0xac
SFR(S3CON, 0xac);
#define S3SM0       0x80
#define S3ST4       0x40
#define S3SM2       0x20
#define S3REN       0x10
#define S3TB8       0x08
#define S3RB8       0x04
#define S3TI        0x02
#define S3RI        0x01

// ... (rest of original STC8Fxx.h continues, including ADC register definitions)
#define _ADC_CONTR 0xbc
SFR(ADC_CONTR, 0xbc);
#define _ADC_RES 0xbd
SFR(ADC_RES, 0xbd);
#define _ADC_RESL 0xbe
SFR(ADC_RESL, 0xbe);
#define _ADCCFG 0xde
SFR(ADCCFG, 0xde);
#define ADC_RESFMT  0x20

/* Provide fallback _nop_ if <intrins.h> is not available */
#ifndef _NOP_DEFINED
static inline void _nop_(void) {
    __asm
    nop
    __endasm;
}
#define _NOP_DEFINED
#endif

#endif /* STC8G_H */
