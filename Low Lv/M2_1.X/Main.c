/*
 * File:   Main.c
 * Author: thans
 *
 * Created on September 14, 2020, 10:15 PM
 */

#include "xc.h"
#include <stdio.h>
#include "configuration.h"

#define Motor1_A 15
#define Motor1_PWM 14
#define Motor1_B 13

#define Motor2_A 0
#define Motor2_PWM 1
#define Motor2_B 2

#define PWM_period 5000 //set period to 15,625 tick per cycle

//interrupt variable
volatile unsigned long millis = 0;
volatile char HomeX = 0;
volatile char HomeY = 0;
void initPLL() {
    PLLFBD = 150; // M  = 152
    CLKDIVbits.PLLPRE = 5; // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1); // Wait for PLL to lock
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {//timer of system
    millis++; //plus every 1 ms
    _T1IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void){
    LATA ^= 0x0001;
    IFS0bits.INT0IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void){
    LATB &= ((0x0001 << Motor1_A)^0xFFFF);
    LATB &= ((0x0001 << Motor1_B)^0xFFFF);
    HomeX++;
    IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void){
    LATB &= ((0x0001 << Motor2_A)^0xFFFF);
    LATB &= ((0x0001 << Motor2_B)^0xFFFF);
    HomeY++;
    IFS1bits.INT2IF = 0;
}

void motor_driveX(char direction,unsigned char speed) {
    unsigned long pwm = speed*((unsigned long)PWM_period);
    pwm/=100;
    OC1RS = pwm;
    printf("%d \n",OC1RS);
    if (direction == 0) {
        LATB &= ((0x0001 << Motor1_A)^0xFFFF);
        LATB &= ((0x0001 << Motor1_B)^0xFFFF);
    } else if (direction == -1) {
        LATB |= 0x0001 << Motor1_A;
        LATB &= ((0x0001 << Motor1_B)^0xFFFF);
    } else if (direction == 1) {
        LATB &= ((0x0001 << Motor1_A)^0xFFFF);
        LATB |= 0x0001 << Motor1_B;
    }
}

void motor_driveY(char direction,unsigned char speed) {
    unsigned long pwm = speed*((unsigned long)PWM_period);
    pwm/=100;
    OC2RS = pwm;
    if (direction == 0) {
        LATB &= ((0x0001 << Motor2_A)^0xFFFF);
        LATB &= ((0x0001 << Motor2_B)^0xFFFF);
    } else if (direction == -1) {
        LATB |= 0x0001 << Motor2_A;
        LATB &= ((0x0001 << Motor2_B)^0xFFFF);
    } else if (direction == 1) {
        LATB &= ((0x0001 << Motor2_A)^0xFFFF);
        LATB |= 0x0001 << Motor2_B;
    }
}

int main(void) {
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 0;
    RPINR0 = 0x0C00; //interrupts1
    RPINR1 = 0x0003; //interrupts2
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 1;

    IEC0 |= 0x0001; //enable interrupts0
    IEC1 |= 0x2010; //enable interrupts1,2
    INTCON2 &= 0xFFF8; //Edge detection
    IPC0bits.INT0IP = 1; //priority
    IPC5bits.INT1IP = 1; //priority
    IPC7bits.INT2IP = 1; //priority

    initPLL();
    T1CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    T2CONbits.TCKPS = 0b11; //set timer prescaler to 1:64
    PR1 = 5000; //set period to 1 ms
    PR2 = PWM_period; //set period to 15,625 tick per cycle 

    OC1RS = 8000;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC2RS = 8000;
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0; //OC2 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP14R = 0b10010; //remap RP11 connect to OC1
    _RP1R = 0b10011; //remap RP11 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    
    AD1PCFGL = 0xFFFF; //analog or digital setup
    TRISB = 0x1FF8; //input or output setup
    TRISA = 0xFFFE;

    _T1IE = 1; //enable interrupt for timer1
    _T1IP = 3; //priority interrupt for timer1

    __builtin_enable_interrupts();

    printf("start main loop\n");
    T1CONbits.TON = 1; //enable timer1
    T2CONbits.TON = 1; //enable timer2

    unsigned int timer_blink = 0;
    while (1) {
        if (HomeX > 0) {
            motor_driveX(-1,100);
            printf("%x %x\n", _RB15, _RB13);
            HomeX = 0;
        }
        if (HomeY > 0) {
            motor_driveX(1,50);
            printf("%x %x\n", _RB15, _RB13);
            HomeY = 0;
        }
        if (millis - timer_blink > 1000) {
            timer_blink = millis;
            TRISA ^= 0x0002;
        }
//        printf("%x %x\n", _RB15, _RB13);
//        printf("TMR2 = %u, RB14 = %d, RB1 = %d, OC1RS = %d, OC2RS = %d\n", TMR2, _RB14 , _RB1 , OC1RS ,OC2RS);
    }
    return 0;
}