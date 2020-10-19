/*
 * File:   outputcompare1.c
 * Author: G_Peav
 *
 * Created on September 4, 2019, 4:29 PM
 */

#include "xc.h"
#include <stdio.h>
#include "configuration.h"
#include "UART_rev3.h"
#include <math.h>

volatile float timer = 0;
volatile float output = 0;
volatile int count = 0;
volatile float cui_pose = 0;
volatile int Em_Stop = 0, set_home_x_f = 0, set_home_y_f = 0;
volatile int32_t pre_x_error = 0, pre_y_error = 0;
volatile unsigned int x_pos = 0 , y_pos = 0;

#define PI 3.14159265
#define INPUT_VOLTAGE 12

#define PWM_period 10000
#define SERVO_period 10000
#define int_period 50000

#define k_p_x 10
#define k_d_x 10
#define k_p_y 0
#define k_d_y 0

void pid_xy(unsigned int x_pos ,unsigned int y_pos){
    int32_t error_x = (int32_t)(x_pos -  (unsigned int)POS1CNT);
    int32_t error_y = (int32_t)(y_pos -  (unsigned int)POS2CNT);
    int32_t d_term_x = error_x - pre_x_error;
    int32_t d_term_y = error_y - pre_y_error;
    int32_t x_pwm = (k_p_x*error_x + k_d_x*d_term_x)/10000 ;
    int32_t y_pwm = (k_p_y*error_y + k_d_y*d_term_y)/10000 ;
    OC1RS = abs(x_pwm);
    OC2RS = abs(y_pwm);
    if (abs(x_pwm) <= 100){
        _LATA0 = 1;
        _LATA1 = 1;
    }
    else if (x_pwm > 0){
        _LATA0 = 1;
        _LATA1 = 0;
    }
    else{
        _LATA0 = 0;
        _LATA1 = 1;
    }
    
    if (abs(y_pwm) <= 100){
        _LATA4 = 1;
        _LATB4 = 1;
    }
    else if (y_pwm > 0){
        _LATA4 = 1;
        _LATB4 = 0;
    }
    else{
        _LATA4 = 0;
        _LATB4 = 1;
    }
    pre_x_error = error_x;
    pre_y_error = error_y;
    
}

void motor_driveX(int speed) {
    if ((int) speed == 1) {
        _LATA0 = 1;
        _LATA1 = 0;
    } else if ((int) speed > 1) {
        _LATA0 = 1;
        _LATA1 = 0;
    } else {
        _LATA0 = 0;
        _LATA1 = 1;
    }
    speed = abs(speed);
    unsigned long pwm = speed * ((unsigned long) PWM_period);
    pwm /= 100;
    OC1RS = pwm;
}

void motor_driveY(int speed) {
    if ((int) speed == 1) {
        _LATA4 = 0;
        _LATB4 = 0;
    } else if ((int) speed > 1) {
        _LATA4 = 1;
        _LATB4 = 0;
    } else {
        _LATA4 = 0;
        _LATB4 = 1;
    }
    speed = abs(speed);
    unsigned long pwm = speed * ((unsigned long) PWM_period);
    pwm /= 100;
    OC2RS = pwm;
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
    Em_Stop = 1;
    IFS0bits.INT0IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    if (set_home_x_f) {
        set_home_x_f = 0;
    }
    IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    if (set_home_y_f) {
        set_home_y_f = 0;
    }
    IFS1bits.INT2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    //timer of system
//    printf("%u %u\n", POS1CNT, POS2CNT);
    //        printf("%u %u\n",_RA4,_RB4);
    pid_xy(x_pos,y_pos);
    _T1IF = 0; //clear interrupt flag
}

void set_home() {
    set_home_x_f = 1;
    int x_set = 1;
    int y_set = 1;
    printf("sethome\n");
    while (_RB2) {
        motor_driveX(20);
    }
    unsigned int i;
    for (i = 0; i < 60000; i++)Nop();
    motor_driveX(-7);
    while (set_home_x_f) {
        if (!set_home_x_f && x_set) {
            motor_driveX(0);
            x_set = 0;
            //            printf("X set\n");
        }
    }
    while (_RB2) {
        motor_driveX(2);
    }
    _LATA0 = 1;
    _LATA1 = 1;
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    POS1CNT = 0;

    set_home_y_f = 1;
    while (_RB3) {
        motor_driveY(20);
    }
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    motor_driveY(-7);
    while (set_home_y_f) {
        if (!set_home_y_f && y_set) {
            motor_driveY(0);
            y_set = 0;
            //            printf("Y set\n");
        }
    }
    while (_RB3) {
        motor_driveY(2);
    }
    _LATA4 = 1;
    _LATB4 = 1;
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    i = 0;
    for (i = 0; i < 60000; i++)Nop();
    POS2CNT = 0;
    printf("set home finish\n");
}

void initPLL() {
    PLLFBD = 150; // M  = 152
    CLKDIVbits.PLLPRE = 5; // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001)
        ; // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    }; // Wait for PLL to lock
}

void start_program(void) {
    __builtin_disable_interrupts();

    initPLL();
    T1CONbits.TCKPS = 0b01;
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 Motor
    T3CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 servo
    PR1 = int_period; //set period to 1 ms
    PR2 = PWM_period; //set period to 15,625 tick per cycle
    PR3 = SERVO_period;

    OC1RS = 0;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC2RS = 0;
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC3RS = 750;
    OC3CONbits.OCM = 0b000; //Disable Output Compare Module
    OC3CONbits.OCTSEL = 1; //OC1 use timer2 as counter source
    OC3CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC4RS = 750;
    OC4CONbits.OCM = 0b000; //Disable Output Compare Module
    OC4CONbits.OCTSEL = 1; //OC1 use timer2 as counter source
    OC4CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    _QEA2R = 9; //remap RP14 connect to QEI1_A
    _QEB2R = 8;
    _QEA1R = 10; //remap RP14 connect to QEI2_A
    _QEB1R = 11;
    _RP0R = 0b10010; //remap RP15 connect to OC1
    _RP1R = 0b10011; //remap RP15 connect to OC2
    _RP13R = 0b10100; //remap RP15 connect to OC3
    _RP12R = 0b10101; //remap RP15 connect to OC4
    _INT1R = 2; //interrupts1
    _INT2R = 3; //interrupts2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    QEI1CONbits.QEIM = 0b000; // QEI Mode disable
    QEI1CONbits.PCDOUT = 0; // no direction pin out
    QEI1CONbits.QEIM = 0b101; // 2x ,no index
    DFLT1CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1;

    QEI2CONbits.QEIM = 0b000; // QEI Mode disable
    QEI2CONbits.PCDOUT = 0; // no direction pin out
    QEI2CONbits.QEIM = 0b101; // 2x ,no index
    DFLT2CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1;

    IEC0 |= 0x0001; //enable interrupts0
    IEC1 |= 0x2010; //enable interrupts1,2
    INTCON2 &= 0xFFFF; //Edge detection
    _INT0IP = 1; //priority
    _INT1IP = 7; //priority
    _INT2IP = 7; //priority

    _T1IE = 1; //enable interrupt for timer1
    _T1IP = 3; //priority interrupt for timer1

    AD1PCFGL = 0xFFFF; //set analog input to digital pin
    TRISB = 0x0FEC;
    TRISA = 0xFFEC;
    //    LATB = 0x0008;
    /*enable global interrupt*/
    UART1_Initialize(86, 347);
    __builtin_enable_interrupts();
    T1CONbits.TON = 1; //enable timer1
    T2CONbits.TON = 1; //enable timer2
    T3CONbits.TON = 1;

    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
}

void demo_move1() {
    while (POS1CNT < 30000) {
        motor_driveX(10);
    }
    while (POS1CNT > 500) {
        motor_driveX(-10);
    }
    motor_driveX(0);
    while (POS2CNT < 30000) {
        motor_driveY(10);
    }
    while (POS2CNT > 500) {
        motor_driveY(-10);
    }
    motor_driveY(0);
    while (POS1CNT < 30000) {
        motor_driveX(10);
        motor_driveY(10);
    }
    while (POS1CNT > 500) {
        motor_driveX(-10);
        motor_driveY(-10);
    }
    motor_driveX(0);
    motor_driveY(0);
    while (true)Nop();
}

void demo_move2(){
    x_pos = 30000;
    y_pos = 30000;
    while(POS2CNT < 30000)Nop();
    x_pos = 500;
    y_pos = 500;
    while(POS2CNT > 500)Nop();
    
    
    
}

int main(void) {
    start_program();
    set_home();
    while (1) {
        if (Em_Stop) {
            Em_Stop = 0;
            printf("Em \n");
        }
//        demo_move1();
    }
    return 0;
}