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

volatile char Em_Stop = 0, set_home_x_f = 0, set_home_y_f = 0;
volatile long int pre_x_error = 0, pre_y_error = 0, i_term_x = 0, i_term_y = 0, x_pos = 500, y_pos = 500;
volatile float x_speed = 0.0, y_speed = 0.0, pre_speed_error_x = 0, pre_speed_error_y = 0, speed_i_term_x = 0, speed_i_term_y = 0;
volatile long int pre_pos = 0;
volatile char x_set_ok = 0,y_set_ok = 0 , print_f = 0;

#define PI 3.14159265

#define PWM_period 16667
#define SERVO_period 10000
#define t1_prescaler 0b01
#define t1_period 2500
#define t4_prescaler 0b01
#define t4_period 50000
#define t5_prescaler 0b01
#define t5_period 60000
#define T_speed 0.01

#define k_p_x 9
#define k_d_x 2
#define k_p_y 14
#define k_d_y 5

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

void delay(int time_ms) {
    unsigned int i = 0;
    for (i = 0; i < time_ms; i++) {
        unsigned int j;
        for (j = 0; j < 4000; j++)
            Nop();
    }
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
    //    else if (T1CONbits.TON == 1){
    //    T1CONbits.TON = 0;
    //    delay(10);
    //    _LATA0 = 0;
    //    _LATA1 = 0;
    //    _LATA4 = 0;
    //    _LATB4 = 0;
    //    }
    //    else if (T1CONbits.TON == 0){
    //    T1CONbits.TON = 1;
    //    }
    IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    if (set_home_y_f) {
        set_home_y_f = 0;
    }
    //    else if (T1CONbits.TON == 1){
    //    T1CONbits.TON = 0;
    //    delay(10);
    //    _LATA0 = 0;
    //    _LATA1 = 0;
    //    _LATA4 = 0;
    //    _LATB4 = 0;
    //    }
    //    else if (T1CONbits.TON == 0){
    //    T1CONbits.TON = 1;
    //    }
    IFS1bits.INT2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    long int error_x = (long int) (x_pos - (long int) POS1CNT);
    long int error_y = (long int) (y_pos - (long int) POS2CNT);
    long int d_term_x = error_x - pre_x_error;
    long int d_term_y = error_y - pre_y_error;
    long int x_pwm = (long int) (k_p_x * error_x + k_d_x * d_term_x);
    long int y_pwm = (long int) (k_p_y * error_y + k_d_y * d_term_y);

    if (x_pwm >= PWM_period) {
        x_pwm = PWM_period;
    } else if (x_pwm <= -PWM_period) {
        x_pwm = PWM_period;
    }

    if (y_pwm >= PWM_period) {
        y_pwm = PWM_period;
    } else if (y_pwm <= -PWM_period) {
        y_pwm = PWM_period;
    }

    OC1RS = (unsigned int) abs(x_pwm);
    OC2RS = (unsigned int) abs(y_pwm);

    if (abs(error_x) <= 50) {
        _LATA0 = 1;
        _LATA1 = 1;
        x_set_ok = 1;
    } else if (x_pos > (unsigned int) POS1CNT) {
        _LATA0 = 1;
        _LATA1 = 0;
        x_set_ok = 0;
    } else {
        _LATA0 = 0;
        _LATA1 = 1;
        x_set_ok = 0;
    }

    if (abs(error_y) <= 50) {
        _LATA4 = 1;
        _LATB4 = 1;
        y_set_ok = 1;
    } else if (y_pos > (unsigned int) POS2CNT) {
        _LATA4 = 1;
        _LATB4 = 0;
        y_set_ok = 0;
    } else {
        _LATA4 = 0;
        _LATB4 = 1;
        y_set_ok = 0;
    }
    pre_x_error = error_x;
    pre_y_error = error_y;
    //    printf("%lu %lu %ld %ld\n", (long int) x_pos, (long int) POS1CNT, x_pwm, error_x);
    //        printf("%lu %lu %ld %ld\n",(long int)y_pos,(long int)POS2CNT,y_pwm,error_y);
    _T1IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    //    long int s = (long int) POS1CNT - pre_pos;
    //    if (s >= 32768) {
    //        s -= 65535;
    //    } else if (s <= -32768) {
    //        s += 65535;
    //    }
    //    float v = (float)s / T_speed;
    _T4IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    print_f = 1;
//    printf("%u %u\n", POS1CNT, POS2CNT);
    _T5IF = 0; //clear interrupt flag
}

void set_home() {
    T1CONbits.TON = 0;
    set_home_x_f = 1;
    int x_set = 1;
    int y_set = 1;
    printf("sethome\n");
    while (_RB2) {
        motor_driveX(30);
    }
    delay(500);
    motor_driveX(-15);
    while (set_home_x_f) {
        if (!set_home_x_f && x_set) {
            motor_driveX(0);
            x_set = 0;
        }
    }
    while (_RB2) {
        motor_driveX(10);
    }
    _LATA0 = 1;
    _LATA1 = 1;
    delay(300);

    POS1CNT = 0;
    printf("X set\n");

    set_home_y_f = 1;
    while (_RB3) {
        motor_driveY(30);
    }
    delay(500);
    motor_driveY(-20);
    while (set_home_y_f) {
        if (!set_home_y_f && y_set) {
            motor_driveY(0);
            y_set = 0;
        }
    }
    while (_RB3) {
        motor_driveY(10);
    }
    _LATA4 = 1;
    _LATB4 = 1;
    delay(300);
    POS2CNT = 0;
    printf("Y set\n");
    printf("set home finish\n");
    T1CONbits.TON = 1;
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
    T1CONbits.TCKPS = t1_prescaler;
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 Motor
    T3CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 servo
    T4CONbits.TCKPS = t4_prescaler;
    T5CONbits.TCKPS = t5_prescaler;
    PR1 = t1_period;
    PR2 = PWM_period;
    PR3 = SERVO_period;
    PR4 = t4_period;
    PR5 = t5_period;

    OC1RS = 0;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC2RS = 0;
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0; //OC2 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC3RS = 750;
    OC3CONbits.OCM = 0b000; //Disable Output Compare Module
    OC3CONbits.OCTSEL = 1; //OC3 use timer3 as counter source
    OC3CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC4RS = 750;
    OC4CONbits.OCM = 0b000; //Disable Output Compare Module
    OC4CONbits.OCTSEL = 1; //OC4 use timer3 as counter source
    OC4CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    _QEA2R = 9; //remap connect to QEI1_A
    _QEB2R = 8; //remap connect to QEI1_B
    _QEA1R = 10; //remap connect to QEI2_A
    _QEB1R = 11; //remap connect to QEI2_B
    _RP0R = 0b10010; //remap connect to OC1
    _RP1R = 0b10011; //remap connect to OC2
    _RP13R = 0b10100; //remap connect to OC3
    _RP12R = 0b10101; //remap connect to OC4
    _INT1R = 2; //remap external interrupts1
    _INT2R = 3; //remap external interrupts2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    QEI1CONbits.QEIM = 0b000; // QEI1 Mode disable
    QEI1CONbits.PCDOUT = 0; // no direction pin out
    QEI1CONbits.QEIM = 0b101; // 2x ,no index
    DFLT1CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1; // QEI1 Enable digital filter

    QEI2CONbits.QEIM = 0b000; // QEI2 Mode disable
    QEI2CONbits.PCDOUT = 0; // no direction pin out
    QEI2CONbits.QEIM = 0b101; // 2x ,no index
    DFLT2CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1; // QEI2 Enable digital filter

    IEC0 |= 0x0001; //enable interrupts0
    IEC1 |= 0x2010; //enable interrupts1,2
    INTCON2 &= 0xFFFF; //Edge detection
    _INT0IP = 1; //priority
    _INT1IP = 7; //priority
    _INT2IP = 7; //priority

    _T1IE = 1; //enable interrupt for timer1
    _T4IE = 1; //enable interrupt for timer4
    _T5IE = 1; //enable interrupt for timer5
    _T1IP = 3; //priority interrupt for timer1
    _T4IP = 1; //priority interrupt for timer4
    _T5IP = 1; //priority interrupt for timer5

    AD1PCFGL = 0xFFFF; //set analog input to digital pin
    TRISB = 0x0FEC;
    TRISA = 0xFFEC;

    UART1_Initialize(86, 347);
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    T2CONbits.TON = 1; //enable timer2
    T3CONbits.TON = 1; //enable timer3
    T4CONbits.TON = 1; //enable timer4
    T5CONbits.TON = 1; //enable timer5

    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
}

void demo_move1() {
    x_pos = 30000;
    y_pos = 30000;
}

int demo_move2(int stage) {

    if (stage == 0) {
        if (POS2CNT <= 29900 || POS1CNT <= 29900) {
            x_pos = 30000;
            y_pos = 30000;
        } else {
            stage = 1;
        }
    }

    if (stage == 1) {
        if (POS2CNT >= 600 || POS1CNT >= 600) {
            x_pos = 500;
            y_pos = 500;
        } else {
            stage = 0;
        }
    }
    return stage;
}

int main(void) {
    start_program();
    set_home();

    //    int stage = 0;
    int numByte;
    uint8_t dataArray[10];
    char status = 0;
    char return_status = 0;
    unsigned int x_buffer=0,y_buffer =0;

    while (1) {
        if (Em_Stop) {
            Em_Stop = 0;
            printf("Em \n");
        }
        //        int output = demo_move2(stage);
        //        stage = output;
//        demo_move1();

        numByte = UART1_ReadBuffer(dataArray, 10);
        if (numByte != 0) {
            int i;
            for (i = 0; i < numByte; i++) {
                if (dataArray[i] == 0xFF && status == 0){
                    status = 1;
                }
                else if (dataArray[i] == 0xFF && status == 1){
                    status = 2;
                }
                else if (status == 2){
                    x_buffer |= dataArray[i];
                    status = 3;
                }
                else if (status == 3){
                    x_buffer |= dataArray[i]<<8;
                    x_pos = (unsigned int)(x_buffer*102.4);
                    x_buffer =0;
                    status = 4;
                }
                else if (status == 4){
                    y_buffer |= dataArray[i];
                    status = 5;
                }
                else if (status == 5){
                    y_buffer |= dataArray[i]<<8;
                    y_pos = (unsigned int)(y_buffer*102.4);
                    y_buffer =0;
                    status = 0;
                }
            }
        }
    //    if (print_f == 1 && return_status == 0){
    //        print_f = 0;
    //        printf("%u %u\n", POS1CNT, POS2CNT);
    //    }
            
        
        if (x_set_ok == 1 && y_set_ok == 1){
            if(return_status == 1){
                printf("ok\n");
                return_status = 0;
            }
        }
        else{
            return_status = 1;
        }
        

        //        if (lastValue != POS1CNT) {
        //            printf("%u\n", POS1CNT);
        //            lastValue = POS1CNT;
        //        }
    }
    return 0;
}