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
volatile long int x_pos = 500, y_pos = 500;
volatile float x_speed = 0.0, y_speed = 0.0, pre_speed_error_x = 0, pre_speed_error_y = 0, speed_i_term_x = 0, speed_i_term_y = 0;
volatile long int pre_pos = 0;
volatile char x_set_ok = 0, y_set_ok = 0, print_f = 0;
float x2_out = 0;
float v_set = 0;
float w_out = 0;
float x_out = 0;

#define PI 3.14159265

#define PWM_period 16667
#define SERVO_period 10000
#define t1_prescaler 0b01
#define t1_period 25000
#define t4_prescaler 0b01  
#define t4_period 50000
#define t5_prescaler 0b01
#define t5_period 60000
#define T_speed 0.01

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
    static long int pre_x_error = 0, pre_y_error = 0, i_term_x = 0, i_term_y = 0;
    long int error_x = (long int) (x_pos - (long int) POS1CNT);
    long int error_y = (long int) (y_pos - (long int) POS2CNT);
    i_term_x = error_x + i_term_x;
    i_term_y = error_y + i_term_y;
    long int d_term_x = error_x - pre_x_error;
    long int d_term_y = error_y - pre_y_error;
    long int x_pwm = (25 * error_x + 0 * i_term_x + 15 * d_term_x);
    long int y_pwm = (22 * error_y + 0 * i_term_y + 12 * d_term_y);

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

    if (abs(error_x) <= 20) {
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

    if (abs(error_y) <= 20) {
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
    //        printf("%lu %lu %ld %ld\n", (long int) x_pos, (long int) POS1CNT, x_pwm, error_x);
    //        printf("%lu %lu %ld %ld\n",(long int)y_pos,(long int)POS2CNT,y_pwm,error_y);
    _T1IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
//        motor_driveX(100);
    static float t = 0;
    #define a3 0.0750
    #define a4 0.0025
    
    float point_tra = (a3*t*t)+(a4*t*t*t);
    float v_tra = (a3*2)+(3*a4)+(t*t);
    
    static float sigma_a = 10; // adjustable
    static float sigma_w = 1; // adjustable
    static float Q, R;
    static float x1_in = 0;
    static float x2_in = 0;
    static float x1 = 0; // orientation
    static float x2 = 0; // w_outKalman (angular_vel)
    static float p11 = 1.0; // adjustable
    static float p12 = 0;
    static float p21 = 0;
    static float p22 = 1.0; // adjustable
    static long int pre_vx_error = 0;
    
    long int position = (long int) POS1CNT;
    long int s = position - pre_pos;
    static float un_position = 0.0;
    if (s >= 32768) {
        s -= 65535;
    } else if (s <= -32768) {
        s += 65535;
    }
    un_position += ((float) s / 17.06667);
    float v = (float) s / T_speed;
    v = v * (0.05859375);
    Q = sigma_a*sigma_a;
    R = sigma_w*sigma_w;
    float x1_new = x1_in + x2_in*T_speed;
    float x2_new = 0 + x2_in;
    float ye = v - x2_new;
    p11 = p11 + T_speed * p21 + (Q * T_speed * T_speed * T_speed * T_speed) / 4 + (T_speed * T_speed * (p12 + T_speed * p22)) / T_speed;
    p12 = p12 + T_speed * p22 + (Q * T_speed * T_speed * T_speed) / 2;
    p21 = (2 * T_speed * p21 + Q * T_speed * T_speed + 2 * p22 * T_speed * T_speed) / (2 * T_speed);
    p22 = Q * T_speed * T_speed + p22;
    x1_new = x1_new + (p12 * ye) / (R + p22);
    x2_new = x2_new + (p22 * ye) / (R + p22);
    p11 = p11 - (p12 * p21) / (R + p22);
    p12 = p12 - (p12 * p22) / (R + p22);
    p21 = -p21 * (p22 / (R + p22) - 1);
    p22 = -p22 * (p22 / (R + p22) - 1);
    x1 = x1_new;
    x2 = x2_new;
    x1_in = x1;
    x2_in = x2;
    
    static long int pre_x_error = 0, i_term_x = 0;
    long int error_x = (long int) (x_pos - (long int) POS1CNT);
    i_term_x = error_x + i_term_x;
    long int d_term_x = error_x - pre_x_error;
    float v_PID = (0.015 * (float)error_x + 0 * i_term_x + 0.001 * d_term_x);
    static float i_term_vx =0;
    static float v_setpoint = 0;
    v_setpoint = v_PID;
    if(v_setpoint == 0.0){
        i_term_vx = 0;
    }
    
//    static float vx_pwm =0;
    float error_vx = v_setpoint - x2;
    float d_term_vx = error_vx - pre_vx_error;
    i_term_vx = error_vx + i_term_vx;
    float vx_pwm = (35 * error_vx + 1 * i_term_vx +  8 * d_term_vx); 
//    vx_pwm += vx_pwm_real;
    printf("%.2f \n",v_PID);
    if ((long int)vx_pwm >= PWM_period) {
        vx_pwm = PWM_period;
    } else if ((long int)vx_pwm <= -PWM_period) {
        vx_pwm = PWM_period;
    }


//    if (abs(vx_pwm) <= 10) {
//        i_term_vx = 0;
//    }
    
    if (vx_pwm >= 0) {
        OC1RS = (unsigned int) abs(vx_pwm);
        _LATA0 = 1;
        _LATA1 = 0;
    } else {
        OC1RS = (unsigned int) abs(vx_pwm);
        _LATA0 = 0;
        _LATA1 = 1;
    }
    pre_pos = position;
    pre_vx_error = error_vx;
    x2_out = x1;
    w_out = x2;
    v_set = v;
    x_out = un_position;
    _T4IF = 0; //clear interrupt flag

}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    print_f = 1;
//    printf("%.2f %.2f %.2f %.2f \n", v_set, w_out, x_out, x2_out);
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
    motor_driveX(-18);
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
//    T1CONbits.TON = 1;
    T4CONbits.TON = 1;
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
    _INT0IP = 7; //priority
    _INT1IP = 7; //priority
    _INT2IP = 7; //priority

    _T1IE = 1; //enable interrupt for timer1
    _T4IE = 1; //enable interrupt for timer4
    _T5IE = 1; //enable interrupt for timer5
    _T1IP = 5; //priority interrupt for timer1
    _T4IP = 5; //priority interrupt for timer4
    _T5IP = 1; //priority interrupt for timer5

    AD1PCFGL = 0xFFFF; //set analog input to digital pin
    TRISB = 0x0FEC;
    TRISA = 0xFFEC;

    UART1_Initialize(86, 347);
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    T2CONbits.TON = 1; //enable timer2
    T3CONbits.TON = 1; //enable timer3
//    T4CONbits.TON = 1; //enable timer4
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
    printf("ok\n");

    //    
    int numByte;
    uint8_t dataArray[10];
    char data_status = 0, data_type = 0;
    char return_status = 0;
    unsigned int x_buffer = 0, y_buffer = 0, z_buffer = 0, thata_buffer = 0;

    while (1) {
        if (Em_Stop) {
            Em_Stop = 0;
            printf("Em \n");
        }
        //        static int stage = 0;
        //        int output = demo_move2(stage);
        //        stage = output;

                demo_move1();


        numByte = UART1_ReadBuffer(dataArray, 10);
        if (numByte != 0) {
            int i;
            for (i = 0; i < numByte; i++) {
                if (dataArray[i] == 0xFF && data_status == 0) {
                    data_status = 1;
                } else if (data_status == 1) {
                    if (dataArray[i] == 0xFF) {
                        data_status = 2;
                        data_type = 1;
                    }
                } else if (data_status == 2) {
                    x_buffer |= (dataArray[i] << 1);
                    data_status = 3;
                } else if (data_status == 3) {
                    int buffer = dataArray[i];
                    x_buffer |= (dataArray[i] >> 7);
                    y_buffer |= (0x01FC & (buffer << 2));
                    data_status = 4;
                } else if (data_status == 4) {
                    int buffer = dataArray[i];
                    y_buffer |= (dataArray[i] >> 6);
                    z_buffer |= (0x01F8 & (buffer << 3));
                    data_status = 5;
                } else if (data_status == 5) {
                    int buffer = dataArray[i];
                    z_buffer |= (dataArray[i] >> 5);
                    thata_buffer |= (0x01F0 & (buffer << 4));
                    data_status = 6;
                } else if (data_status == 6) {
                    //                    int buffer = dataArray[i];
                    thata_buffer |= (dataArray[i] >> 4);
                    //                    int chack_sum = (0x0F & buffer);
                    //                    if (chack_sum == ((y_buffer + y_buffer + z_buffer+ thata_buffer)%15)){
                    y_pos = (unsigned int) (y_buffer * 102.4);
                    x_pos = (unsigned int) (x_buffer * 102.4);
                    //                    printf("%u %u %u %u\n", x_buffer, y_buffer, z_buffer, thata_buffer);
                    //                    }    
                    x_buffer = 0;
                    y_buffer = 0;
                    z_buffer = 0;
                    thata_buffer = 0;
                    data_status = 0;
                }
            }
        }
        //    if (print_f == 1 && return_status == 0){
        //        print_f = 0;
        //        printf("%u %u\n", POS1CNT, POS2CNT);
        //    }


        if (x_set_ok == 1 && y_set_ok == 1) {
            if (return_status == 1) {
                printf("ok\n");
                return_status = 0;
            }
        } else {
            return_status = 1;
        }


        //        if (lastValue != POS1CNT) {
        //            printf("%u\n", POS1CNT);
        //            lastValue = POS1CNT;
        //        }
    }
    return 0;
}