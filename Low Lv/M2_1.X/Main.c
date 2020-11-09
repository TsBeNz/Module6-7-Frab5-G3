/*
 * File:   outputcompare1.c
 * Author: G_Peav
 *
 * Created on September 4, 2019, 4:29 PM
 */

#include "xc.h"
#include <stdio.h>
#include "configuration.h"
#include <math.h>
#include "uart1Drv.c"

volatile char Em_Stop = 0, set_home_x_f = 0, set_home_y_f = 0;
volatile long int x_pos = 500, y_pos = 500;
volatile float x_speed = 0.0, y_speed = 0.0, pre_speed_error_x = 0, pre_speed_error_y = 0, speed_i_term_x = 0, speed_i_term_y = 0;
volatile long int pre_pos = 0;
volatile char x_set_ok = 0, y_set_ok = 0, print_f = 0;

volatile float unwrapped_position[2] = {0.0, 0.0};
volatile float velocity_set_point[2] = {0, 0}; // max 90 mm/s
volatile float position_set_point[2] = {0, 0}; // max 430 mm
volatile char position_enable[2] = {1, 1};
volatile float co_trajectory[4][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
}; //[position in matix][axis]
volatile float trajectory_time_set = 0;
volatile char trajectory_finish_move = 1;
volatile float a = 0, b = 0, c = 0, d = 0, e = 0;
volatile char next = 0;

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
#define T_speed_pow2 0.001
#define T_speed_pow3 0.0001
#define T_speed_pow4 0.00001

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
    static float sigma_a[2] = {6, 6}; // adjustable
    static float sigma_w[2] = {1, 1}; // adjustable
    static float x1[2] = {0, 0}; // orientation
    static float x2[2] = {0, 0}; // w_outKalman (angular_vel)
    static float p11[2] = {1.0, 1.0}; // adjustable
    static float p12[2] = {0, 0};
    static float p21[2] = {0, 0};
    static float p22[2] = {1.0, 1.0}; // adjustable
    static long int pre_position[2] = {0, 0};
    static float pre_position_error[2] = {0, 0}, position_i_term[2] = {0, 0}, cmd_velocity[2] = {0, 0};
    static float position_kp[2] = {0.9, 0.9}, position_ki[2] = {0, 0}, position_kd[2] = {0, 0};
    static float pre_velocity_error[2] = {0, 0}, velocity_i_term[2] = {0, 0}, pwm_output[2] = {0, 0};
    static int velocity_kp[2] = {600, 600}, velocity_ki[2] = {30, 30}, velocity_kd[2] = {330, 330};
    static float trajectory_time = 0;

    //calculation command part

    long int position[2] = {(long int) POS1CNT, (long int) POS2CNT}; //read data form encoder
    float v[2] = {0, 0};
    float trajectory_time_pow2 = trajectory_time * trajectory_time;
    float trajectory_time_pow3 = trajectory_time_pow2 * trajectory_time;
    int i = 0;
    for (i = 0; i < 2; i++) {
        if (!trajectory_finish_move) {
            // Trajectory motion
            position_set_point[i] = co_trajectory[0][i] + co_trajectory[1][i] * trajectory_time + co_trajectory[2][i] * trajectory_time_pow2 + co_trajectory[3][i] * trajectory_time_pow3;
            velocity_set_point[i] = co_trajectory[1][i] + 2 * co_trajectory[2][i] * trajectory_time + 3 * co_trajectory[3][i] * trajectory_time_pow2;
        }

        long int ds = position[i] - pre_position[i]; //find delta s
        //Unwrapping position
        if (ds >= 32768) {
            ds -= 65535;
        } else if (ds <= -32768) {
            ds += 65535;
        }
        float ds_mm = (float) ds / 102.4; // change pulse to mm
        v[i] = ds_mm / T_speed; // velocity [mm/s]
        unwrapped_position[i] += ds_mm; // update new position

        // kaman filter

        float Q = sigma_a[i] * sigma_a[i];
        float R = sigma_w[i] * sigma_w[i];
        float x1_new = x1[i] + x2[i] * T_speed;
        float x2_new = 0 + x2[i];
        float ye = v[i] - x2_new;
        p11[i] = p11[i] + T_speed * p21[i] + (Q * T_speed_pow4) / 4 + (T_speed_pow2 * (p12[i] + T_speed * p22[i])) / T_speed;
        p12[i] = p12[i] + T_speed * p22[i] + (Q * T_speed_pow3) / 2;
        p21[i] = (2 * T_speed * p21[i] + Q * T_speed_pow2 + 2 * p22[i] * T_speed_pow2) / (2 * T_speed);
        p22[i] = Q * T_speed_pow2 + p22[i];
        x1_new = x1_new + (p12[i] * ye) / (R + p22[i]);
        x2_new = x2_new + (p22[i] * ye) / (R + p22[i]);
        p11[i] = p11[i] - (p12[i] * p21[i]) / (R + p22[i]);
        p12[i] = p12[i] - (p12[i] * p22[i]) / (R + p22[i]);
        p21[i] = -p21[i] * (p22[i] / (R + p22[i]) - 1);
        p22[i] = -p22[i] * (p22[i] / (R + p22[i]) - 1);
        //update variable of kaman filter
        pre_position[i] = position[i];
        x1[i] = x1_new; // position
        x2[i] = x2_new; // velocity

        // position control //
        //        float position_error = position_set_point[i] - unwrapped_position[i];
        //        position_i_term[i] += position_error;
        //        float position_d_term = position_error - pre_position_error[i];
        //        cmd_velocity[i] = (position_kp[i] * position_error)+(position_ki[i] * position_i_term[i])+(position_kd[i] * position_d_term);

        float position_error = position_set_point[i] - unwrapped_position[i];
        cmd_velocity[i] = (position_kp[i] * position_error);
        //update variable of position control
        pre_position_error[i] = position_error;

        // calculation velocity from (velocity form position control) + (velocity feedforward)

        float velocity_input = velocity_set_point[i] + position_enable[i] * cmd_velocity[i];
        if (velocity_input >= 90.0) {
            velocity_input = 90.0;
        }

        // velocity control //
        float velocity_error = velocity_input - x2[i];
        velocity_i_term[i] += velocity_error;
        float velocity_d_term = velocity_error - pre_velocity_error[i];
        pwm_output[i] = (velocity_kp[i] * velocity_error) + (velocity_ki[i] * velocity_i_term[i]) + (velocity_kd[i] * velocity_d_term);
        //update variable of velocity control
        pre_velocity_error[i] = velocity_error;
    }

    //end of calculation command part

    // drive command
    if (pwm_output[0] >= PWM_period) {
        pwm_output[0] = PWM_period;
    } else if (pwm_output[0] <= -PWM_period) {
        pwm_output[0] = PWM_period;
    }
    if (pwm_output[1] >= PWM_period) {
        pwm_output[1] = PWM_period;
    } else if (pwm_output[1] <= -PWM_period) {
        pwm_output[1] = PWM_period;
    }

    if (pwm_output[0] >= 0) {
        OC1RS = (unsigned int) pwm_output[0];
        _LATA0 = 1;
        _LATA1 = 0;
    } else {
        OC1RS = (unsigned int) abs(pwm_output[0]);
        _LATA0 = 0;
        _LATA1 = 1;
    }

    if (pwm_output[1] >= 0) {
        OC2RS = (unsigned int) pwm_output[1];
        _LATA4 = 1;
        _LATB4 = 0;
    } else {
        OC2RS = (unsigned int) abs(pwm_output[1]);
        _LATA4 = 0;
        _LATB4 = 1;
    }

    //update time trajectory
    if (trajectory_time >= trajectory_time_set) {
        trajectory_time = 0;
        trajectory_finish_move = 1;
        _LATA4 = 1;
        _LATB4 = 1;
        _LATA0 = 1;
        _LATA1 = 1;
        if (next == 1) {
            next = 2;
        }
        else if (next == 3) {
            next = 4;
        }
        else if (next == 5) {
            next = 6;
        }
        else if (next == 7) {
            next = 2;
        }
    }
    if (!trajectory_finish_move) {
        trajectory_time += T_speed;
    }

    //debug
    a = x1[1];
    b = unwrapped_position[1];
    c = x2[1];
    d = v[1];
    e = position_set_point[1];
    _T4IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    print_f = 1;
    printf("%.2f %.2f %.2f %.2f %.2f \n", b, a, d, c, e);
    //    printf("%.2f\n", e);
    //    printf("%u %u\n", POS1CNT, POS2CNT);
    _T5IF = 0; //clear interrupt flag
}

void set_home() {
    T1CONbits.TON = 0;
    T4CONbits.TON = 0;
    T5CONbits.TON = 0;
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
    T5CONbits.TON = 1;
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

//    UART1_Initialize(86, 347);    // UxBRG = 4 high speed = 1 buadrate 2,000,000
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    cfgDma0UartTx();
	cfgDma1UartRx();
    cfgUart1();
    
    T2CONbits.TON = 1; //enable timer2
    T3CONbits.TON = 1; //enable timer3
    //    T4CONbits.TON = 1; //enable timer4
    //    T5CONbits.TON = 1; //enable timer5

    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
}

void trajectory() {
    int i = 0;
    for (i = 0; i < 2; i++) {
        velocity_set_point[i] = 0; // max 90 mm/s
        position_set_point[i] = 0; // max 430 mm
        position_enable[i] = 1;
    }
    co_trajectory[0][0] = 0;
    co_trajectory[1][0] = 0;
    co_trajectory[2][0] = 37.5;
    co_trajectory[3][0] = -12.5;
    co_trajectory[0][1] = 0;
    co_trajectory[1][1] = 0;
    co_trajectory[2][1] = 37.5;
    co_trajectory[3][1] = -12.5;
    trajectory_time_set = 2;
    trajectory_finish_move = 0;
}

void trajectory2() {
    int i = 0;
    for (i = 0; i < 2; i++) {
        velocity_set_point[i] = 0; // max 90 mm/s
        position_set_point[i] = 0; // max 430 mm
        position_enable[i] = 1;
    }
    co_trajectory[0][0] = 50;
    co_trajectory[1][0] = 0;
    co_trajectory[2][0] = 7.9592;
    co_trajectory[3][0] = -0.758;
    co_trajectory[0][1] = 50;
    co_trajectory[1][1] = 0;
    co_trajectory[2][1] = 21.4286;
    co_trajectory[3][1] = -2.0408;
    trajectory_time_set = 7;
    trajectory_finish_move = 0;
}

void trajectory3() {
    int i = 0;
    for (i = 0; i < 2; i++) {
        velocity_set_point[i] = 0; // max 90 mm/s
        position_set_point[i] = 0; // max 430 mm
        position_enable[i] = 1;
    }
    co_trajectory[0][0] = 180;
    co_trajectory[1][0] = 0;
    co_trajectory[2][0] = 14.0816;
    co_trajectory[3][0] = -1.3411;
    co_trajectory[0][1] = 400;
    co_trajectory[1][1] = 0;
    co_trajectory[2][1] = -21.4286;
    co_trajectory[3][1] = 2.0408;
    trajectory_time_set = 7;
    trajectory_finish_move = 0;
}

void trajectory4() {
    int i = 0;
    for (i = 0; i < 2; i++) {
        velocity_set_point[i] = 0; // max 90 mm/s
        position_set_point[i] = 0; // max 430 mm
        position_enable[i] = 1;
    }
    co_trajectory[0][0] = 410;
    co_trajectory[1][0] = 0;
    co_trajectory[2][0] = -22.0408;
    co_trajectory[3][0] = 2.0991;
    co_trajectory[0][1] = 50;
    co_trajectory[1][1] = 0;
    co_trajectory[2][1] = 0;
    co_trajectory[3][1] = 0;
    trajectory_time_set = 7;
    trajectory_finish_move = 0;
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
        if (next == 0){
            trajectory();
            next = 1;
        }
        else if (next == 2) {
            trajectory2();
            next = 3;
        }
        else if (next == 4) {
            trajectory3();
            next = 5;
        }
        else if (next == 6) {
            trajectory4();
            next = 7;
        }
        if (Em_Stop) {
            Em_Stop = 0;
            printf("Em \n");
        }
        //        static int stage = 0;
        //        stage = demo_move2(stage);

        //        demo_move1();

//        numByte = UART1_ReadBuffer(dataArray, 10);
//        if (numByte != 0) {
//            int i;
//            for (i = 0; i < numByte; i++) {
//                if (dataArray[i] == 0xFF && data_status == 0) {
//                    data_status = 1;
//                } else if (data_status == 1) {
//                    if (dataArray[i] == 0xFF) {
//                        data_status = 2;
//                        data_type = 1;
//                    }
//                } else if (data_status == 2) {
//                    x_buffer |= (dataArray[i] << 1);
//                    data_status = 3;
//                } else if (data_status == 3) {
//                    int buffer = dataArray[i];
//                    x_buffer |= (dataArray[i] >> 7);
//                    y_buffer |= (0x01FC & (buffer << 2));
//                    data_status = 4;
//                } else if (data_status == 4) {
//                    int buffer = dataArray[i];
//                    y_buffer |= (dataArray[i] >> 6);
//                    z_buffer |= (0x01F8 & (buffer << 3));
//                    data_status = 5;
//                } else if (data_status == 5) {
//                    int buffer = dataArray[i];
//                    z_buffer |= (dataArray[i] >> 5);
//                    thata_buffer |= (0x01F0 & (buffer << 4));
//                    data_status = 6;
//                } else if (data_status == 6) {
//                    //                    int buffer = dataArray[i];
//                    thata_buffer |= (dataArray[i] >> 4);
//                    //                    int chack_sum = (0x0F & buffer);
//                    //                    if (chack_sum == ((y_buffer + y_buffer + z_buffer+ thata_buffer)%15)){
//                    y_pos = (unsigned int) (y_buffer * 102.4);
//                    x_pos = (unsigned int) (x_buffer * 102.4);
//                    //                    printf("%u %u %u %u\n", x_buffer, y_buffer, z_buffer, thata_buffer);
//                    //                    }
//                    x_buffer = 0;
//                    y_buffer = 0;
//                    z_buffer = 0;
//                    thata_buffer = 0;
//                    data_status = 0;
//                }
//            }
//        }
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