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
#include <stdlib.h>

#define FCY 40000000
#define BAUDRATE 2000000
#define BRGVAL ((FCY / BAUDRATE) / 4) - 1

#define PI 3.14159265
#define PWM_period 16667
#define SERVO_period 800
#define t1_prescaler 0b01
#define t1_period 10000
#define t4_prescaler 0b01
#define t4_period 50000
#define t5_prescaler 0b10
#define t5_period 12500

// #ifdef

// #endif

#define T_speed 0.01
#define T_speed_pow2 (T_speed * T_speed)
#define T_speed_pow3 (T_speed * T_speed * T_speed)
#define T_speed_pow4 (T_speed * T_speed * T_speed * T_speed)
unsigned char BufferA[8] __attribute__((space(dma)));
unsigned char BufferB[50] __attribute__((space(dma)));

volatile char Em_Stop = 0, set_home_x_f = 0, set_home_y_f = 0, griper_status = 0;

volatile float z_before = 410;

volatile int x = 0, y = 0, z = 0, theta = 0, update_position_trajectory = 0;
volatile char sethomef = 0, driver = 0;
volatile long int pre_position[2] = {0, 0};                                 // for find ds
volatile float x1[2] = {0, 0};                                              // posiition
volatile float x2[2] = {0, 0};                                              // velocity
volatile float pre_velocity_error[2] = {0, 0}, velocity_i_term[2] = {0, 0}; // velocity PID
volatile float unwrapped_position[3] = {0.0, 0.0, 0.0};
volatile float velocity_set_point[3] = {0, 0, 0}; // max +-90 mm/s
volatile float position_set_point[3] = {0, 0, 0}; // max 430 mm
volatile char position_enable[3] = {1, 1, 1}, velocity_enable[3] = {1, 1, 1};
volatile float co_trajectory[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
}; //[position in matix][axis]
volatile float trajectory_time_set = 0;
volatile char trajectory_finish_move = 1;
volatile float a = 0, b = 0, c = 0, d = 0, e = 0;
// volatile char next = 0;

void motor_driveX(int speed)
{
    if ((int)speed == 1)
    {
        _LATA0 = 1;
        _LATA1 = 0;
    }
    else if ((int)speed > 1)
    {
        _LATA0 = 1;
        _LATA1 = 0;
    }
    else
    {
        _LATA0 = 0;
        _LATA1 = 1;
    }
    speed = abs(speed);
    unsigned long pwm = speed * ((unsigned long)PWM_period);
    pwm /= 100;
    OC1RS = pwm;
}

void motor_driveY(int speed)
{
    if ((int)speed == 1)
    {
        _LATA4 = 0;
        _LATB4 = 0;
    }
    else if ((int)speed > 1)
    {
        _LATA4 = 1;
        _LATB4 = 0;
    }
    else
    {
        _LATA4 = 0;
        _LATB4 = 1;
    }
    speed = abs(speed);
    unsigned long pwm = speed * ((unsigned long)PWM_period);
    pwm /= 100;
    OC2RS = pwm;
}

void dma_print()
{
    DMA0STA = __builtin_dmaoffset(BufferB);
    DMA0CNT = strlen(BufferB) - 1;
    DMA0CONbits.CHEN = 1;  // Re-enable DMA0 Channel
    DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
}

void delay(int time_ms)
{
    unsigned int i = 0;
    for (i = 0; i < time_ms; i++)
    {
        unsigned int j;
        for (j = 0; j < 4000; j++)
            Nop();
    }
}

void set_home()
{
    position_enable[0] = 0;
    position_enable[1] = 0;
    velocity_enable[0] = 0;
    velocity_enable[1] = 0;
    velocity_enable[2] = 0;
    set_home_x_f = 1;
    int x_set = 1;
    int y_set = 1;
    printf("sethome\n");
    while (_RA2)
    {
        PR3 = 500;
        OC3RS = 250;
        _LATB13 = 0; //down
    }
    delay(200);
    while (!_RA2)
    {
        PR3 = 1000;
        OC3RS = 500;
        _LATB13 = 1; //up
    }
    int i = 0;

    // for (i = 0; i < 20; i++)
    // {
    //     PR3 = 1000;
    //     OC3RS = 500;
    //     _LATB13 = 0; //down
    //     delay(2000);
    //     int j = 500;
    //     while (!_RA2)
    //     {
    //         PR3 = j;
    //         OC3RS = j / 2;
    //         _LATB13 = 1; //up
    //         delay(1);
    //         j += 2;
    //         if (j >= 50000)
    //         {
    //             j = 50000;
    //         }
    //     }
    // }

    PR3 = 0;     //stop
    OC3RS = 0;   //stop
    _LATB13 = 1; //up
    printf("Z set\n");

    while (_RB2)
    {
        motor_driveX(30);
    }
    delay(500);
    motor_driveX(-18);
    while (set_home_x_f)
    {
        if (!set_home_x_f && x_set)
        {
            motor_driveX(0);
            x_set = 0;
        }
    }
    while (_RB2)
    {
        motor_driveX(10);
    }
    _LATA0 = 1;
    _LATA1 = 1;
    delay(300);

    POS1CNT = 0;
    printf("X set\n");

    set_home_y_f = 1;
    while (_RB3)
    {
        motor_driveY(30);
    }
    delay(500);
    motor_driveY(-20);
    while (set_home_y_f)
    {
        if (!set_home_y_f && y_set)
        {
            motor_driveY(0);
            y_set = 0;
        }
    }
    while (_RB3)
    {
        motor_driveY(10);
    }
    _LATA4 = 1;
    _LATB4 = 1;
    delay(300);
    POS2CNT = 0;
    printf("Y set\n");
    printf("set home finish\n");
    i = 0;
    for (i = 0; i < 2; i++)
    {
        pre_velocity_error[i] = 0;
        velocity_i_term[i] = 0;
        pre_position[i] = 0;
        x1[i] = 0;
        x2[i] = 0;
        unwrapped_position[i] = 0;
        velocity_set_point[i] = 0;
        position_set_point[i] = 0;
        position_enable[i] = 1;
        velocity_enable[i] = 1;
    }
    unwrapped_position[2] = 410;
    z_before = 410;
    velocity_set_point[2] = 0;
    velocity_enable[2] = 1;

    // T4CONbits.TON = 1;
    // T5CONbits.TON = 1;
}

void trajectory_gen(unsigned int x, unsigned int y, unsigned int z)
{
    if (trajectory_finish_move)
    {
        float t = 0;
        float setpoint[3] = {(float)x, (float)y, (float)z};
        int i = 0;
        float ds_set[3] = {0, 0, 0};
        float tf = 0;
        for (i = 0; i < 3; i++)
        {
            if (i >= 1.5)
            {
                ds_set[i] = setpoint[i] - z_before;
            }
            else
            {
                ds_set[i] = setpoint[i] - unwrapped_position[i];
            }
            tf = abs(ds_set[i]) / 42;
            if (tf > t)
            {
                t = tf;
            }
        }
        if (t < 1.5)
        {
            t = 1.5;
        }
        float t_pow3 = t * t * t;
        for (i = 0; i < 2; i++)
        {
            co_trajectory[0][i] = unwrapped_position[i];
            co_trajectory[1][i] = ((3 * t * (ds_set[i])) / (t_pow3));
            co_trajectory[2][i] = ((-2 * (ds_set[i])) / (t_pow3));
            velocity_set_point[i] = 0; // max 90 mm/s
            position_set_point[i] = 0; // max 430 mm
            position_enable[i] = 1;
        }
        co_trajectory[0][2] = z_before;
        co_trajectory[1][2] = ((3 * t * (ds_set[2])) / (t_pow3));
        co_trajectory[2][2] = ((-2 * (ds_set[2])) / (t_pow3));
        velocity_set_point[2] = 0; // max 90 mm/s
        position_set_point[2] = 0; // max 430 mm
        position_enable[2] = 1;
        // printf("%.2f %.2f %.2f\n",z_before,setpoint[2],ds_set[2]);
        z_before = z;
        trajectory_time_set = t;
        trajectory_finish_move = 0;

        // unwrapped_position[2] = (float)z;
        // T2CONbits.TON = 1;
    }
}

void Griper(char input)
{
    griper_status = input;
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
    Em_Stop = 1;
    IFS0bits.INT0IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    if (set_home_x_f)
    {
        set_home_x_f = 0;
    }
    IFS1bits.INT1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
{
    if (set_home_y_f)
    {
        set_home_y_f = 0;
    }
    IFS1bits.INT2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    float aa, bb, cc, dd, ee;         // for print debug
    static float sigma_a[2] = {6, 6}; // adjustable
    static float sigma_w[2] = {1, 1}; // adjustable
    static float p11[2] = {1.0, 1.0}; // adjustable
    static float p12[2] = {0, 0};
    static float p21[2] = {0, 0};
    static float p22[2] = {1.0, 1.0}; // adjustable
    static float pre_position_error[2] = {0, 0}, position_i_term[2] = {0, 0}, cmd_velocity[2] = {0, 0};
    static float position_kp[2] = {4, 4}, position_ki[2] = {0, 0}, position_kd[2] = {0, 0};
    static float pwm_output[2] = {0, 0};
    static int velocity_kp[2] = {450, 150}, velocity_ki[2] = {25, 8}, velocity_kd[2] = {390, 94};
    static float trajectory_time = 0;
    float position_error[2] = {0, 0};

    long int position[2] = {(long int)POS1CNT, (long int)POS2CNT}; //read data form encoder

    //calculation command part

    float v[2] = {0, 0};
    float trajectory_time_pow2 = trajectory_time * trajectory_time;
    float trajectory_time_pow3 = trajectory_time_pow2 * trajectory_time;
    int i = 0;
    for (i = 0; i < 2; i++)
    {
        if (!trajectory_finish_move)
        {
            // Trajectory motion
            position_set_point[i] = co_trajectory[0][i] + (co_trajectory[1][i] * trajectory_time_pow2) + (co_trajectory[2][i] * trajectory_time_pow3);
            velocity_set_point[i] = (2 * co_trajectory[1][i] * trajectory_time) + (3 * co_trajectory[2][i] * trajectory_time_pow2);
        }

        long int ds = position[i] - pre_position[i]; //find delta s
        //Unwrapping position
        if (ds >= 32768)
        {
            ds -= 65535;
        }
        else if (ds <= -32768)
        {
            ds += 65535;
        }
        float ds_mm = (float)ds / 102.4; // change pulse to mm
        v[i] = ds_mm / T_speed;          // velocity [mm/s]
        unwrapped_position[i] += ds_mm;  // update new position

        // kaman filter By -----> pokpong.c

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
        // float position_error = position_set_point[i] - unwrapped_position[i];
        // position_i_term[i] += position_error;
        // float position_d_term = position_error - pre_position_error[i];
        // cmd_velocity[i] = (position_kp[i] * position_error) + (position_ki[i] * position_i_term[i]) + (position_kd[i] * position_d_term);
        // update variable of position control
        //     pre_position_error[i] = position_error;

        position_error[i] = position_set_point[i] - unwrapped_position[i];
        cmd_velocity[i] = (position_kp[i] * position_error[i]);

        // calculation velocity_cmd = (velocity feedforward) + (velocity output form position control)

        float velocity_input = velocity_set_point[i] + position_enable[i] * cmd_velocity[i];
        if (velocity_input >= 90.0)
        {
            velocity_input = 90.0;
        }
        else if (velocity_input <= -90.0)
        {
            velocity_input = -90.0;
        }

        // velocity control //
        float velocity_error = velocity_input - x2[i];
        velocity_i_term[i] += velocity_error;
        float velocity_d_term = velocity_error - pre_velocity_error[i];
        pwm_output[i] = (velocity_kp[i] * velocity_error) + (velocity_ki[i] * velocity_i_term[i]) + (velocity_kd[i] * velocity_d_term);
        aa = pwm_output[i];
        //update variable of velocity control
        pre_velocity_error[i] = velocity_error;
    }
    unwrapped_position[2] = co_trajectory[0][2] + (co_trajectory[1][2] * trajectory_time_pow2) + (co_trajectory[2][2] * trajectory_time_pow3);
    velocity_set_point[2] = (2 * co_trajectory[1][2] * trajectory_time) + (3 * co_trajectory[2][2] * trajectory_time_pow2);

    //end of calculation command part

    // drive command
    if (velocity_enable[2])
    {
        if (abs(velocity_set_point[2]) >= 1.8)
        {
            unsigned int buffer = (unsigned int)(100000/ abs(velocity_set_point[2]));
            PR3 = buffer;
            OC3RS = buffer / 2;
            // printf("%.2f %.2f\n", velocity_set_point[2], buffer);
            if (velocity_set_point[2] > 0)
            {
                _LATB13 = 1; //up
            }
            else
            {
                _LATB13 = 0; //down
            }
        }
        else
        {
            PR3 = 0;
            OC3RS = 0;
        }
    }

    if (!PR3)
    {
        _LATB13 ^= 1;
    }

    if (pwm_output[0] >= PWM_period)
    {
        pwm_output[0] = PWM_period;
    }
    else if (pwm_output[0] <= -PWM_period)
    {
        pwm_output[0] = -PWM_period;
    }
    if (pwm_output[1] >= PWM_period)
    {
        pwm_output[1] = PWM_period;
    }
    else if (pwm_output[1] <= -PWM_period)
    {
        pwm_output[1] = -PWM_period;
    }

    if (velocity_enable[0])
    {
        if (abs(pwm_output[0] <= 800 && abs(position_error[0]) <= 2) && trajectory_finish_move)
        {
            velocity_i_term[0] = 0;
            _LATA4 = 1;
            _LATB4 = 1;
        }
        if (pwm_output[0] >= 0)
        {
            OC1RS = (unsigned int)pwm_output[0];
            _LATA0 = 1;
            _LATA1 = 0;
        }
        else
        {
            OC1RS = (unsigned int)abs(pwm_output[0]);
            _LATA0 = 0;
            _LATA1 = 1;
        }
    }

    if (velocity_enable[1])
    {
        if (abs(pwm_output[1] <= 800 && abs(position_error[1]) <= 2) && trajectory_finish_move)
        {
            velocity_i_term[1] = 0;
            _LATA4 = 1;
            _LATB4 = 1;
        }
        else if (pwm_output[1] >= 0)
        {
            OC2RS = (unsigned int)pwm_output[1];
            _LATA4 = 1;
            _LATB4 = 0;
        }
        else
        {
            OC2RS = (unsigned int)abs(pwm_output[1]);
            _LATA4 = 0;
            _LATB4 = 1;
        }
    }

    //update time trajectory
    if (!trajectory_finish_move)
    {
        trajectory_time += T_speed;
    }
    if (trajectory_time > trajectory_time_set)
    {
        trajectory_time = 0;
        trajectory_finish_move = 1;
        // T2CONbits.TON = 0;
        // _LATA0 = 1;
        // _LATA1 = 1;
        // _LATA4 = 1;
        // _LATB4 = 1;
        // printf("ok\n");
    }

    //debug
    //    a = x1[0];
    //    b = unwrapped_position[0];
    a = v[1];
    b = x2[1];
    c = aa;
    d = bb;
    //    e = position_set_point[0];
    _T4IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void)
{
    //    print_f = 1;
    // printf("%.2f %.2f %.2f %.2f %.2f \n", a, b, c, d, e);
    //        printf("%.2f\n", e);
    //    printf("%u %u\n", POS1CNT, POS2CNT);
    // printf("%u %u %u \n", PR3, OC3RS, _LATB13);
    // sprintf(BufferB, "Error cmd\n");
    // DMA0STA = __builtin_dmaoffset(BufferB);
    // DMA0CONbits.CHEN = 1;  // Re-enable DMA0 Channel
    // DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
    _T5IF = 0; //clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
    static char stage = 0;
    switch (BufferA[0])
    {
    case 0xFF:
        switch (BufferA[7])
        {
        case 0xFF:
            sethomef = 1;
            break;
        case 0xEE:
            sprintf(BufferB, "%.2f %.2f %.2f 0 %d\n", unwrapped_position[0] - 10, unwrapped_position[1] - 10, z_before - 15, trajectory_finish_move);
            dma_print();
            break;
        default:
            sprintf(BufferB, "Error cmd\n");
            dma_print();
        }
        break;
    case 0x99:
        if(stage == 4){
            stage = 0;
            update_position_trajectory = 1;
        }
        break;
    case 0xFD:
        if (trajectory_finish_move && stage == 0)
        {
            x = (BufferA[1] << 8) | (BufferA[2]) + 10;
            y = (BufferA[3] << 8) | (BufferA[4]) + 10;
            z = (BufferA[5] << 8) | (BufferA[6]) + 10;
            sprintf(BufferB, "%d %d %d\n", x - 10, y - 10, z - 10);
            dma_print();
            stage = 4;
        }
        else
        {
            sprintf(BufferB, "can't move\n");
            dma_print();
        }
        break;
    case 0xFC:
        if (BufferA[6] == 0xFF)
        {
            if (BufferA[7] == 0xFF)
            {
                Griper(1);
            }
            else if (BufferA[7] == 0x00)
            {
                Griper(0);
            }
        }
        if (BufferA[1] == 0xFF)
        {
            int theta = (BufferA[2] << 8) | (BufferA[3]);
            // something about thetas
        }

        break;
    default:
        sprintf(BufferB, "Error Type\n");
        dma_print();
    }
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{
    IFS4bits.U1EIF = 0; // Clear the UART2 Error Interrupt Flag
}

void initPLL()
{
    PLLFBD = 150;           // M  = 152
    CLKDIVbits.PLLPRE = 5;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001)
        ; // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1)
    {
    }; // Wait for PLL to lock
}

void start_program(void)
{
    __builtin_disable_interrupts();

    initPLL();
    T1CONbits.TCKPS = t1_prescaler;
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 Motor
    T3CONbits.TCKPS = 0b01; //set timer prescaler to 1:8 servo
    T4CONbits.TCKPS = t4_prescaler;
    T5CONbits.TCKPS = t5_prescaler;
    PR1 = t1_period;
    PR2 = PWM_period;
    PR3 = 0;
    PR4 = t4_period;
    PR5 = t5_period;

    OC1RS = 0;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0;  //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC2RS = 0;
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 0;  //OC2 use timer2 as counter source
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC3RS = 0;
    OC3CONbits.OCM = 0b000; //Disable Output Compare Module
    OC3CONbits.OCTSEL = 1;  //OC3 use timer3 as counter source
    OC3CONbits.OCM = 0b110; //set to pwm without fault pin mode

    OC4RS = SERVO_period / 2;
    OC4CONbits.OCM = 0b000; //Disable Output Compare Module
    OC4CONbits.OCTSEL = 1;  //OC4 use timer3 as counter source
    OC4CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    _QEA2R = 9;                             //remap connect to QEI1_A
    _QEB2R = 8;                             //remap connect to QEI1_B
    _QEA1R = 10;                            //remap connect to QEI2_A
    _QEB1R = 11;                            //remap connect to QEI2_B
    _RP0R = 0b10010;                        //remap connect to OC1
    _RP1R = 0b10011;                        //remap connect to OC2
    _RP12R = 0b10100;                       //remap connect to OC3
    _RP15R = 0b10101;                       //remap connect to OC4
    _INT1R = 2;                             //remap external interrupts1
    _INT2R = 3;                             //remap external interrupts2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    QEI1CONbits.QEIM = 0b000;  // QEI1 Mode disable
    QEI1CONbits.PCDOUT = 0;    // no direction pin out
    QEI1CONbits.QEIM = 0b101;  // 2x ,no index
    DFLT1CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1;    // QEI1 Enable digital filter

    QEI2CONbits.QEIM = 0b000;  // QEI2 Mode disable
    QEI2CONbits.PCDOUT = 0;    // no direction pin out
    QEI2CONbits.QEIM = 0b101;  // 2x ,no index
    DFLT2CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1;    // QEI2 Enable digital filter

    
    U1MODEbits.STSEL = 0;   // 1 Stop bit
    U1MODEbits.PDSEL = 0;   // No Parity, 8 data bits
    U1MODEbits.BRGH = 1;    // High Speed mode
    U1MODEbits.URXINV = 0;  // UxRX idle state is '1'

    U1BRG = BRGVAL;         // BAUD Rate Setting for 2000000
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1;   // Enable UART Tx
    IEC4bits.U1EIE = 0;

    DMA0REQ = 0x000C; // Select UART1 Transmitter
    DMA0PAD = (volatile unsigned int)&U1TXREG;
    DMA0CONbits.AMODE = 0;
    DMA0CONbits.MODE = 1;
    DMA0CONbits.DIR = 1;
    DMA0CONbits.SIZE = 1;
    DMA0CNT = 14; // 15 DMA requests
    DMA0STA = __builtin_dmaoffset(BufferB);
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

    DMA1REQ = 0x000B; // Select UART1 Receiver
    DMA1PAD = (volatile unsigned int)&U1RXREG;
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 0;
    DMA1CONbits.DIR = 0;
    DMA1CONbits.SIZE = 1;
    DMA1CNT = 7; // 8 DMA requests
    DMA1STA = __builtin_dmaoffset(BufferA);
    IFS0bits.DMA1IF = 0;  // Clear DMA interrupt
    IEC0bits.DMA1IE = 1;  // Enable DMA interrupt
    DMA1CONbits.CHEN = 1; // Enable DMA Channel

    IEC0 |= 0x0001;    //enable interrupts0
    IEC1 |= 0x2010;    //enable interrupts1,2
    INTCON2 &= 0xFFFF; //Edge detection
    _INT0IP = 7;       //priority
    _INT1IP = 6;       //priority
    _INT2IP = 6;       //priority

    _T1IE = 1; //enable interrupt for timer1
    _T4IE = 1; //enable interrupt for timer4
    _T5IE = 1; //enable interrupt for timer5
    _T1IP = 5; //priority interrupt for timer1
    _T4IP = 5; //priority interrupt for timer4
    _T5IP = 1; //priority interrupt for timer5

    AD1PCFGL = 0xFFFF; //set analog input to digital pin
    TRISB = 0x0FEC;
    TRISA = 0xFFEC;
    __builtin_enable_interrupts();
    T2CONbits.TON = 1; //enable timer2
    T3CONbits.TON = 1; //enable timer3
    T4CONbits.TON = 1; //enable timer4
    // T5CONbits.TON = 1; //enable timer5

    _LATA0 = 0;
    _LATA1 = 0;
    _LATA4 = 0;
    _LATB4 = 0;
}

int main(void)
{
    start_program();
    printf("start\n");
    //    T4CONbits.TON = 1;
    // set_home();
    while (1)
    {
        if (sethomef)
        {
            _LATA0 = 0;
            _LATA1 = 0;
            _LATA4 = 0;
            _LATB4 = 0;
            sethomef = 0;
            set_home();
        }

        if (update_position_trajectory)
        {
            update_position_trajectory = 0;
            trajectory_gen(x, y, z);
            x = 0;
            y = 0;
            z = 0;
        }

        //        velocity_set_point[1] = 45; // max 90 mm/s
        //        position_set_point[1] = 0; // max 430 mm
        //        position_enable[1] = 0;

        if (Em_Stop)
        {
            Em_Stop = 0;
            printf("Em \n");
        }
    }
    return 0;
}