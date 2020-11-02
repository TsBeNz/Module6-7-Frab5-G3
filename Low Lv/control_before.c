//        motor_driveX(100);
    static float t = 0;
    
    #define a3 0.0750
    #define a4 0.0025
    
    float point_tra = (a3*t*t)+(a4*t*t*t);
    float v_tra = (a3*2)+(3*a4)+(t*t);
    int i = 0;
    static float sigma_a[2] = {10,10}; // adjustable
    static float sigma_w[2] = {1,1}; // adjustable
    static float Q[2] = {0,0}, R[2] = {0,0};
    static float x1_in[2] = {0,0};
    static float x2_in[2] = {0,0};
    static float x1[2] = {0,0}; // orientation
    static float x2[2] = {0,0}; // w_outKalman (angular_vel)
    static float p11[2] = {1,1}; // adjustable
    static float p12[2] = {0,0};
    static float p21[2] = {0,0};
    static float p22[2] = {1.0,1.0}; // adjustable
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
    float x1_new = x1_in[i] + x2_in[i]*T_speed;
    float x2_new = 0 + x2_in[i];
    float ye = v - x2_new[i];
    p11[i] = p11 + T_speed * p21 + (Q * T_speed * T_speed * T_speed * T_speed) / 4 + (T_speed * T_speed * (p12[i] + T_speed * p22[i])) / T_speed;
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