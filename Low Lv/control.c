//        motor_driveX(100);
static float t = 0;
#define T_speed 0.02
#define a3 0.0750
#define a4 0.0025






float point_tra = (a3 * t * t) + (a4 * t * t * t);
float v_tra = (a3 * 2) + (3 * a4) + (t * t);


static float sigma_a[2] = {10, 10}; // adjustable
static float sigma_w[2] = {1, 1};   // adjustable
static float Q[2] = {0, 0}, R[2] = {0, 0};
static float x1_in[2] = {0, 0};
static float x2_in[2] = {0, 0};
static float x1[2] = {0, 0};  // orientation
static float x2[2] = {0, 0};  // w_outKalman (angular_vel)
static float p11[2] = {1, 1}; // adjustable
static float p12[2] = {0, 0};
static float p21[2] = {0, 0};
static float p22[2] = {1.0, 1.0}; // adjustable
static long int pre_position[2] = {0, 0};
static float un_position = 0.0;

long int ds[2] = {0, 0};
float v[2] = {0,0};
int i = 0;

long int position[2] = {(long int)POS1CNT, (long int)POS2CNT};
ds[i] = position[i] - pre_position[i];

if (ds[i] >= 32768)
{
    ds[i] -= 65535;
}
else if (s <= -32768)
{
    ds[i] += 65535;
}
float ds_mm = (float)ds[i] / 17.06667; // change pulse to mm
un_position += ds_mm;

v[i]= ds_mm / T_speed; // [mm/s]

Q = sigma_a[i] * sigma_a[i];
R = sigma_w[i] * sigma_w[i];
float x1_new = x1[i] + x2[i] * T_speed;
float x2_new = 0 + x2[i];
float ye = v[i] - x2_new[i];
p11[i] = p11[i] + T_speed * p21[i] + (Q[i] * T_speed * T_speed * T_speed * T_speed) / 4 + (T_speed * T_speed * (p12[i] + T_speed * p22[i])) / T_speed;
p12[i] = p12[i] + T_speed * p22[i] + (Q[i] * T_speed * T_speed * T_speed) / 2;
p21[i] = (2 * T_speed * p21[i] + Q[i] * T_speed * T_speed + 2 * p22[i] * T_speed * T_speed) / (2 * T_speed);
p22[i] = Q[i] * T_speed * T_speed + p22[i];
x1_new = x1_new + (p12 * ye) / (R + p22[i]);
x2_new = x2_new + (p22 * ye) / (R + p22[i]);
p11 = p11[i] - (p12[i] * p21[i]) / (R + p22[i]);
p12 = p12[i] - (p12[i] * p22[i]) / (R + p22[i]);
p21 = -p21[i] * (p22[i] / (R + p22[i]) - 1);
p22 = -p22[i] * (p22[i] / (R + p22[i]) - 1);
x1[i] = x1_new;
x2[i] = x2_new;
// x1_in = x1;
// x2_in = x2;

static long int pre_x_error = 0, i_term_x = 0;
long int error_x = (long int)(x_pos - (long int)POS1CNT);
i_term_x = error_x + i_term_x;
long int d_term_x = error_x - pre_x_error;
float v_PID = (0.015 * (float)error_x + 0 * i_term_x + 0.001 * d_term_x);
static float i_term_vx = 0;
static float v_setpoint = 0;
v_setpoint = v_PID;
if (v_setpoint == 0.0)
{
    i_term_vx = 0;
}

//    static float vx_pwm =0;
float error_vx = v_setpoint - x2;
float d_term_vx = error_vx - pre_vx_error;
i_term_vx = error_vx + i_term_vx;
float vx_pwm = (35 * error_vx + 1 * i_term_vx + 8 * d_term_vx);
//    vx_pwm += vx_pwm_real;
printf("%.2f \n", v_PID);
if ((long int)vx_pwm >= PWM_period)
{
    vx_pwm = PWM_period;
}
else if ((long int)vx_pwm <= -PWM_period)
{
    vx_pwm = PWM_period;
}

//    if (abs(vx_pwm) <= 10) {
//        i_term_vx = 0;
//    }

if (vx_pwm >= 0)
{
    OC1RS = (unsigned int)abs(vx_pwm);
    _LATA0 = 1;
    _LATA1 = 0;
}
else
{
    OC1RS = (unsigned int)abs(vx_pwm);
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