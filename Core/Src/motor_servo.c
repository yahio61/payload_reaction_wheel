#include "motor_servo.h"
#include <math.h>

// Global değişkenler
static TIM_HandleTypeDef *pwm_timer;
static PID_Controller motor_pid;
static uint32_t last_time = 0;

float error = 0;
uint32_t pulse = 0;

// Motor servo sistemini başlat
void Motor_Servo_Init(TIM_HandleTypeDef *htim_pwm)
{
    pwm_timer = htim_pwm;
    
    // PID parametrelerini ayarla (ihtiyacınıza göre ayarlayın)
    PID_Init(&motor_pid, KP, KI, KD);
    
    // PWM'i başlat (MD_PWM kanalını kullanıyoruz)
    HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_3); // Kanal numarasını kendi setup'ınıza göre değiştirin

    // İlk zamanı al
    last_time = HAL_GetTick();
}

// Ana servo kontrol fonksiyonu
void Motor_Servo_Control(float current_angle, float target_angle)
{
    // Zaman farkını hesapla
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Saniye cinsinden
    last_time = current_time;
    
    // Açı farkını hesapla
    error = target_angle - current_angle;
    
    // Açı farkını -180 ile +180 arasına normalize et
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    
    // PID hesapla
    float pid_output = PID_Calculate(&motor_pid, error, dt);
    
    // Dead zone - küçük hatalar için motoru durdur
    if (fabs(error) < 1.0f) {
        Motor_Stop();
        return;
    }
    
    // Motor yönünü ve hızını ayarla
    if (pid_output > 0) {
        Motor_Set_Direction(MOTOR_FORWARD);
        Motor_Set_Speed((uint16_t)fabs(pid_output));
    } else if (pid_output < 0) {
        Motor_Set_Direction(MOTOR_BACKWARD);
        Motor_Set_Speed((uint16_t)fabs(pid_output));
    } else {
        Motor_Stop();
    }
}

// Motor yönünü ayarla
void Motor_Set_Direction(Motor_Direction direction)
{
    switch (direction) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(MD_INA_PORT, MD_INA_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MD_INB_PORT, MD_INB_PIN, GPIO_PIN_SET);

            break;
            
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(MD_INA_PORT, MD_INA_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MD_INB_PORT, MD_INB_PIN, GPIO_PIN_RESET);
            break;
            
        case MOTOR_STOP:
        default:
            HAL_GPIO_WritePin(MD_INA_PORT, MD_INA_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MD_INB_PORT, MD_INB_PIN, GPIO_PIN_RESET);
            break;
    }
}

// Motor hızını ayarla (0-1000 arası değer)
void Motor_Set_Speed(uint16_t speed)
{
    // Hızı sınırla
    if (speed > 1000) speed = 1000;
    
    // PWM duty cycle'ını ayarla
    pulse = (speed * (pwm_timer->Init.Period)) / 1000;
    __HAL_TIM_SET_COMPARE(pwm_timer, TIM_CHANNEL_3, pulse);
}

// Motoru durdur
void Motor_Stop(void)
{
    Motor_Set_Direction(MOTOR_STOP);
    Motor_Set_Speed(0);
}

// PID controller'ı başlat
void PID_Init(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = 1000.0f;  // Maksimum motor hızı
    pid->min_output = -1000.0f; // Minimum motor hızı
}

// PID hesaplama
float PID_Calculate(PID_Controller *pid, float error, float dt)
{
    // Proportional
    float proportional = pid->kp * error;
    
    // Integral
    pid->integral += error * dt;
    float integral = pid->ki * pid->integral;
    
    // Derivative
    float derivative = 0.0f;
    if (dt > 0) {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;
    
    // Toplam çıkış
    float output = proportional + integral + derivative;
    
    // Çıkışı sınırla
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    return output;
}
