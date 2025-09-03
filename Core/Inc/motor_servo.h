#ifndef MOTOR_SERVO_H
#define MOTOR_SERVO_H

#include "stm32f4xx_hal.h"  // STM32 modelinize göre değiştirin
#include <stdbool.h>
#include "main.h"


// Kullanici parametreleri
#define KP				(float) 2
#define KI				(float) 0.01
#define KD				(float) 0
#define TARGET_ANGLE	(float) 0



// Motor kontrol pinleri - CubeIDE'de tanımladığınız adlar
#define MD_INA_PIN    MD_IN_A_Pin
#define MD_INA_PORT   MD_IN_A_GPIO_Port
#define MD_INB_PIN    MD_IN_B_Pin
#define MD_INB_PORT   MD_IN_B_GPIO_Port



// PID kontrol parametreleri
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral toplam
    float prev_error;   // Önceki hata
    float max_output;   // Maximum çıkış değeri
    float min_output;   // Minimum çıkış değeri
} PID_Controller;

// Motor yönleri
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} Motor_Direction;

// Fonksiyon prototipleri
void Motor_Servo_Init(TIM_HandleTypeDef *htim_pwm);
void Motor_Servo_Control(float current_angle, float target_angle);
void Motor_Set_Direction(Motor_Direction direction);
void Motor_Set_Speed(uint16_t speed);
void Motor_Stop(void);
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Calculate(PID_Controller *pid, float error, float dt);

#endif // MOTOR_SERVO_H
