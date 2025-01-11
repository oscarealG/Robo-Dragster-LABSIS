#define F_CPU 16000000UL // Frequência do ATmega328p (16 MHz)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2c.h"
// #include "ssd1306.h"
#include <stdbool.h>
#include "L3GD20.h"
#include "UART.h"
#include "LSM303DLH.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int16_t gyro_X = 0;
int16_t gyro_Y = 0;
int16_t gyro_Z = 0;
int16_t accel_X = 0;
int16_t accel_Y = 0;
int16_t accel_Z = 0;
int16_t mag_X = 0;
int16_t mag_Y = 0;
int16_t mag_Z = 0;

float real_gyro_X = 0.0;
float real_gyro_Y = 0.0;
float real_gyro_Z = 0.0;
float real_accel_X = 0.0;
float real_accel_Y = 0.0;
float real_accel_Z = 0.0;
float real_mag_X = 0.0;
float real_mag_Y = 0.0;
float real_mag_Z = 0.0;

int16_t real_accel_X_I = 0;
int16_t real_accel_X_D = 0;
int16_t real_accel_Y_I = 0;
int16_t real_accel_Y_D = 0;
int16_t real_accel_Z_I = 0;
int16_t real_accel_Z_D = 0;
//
int16_t real_gyro_X_I = 0;
int16_t real_gyro_X_D = 0;
int16_t real_gyro_Y_I = 0;
int16_t real_gyro_Y_D = 0;
int16_t real_gyro_Z_I = 0;
int16_t real_gyro_Z_D = 0;
//
int16_t max_accel_X = INT16_MIN, max_accel_Y = INT16_MIN, max_accel_Z = INT16_MIN;
int16_t min_accel_X = INT16_MAX, min_accel_Y = INT16_MAX, min_accel_Z = INT16_MAX;
//
float gyroHeading = 0;
float PitchAngle = 0;
float RollAngle = 0;
//
// float dt = 0.1;
// float PI = 3.14;
// float teste = 0.0;

int8_t LDR = 6;
char buffer[100];
#define BUFFER_SIZE 10 // Tamanho suficiente para armazenar os números convertidos

volatile uint32_t elapsedTime = 0; // Variável global para armazenar o tempo em milissegundos
volatile uint32_t StartingTime = 0;
volatile uint32_t lastSampleTime = 0;
volatile uint32_t previous_FailSafe_White = 0;
volatile uint32_t previous_FailSafe_Black = 0;
volatile uint32_t delay_FailSafe_White = 75; // tempo minimo de deteção de branco para parar (todos os sensores a ler branco)(Provavelmente meio da pista)
volatile uint32_t delay_FailSafe_Black = 10; // tempo minimo de deteção da linha preta para parar(todos os sensores a ler preto)(Provavelmente linha da meta)

// rampa para aumetar PWM
bool isToIncrement = false;
bool isToCancel_White = false;
bool isToCancel_Black = false;
unsigned long previous_Increment = 0;
const long timeForIncrement = 70;

int motorSpeed = 0;
int MaxMotorSpeed = 40;

size_t i = 0;

volatile bool estado_sinal = false; // Estado do sinal PWM
volatile uint8_t largura_pulso = 1; // Largura do pulso em ms (1-2 ms)
volatile uint8_t timerCount = 0;    // Contador de tempo em ms

bool pid_compute_timer = false;

typedef struct
{
    float kp;         // proporcional
    float ki;         // integral
    float kd;         // derivativa
    float setpoint;   // Valor desejado
    float last_error; // Erro anterior
    float integral;   // Soma dos erros (componente integral)
    float output_min; // Saida minima
    float output_max; // Saida maxima
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0;
    pid->last_error = 0.0;
    pid->integral = 0.0;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

float pid_compute(PIDController *pid, float measured_value, float dt)
{
    // Calcula o erro
    float error = pid->setpoint - measured_value;

    // Componente proporcional
    float proportional = pid->kp * error;

    // Componente integral
    pid->integral += error * dt;
    float integral = pid->ki * pid->integral;

    // Componente derivativo
    float derivative = pid->kd * (error - pid->last_error) / dt;

    // Atualiza o erro anterior
    pid->last_error = error;

    // Calcula a saída total
    float output = proportional + integral + derivative;

    // Limita a saída para dentro do intervalo permitido
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    return output;
}
PIDController Motor_FR_RPMs;
PIDController Motor_FL_RPMs;
PIDController Motor_B_RPMs;
PIDController Motor_FR_POS;
PIDController Motor_FL_POS;
PIDController Motor_B_POS;
PIDController Motor_FR_Dist;
PIDController Motor_FL_Dist;
PIDController Motor_B_Dist;
float pwm_L = 0.0;
float pwm_R = 0.0;

// INIT timer0
void timer0_init()
{
    // CTC 1ms
    TCCR0A = 0;
    TCCR0B |= (1 << WGM01);
    TCCR0B |= (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 |= (1 << OCIE0A);
}

// Evento da interrupção TIMER0_COMPA_vect
ISR(TIMER0_COMPA_vect)
{
    elapsedTime++; // a cada 1ms
}

// INIT TIMER1 para 50Hz, 1ms = 0% 2ms = 100%,
void init_pwm(void)
{
    // Motores da frente Timer 1
    //  Configura os pinos OC1A (PB1)(D9) e OC1B (PB2)(D10) como saída
    DDRB |= (1 << PORTB1) | (1 << PORTB2);

    // Configura o Timer 1 para Fast PWM, modo 14 (Top = ICR1)
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1); //| (1 << COM1A0) | (1<< COM1B0); // OC1A e OC1B set on compare match (inverting mode)
    TCCR1A |= (1 << WGM11);                  // Set Fast PWM junto com TCCR1B
    TCCR1B |= (1 << WGM12) | (1 << WGM13);   // Fast PWM (MODO 14)
    TCCR1B |= (1 << CS12);                   // Prescaler de 1024
    ICR1 = 1249;
    // Configura OCR1A e OCR1B para 1ms (50 Hz, 20ms)
    OCR1A = 61;
    OCR1B = 61;
}

void init_timer2_pwm(void)
{
    // Motor traseiro Timer 2
    // pinos OC2B (PB3)(D11) como saída
    DDRD |= (1 << PORTD3);

    TCCR2A = (1 << WGM21);              // Modo CTC
    TCCR2B = (1 << CS22) | (1 << CS21); // Prescaler de 256 (64 us por tick)
    OCR2A = 61;                         // 1 ms
    TIMSK2 = (1 << OCIE2A);             // Habilita interrupção de comparação do Timer 2

    sei();
}

// Função que atualiza PWM (OCR), 61 = 0% 125 = 100%, motores frente direita e frente esquerda
void set_PWM_FR_FL_Motors(uint16_t FR_PWM, uint16_t FL_PWM)
{
    // Canal A (OC1A)
    if (FR_PWM < 61)
        FR_PWM = 61; // 1ms 0%
    if (FR_PWM > 125)
        FR_PWM = 125; // 2ms 100%
    OCR1A = FR_PWM;   // Atualiza OCR1A

    // Canal B (OC1B)
    if (FL_PWM < 61)
        FL_PWM = 61; // 1ms 0%
    if (FL_PWM > 125)
        FL_PWM = 125; // 2ms 100%
    OCR1B = FL_PWM;   // Atualiza OCR1B
}

uint8_t delay_contagem = 0;
// Evento da interrupção PCINT0_vect PWM para motor traseiro (ON/OFF) , calculo RPMs
ISR(TIMER2_COMPA_vect)
{
    timerCount++;
    delay_contagem++;
    if (estado_sinal == false) // acontece a cada 20ms (19x\18x no ISR)
    {
        if (timerCount == 20)        // acontece a cada 20ms (1x no ISR)
        {                            // Após 20 ms, vai para HIGH
            PORTD |= (1 << PORTD3);  // Sinal HIGH
            estado_sinal = true;     // Alterna o estado do sinal
            timerCount = 0;          // Reseta o contador
            if (delay_contagem >= 2) // 2x 20ms
            {
                RPMs(40); // Calcula RPMs a cada 40ms
                pid_compute_timer = true;
                delay_contagem = 0;
            }
        }
    }
    else
    { // acontece a cada 20ms (1x\2x no ISR)

        if (timerCount >= largura_pulso) // acontece a cada 20ms\19ms (1x no ISR)
        {                                // Após a largura do pulso (1-2 ms)
            PORTD &= ~(1 << PORTD3);     // Sinal LOW
            estado_sinal = false;        // Alterna o estado do sinal
        }
    }
}

// Função delay
void delay_ms(uint32_t ms)
{
    uint32_t startTime = elapsedTime;
    while ((elapsedTime - startTime) < ms)
    {
        ;
    }
}

// Função converte FLOAT para dois inteiros [função foi usada para debug] [tem um "BUG" caso num = -0.5 -> parte_inteira = 0  parte_decimal = -5 de outra forma não saberia o sinal de num]
void float_2Int(float num, int16_t *parte_inteira, int16_t *parte_decimal)
{
    float inteiro;
    float decimal;

    decimal = modff(num, &inteiro);
    *parte_inteira = (int16_t)inteiro;

    if ((int16_t)inteiro == 0)
        *parte_decimal = (int16_t)((decimal) * 1000);
    else
        *parte_decimal = (uint16_t)(fabs(decimal) * 1000);
}

volatile bool current_state_FL = false;
volatile bool current_state_FR = false;
volatile bool current_state_B = false;
volatile bool last_state_FL = false;
volatile bool last_state_FR = false;
volatile bool last_state_B = false;
volatile float ticks_counter_FL = 0; // Nr absoluto de pulsos do encoder
volatile float ticks_counter_FR = 0; // Nr absoluto de pulsos do encoder
volatile float ticks_counter_B = 0;  // Nr absoluto de pulsos do encoder
volatile float ticks_FL = 0;         // Nr de pulsos para cada encoder variavel para calculo
volatile float ticks_FR = 0;         // Nr de pulsos para cada encoder variavel para calculo
volatile float ticks_B = 0;          // Nr de pulsos para cada encoder variavel para calculo
volatile float RPMs_FL = 0;          // RPMs de cada roda
volatile float RPMs_FR = 0;          // RPMs de cada roda
volatile float RPMs_B = 0;           // RPMs de cada roda

// INIT interrupções dos encoders
void init_interrupts()
{
    DDRB &= ~(1 << PB4);
    DDRD &= ~(1 << PD7);

    PORTB |= (1 << PB4);
    PORTD |= (1 << PD7);

    PCICR |= (1 << PCIE0) | (1 << PCIE2);
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT0);
    PCMSK2 |= (1 << PCINT23);

    EICRA |= (1 << ISC01) | (1 << ISC00);
    EIMSK |= (1 << INT0);
}

// Evento da interrupção PCINT0_vect incrementa contador roda frente Esquerda
ISR(PCINT0_vect)
{
    current_state_FR = PINB & (1 << PB4);

    if (current_state_FR && !last_state_FR)
    {
        ticks_counter_FR++;
    }

    last_state_FR = current_state_FR;
}

// Evento da interrupção PCINT0_vect incrementa contador roda frente direita
ISR(PCINT2_vect)
{
    if (!(PIND & (1 << PD7)))
    {
        ticks_counter_FL++;
    }
}

// Evento da interrupção PCINT2_vect incrementa contador traseira
ISR(INT0_vect)
{
    current_state_B = PIND & (1 << PD2);

    if (current_state_B && !last_state_B)
    {
        ticks_counter_B++;
    }

    last_state_B = current_state_B;
}

// Função que calcula RPMs das roda, tem que ser chamada periodicamente 0-255ms //deixou de ser /min e é /seg
void RPMs(uint8_t time_ms)
{
    // RPMs = ticks * 60 -> para um tick por Rev e a cada segundo (1000ms)
    // RPMs = ticks * 60 * 100 -> para um tick por Rev e a cada 10ms
    // ticks = ticks_counter_FL (valor absoluto) - ticks_FL (nr de tick entre medições) / 2 (Nr de tick por rotação)(fisicamente)
    //  60 = multiplicador para saber Rev por minuto
    //(1000/time_ms) = converção para a função funcionar com diferentes periodicidades

    float Rate = (1000 / time_ms);

    RPMs_FL = ((ticks_counter_FL - ticks_FL)) * Rate;
    RPMs_FR = ((ticks_counter_FR - ticks_FR)) * Rate;
    RPMs_B = ((ticks_counter_B - ticks_B)) * Rate;

    ticks_FL = ticks_counter_FL;
    ticks_FR = ticks_counter_FR;
    ticks_B = ticks_counter_B;
}

// INIT do ADC prescaler para 64 (16MHz / 64 = 250kHz)
void ADC_init()
{
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

// Função que retorna valor uint8_t para leitura analogica de channel do ADC
uint16_t ADC_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
    return ADC;
}

// INIT pinos [0,1,2,3] porto C como entrada
void init_sensors()
{
    DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3));
    PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
}

// Função que Retorna valor digital de pino para o porto C
uint8_t Read_Sensor(uint8_t pino)
{
    return (PINC & (1 << pino)) ? true : false;
}

// Função que retorna posição [1..5] de um sensor seguidor de linha com 5 sensores, caso haja mais que um sensor ligado ou nenhum ligado retorna 0
uint8_t Sensors_Pos()
{
    bool estado_A0 = !Read_Sensor(PC0);
    bool estado_A1 = !Read_Sensor(PC1);
    bool estado_A2 = !Read_Sensor(PC2);
    bool estado_A3 = !Read_Sensor(PC3);
    bool estado_A7 = (ADC_read(7) >= 50) ? false : true;

    bool valid = estado_A0 ^ estado_A1 ^ estado_A2 ^ estado_A3 ^ estado_A7;
    if (valid)
    {
        if (estado_A0)
        {
            return 4;
        }
        if (estado_A1)
        {
            return 1;
        }
        if (estado_A2)
        {
            return 2;
        }
        if (estado_A3)
        {
            return 3;
        }
        if (estado_A7)
        {
            return 5;
        }
    }
    else
    {
        return 0;
    }
}
uint8_t Sensors_Pos_inv()
{
    bool estado_A0 = !Read_Sensor(PC0);
    bool estado_A1 = !Read_Sensor(PC1);
    bool estado_A2 = !Read_Sensor(PC2);
    bool estado_A3 = !Read_Sensor(PC3);
    bool estado_A7 = (ADC_read(7) >= 50) ? false : true;

    bool valid = estado_A0 ^ estado_A1 ^ estado_A2 ^ estado_A3 ^ estado_A7;
    if (valid)
    {
        if (estado_A0)
        {
            return 2;
        }
        if (estado_A1)
        {
            return 5;
        }
        if (estado_A2)
        {
            return 4;
        }
        if (estado_A3)
        {
            return 3;
        }
        if (estado_A7)
        {
            return 1;
        }
    }
    else
    {
        return 0;
    }
}

// Função que retorna valor FLOAT compreendido entre newMax e newMin de um valor value que é compreendido entre maxValue e minValue
float fmap(float value, float maxValue, float minValue, float newMax, float newMin)
{
    if (maxValue == minValue)
    {
        return newMin;
    }
    return (value - minValue) * (newMax - newMin) / (maxValue - minValue) + newMin;
}

// Função que retorna valor UINT16_T compreendido entre newMax e newMin de um valor value que é compreendido entre maxValue e minValue
uint16_t imap(uint16_t value, uint16_t maxValue, uint16_t minValue, uint16_t newMax, uint16_t newMin)
{
    if (maxValue == minValue)
    {
        return newMin;
    }
    return (value - minValue) * (newMax - newMin) / (maxValue - minValue) + newMin;
}

// Função de segurança, Sensores todos ligados - isToCancel_White, Sensores todos desligados - isToCancel_Black
void sensorSafety()
{
    bool _estado_A0 = !Read_Sensor(PC0);
    bool _estado_A1 = !Read_Sensor(PC1);
    bool _estado_A2 = !Read_Sensor(PC2);
    bool _estado_A3 = !Read_Sensor(PC3);
    bool _estado_A7 = (ADC_read(7) >= 50) ? true : false;
    sprintf(buffer, "%d,%d,%d,%d,%d\n",

            _estado_A0,
            _estado_A1,
            _estado_A2,
            _estado_A3,
            _estado_A7);

    uart_send_string(buffer);

    if (_estado_A0 && _estado_A1 && _estado_A2 && _estado_A3 && _estado_A7)
    {
        isToCancel_White = true;
    }

    if (!_estado_A0 && !_estado_A1 && !_estado_A2 && !_estado_A3 && !_estado_A7)
    {
        isToCancel_Black = true;
    }
}

int main(void)
{
    uart_init(9600); // Inicializa UART com baudrate de 9600

    timer0_init();
    init_pwm();
    init_timer2_pwm();
    init_interrupts();
    uart_send_string("INIT ");
    // timer2_init();
    sei();

    // I2C_init();

    // LSM303_Accel_Init();
    uart_send_string("LSM303_Accel_Init ");
    delay_ms(100);
    // L3GD20_init();
    uart_send_string("L3GD20_init ");
    delay_ms(100);

    // CalibAccValues(2000);
    uart_send_string("CalibAccValues ");
    delay_ms(150);
    // CalibGyroValues(200);
    uart_send_string("CalibGyroValues ");
    delay_ms(100);

    uart_send_string("done ");

    // Motores frente
    set_PWM_FR_FL_Motors(61, 61); // desligado
    // Motor traseiro
    largura_pulso = 1; // 2 ms

     _delay_ms(5000);
     _delay_ms(1000);

    set_PWM_FR_FL_Motors(61, 61); //  [3º - 2º] [DIR - ESQ]

    ADC_init();
    init_sensors();
    uint16_t adc_result;
    uint8_t sensor_pos_result;
    uint8_t modoCorrida = 0;
    int8_t error = 0;
    int8_t error_inv = 0;

    bool FailSafe_White = false;
    bool FailSafe_Black = false;

    // RPMs de erro 0-80
    pid_init(&Motor_FL_RPMs, 1, 0.5, 0.1, 0, 80); // obj, kP, kI, Kd, min, max
    pid_init(&Motor_FR_RPMs, 1, 0.5, 0.1, 0, 80); // obj, kP, kI, Kd, min, max
    pid_init(&Motor_B_RPMs, 1, 0.5, 0.1, 0, 80);  // obj, kP, kI, Kd, min, max

    // POS de erro 1-5
    pid_init(&Motor_FL_POS, 3, 1, 0.1, 0, 3);  // obj, kP, kI, Kd, min, max
    pid_init(&Motor_FR_POS, 3, 1, 0.1, 0, 3);  // obj, kP, kI, Kd, min, max
    pid_init(&Motor_B_POS, 1, 0.5, 0.1, 1, 5); // obj, kP, kI, Kd, min, max

    // Dist de erro 0-7
    pid_init(&Motor_FL_Dist, 3, 1.0, 0.1, 0, 7); // obj, kP, kI, Kd, min, max
    pid_init(&Motor_FR_Dist, 3, 1.0, 0.1, 0, 7); // obj, kP, kI, Kd, min, max
    pid_init(&Motor_B_Dist, 1, 0.5, 0.1, 0, 7);  // obj, kP, kI, Kd, min, max

    Motor_FR_RPMs.setpoint = 0;
    Motor_FL_RPMs.setpoint = 0;
    Motor_B_RPMs.setpoint = 0;
    Motor_FR_POS.setpoint = 3;
    Motor_FL_POS.setpoint = 3;
    Motor_B_POS.setpoint = 3;
    Motor_FR_Dist.setpoint = 0;
    Motor_FL_Dist.setpoint = 0;
    Motor_B_Dist.setpoint = 0;

    while (1)
    {

        switch (modoCorrida)
        {
        case 0:                           // INIT
            set_PWM_FR_FL_Motors(61, 61); // motores desligados
            modoCorrida = 1;
            break;

        case 1: // Esperar semaforo

            // 500 é mera estimativa pendente de calibração
            // 1020 para evitar maus contactos, sensor desligado - adc_result == 1024
            adc_result = ADC_read(LDR);
            sprintf(buffer, "%d\n",
                    adc_result);

            uart_send_string(buffer);
            if ((adc_result >= 500) && !(adc_result >= 1020))
            {
                // Proximo passo
                 set_PWM_FR_FL_Motors(69, 69); // ligar motores
                modoCorrida = 2;
                StartingTime = elapsedTime;
            }

            break;
        case 2: // passo para sair da linha
                // motores ligados 50ms
            if (elapsedTime - StartingTime >= 50)
            {
                StartingTime = elapsedTime;
                modoCorrida = 3;
            }

            break;

        case 3:
            error = Sensors_Pos();
            error_inv = Sensors_Pos_inv();
            if (error == 0)
            {
                sensorSafety();
                // uart_send_string("sensorSafety\n");
            }
            else
            {
                // modoCorrida = 3;
                Motor_FL_POS.setpoint = 3;
                Motor_FL_POS.setpoint = 3;

                if (pid_compute_timer)
                {
                    pwm_L = pid_compute(&Motor_FL_POS, error, 0.04);
                    pwm_R = pid_compute(&Motor_FR_POS, error_inv, 0.04);
                    // set_PWM_FR_FL_Motors(67 - pwm_R, 67 - pwm_L);
                    pid_compute_timer = false;
                }

                float_2Int(RPMs_FL, &real_gyro_X_I, &real_gyro_X_D);
                float_2Int(RPMs_FR, &real_gyro_Y_I, &real_gyro_Y_D);
                float_2Int(RPMs_B, &real_gyro_Z_I, &real_gyro_Z_D);
                float_2Int(pwm_L, &real_accel_X_I, &real_accel_X_D);
                float_2Int(pwm_R, &real_accel_Y_I, &real_accel_Y_D);

                sprintf(buffer, "%d,%d,%d|%d,%d\n",

                        0,
                        error_inv,
                        error,
                        real_accel_X_I,
                        real_accel_Y_I);

                uart_send_string(buffer);
            }

            break;

        case 255: // desligar tudo
            sprintf(buffer, "FIM - %d,%d\n",

                    FailSafe_White,
                    FailSafe_Black);

            uart_send_string(buffer);
            set_PWM_FR_FL_Motors(61, 61); // motores desligados

            break;
        }

#pragma region FailSafe_White

        if (!isToCancel_White)
        {
            previous_FailSafe_White = elapsedTime;
        }
        if (elapsedTime - previous_FailSafe_White >= delay_FailSafe_White)
        {
            FailSafe_White = true;
            modoCorrida = 255;
            previous_FailSafe_White = elapsedTime;
        }

#pragma endregion comment

#pragma region FailSafe_Black
        if (!isToCancel_Black)
        {
            previous_FailSafe_Black = elapsedTime;
        }
        if (elapsedTime - previous_FailSafe_Black >= delay_FailSafe_Black)
        {
            FailSafe_Black = true;
            modoCorrida = 255;
            previous_FailSafe_Black = elapsedTime;
        }

#pragma endregion comment

#pragma region Incremeto de velociade dos Motores
        if (!isToIncrement)
        {
            previous_Increment = elapsedTime;
        }
        if (elapsedTime - previous_Increment >= timeForIncrement)
        {
            if (motorSpeed <= MaxMotorSpeed)
            {
                motorSpeed += 1;
            }
            previous_Increment = elapsedTime;
        }
#pragma endregion comment

    } // while
}

// adc_result = ADC_read(6);
// sensor_pos_result = Sensors_Pos();
// sprintf(buffer, "%d|%d\n",
//         sensor_pos_result, adc_result);
// uart_send_string(buffer);
//_delay_ms(500);
//// Leitura do acelerômetro
// LSM303_ReadAccel(&accel_X, &accel_Y, &accel_Z);

//// Conversão para valores reais (g)
// real_accel_X = ((float)accel_X / 4096.0) + 0.063;
// real_accel_Y = ((float)accel_Y / 4096.0) + 0;
// real_accel_Z = ((float)accel_Z / 4096.0) + 0.180;

// float_2Int(real_accel_X, &real_accel_X_I, &real_accel_X_D);
// float_2Int(real_accel_Y, &real_accel_Y_I, &real_accel_Y_D);
// float_2Int(real_accel_Z, &real_accel_Z_I, &real_accel_Z_D);
//  Imprimir valores no formato correto
// sprintf(buffer, "%d.%03d,%d.%03d,%d.%03d\n",
//         real_accel_X_I, real_accel_X_D,
//         real_accel_Y_I, real_accel_Y_D,
//         real_accel_Z_I, real_accel_Z_D);

// accelRoll = atan(real_accel_Y / sqrt(real_accel_X * real_accel_X + real_accel_Z * real_accel_Z)) * 1 / (PI / 180);
// accelPitch = atan(real_accel_X / sqrt(real_accel_Y * real_accel_Y + real_accel_Z * real_accel_Z)) * 1 / (PI / 180);

// accelPitch();
// accelRoll();

// float_2Int(accelRoll, &real_accel_X_I, &real_accel_X_D);
// float_2Int(accelPitch, &real_accel_Y_I, &real_accel_Y_D);
// sprintf(buffer, "%d.%03d,%d.%03d\n",
//         real_accel_X_I, real_accel_X_D,
//         real_accel_Y_I, real_accel_Y_D);

// getGyroValues(&gyro_X, &gyro_Y, &gyro_Z);
// real_gyro_X = ((float)gyro_X / 8.75) - RateCalibrationRoll;
// real_gyro_Y = ((float)gyro_Y / 8.75) - RateCalibrationPitch;
// real_gyro_Z = ((float)gyro_Z * 8.75) - RateCalibrationYaw;
// real_gyro_Z /= 1000; // de mdps para dps

// float_2Int(real_gyro_X, &real_gyro_X_I, &real_gyro_X_D);
// float_2Int(real_gyro_Y, &real_gyro_Y_I, &real_gyro_Y_D);
// float_2Int(real_gyro_Z, &real_gyro_Z_I, &real_gyro_Z_D);
//
// sprintf(buffer, "raw %03d,%03d,%03d |real: %03d.%03d,%03d.%03d,%03d.%03d\n",
//        gyro_X, gyro_Y, gyro_Z,
//        real_gyro_X_I, real_gyro_X_D,
//        real_gyro_Y_I, real_gyro_Y_D,
//        real_gyro_Z_I, real_gyro_Z_D);

// uart_send_string(buffer);
// real_gyro_Z = getRealZGyro();
// gyroHeading += (real_gyro_Z) * ((float)(elapsedTime - lastSampleTime) / 1000);
//
// if (gyroHeading > 360)
//{
//    gyroHeading = 0;
//}
// if (gyroHeading < 0)
//{
//    gyroHeading = 360;
//}
//
// lastSampleTime = elapsedTime;
// float_2Int(gyroHeading, &real_gyro_Y_I, &real_gyro_Y_D);
// float_2Int(real_gyro_Z, &real_gyro_Z_I, &real_gyro_Z_D);
// sprintf(buffer, "raw %03d.%03d|real: %03d.%03d| elapsedTime: %ld |lastSampleTime: %ld \n",
//        real_gyro_Y_I, real_gyro_Y_D,
//        real_gyro_Z_I, real_gyro_Z_D,
//        elapsedTime,
//        lastSampleTime);
//
// uart_send_string(buffer);
////
// delay_ms(100); // Delay para estabilização