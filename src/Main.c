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

// int16_t gyro_X = 0;
// int16_t gyro_Y = 0;
// int16_t gyro_Z = 0;
// int16_t accel_X = 0;
// int16_t accel_Y = 0;
// int16_t accel_Z = 0;
// int16_t mag_X = 0;
// int16_t mag_Y = 0;
// int16_t mag_Z = 0;
//
// float real_gyro_X = 0.0;
// float real_gyro_Y = 0.0;
// float real_gyro_Z = 0.0;
// float real_accel_X = 0.0;
// float real_accel_Y = 0.0;
// float real_accel_Z = 0.0;
// float real_mag_X = 0.0;
// float real_mag_Y = 0.0;
// float real_mag_Z = 0.0;
//
// int16_t real_accel_X_I = 0;
// int16_t real_accel_X_D = 0;
// int16_t real_accel_Y_I = 0;
// int16_t real_accel_Y_D = 0;
// int16_t real_accel_Z_I = 0;
// int16_t real_accel_Z_D = 0;
//
// int16_t real_gyro_X_I = 0;
// int16_t real_gyro_X_D = 0;
// int16_t real_gyro_Y_I = 0;
// int16_t real_gyro_Y_D = 0;
// int16_t real_gyro_Z_I = 0;
// int16_t real_gyro_Z_D = 0;
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
char buffer[100];
#define BUFFER_SIZE 10 // Tamanho suficiente para armazenar os números convertidos

volatile uint32_t elapsedTime = 0; // Variável global para armazenar o tempo em milissegundos
volatile uint32_t lastSampleTime = 0;
size_t i = 0;

volatile bool estado_sinal = false; // Estado do sinal PWM
volatile uint8_t largura_pulso = 2; // Largura do pulso em ms (1-2 ms)
volatile uint8_t timerCount = 0;    // Contador de tempo em ms

void timer0_init()
{
    TCCR0A = 0;
    TCCR0B |= (1 << WGM01);
    TCCR0B |= (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
    elapsedTime++;
}
void inic(void)
{

    // Configuração das portas como saídas
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3); // 14,15,16,17
    DDRB |= (1 << PB4) | (1 << PB5);                           // 22,23

    // Colocar as saídas em nível lógico alto
    PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
    PORTB |= (1 << PB4) | (1 << PB5);
}
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

    sei(); // Habilita interrupções globais
}
void set_pwm_duty_cycle(uint16_t duty_cycle_A, uint16_t duty_cycle_B)
{

    // Canal A (OC1A)
    if (duty_cycle_A < 61)
        duty_cycle_A = 61; // 1ms
    if (duty_cycle_A > 125)
        duty_cycle_A = 125; // 2ms
    OCR1A = duty_cycle_A;   // Atualiza OCR1A

    // Canal B (OC1B)
    if (duty_cycle_B < 61)
        duty_cycle_B = 61;
    if (duty_cycle_B > 125)
        duty_cycle_B = 125;
    OCR1B = duty_cycle_B; // Atualiza OCR1B
}

ISR(TIMER2_COMPA_vect)
{

    timerCount++;

    if (estado_sinal == false)
    { // Se o sinal está em LOW
        if (timerCount == 19)
        {                           // Após 19 ms, vai para HIGH
            PORTD |= (1 << PORTD3); // Sinal HIGH
            estado_sinal = true;    // Alterna o estado do sinal
            timerCount = 0;         // Reseta o contador
        }
    }
    else
    { // Se o sinal está em HIGH

        if (timerCount >= largura_pulso)
        {                            // Após a largura do pulso (1-2 ms)
            PORTD &= ~(1 << PORTD3); // Sinal LOW
            estado_sinal = false;    // Alterna o estado do sinal
            timerCount = 0;          // Reseta o contador
        }
    }
}

void delay_ms(uint32_t ms)
{
    uint32_t startTime = elapsedTime; // Armazenar o tempo de início
    while ((elapsedTime - startTime) < ms)
    {
        ;
    }
}

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
// Variáveis globais para contagem
volatile uint16_t contador_pb4 = 0;
volatile uint16_t contador_pb0 = 0;
volatile uint16_t contador_pd7 = 0;

// Função de inicialização
void init_interrupts()
{
    DDRB &= ~((1 << PB4) | (1 << PB0));
    DDRD &= ~(1 << PD7);

    PORTB |= (1 << PB4) | (1 << PB0);
    PORTD |= (1 << PD7);

    PCICR |= (1 << PCIE0) | (1 << PCIE2);
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT0);
    PCMSK2 |= (1 << PCINT23);
}

ISR(PCINT0_vect)
{
    uart_send_string("PCINT0_vect ");
    if (!(PINB & (1 << PB4)))
    {
        // Frente direito
        // contador_pb4++;
    }
    if (!(PINB & (1 << PB0)))
    {
        uart_send_string("contador_pb0 ");
        // contador_pb0++;
    }
}

ISR(PCINT2_vect)
{

    uart_send_string("PCINT2_vect ");
    if (!(PIND & (1 << PD7)))
    {
        // contador_pd7++;
    }
}

void ADC_init()
{
    // Habilita o ADC e configura o prescaler para 64 (16MHz / 64 = 250kHz -> adequado para ADC)
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t ADC_read(uint8_t channel)
{
    // Seleciona o canal (neste caso, ADC7)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // Inicia a conversão
    ADCSRA |= (1 << ADSC);

    // Espera a conversão terminar
    while (ADCSRA & (1 << ADSC))
        ;

    // Retorna o valor de 10 bits (do ADC)
    return ADC;
}

int main(void)
{
    uart_init(9600); // Inicializa UART com baudrate de 9600

    timer0_init();
    inic();
    init_pwm();
    init_timer2_pwm();
    init_interrupts();
    uart_send_string("INIT ");
    // timer2_init();
    sei();

    // I2C_init();

    // LSM303_Accel_Init();
    delay_ms(100);
    // L3GD20_init();
    delay_ms(100);

    // CalibAccValues(2000);
    delay_ms(100);
    // CalibGyroValues(2000);
    delay_ms(100);

    uart_send_string("done ");

    // Motores frente
    set_pwm_duty_cycle(61, 61); // desligado
    _delay_ms(1000);

    // Motor traseiro
    largura_pulso = 1; // 2 ms
    _delay_ms(1000);

    ADC_init();

    uint16_t adc_result;

    while (1)
    {
        adc_result = ADC_read(6);
        sprintf(buffer, "%d\n",
                adc_result);
       // uart_send_string(buffer);
        _delay_ms(500);
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

        // gyroHeading += (real_gyro_Z) * ((float)(elapsedTime - lastSampleTime) / 1000);
        if (gyroHeading > 360)
        {
            gyroHeading = 0;
        }
        if (gyroHeading < 0)
        {
            gyroHeading = 360;
        }
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
        //// uart_send_string(buffer);
        //
        // delay_ms(100); // Delay para estabilização
    }
}