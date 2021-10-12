/*
 * Sprint10_Igor_118111142.c
 *
 * Created: 12/10/2021 10:53:44
 * Author : igor_
 */ 

#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include "nokia5110.h"

//Novos tipos
typedef enum enum_parametros {Sel_modo, Sel_tempo_verde, Sel_tempo_vermelho, Sel_tempo_amarelo, Size_enum_param} enum_parametros;
typedef struct stc_semaforo{
	uint8_t modo;
	uint16_t tempo_verde;
	uint16_t tempo_vermelho;
	uint16_t tempo_amarelo;
	uint16_t carros_por_min;
	uint16_t sensor_lux;
	
} stc_semaforo;

//Variaveis Globais
stc_semaforo semaforo = {.modo = 0, .tempo_verde = 5000, .tempo_vermelho = 3000, .tempo_amarelo = 1000, .carros_por_min = 0, .sensor_lux = 0};
enum_parametros selecao_parametro = Sel_modo;
uint8_t flag_5000ms = 0, flag_500ms = 0;
uint32_t tempo_ms = 0;
uint16_t num_carros = 0;

void anima_semaforo(stc_semaforo semaforo, uint32_t Tempo_ms);
void anima_LCD(stc_semaforo semaforo);
void estima_carros_por_min(uint8_t *flag_disparo);
void leitura_ADC_sensor_LUX(uint8_t *flag_disparo);

ISR(TIMER0_COMPA_vect){
	tempo_ms++;
	
	if((tempo_ms % 5000)==0)
	flag_5000ms = 1;
	if((tempo_ms % 500)==0)
	flag_500ms = 1;
}

ISR(INT0_vect){
	num_carros++;
}


ISR(PCINT2_vect){
	
	if((PIND&0b00010000)==0)
	{
		switch(selecao_parametro)
		{
			case Sel_modo:
			semaforo.modo = !semaforo.modo;
			break;
			case Sel_tempo_verde:
			if(semaforo.tempo_verde <= 8000)
			semaforo.tempo_verde += 1000;
			break;
			case Sel_tempo_vermelho:
			if(semaforo.tempo_vermelho <= 8000)
			semaforo.tempo_vermelho += 1000;
			break;
			case Sel_tempo_amarelo:
			if(semaforo.tempo_amarelo <= 8000)
			semaforo.tempo_amarelo += 1000;
			break;
		}
	}
	if((PIND&0b00100000)==0)
	{
		switch(selecao_parametro)
		{
			case Sel_modo:
			semaforo.modo = !semaforo.modo;
			break;
			case Sel_tempo_verde:
			if(semaforo.tempo_verde >= 2000)
			semaforo.tempo_verde -= 1000;
			break;
			case Sel_tempo_vermelho:
			if(semaforo.tempo_vermelho >= 2000)
			semaforo.tempo_vermelho -= 1000;
			break;
			case Sel_tempo_amarelo:
			if(semaforo.tempo_amarelo >= 2000)
			semaforo.tempo_amarelo -= 1000;
			break;
		}
	}
	if((PIND&0b01000000)==0)
	{
		if(selecao_parametro < (Size_enum_param-1))
		selecao_parametro++;
		else
		selecao_parametro = Sel_modo;
	}
	
	anima_LCD(semaforo);
	
}

void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms)
{
	const uint16_t estados[9] = {0b000001111, 0b000000111, 0b000000011, 0b000000001, 0b100000000, 0b011110000, 0b001110000, 0b000110000, 0b000010000};
	static int8_t i = 0;
	static uint32_t tempo_anterior_ms = 0;
	
	PORTB = estados[i] & 0b011111111;
	if (estados[i] & 0b100000000)
	PORTD |= 0b10000000;
	else
	PORTD &= 0b01111111;
	
	//Sinal Pedestre LED Vermelho
	if (estados[i] & 0b100001111)
	PORTD |= 0b00000001;
	else
	PORTD &= 0b11111110;
	
	//Sinal Pedestre LED Verde
	if (estados[i] & 0b011110000)
	PORTD |= 0b00000010;
	else
	PORTD &= 0b11111101;
	
	if(i<=3)
	{
		if((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_verde/4))
		{
			i++;
			tempo_anterior_ms += (Semaforo.tempo_verde/4);
		}
	}
	else
	{
		if(i<=4)
		{
			if((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_amarelo))
			{
				i++;
				tempo_anterior_ms += (Semaforo.tempo_amarelo);
				
			}
		}
		else
		{
			if(i<=8)
			{
				if((Tempo_ms - tempo_anterior_ms) >= (Semaforo.tempo_vermelho/4))
				{
					i++;
					tempo_anterior_ms += (Semaforo.tempo_vermelho/4);
					
				}
			}
			else
			{
				i=0;
				tempo_anterior_ms = Tempo_ms;
			}
		}
	}
}

void anima_LCD(stc_semaforo Semaforo){ // Atualiza o display de acordo com o tempo de cada led
	
	unsigned char modo_string[2];
	unsigned char tempo_verde_s_string[2];
	unsigned char tempo_vermelho_s_string[2];
	unsigned char tempo_amarelo_s_string[2];
	unsigned char carros_por_min_string[4];
	unsigned char sensor_LUX_string[5];
	modo_string[0] = (Semaforo.modo) ? 'A' : 'M'; modo_string[1]='\0';
	sprintf (&tempo_verde_s_string, "%u", Semaforo.tempo_verde/1000);
	sprintf (&tempo_vermelho_s_string, "%u", Semaforo.tempo_vermelho/1000);
	sprintf (&tempo_amarelo_s_string, "%u", Semaforo.tempo_amarelo/1000);
	sprintf (&carros_por_min_string, "%u", Semaforo.carros_por_min);
	sprintf (&sensor_LUX_string, "%u", Semaforo.sensor_lux);

	nokia_lcd_clear();

	nokia_lcd_set_cursor(0, 5);
	nokia_lcd_write_string("Modo", 1);
	nokia_lcd_set_cursor(30, 5);
	nokia_lcd_write_char(modo_string[0], 1);
	nokia_lcd_set_cursor(0, 15);
	nokia_lcd_write_string("T.Vd", 1);
	nokia_lcd_set_cursor(30,15);
	nokia_lcd_write_string(tempo_verde_s_string, 1);nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0,25);
	nokia_lcd_write_string("T.Vm", 1);
	nokia_lcd_set_cursor(30,25);
	nokia_lcd_write_string(tempo_vermelho_s_string, 1);nokia_lcd_write_string("s", 1);
	nokia_lcd_set_cursor(0,35);
	nokia_lcd_write_string("T.Am", 1);
	nokia_lcd_set_cursor(30,35);
	nokia_lcd_write_string(tempo_amarelo_s_string, 1);nokia_lcd_write_string("s", 1);
	
	nokia_lcd_set_cursor(42, 5+selecao_parametro*10);
	nokia_lcd_write_string("<", 1);
	
	nokia_lcd_draw_Hline(50, 2, 47);
	
	
	nokia_lcd_set_cursor(52, 25);
	nokia_lcd_write_string(carros_por_min_string, 2);
	nokia_lcd_set_cursor(55,40);
	nokia_lcd_write_string("c/min", 1);
	
	nokia_lcd_set_cursor(52,0);
	nokia_lcd_write_string(sensor_LUX_string, 2);
	nokia_lcd_set_cursor(55,15);
	nokia_lcd_write_string("lux", 1);
	
	nokia_lcd_render();
	
	
}

void estima_carros_por_min(uint8_t *flag_disparo)
{
	static uint16_t aux = 0;
	
	if(*flag_disparo)
	{
		*flag_disparo = 0;
		aux = num_carros;
		num_carros = 0;
		semaforo.carros_por_min = aux*12;
		
		if(semaforo.modo)
		{
			semaforo.tempo_verde = 1000 + ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if(semaforo.tempo_verde>9000)
			semaforo.tempo_verde = 9000;
			semaforo.tempo_vermelho = 9000 - ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if(semaforo.tempo_vermelho>32000)
			semaforo.tempo_vermelho = 1000;
		}
	}
}

void leitura_ADC_sensor_LUX(uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		semaforo.sensor_lux = 1023000/ADC -1000;
		if(semaforo.sensor_lux > 300)
		{
			OCR2B = 0;
		}
		else
		{
			if(((PINC & 0b1000000)==0) || (semaforo.carros_por_min > 0))
			{
				OCR2B = 255;
			}
			else
			{
				OCR2B = 85;
			}
		}
		*flag_disparo = 0;
		anima_LCD(semaforo);
	}
}

int main(void){
	
	// Config. GPIO
	DDRB = 0b11111111; // Habilita os pinos PB0 ao PB7 todos como saida
	DDRD = 0b10001011; // Habilita o pino PD7, PD3 PD0 e PD1 como saidas
	DDRC &=0b0111111;
	PORTD = 0b01110100; // Habilita o resistor de pull up dos pinos PD2, PD4, PD5 e PD6
	PORTC|= 0b1000000; // Habilita o resistor de pull up do pino PC6
	
	//Configurações das interrupções
	EICRA = 0b00001010;
	EIMSK = 0b00000011;
	PCICR  = 0b00000110;
	PCMSK2 = 0b01110000;
	
	// Configuração da TCT
	TCCR0A = 0b00000010; // habilita o CTC
	TCCR0B = 0b00000011; // Liga TC0 com prescaler = 64
	OCR0A  = 249;        // ajusta o comparador para TC0 contar até 249
	TIMSK0 = 0b00000010; // habilita a interrupção na igualdade de comparação com OCR0A. A interupção ocorre a cada 1ms = (65*(249+1))/16MHz

	//Configura ADC
	ADMUX = 0b01000000; // Vcc com referencia canal PC0
	ADCSRA= 0b11100111; // Habilita o AD, habilita interrupção, modo de conversão continua, prescaler = 128
	ADCSRB= 0b00000000; // Modo de conversão contínua
	DIDR0 = 0b00000001; // habilita pino PC0 e PC1 como entrada digitais
	
	//CONFIGURAÇÃO PWM
	TCCR2A = 0b00100011;
	TCCR2B = 0b00000110;
	OCR2B = 128;

	sei(); //Habilita interrupção globais, ativando o bit I do SREG
	
	nokia_lcd_init();
	
	while (1){
		
		anima_semaforo(semaforo, tempo_ms);
		estima_carros_por_min(&flag_5000ms);
		leitura_ADC_sensor_LUX(&flag_500ms);
	}

}

