#include <p18f4550.h>

#define QNT_MOTORS 4
#define BIT_ON 1
#define BIT_OFF 0
#define PWM_RESOLUTION 20


#pragma config FOSC = HSPLL_HS 
#pragma config IESO = OFF
#pragma config PLLDIV = 4 
#pragma config CPUDIV = OSC1_PLL2 
#pragma config WDT = OFF

struct _strmotor{
	char velocity;
	unsigned char *port;
	char pin;
};

typedef struct _strmotor TMotor;

char pwm_output = 0b00001111;
char timecounter = -1;
char vel_percent[101]; //tabela com valores de resolução para cada duty_cicle
TMotor motors[4];
int analog_vals[6];

//Business
void cfg_motors(void);

//Interrupts
void handle_timecount(void);
void handle_int(void);
void handle_isec(void);

//Utils
void pin_send(unsigned char*, char, char);


#pragma code int_toogle = 0x0008
	void int_toogle(void){
		_asm
			GOTO handle_int
		_endasm
	}
#pragma code

#pragma code int_sec = 0x0018
	void int_sec(void){
		_asm
			GOTO handle_isec
		_endasm
	}
#pragma code

#pragma interrupt handle_int
void handle_int(void){
	if(PIR1bits.TMR2IF){			
		PIR1bits.TMR2IF = 0;
		
		timecounter++;

		if(timecounter == PWM_RESOLUTION){
			timecounter = 0;
			pwm_output = 0b00000000;

			if(motors[0].velocity){
				pwm_output |= 0b00000001;
			}
			if(motors[1].velocity){
				pwm_output |= 0b00000010;
			}
			if(motors[2].velocity){
				pwm_output |= 0b00000100;
			}
			if(motors[3].velocity){
				pwm_output |= 0b00001000;
			}

			LATD |= pwm_output;	
		}
		else{
			if(motors[0].velocity == timecounter){
				pwm_output &= 0b11111110;
			}
			if(motors[1].velocity == timecounter){
				pwm_output &= 0b11111101;
			}
			if(motors[2].velocity == timecounter){
				pwm_output &= 0b11111011;
			}
			if(motors[3].velocity == timecounter){
				pwm_output &= 0b11110111;
			}

			LATD &= pwm_output;
		}
	}

}

#pragma interrupt handle_isec
void handle_isec(void){
	char pin;
	PIR1bits.ADIF = 0;
	
	pin = ADCON0 >> 2;

	if(pin)
		ADCON0 = ((pin-1) << 2) + 0b00000011;
	else
		ADCON0 = 0b00010111;
	
	analog_vals[pin] = ((int)ADRESH << 8) + ADRESL;
}

void main(){
	//Pinos digitais
	TRISDbits.RD0 = 0;
	TRISDbits.RD1 = 0;
	TRISDbits.RD2 = 0;
	TRISDbits.RD3 = 0;

	TRISDbits.RD4 = 1;
	TRISDbits.RD5 = 0;

	//Pinos analógicos
	TRISAbits.RA0 = 1;
	TRISAbits.RA1 = 1;
	TRISAbits.RA2 = 1;
	TRISAbits.RA3 = 1;
	TRISAbits.RA4 = 1;
	TRISEbits.RE0 = 1;

	//Configuração para conversor A/D: Obtenção de dados do giroscópio e acelerômetro
	ADCON1 = 0b00001001;
	ADCON2 = 0b10001110;

	cfg_motors();
	
	//Interrupções
	RCONbits.IPEN = 1;
	INTCONbits.GIE = 1; //Habilita interruções globalmente
    INTCONbits.GIEL = 1; //habilita interrupção dos periféricos

	//Interrupção - Timer para PWM
    PIE1bits.TMR2IE = 1; //habilita Interrupção para Timer2
	IPR1bits.TMR2IP = 1; // Flag de prioridade
    T2CON = 0b01100100; // Ignora primeiro bit (0), Postscaler 1:12 (1100), Timer2 ligado (1), Prescaler 1:1 (00)
	PR2 = 9; //Define valor de PR2 para 9. Assim, temos t = (4/48) * 1 * 12 * 9 = 9~10uS (t(uS) = (clock/4) * (1/prescale) * (1/postscale) * PR2)
	
	//Interrupção - A/D
	PIR1bits.ADIF = 0;
	IPR1bits.ADIP = 0;
	PIE1bits.ADIE = 1;
	
	ADCON0 = 0b00010111; //Inicia varredura de campos analógicos

	while(1){
		
	}
}

void cfg_motors(){
	int i = 0;

	for(i = 0; i < 101; i++)
		vel_percent[i] = (PWM_RESOLUTION * i) / 100;

	motors[0].velocity = vel_percent[5];
	motors[0].port = (unsigned char*) &LATD;
	motors[0].pin = 0;

	motors[1].velocity = vel_percent[25];
	motors[1].port = (unsigned char*) &LATD;
	motors[1].pin = 1;

	motors[2].velocity = vel_percent[75];
	motors[2].port = (unsigned char*) &LATD;
	motors[2].pin = 2;

	motors[3].velocity = vel_percent[100];
	motors[3].port = (unsigned char*) &LATD;
	motors[3].pin = 3;
}




/*	


*/


/*

if(PIR1bits.TMR2IF){			
		PIR1bits.TMR2IF = 0;
		
		timecounter++;

		if(timecounter == 20){
			timecounter = 0;
		
			if(motors[0].velocity){
				LATDbits.LATD0 = 1;
			}
			if(motors[1].velocity){
				LATDbits.LATD1 = 1;
			}
			if(motors[2].velocity){
				LATDbits.LATD2 = 1;
			}
			if(motors[3].velocity){
				LATDbits.LATD3 = 1;
			}
		}
		else{
			if(motors[0].velocity == timecounter){
				LATDbits.LATD0 = 0;
			}
			if(motors[1].velocity == timecounter){
				LATDbits.LATD1 = 0;
			}
			if(motors[2].velocity == timecounter){
				LATDbits.LATD2 = 0;
			}
			if(motors[3].velocity == timecounter){
				LATDbits.LATD3 = 0;
			}
		}
	}

*/
