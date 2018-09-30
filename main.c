#include <p18f4550.h>
#include "i2c.h"

#define NULL 0

#define PWM_RESOLUTION 20
#define PWM_MAXOFFSET 0

//OUTPUT_NORMALIZING_TERM: 1023 = max_gyro_value, 100 = max_motor_velocity
//max_gyro_value/max_motor_velocity
#define GYRO_NORMALIZING_TERM 51.15
//0x68
#define ADDR_GYRO 0x50
#define GYRO_MSGBUF_LENGTH 2
#define I2C_BITRATE 1249
#define GYRO_REG_SMPLRT_DIV 0x19
#define GYRO_REG_PWR_MGMT_1 0x6B
#define GYRO_REG_CONFIG 0x1A
#define GYRO_REG_GYRO_CONFIG 0x1B
#define GYRO_REG_ACCEL_CONFIG 0x1C


//Por convenção do projeto, o eixo X está representado pelo índice zero no array AXES
#define AXIS_X 0
//Por convenção do projeto, o eixo Y está representado pelo índice um no array AXES
#define AXIS_Y 1

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ABS(x) (((x) > 0) ? (x) : -(x))

#pragma config FOSC = HSPLL_HS
#pragma config IESO = OFF
#pragma config PLLDIV = 4
#pragma config CPUDIV = OSC1_PLL2
#pragma config WDT = OFF
#pragma config MCLRE = OFF

typedef struct {
	char velocity;
	char pwm_timecounter;
	unsigned char *port;
	char pin;
} TMotor;

struct {
	int acx;
	int acy;
	int acz;
	int temperature;
	int gyx;
	int gyy;
	int gyz;
} GYRO = {1, 1, 1, 1, 1, 1, 1};

struct {
	// Motores DC que fazem parte do eixo
	TMotor motor_positive; //Por convenção do projeto, o motor positivo é o que faz o valor do eixo crescer positivamente e o que gira no sentido horário dentro do eixo
	TMotor motor_negative; //Por convenção do projeto, o motor negativo é o que faz o valor do eixo crescer negativamente e o que gira no sentido anti-horário dentro do eixo
	int err;         	   // <saída esperada> - <saída real>
	int lasterr;      	   // Erro do loop anterior. Começa com zero.
	int reset;        	   // Soma dos erros. Começa com zero. reset := reset + (gain/tau_i) * err
	int currstate;    	   // Estado atual do eixo, ou seja, saída real (Angulo)
	int setpoint;     	   // Saída desejada. (Angulo)
} AXES[2];

char millis_counter = 5;
unsigned int wait_millis_counter = 0;
char pwm_output = 0b00000000; //Saída pwm por pino
char timecounter = 0; //Contador PWM geral
char i2c_gyro_msgbuf[GYRO_MSGBUF_LENGTH]; //Espaço temporário para guardar bytes recebidos pelo i2c do giroscópio
char i2c_gyro_bufidx = GYRO_MSGBUF_LENGTH; //Variável para controle da comunicação i2c com o giroscópio
char gyro_nextidx = 0;
int *gyro_ptr = (int*) &GYRO;
char gyro_config = 0;
rom char vel_percent[101]; //tabela com valores de resolução para cada duty_cicle
char hover_velocity; //Velocidade para manter o drone parado no ar
char velocity_setpoint; //Setpoint da velocidade do drone, ou seja, a velocidade que o drone tem que estar.
char i2c_status = 0;

//Váriáveis usadas para implementar a equação PID (Proporcional integral derivada) para estabilização do drone.
// Equação PID: output := gain * err + reset + (gain * (err - lasterr) / tau_d)

float gain = 4.0; //Constante base para calcular os ganhos proporcionais, integrais, e derivativos
float tau_i = 16.0; // gain/tau_i para dar o ganho integral do sistema. (Ki = gain/tau_i)
float tau_d = 2.0; // gain/tau_d para dar o ganho derivativo do sistema (Kd = gain/tau_d)

// Equação PID: output = Kp * err + (Ki * interr * dt) + (Kd * dererr /dt);
//double interr = 0.0; //err + <interr do loop anterior> (erro integral. começa com zero)
//double dererr = 0.0; //err - <err do loop anterior> (erro diferencial. começa com zero)
//double dt; //Tempo de execução do loop em segundos
//double setpoint; //Valor esperado
//double errorsum = 0.0; //Soma dos erros. Começa em zero.

//Initializations and configurations
void cfg_axes(void);
void start_PID_control();


//Interrupts
void handle_timecount(void);
void handle_int(void);
void handle_isec(void);

//Utils
void wait_millis(unsigned int millis);
void pin_send(unsigned char*, char, char);

//I2C
void cfg_gyro(const char reg, const char value);
void on_reg_start_complete(int tag, char error);
void on_reg_reg_complete(int tag, char error);
void on_reg_value_complete(int tag, char error);
void loopstep_on_complete(int tag, char error);

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
	PIR1bits.TMR2IF = 0;

	if(timecounter){
		timecounter--;

		if(!--AXES[AXIS_X].motor_positive.pwm_timecounter){
			pwm_output &= 0b00001110;
		}
		if(!--AXES[AXIS_X].motor_negative.pwm_timecounter){
			pwm_output &= 0b00001101;
		}
		if(!--AXES[AXIS_Y].motor_positive.pwm_timecounter){
			pwm_output &= 0b00001011;
		}
		if(!--AXES[AXIS_Y].motor_negative.pwm_timecounter){
			pwm_output &= 0b00000111;
		}

		LATD &= pwm_output;
	}
	else{
		if(AXES[AXIS_X].motor_positive.velocity){
			pwm_output |= 0b00000001;
		}
		if(AXES[AXIS_X].motor_negative.velocity){
			pwm_output |= 0b00000010;
		}
		if(AXES[AXIS_Y].motor_positive.velocity){
			pwm_output |= 0b00000100;
		}
		if(AXES[AXIS_Y].motor_negative.velocity){
			pwm_output |= 0b00001000;
		}

		LATD |= pwm_output;

		if(wait_millis_counter){
			if(!--millis_counter){
				wait_millis_counter--;
				millis_counter = 5;
			}
		}

		timecounter = PWM_RESOLUTION - 1;

		AXES[AXIS_X].motor_positive.pwm_timecounter = AXES[AXIS_X].motor_positive.velocity;
		AXES[AXIS_X].motor_negative.pwm_timecounter = AXES[AXIS_X].motor_negative.velocity;
		AXES[AXIS_Y].motor_positive.pwm_timecounter = AXES[AXIS_Y].motor_positive.velocity;
		AXES[AXIS_Y].motor_negative.pwm_timecounter = AXES[AXIS_Y].motor_negative.velocity;
	}
}

#pragma interrupt handle_isec
void handle_isec(void){
	if(PIR1bits.SSPIF){
		PIR1bits.SSPIF = 0;

		if(gyro_config){
			i2c_int_handle();
		}
		else{
			if(SSPCON2bits.ACKSTAT){
				LATAbits.LATA1 = 1;
			}
			else{
				LATAbits.LATA1 = 0;
			}

			if(gyro_nextidx == 7){
				gyro_nextidx = 0;
				SSPCON2bits.PEN = 1; //Para a comunicação com o giroscópio e acelerômetro
			}
			else if(SSPSTATbits.BF || SSPSTATbits.R_NOT_W){
				if(!--i2c_gyro_bufidx){
					i2c_gyro_msgbuf[i2c_gyro_bufidx] = SSPBUF;
	                i2c_gyro_bufidx = GYRO_MSGBUF_LENGTH;

					gyro_ptr[gyro_nextidx++] = (int) *i2c_gyro_msgbuf;
	            }
				else{
					i2c_gyro_msgbuf[i2c_gyro_bufidx] = SSPBUF;
				}

				SSPCON2bits.ACKDT = 0;		/* Acknowledge data 1:NACK,0:ACK */
	    		SSPCON2bits.ACKEN = 1;	    /* Enable ACK to send */
			}
			else if(SSPSTATbits.P){
				i2c_status = 0;
				SSPCON2bits.SEN = 1; //Inicia comunicação I2C com giroscópio e acelerômetro
			}
			else if(SSPSTATbits.S && i2c_status == 1){
				SSPSTATbits.S = 0;
				i2c_status = 2;
				SSPBUF = 0x3b; //Endereço do primeiro registrador do giroscópio, para leitura. Coloca-se o primeiro, pois consequentes leituras lerão os outros registradores um à um, até ter lido todos.
			}
			else if(SSPSTATbits.S && i2c_status == 2){
				SSPSTATbits.S = 0;
				i2c_status = 3;
				SSPCON2bits.RSEN = 1;	//Restart para modo leitura
			}
			else if(SSPSTATbits.S && i2c_status != 4){
				SSPSTATbits.S = 0;
				if(i2c_status == 3){
					i2c_status = 4;
					SSPBUF = (ADDR_GYRO << 1) + I2C_BIT_R; //Envia endereço do giroscópio e acelerômetro, com o bit RW setado para leitura
				}
				else{
					i2c_status = 1;
					SSPBUF = (ADDR_GYRO << 1) + I2C_BIT_W; //Envia endereço do giroscópio e acelerômetro, com o bit RW setado para escrita
				}
			}
			else if(!SSPCON2bits.RCEN && !SSPCON2bits.ACKDT){
				SSPCON2bits.RCEN = 1; //Inicia leitura de dados do giroscópio e acelerômetro
			}
		}
	}
}


/*
	PIR1bits.ADIF = 0;

		pin = ADCON0 >> 2;
		ADCON0 = pin ? adpins[0] : adpins[pin+1];*/

void main(){ //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ NÃO DEVO MAIS MEXER NESSE CÓDIGO ATÉ ESTÁ COM EQUIPE @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

	TRISAbits.RA0 = 1;
	TRISAbits.RA1 = 0;
	TRISAbits.RA2 = 0;
	TRISAbits.RA3 = 0;

	//Pinos digitais
	TRISDbits.RD0 = 0;
	TRISDbits.RD1 = 0;
	TRISDbits.RD2 = 0;
	TRISDbits.RD3 = 0;

	TRISDbits.RD4 = 1;
	TRISDbits.RD5 = 0;

	//Pinos para comunicação I2C
	TRISBbits.RB0 = 1;
	TRISBbits.RB1 = 1;

	//Init variables
	cfg_axes();
	//init_wifi();

	//Interrupções
	RCONbits.IPEN = 1;
	INTCONbits.GIE = 1; //Habilita interruções globalmente
    INTCONbits.GIEL = 1; //habilita interrupção dos periféricos

	//Interrupção - Timer para PWM
    PIE1bits.TMR2IE = 1; //habilita Interrupção para Timer2
	IPR1bits.TMR2IP = 1; // Flag de prioridade
    T2CON = 0b01100100; // Ignora primeiro bit (0), Postscaler 1:12 (1100), Timer2 ligado (1), Prescaler 1:1 (00)
	PR2 = 9; //Define valor de PR2 para 9. Assim, temos t = (4/48) * 1 * 12 * 9 = 9~10uS (t(uS) = (clock/4) * (1/prescale) * (1/postscale) * PR2)

	//Espera 1 segundo para os outros componentes de hardware inicializarem
	LATAbits.LATA1 = 1;
	LATAbits.LATA2 = 0;
	wait_millis(1000);
	LATAbits.LATA1 = 0;

	//Inicialização do módulo de comunicação serial I2C (SDA e SCL), para comunicação com o giroscópio e acelerômetro
	i2c_init_master(400000);
	//SSPSTAT |= 0b01000000;
	//SSPCON1  = 0b00101000;
	//SSPCON2  = 0b00000000;
	//i2c_init(400000); //Inicializa comunicação i2c com 400kHz

	cfg_gyro(GYRO_REG_SMPLRT_DIV, 0x07);
	cfg_gyro(GYRO_REG_PWR_MGMT_1, 0x00); //Highest sensitivity
	cfg_gyro(GYRO_REG_CONFIG, 0x00);
	cfg_gyro(GYRO_REG_GYRO_CONFIG, 0x18);
	cfg_gyro(GYRO_REG_ACCEL_CONFIG, 0x00);

	i2c_loopstep();

	//Configuração para conversor A/D
	//ADCON1 |= 0x0F;   //Define todos pinos AN como digital I/O
	//ADCON1 = 0b00001001;
	//ADCON2 = 0b10111110;

	//Interrupção - A/D
//	PIR1bits.ADIF = 0;
//	IPR1bits.ADIP = 0;
//	PIE1bits.ADIE = 1;

//	ADCON0 = 0b00010111; //Inicia varredura de campos analógicos

	while(1){ //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ NÃO DEVO MAIS MEXER NESSE CÓDIGO ATÉ ESTÁ COM EQUIPE @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		LATAbits.LATA2 = ABS(GYRO.gyx) || ABS(GYRO.gyz) || ABS(GYRO.gyy) || ABS(GYRO.temperature) || ABS(GYRO.acx) || ABS(GYRO.acy) || ABS(GYRO.acz);
	}

	start_PID_control();
}

void cfg_gyro(const char reg, const char value){
	char tag[2] = {reg, value};
	gyro_config++;
	i2c_start(ADDR_GYRO, I2C_BIT_W, *((int*)tag), &on_reg_start_complete);
}
void on_reg_start_complete(int tag, char error){
	char *params = (char*)&tag;
	LATAbits.LATA1 = error;
	i2c_write(params[0], &on_reg_reg_complete);
}
void on_reg_reg_complete(int tag, char error){
	char *params = (char*)&tag;
	LATAbits.LATA1 = error;
	i2c_write(params[1], &on_reg_value_complete);	/* X axis gyroscope reference frequency */
}
void on_reg_value_complete(int tag, char error){
	LATAbits.LATA1 = error;
	i2c_stop(&loopstep_on_complete);
}
void loopstep_on_complete(int tag, char error){
	gyro_config--;

	if(!gyro_config)
		SSPCON2bits.SEN = 1;
}


void cfg_pins(){

}

void init_extern_hardware(){

}

void init_intern_hardware(){

}

void start_PID_control(){
	while(1){
		int i;
		char pid_output;
		char motorPositive[2], motorNegative[2];
		char *motorGvel, *motorLvel;

		/*
		 *	Usando PID controller para estabilizar o drone, com base nos dados fornecidos pelo giroscópio.
		 *  Aqui, ele atualiza os valores dos eixos e as velocidades dos motores, aplicando a saída do algoritmo PID.
		 */
		for(i = 0; i < 2; i++){
			AXES[i].err = AXES[i].setpoint - AXES[i].currstate;
			AXES[i].reset += AXES[i].err/tau_i;
			pid_output = gain * (AXES[i].err + AXES[i].reset + ((AXES[i].err - AXES[i].lasterr) / tau_d)) / GYRO_NORMALIZING_TERM; // ganho proporcional + ganho integral + ganho
			AXES[i].lasterr = AXES[i].err; //Guarda o erro para a próxima iteração

			motorPositive[i] = AXES[i].motor_positive.velocity + pid_output;
			motorNegative[i] = AXES[i].motor_negative.velocity - pid_output;

			if(pid_output > 0){
				motorGvel = &motorPositive[i];
				motorLvel = &motorNegative[i];
			}
			else{
				motorLvel = &motorPositive[i];
				motorGvel = &motorNegative[i];
			}

			if(*motorGvel > PWM_RESOLUTION){
				char incr = *motorGvel - PWM_RESOLUTION;
				*motorGvel -= incr;
				*motorLvel -= incr;
			}
			if(*motorLvel < 0){
				*motorGvel = MIN(*motorGvel - *motorLvel, PWM_RESOLUTION);
				*motorLvel = 0;
			}
		}

		AXES[AXIS_X].motor_positive.velocity = motorPositive[AXIS_X];
		AXES[AXIS_X].motor_negative.velocity = motorNegative[AXIS_X];
		AXES[AXIS_Y].motor_positive.velocity = motorPositive[AXIS_Y];
		AXES[AXIS_Y].motor_negative.velocity = motorNegative[AXIS_Y];
	}
}

void wait_millis(unsigned int millis){
	wait_millis_counter = millis;
	while(wait_millis_counter);
}

void cfg_axes(){
	int i = 0;

	//Inicializando valores dos eixos
	AXES[AXIS_X].err = 0;
	AXES[AXIS_X].lasterr = 0;
	AXES[AXIS_X].reset = 0;
	AXES[AXIS_X].currstate = 0;
	AXES[AXIS_X].setpoint = 0;

	AXES[AXIS_Y].err = 0;
	AXES[AXIS_Y].lasterr = 0;
	AXES[AXIS_Y].reset = 0;
	AXES[AXIS_Y].currstate = 0;
	AXES[AXIS_Y].setpoint = 0;

	//Inicializa tabela de velocidades por porcentagem
	for(i = 0; i < 101; i++)
		vel_percent[i] = (PWM_RESOLUTION * i) / 100;

	//Remove as ultimas PWM_MAX_OFFSET velocidades. Nova velocidade máxima = PWM_RESOLUTION - PWM_MAX_OFFSET
	for(i = 0; i < (100 / PWM_RESOLUTION * PWM_MAXOFFSET); i++)
		vel_percent[100-i] = PWM_RESOLUTION - PWM_MAXOFFSET;


	//Valores iniciais dos motores
	AXES[AXIS_X].motor_positive.velocity = vel_percent[0];
	AXES[AXIS_X].motor_positive.port = (unsigned char*) &LATD;
	AXES[AXIS_X].motor_positive.pin = 0;

	AXES[AXIS_X].motor_negative.velocity = vel_percent[0];
	AXES[AXIS_X].motor_negative.port = (unsigned char*) &LATD;
	AXES[AXIS_X].motor_negative.pin = 1;

	AXES[AXIS_Y].motor_positive.velocity = vel_percent[25];
	AXES[AXIS_Y].motor_positive.port = (unsigned char*) &LATD;
	AXES[AXIS_Y].motor_positive.pin = 2;

	AXES[AXIS_Y].motor_negative.velocity = vel_percent[80];
	AXES[AXIS_Y].motor_negative.port = (unsigned char*) &LATD;
	AXES[AXIS_Y].motor_negative.pin = 3;
}




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



/*

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
	}*/
