#include <p18f4550.h>

#define PWM_RESOLUTION 20
#define PWM_MAXOFFSET 0

//OUTPUT_NORMALIZING_TERM: 1023 = max_gyro_value, 100 = max_motor_velocity
//max_gyro_value/max_motor_velocity
#define GYRO_NORMALIZING_TERM 51.15
#define GYRO_ADDRESS 0x68
#define I2C_BITRATE 1249
#define I2C_READ_BIT 0x01
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

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
} GYRO = {0, 0, 0, 0, 0, 0, 0};

struct {
	int err;       // <saída esperada> - <saída real>
	int lasterr;   // Erro do loop anterior. Começa com zero.
	int reset;     // Soma dos erros. Começa com zero. reset := reset + (gain/tau_i) * err
	int setpoint;  // Saída desejada.
} AXIS_X = {0, 0, 0, 0}, AXIS_Y = {0, 0, 0, 0};

char pwm_output = 0b00000000; //Saída pwm por pino
char timecounter = 0; //Contador PWM geral
char i2c_gyro_msgbuf[4]; //Espaço temporário para guardar bytes recebidos pelo i2c do giroscópio
char i2c_gyro_nextidx = 4; //Variável para controle da comunicação i2c com o giroscópio
char vel_percent[101]; //tabela com valores de resolução para cada duty_cicle
TMotor motors[4];  //Motores DC do drone
char adpins[] = {0b00000011, 0b00000111}; //pinos analogicos para serem usados em ADCON0
char hover_velocity; //Velocidade para manter o drone parado no ar
char velocity_setpoint; //Setpoint da velocidade do drone, ou seja, a velocidade que o drone tem que estar.

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
	PIR1bits.TMR2IF = 0;

	if(timecounter){
		timecounter--;

		if(!--motors[0].pwm_timecounter){
			pwm_output &= 0b11111110;
		}
		if(!--motors[1].pwm_timecounter){
			pwm_output &= 0b11111101;
		}
		if(!--motors[2].pwm_timecounter){
			pwm_output &= 0b11111011;
		}
		if(!--motors[3].pwm_timecounter){
			pwm_output &= 0b11110111;
		}

		LATD &= pwm_output;
	}
	else{
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

		timecounter = PWM_RESOLUTION - 1;

		motors[0].pwm_timecounter = motors[0].velocity;
		motors[1].pwm_timecounter = motors[1].velocity;
		motors[2].pwm_timecounter = motors[2].velocity;
		motors[3].pwm_timecounter = motors[3].velocity;
	}
}

#pragma interrupt handle_isec
void handle_isec(void){
	if(PIR1bits.SSPIF){
		PIR1bits.SSPIF = 0;

		if(SSPSTATbits.BF || SSPSTATbits.R_NOT_W){	
			if(!--i2c_gyro_nextidx){
				i2c_gyro_msgbuf[i2c_gyro_nextidx] = SSPBUF;
                i2c_gyro_nextidx = 4;
				SSPCON2bits.PEN = 1; //Para a comunicação com o giroscópio e acelerômetro
            }
			else{
				i2c_gyro_msgbuf[i2c_gyro_nextidx] = SSPBUF;
			}
			
			SSPCON2bits.ACKDT = 0;		/* Acknowledge data 1:NACK,0:ACK */
    		SSPCON2bits.ACKEN = 1;	    /* Enable ACK to send */
		}
		else if(SSPSTATbits.P){
			SSPCON2bits.SEN = 1; //Inicia comunicação I2C com giroscópio e acelerômetro
		}
		else if(!SSPCON2bits.RCEN && !SSPCON2bits.ACKDT){
			SSPCON2bits.RCEN = 1; //Inicia leitura de dados do giroscópio e acelerômetro
		}
		else if(SSPSTATbits.S){
			SSPBUF = (GYRO_ADDRESS << 1) | I2C_READ_BIT; //Envia endereço do giroscópio e acelerômetro, com o bit RW setado para leitura
		}
	}

/*
	PIR1bits.ADIF = 0;

		pin = ADCON0 >> 2;
		ADCON0 = pin ? adpins[0] : adpins[pin+1];

	gyro_input[pin] = (((int)ADRESH << 8) + ADRESL);

	//Usando PID controller para estabilizar o drone, com base nos dados fornecidos pelo giroscópio
	if(pin){
		errZ = setpointZ - gyro_input[pin]; //calcula o erro
		resetZ += errZ/tau_i; // Ganho integral
		output = gain * (errZ + resetZ + ((errZ - lasterrZ) / tau_d)) / GYRO_NORMALIZING_TERM; // ganho proporcional + ganho integral + ganho
		lasterrZ = errZ; //Guarda o erro para a próxima iteração

		idxG = output > 0 ? 0 : 2;
		idxL = idxG ? 0 : 2;
	}
	else{
		errY = setpointY - gyro_input[pin];
		resetY += errY/tau_i;
		output = gain * (errY + resetY + ((errY - lasterrY) / tau_d)) / GYRO_NORMALIZING_TERM;
		lasterrY = errY;

		idxG = output > 0 ? 1 : 3;
		idxL = idxG - 1 ? 1 : 3;
	}

	if(output){
		char motorGvel = motors[idxG].velocity;
		char motorLvel = motors[idxL].velocity;

		motorGvel += output;
		motorLvel -= output;

		if(motorGvel > PWM_RESOLUTION){
			char incr = motorGvel - PWM_RESOLUTION;
			motorGvel -= incr;
			motorLvel -= incr;
		}
		if(motorLvel < 0){
			motorGvel = MIN(motorGvel - motorLvel, PWM_RESOLUTION);
			motorLvel = 0;
		}

		motors[idxL].velocity = motorLvel;
		motors[idxG].velocity = motorGvel;
	}*/
}

void main(){
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
	cfg_motors();

	//Configuração para conversor A/D
	//ADCON1 = 0b00001001;
	//ADCON2 = 0b10111110;

	//Inicialização do módulo de comunicação serial I2C (SDA e SCL), para comunicação com o giroscópio e acelerômetro
	SSPSTAT |= 0b11000000;
	SSPCON1  = 0b00101000;
	SSPCON2  = 0b00000000;
	SSPADD = 0x77; // I2C bit clock de 400kHz, obtido com ((Fosc/4)/BitRate)-1 = ((48M/4)/100k)-1 = (12M/0.1M) - 1 = 120 - 1 = 119 = 1Dh
	IPR1bits.SSPIP = 0;
	PIE1bits.SSPIE = 1;
	PIR1bits.SSPIF = 0;
	SSPCON2bits.SEN = 1;

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
//	PIR1bits.ADIF = 0;
//	IPR1bits.ADIP = 0;
//	PIE1bits.ADIE = 1;

//	ADCON0 = 0b00010111; //Inicia varredura de campos analógicos

	//Implementa PID control
	while(1){
		
	}
}

void cfg_motors(){
	int i = 0;

	//Inicializa vetor de velocidades por porcentagem
	for(i = 0; i < 101; i++)
		vel_percent[i] = (PWM_RESOLUTION * i) / 100;

	//Remove as ultimas PWM_MAX_OFFSET velocidades. Nova velocidade máxima = PWM_RESOLUTION - PWM_MAX_OFFSET
	for(i = 0; i < (100 / PWM_RESOLUTION * PWM_MAXOFFSET); i++)
		vel_percent[100-i] = PWM_RESOLUTION - PWM_MAXOFFSET;

	//Valores iniciais dos motores
	motors[0].velocity = vel_percent[0];
	motors[0].port = (unsigned char*) &LATD;
	motors[0].pin = 0;

	motors[1].velocity = vel_percent[0];
	motors[1].port = (unsigned char*) &LATD;
	motors[1].pin = 1;

	motors[2].velocity = vel_percent[25];
	motors[2].port = (unsigned char*) &LATD;
	motors[2].pin = 2;

	motors[3].velocity = vel_percent[80];
	motors[3].port = (unsigned char*) &LATD;
	motors[3].pin = 3;
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
