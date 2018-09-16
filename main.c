#include <p18f4550.h>

#define PWM_RESOLUTION 20
#define PWM_MAXOFFSET 0

//OUTPUT_NORMALIZING_TERM: 1023 = max_gyro_value, 100 = max_motor_velocity
//max_gyro_value/max_motor_velocity
#define GYRO_NORMALIZING_TERM 51.15
#define ADDR_GYRO 0x68
#define I2C_BITRATE 1249
#define I2C_READ_BIT 0x01

//Por conven��o do projeto, o motor positivo � o motor que gira no sentido hor�rio dentro do eixo, e seu �ndice � 0 
#define MOTOR_POSITIVE 0
//Por conven��o do projeto, o motor negativo � o motor que gira no sentido anti-hor�rio dentro do eixo, e seu �ndice � 1 
#define MOTOR_NEGATIVE 1

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
	TMotor motors[2]; // Motores DC que fazem parte do eixo
	int err;          // <sa�da esperada> - <sa�da real>
	int lasterr;      // Erro do loop anterior. Come�a com zero.
	int reset;        // Soma dos erros. Come�a com zero. reset := reset + (gain/tau_i) * err
	int currstate;    // Estado atual do eixo, ou seja, sa�da real (Angulo)
	int setpoint;     // Sa�da desejada. (Angulo)
} AXIS_X, AXIS_Y;

char pwm_output = 0b00000000; //Sa�da pwm por pino
char timecounter = 0; //Contador PWM geral
char i2c_gyro_msgbuf[4]; //Espa�o tempor�rio para guardar bytes recebidos pelo i2c do girosc�pio
char i2c_gyro_bufidx = 4; //Vari�vel para controle da comunica��o i2c com o girosc�pio
char vel_percent[101]; //tabela com valores de resolu��o para cada duty_cicle
char adpins[] = {0b00000011, 0b00000111}; //pinos analogicos para serem usados em ADCON0
char hover_velocity; //Velocidade para manter o drone parado no ar
char velocity_setpoint; //Setpoint da velocidade do drone, ou seja, a velocidade que o drone tem que estar.
char pid_output[2]; //Sa�da do controlador PID para os eixos X e Y

//V�ri�veis usadas para implementar a equa��o PID (Proporcional integral derivada) para estabiliza��o do drone.
// Equa��o PID: output := gain * err + reset + (gain * (err - lasterr) / tau_d)

float gain = 4.0; //Constante base para calcular os ganhos proporcionais, integrais, e derivativos
float tau_i = 16.0; // gain/tau_i para dar o ganho integral do sistema. (Ki = gain/tau_i)
float tau_d = 2.0; // gain/tau_d para dar o ganho derivativo do sistema (Kd = gain/tau_d)

// Equa��o PID: output = Kp * err + (Ki * interr * dt) + (Kd * dererr /dt);
//double interr = 0.0; //err + <interr do loop anterior> (erro integral. come�a com zero)
//double dererr = 0.0; //err - <err do loop anterior> (erro diferencial. come�a com zero)
//double dt; //Tempo de execu��o do loop em segundos
//double setpoint; //Valor esperado
//double errorsum = 0.0; //Soma dos erros. Come�a em zero.

//Initializations and configurations
void cfg_axes(void);

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

		if(!--AXIS_X.motors[MOTOR_POSITIVE].pwm_timecounter){
			pwm_output &= 0b11111110;
		}
		if(!--AXIS_X.motors[MOTOR_NEGATIVE].pwm_timecounter){
			pwm_output &= 0b11111101;
		}
		if(!--AXIS_Y.motors[MOTOR_POSITIVE].pwm_timecounter){
			pwm_output &= 0b11111011;
		}
		if(!--AXIS_Y.motors[MOTOR_NEGATIVE].pwm_timecounter){
			pwm_output &= 0b11110111;
		}

		LATD &= pwm_output;
	}
	else{
		if(AXIS_X.motors[MOTOR_POSITIVE].velocity){
			pwm_output |= 0b00000001;
		}
		if(AXIS_X.motors[MOTOR_NEGATIVE].velocity){
			pwm_output |= 0b00000010;
		}
		if(AXIS_Y.motors[MOTOR_POSITIVE].velocity){
			pwm_output |= 0b00000100;
		}
		if(AXIS_Y.motors[MOTOR_NEGATIVE].velocity){
			pwm_output |= 0b00001000;
		}

		LATD |= pwm_output;

		timecounter = PWM_RESOLUTION - 1;

		AXIS_X.motors[MOTOR_POSITIVE].pwm_timecounter = AXIS_X.motors[MOTOR_POSITIVE].velocity;
		AXIS_X.motors[MOTOR_NEGATIVE].pwm_timecounter = AXIS_X.motors[MOTOR_NEGATIVE].velocity;
		AXIS_Y.motors[MOTOR_POSITIVE].pwm_timecounter = AXIS_Y.motors[MOTOR_POSITIVE].velocity;
		AXIS_Y.motors[MOTOR_NEGATIVE].pwm_timecounter = AXIS_Y.motors[MOTOR_NEGATIVE].velocity;
	}
}

#pragma interrupt handle_isec
void handle_isec(void){
	if(PIR1bits.SSPIF){
		PIR1bits.SSPIF = 0;

		if(SSPSTATbits.BF || SSPSTATbits.R_NOT_W){	
			if(!--i2c_gyro_bufidx){
				i2c_gyro_msgbuf[i2c_gyro_bufidx] = SSPBUF;
                i2c_gyro_bufidx = 4;
				SSPCON2bits.PEN = 1; //Para a comunica��o com o girosc�pio e aceler�metro
            }
			else{
				i2c_gyro_msgbuf[i2c_gyro_bufidx] = SSPBUF;
			}
			
			SSPCON2bits.ACKDT = 0;		/* Acknowledge data 1:NACK,0:ACK */
    		SSPCON2bits.ACKEN = 1;	    /* Enable ACK to send */
		}
		else if(SSPSTATbits.P){
			SSPCON2bits.SEN = 1; //Inicia comunica��o I2C com girosc�pio e aceler�metro
		}
		else if(!SSPCON2bits.RCEN && !SSPCON2bits.ACKDT){
			SSPCON2bits.RCEN = 1; //Inicia leitura de dados do girosc�pio e aceler�metro
		}
		else if(SSPSTATbits.S){
			SSPBUF = (ADDR_GYRO << 1) | I2C_READ_BIT; //Envia endere�o do girosc�pio e aceler�metro, com o bit RW setado para leitura
		}
	}

/*
	PIR1bits.ADIF = 0;

		pin = ADCON0 >> 2;
		ADCON0 = pin ? adpins[0] : adpins[pin+1];

	gyro_input[pin] = (((int)ADRESH << 8) + ADRESL);
*/
}

void main(){
	//Pinos digitais
	TRISDbits.RD0 = 0;
	TRISDbits.RD1 = 0;
	TRISDbits.RD2 = 0;
	TRISDbits.RD3 = 0;

	TRISDbits.RD4 = 1;
	TRISDbits.RD5 = 0;

	//Pinos para comunica��o I2C
	TRISBbits.RB0 = 1;
	TRISBbits.RB1 = 1;

	//Init variables
	cfg_axes();

	//Configura��o para conversor A/D
	//ADCON1 = 0b00001001;
	//ADCON2 = 0b10111110;

	//Inicializa��o do m�dulo de comunica��o serial I2C (SDA e SCL), para comunica��o com o girosc�pio e aceler�metro
	SSPSTAT |= 0b11000000;
	SSPCON1  = 0b00101000;
	SSPCON2  = 0b00000000;
	SSPADD = 0x77; // I2C bit clock de 400kHz, obtido com ((Fosc/4)/BitRate)-1 = ((48M/4)/100k)-1 = (12M/0.1M) - 1 = 120 - 1 = 119 = 1Dh
	IPR1bits.SSPIP = 0;
	PIE1bits.SSPIE = 1;
	PIR1bits.SSPIF = 0;
	SSPCON2bits.SEN = 1;

	//Interrup��es
	RCONbits.IPEN = 1;
	INTCONbits.GIE = 1; //Habilita interru��es globalmente
    INTCONbits.GIEL = 1; //habilita interrup��o dos perif�ricos

	//Interrup��o - Timer para PWM
    PIE1bits.TMR2IE = 1; //habilita Interrup��o para Timer2
	IPR1bits.TMR2IP = 1; // Flag de prioridade
    T2CON = 0b01100100; // Ignora primeiro bit (0), Postscaler 1:12 (1100), Timer2 ligado (1), Prescaler 1:1 (00)
	PR2 = 9; //Define valor de PR2 para 9. Assim, temos t = (4/48) * 1 * 12 * 9 = 9~10uS (t(uS) = (clock/4) * (1/prescale) * (1/postscale) * PR2)

	//Interrup��o - A/D
//	PIR1bits.ADIF = 0;
//	IPR1bits.ADIP = 0;
//	PIE1bits.ADIE = 1;

//	ADCON0 = 0b00010111; //Inicia varredura de campos anal�gicos

	/*
	 *	Usando PID controller para estabilizar o drone, com base nos dados fornecidos pelo girosc�pio.
	 *  Aqui, ele atualiza os valores dos eixos e as velocidades dos motores, aplicando a sa�da do algoritmo PID.		
	 */
	while(1){
		int i;
		char idxG, idxL;
		char motorGvel, motorLvel;

		AXIS_X.err = AXIS_X.setpoint - AXIS_X.currstate;
		AXIS_X.reset += AXIS_X.err/tau_i;
		pid_output[0] = gain * (AXIS_X.err + AXIS_X.reset + ((AXIS_X.err - AXIS_X.lasterr) / tau_d)) / GYRO_NORMALIZING_TERM; // ganho proporcional + ganho integral + ganho
		AXIS_X.lasterr = AXIS_X.err; //Guarda o erro para a pr�xima itera��o

		AXIS_Y.err = AXIS_Y.setpoint - AXIS_Y.currstate;
		AXIS_Y.reset += AXIS_Y.err/tau_i;
		pid_output[1] = gain * (AXIS_Y.err + AXIS_Y.reset + ((AXIS_Y.err - AXIS_Y.lasterr) / tau_d)) / GYRO_NORMALIZING_TERM; // ganho proporcional + ganho integral + ganho
		AXIS_Y.lasterr = AXIS_Y.err; //Guarda o erro para a pr�xima itera��o

		for(i = 0; i < 2; i++){
			TMotor *motors = i ? AXIS_X.motors : AXIS_Y.motors;
			idxG = pid_output[i] > 0 ? MOTOR_POSITIVE : MOTOR_NEGATIVE;
			idxL = idxG == MOTOR_NEGATIVE ? MOTOR_POSITIVE : MOTOR_NEGATIVE;

			motorGvel = motors[idxG].velocity + pid_output[i];
			motorLvel = motors[idxL].velocity - pid_output[i];
	
			if(motorGvel > PWM_RESOLUTION){
				char incr = motorGvel - PWM_RESOLUTION;
				motorGvel -= incr;
				motorLvel -= incr;
			}
			if(motorLvel < 0){
				motorGvel = MIN(motorGvel - motorLvel, PWM_RESOLUTION);
				motorLvel = 0;
			}
			
			motors[idxG].velocity = motorGvel;
			motors[idxL].velocity = motorLvel;
		}
	}
}

void cfg_axes(){
	int i = 0;
	
	//Inicializando valores dos eixos
	AXIS_X.err = 0;          
	AXIS_X.lasterr = 0; 
	AXIS_X.reset = 0;   
	AXIS_X.currstate = 0;
	AXIS_X.setpoint = 0;

	AXIS_Y.err = 0;          
	AXIS_Y.lasterr = 0;   
	AXIS_Y.reset = 0;     
	AXIS_Y.currstate = 0; 
	AXIS_Y.setpoint = 0;

	//Inicializa tabela de velocidades por porcentagem
	for(i = 0; i < 101; i++)
		vel_percent[i] = (PWM_RESOLUTION * i) / 100;

	//Remove as ultimas PWM_MAX_OFFSET velocidades. Nova velocidade m�xima = PWM_RESOLUTION - PWM_MAX_OFFSET
	for(i = 0; i < (100 / PWM_RESOLUTION * PWM_MAXOFFSET); i++)
		vel_percent[100-i] = PWM_RESOLUTION - PWM_MAXOFFSET;


	//Valores iniciais dos motores
	AXIS_X.motors[0].velocity = vel_percent[0];
	AXIS_X.motors[0].port = (unsigned char*) &LATD;
	AXIS_X.motors[0].pin = 0;

	AXIS_X.motors[1].velocity = vel_percent[0];
	AXIS_X.motors[1].port = (unsigned char*) &LATD;
	AXIS_X.motors[1].pin = 1;

	AXIS_Y.motors[0].velocity = vel_percent[25];
	AXIS_Y.motors[0].port = (unsigned char*) &LATD;
	AXIS_Y.motors[0].pin = 2;

	AXIS_Y.motors[1].velocity = vel_percent[80];
	AXIS_Y.motors[1].port = (unsigned char*) &LATD;
	AXIS_Y.motors[1].pin = 3;
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
