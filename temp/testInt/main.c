#include <p18f4550.h>


void handle_int_toogle(void);

#pragma interrupt handle_int_toogle

#pragma code int_toogle = 0x0008
	void int_toogle(void){
		_asm
            GOTO handle_int_toogle
        _endasm
	}
#pragma code

char i = 0;

void handle_int_toogle(void){
    INTCON3bits.INT2IF = 0;

	i = PORTD;

	LATDbits.LATD0 = ~LATDbits.LATD0;
	LATDbits.LATD1 = ~LATDbits.LATD1;
	LATDbits.LATD2 = ~LATDbits.LATD2;
	LATDbits.LATD3 = ~LATDbits.LATD3;
	LATDbits.LATD5 = ~LATDbits.LATD5;
	LATDbits.LATD6 = ~LATDbits.LATD6;
	LATDbits.LATD7 = ~LATDbits.LATD7;

	i = PORTD;
}

void main(){

	TRISDbits.RD0 = 0;
	TRISDbits.RD1 = 0;
	TRISDbits.RD2 = 0;
	TRISDbits.RD3 = 0;

	TRISDbits.RD4 = 1;
	TRISDbits.RD5 = 0;

	ADCON1 = 0xFF;

	INTCON3bits.INT2IF = 0;
	INTCON3bits.INT2IE = 1;
	INTCON3bits.INT2IP = 1;
	INTCONbits.GIE = 1;

	while(1){
		i = PORTDbits.RD4;
	}
}


/*
		for(i = 0; i < 6666; i++);
		if(PORTDbits.RD2){
			LATC = 0b00011011;
		}
		else{
			LATC = 0b00000000;
		}

		//for(i = 0; i < 6666; i++);
		//LATD = 0b00000000;
*/

