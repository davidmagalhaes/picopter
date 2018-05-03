#include <p18f4550.h>

void main(){
	int i = 0;
	TRISDbits.RD0 = 0;
	TRISDbits.RD1 = 0;
	TRISDbits.RD2 = 1;
	TRISDbits.RD3 = 0;
	TRISDbits.RD4 = 0;
		
	while(1){
		for(i = 0; i < 6666; i++);
		if(PORTDbits.RD2){
			LATD = 0b00011011;
		}
		else{
			LATD = 0b00000000;
		}
		
		//for(i = 0; i < 6666; i++);
		//LATD = 0b00000000;
	}	
}


