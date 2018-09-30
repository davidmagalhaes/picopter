#define I2C_BIT_R 0x01
#define I2C_BIT_W 0x00

#define I2C_ACK 0x00
#define I2C_NACK 0x01

#define I2C_BTSEL_START 0
#define I2C_BTSEL_RESTART 1
#define I2C_BTSEL_STOP 2
#define I2C_BTSEL_READ 3

#pragma romdata bitselection_table = 0x2FFA

rom char bitselection_table[4] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};

#pragma romdata

typedef struct {
	char controlbit : 1;
	char bitselect : 3;
	char use_bytestream : 1;
	char value;
	union {
		char (*wrstream)(int tag, char error, char *keepgoing);
		char (*rdstream)(int tag, char error, char byte, char *keepgoing);
		void (*on_complete)(int tag, char error);
		char (*callback)(int tag, char error, char byte);
	} ret;
} I2C_DATA;

typedef struct {
	I2C_DATA start;
	I2C_DATA regaddr;
	I2C_DATA regval;
	I2C_DATA stop;
} I2C_REG_WDATABUF;

typedef struct {
	I2C_DATA start;
	I2C_DATA regaddr;
	I2C_DATA restart;
	I2C_DATA read;
	I2C_DATA stop;
} I2C_REG_RDATABUF;

typedef struct {
	int tag;
	char use_datastream : 1;
	I2C_DATA *databuf;	
	union{
		I2C_DATA (*stream)(int tag, char error, char *keepgoing);
		void (*on_complete)(int tag, char error);
	} func;
} I2C_MSG;

struct {
	I2C_MSG  *msgbuf;
	I2C_MSG  *last_msg;
	I2C_DATA last_data;
	char bufsize;
	char bufidx;
	char databuf_idx;
	char transmiting : 1;
	char bytestreaming : 1;
	char msgstreaming : 1;
	I2C_MSG* (*msgstream)(char *keepgoing);
	void (*on_complete)(int tag, char error);
	I2C_DATA streamaux[2];
} __I2C_CUR_STATUS;

//Private functions. DO NOT USE
void __i2c_loopstep();
void __bufstreamer(char);

//Interruption handling function
void i2c_int_handle();

//i2c communication functions
void i2c_init_master(int frequency); //Inicia conexão i2c, com a frequencia desejada
//Envia bit de START da conexão e, logo após, envia byte de endereço com bit rw. Permite receber um parametro através de tag, que será acessível por todas as msgs dentro desse start.
I2C_DATA i2c_start(char address, char rw, void (*on_complete)(int tag, char error));
I2C_DATA i2c_restart(char address, char rw, void (*on_complete)(int tag, char error));
I2C_DATA i2c_write(char byte, void (*on_complete)(int tag, char error)); // Escreve byte no Buffer de envio. Invoca on_complete no Ack ou Nack, ou em caso de erro.
I2C_DATA i2c_wrstream(char (*wrstream)(int tag, char error, char *keepgoing)); //Escreve uma sequencia de bytes no buffer de envio.
I2C_DATA i2c_read(char nread, char (*callback)(int tag, char error, char byte)); // Lê <nread> bytes do buffer de recebimento do endereço indicado. Invoca callback com o resultado de cada leitura
I2C_DATA i2c_rdstream(char (*rdstream)(int tag, char error, char byte, char *keepgoing));
I2C_DATA i2c_stop(); // Envia bit de STOP
I2C_REG_RDATABUF i2c_reg_read(char address, char regaddr, char nread, char (*callback)(int tag, char error, char byte), void (*on_complete)(int tag, char error)); //Açucar sintático: cria um buffer de dados de leitura de registrador que é comum em vários dispositivos I2C
I2C_REG_WDATABUF i2c_reg_write(char address, char regaddr, char regval, void (*on_complete)(int tag, char error));
I2C_REG_RDATABUF i2c_reg_rdstream(char address, char regaddr, char (*rdstream)(int tag, char error, char byte, char *keepgoing), void (*on_complete)(int tag, char error)); //Açucar sintático: cria um stream de leitura de registrador que é comum em vários dispositivos I2C
I2C_REG_WDATABUF i2c_reg_wrstream(char address, char regaddr, char (*stream)(int tag, char error, char *keepgoing), void (*on_complete)(int tag, char error)); //Açucar sintático: cria um stream de escritas que é comum em vários dispositivos i2c
I2C_MSG i2c_msg(I2C_DATA *databuf, int tag, void (*on_complete)(int tag, char error)); //Cria um I2C_MSG, um wrapper da mensagem i2c, que possui um rótulo para fácil identificação. Depois de transmitir, invoca o método on_complete
void i2c_transmit(I2C_MSG *msgbuf, char bufsize, void (*on_complete)(int tag, char error)); //Envia um buffer de mensagens i2c sequencialmente, sempre depois do on_complete do anterior
void i2c_datastream(char address, char rw, int tag, I2C_DATA (*datastream)(int tag, char error, char *keepgoing), void (*on_complete)(int tag, char error)); //Cria uma stream de dados para uma única mensagem, usado para ter um controle preciso da comunicação i2c
void i2c_msgstream(int tag, I2C_MSG* (*msgstream)(char *keepgoing), void (*on_complete)(int tag, char error)); //Cria uma stream de mensagens, usado para ter um controle preciso da comunicação i2c
void i2c_wait_cycle(); //Espera o buffer de mensagens ser todo enviado antes de retornar o controle.

//Implementations
void i2c_init_master(int frequency){
	//Pinos para comunicação I2C
	TRISBbits.RB0 = 1;
	TRISBbits.RB1 = 1;

	//__I2C_MSG_QUEUE.transmiting = 0;
	__I2C_CUR_STATUS.transmiting = 0;

	//Inicialização do módulo de comunicação serial I2C (SDA e SCL), para comunicação com o giroscópio e acelerômetro
	SSPSTAT |= 0b01000000;
	SSPCON1  = 0b00101000;
	SSPCON2  = 0b00000000;

	SSPADD = 0x77;
	//SSPADD = (12000000/frequency - 1); // I2C bit clock de 400kHz, obtido com ((Fosc/4)/BitRate)-1 = ((48M/4)/400k)-1 = (12M/0.4M) - 1 = 30 - 1 = 29 = 1Dh
	IPR1bits.SSPIP = 1;
	PIR1bits.SSPIF = 0;
	PIE1bits.SSPIE = 1;
}

I2C_DATA i2c_start(char address, char rw, void (*on_complete)(int tag, char error)){
	I2C_DATA data;

	data.controlbit = 1;
	data.bitselect = I2C_BTSEL_START;
	data.use_bytestream = 0;
	data.value = (address << 1) | rw;
	data.ret.on_complete = on_complete;

	return data;
}

I2C_DATA i2c_restart(char address, char rw, void (*on_complete)(int tag, char error)){
	I2C_DATA data;

	data.controlbit = 1;
	data.bitselect = I2C_BTSEL_RESTART;
	data.use_bytestream = 0;
	data.value = (address << 1) | rw;
	data.ret.on_complete = on_complete;

	return data;
}

I2C_DATA i2c_write(char byte, void (*on_complete)(int tag, char error)){
	I2C_DATA data;

	data.controlbit = 0;
	data.use_bytestream = 0;
	data.value = byte;
	data.ret.on_complete = on_complete;

	return data;
}

I2C_DATA i2c_wrstream(char (*stream)(int tag, char error, char *keepgoing)){
	I2C_DATA data;

	data.controlbit = 0;
	data.use_bytestream = 1;
	data.ret.wrstream = stream;

	return data;
}

I2C_DATA i2c_read(char nread, char (*callback)(int tag, char error, char byte)){
	I2C_DATA data;

	data.controlbit = 1;
	data.bitselect = I2C_BTSEL_READ;
	data.use_bytestream = 0;
	data.value = nread;
	data.ret.callback = callback;

	return data;
}

I2C_DATA i2c_rdstream(char (*rdstream)(int tag, char error, char byte, char *keepgoing)){
	I2C_DATA data;

	data.controlbit = 1;
	data.bitselect = I2C_BTSEL_READ;
	data.value = 0;
	data.use_bytestream = 1;
	data.ret.rdstream = rdstream;

	return data;
}

I2C_DATA i2c_stop(){
	I2C_DATA data;

	data.controlbit = 1;
	data.use_bytestream = 0;
	data.bitselect = I2C_BTSEL_STOP;

	return data;
}

I2C_REG_RDATABUF i2c_reg_read(char address, char regaddr, char nread, char (*callback)(int tag, char error, char byte), void (*on_complete)(int tag, char error)){
	I2C_REG_RDATABUF databuf;

	databuf.start = i2c_start(address, I2C_BIT_W, 0);
	databuf.regaddr = i2c_write(regaddr, 0);
	databuf.restart = i2c_restart(address, I2C_BIT_R, 0);
    databuf.read = i2c_read(nread, callback);
    databuf.stop = i2c_stop();

	return databuf;
}

I2C_REG_WDATABUF i2c_reg_write(char address, char regaddr, char regval, void (*on_complete)(int tag, char error)){
	I2C_REG_WDATABUF databuf;

    databuf.start = i2c_start(address, I2C_BIT_W, 0);
	databuf.regaddr = i2c_write(regaddr, 0);
    databuf.regval = i2c_write(regval, 0);
    databuf.stop = i2c_stop();

	return databuf;
}

I2C_REG_WDATABUF i2c_reg_wrstream(char address, char regaddr, char (*stream)(int tag, char error, char *keepgoing), void (*on_complete)(int tag, char error)){
	I2C_REG_WDATABUF databuf;

    databuf.start = i2c_start(address, I2C_BIT_W, 0);
	databuf.regaddr = i2c_write(regaddr, 0);
    databuf.regval = i2c_wrstream(stream);
    databuf.stop = i2c_stop();

	return databuf;
}

I2C_REG_RDATABUF i2c_reg_rdstream(char address, char regaddr, char (*rdstream)(int tag, char error, char byte, char *keepgoing), void (*on_complete)(int tag, char error)){
	I2C_REG_RDATABUF databuf;

	databuf.start = i2c_start(address, I2C_BIT_W, 0);
	databuf.regaddr = i2c_write(regaddr, 0);
	databuf.restart = i2c_restart(address, I2C_BIT_R, 0);
    databuf.read = i2c_rdstream(rdstream);
    databuf.stop = i2c_stop();

	return databuf;
}

I2C_MSG i2c_msg(I2C_DATA *databuf, int tag, void (*on_complete)(int tag, char error)){
	I2C_MSG msg;

	msg.tag = tag;
	msg.databuf = databuf;
	msg.func.on_complete = on_complete;

	return msg;
}

void i2c_transmit(I2C_MSG *msgbuf, char bufsize, void (*on_complete)(int tag, char error)){
	i2c_wait_cycle();

	__I2C_CUR_STATUS.transmiting = 1;
	__I2C_CUR_STATUS.msgbuf = msgbuf;
	__I2C_CUR_STATUS.bufsize = bufsize;
	__I2C_CUR_STATUS.bufidx = 0;
	__I2C_CUR_STATUS.databuf_idx = -1;
	__I2C_CUR_STATUS.bytestreaming = 0;
	__I2C_CUR_STATUS.on_complete = on_complete;

	__i2c_loopstep();
}

void i2c_datastream(char address, char rw, int tag, I2C_DATA (*datastream)(int tag, char error, char *keepgoing), void (*on_complete)(int tag, char error)){
	static I2C_MSG msg;
	i2c_wait_cycle();

	msg.tag = tag;
	msg.func.stream = datastream;

	__I2C_CUR_STATUS.transmiting = 1;
	__I2C_CUR_STATUS.msgbuf = &msg;
	__I2C_CUR_STATUS.bufsize = 1;
	__I2C_CUR_STATUS.bufidx = 0;
	__I2C_CUR_STATUS.databuf_idx = -1;
	__I2C_CUR_STATUS.bytestreaming = 0;
	__I2C_CUR_STATUS.on_complete = on_complete;
	__I2C_CUR_STATUS.streamaux[0] = i2c_start(address, rw, 0);
	__I2C_CUR_STATUS.streamaux[1] = i2c_stop();

	msg.databuf = __I2C_CUR_STATUS.streamaux;

	__i2c_loopstep();
}

void i2c_msgstream(int tag, I2C_MSG* (*msgstream)(int tag, char *keepgoing), void (*on_complete)(int tag, char error)){
	i2c_wait_cycle();

	__I2C_CUR_STATUS.transmiting = 1;
	__I2C_CUR_STATUS.msgbuf = 0;
	__I2C_CUR_STATUS.bufsize = 0;
	__I2C_CUR_STATUS.bufidx = 0;
	__I2C_CUR_STATUS.databuf_idx = -1;
	__I2C_CUR_STATUS.bytestreaming = 0;
	__I2C_CUR_STATUS.msgstreaming = 1;
	__I2C_CUR_STATUS.msgstream = msgstream;
	__I2C_CUR_STATUS.on_complete = on_complete;

	__i2c_loopstep();
}

void i2c_wait_cycle(){
	while(__I2C_CUR_STATUS.transmiting);
}

void i2c_int_handle(){
	I2C_MSG *msg = __I2C_CUR_STATUS.last_msg;
	I2C_DATA *data = &__I2C_CUR_STATUS.last_data;
	char error = 0b00000000;

	if(data->controlbit && (data->bitselect == I2C_BTSEL_START || data->bitselect == I2C_BTSEL_RESTART) && SSPSTATbits.S){
		SSPBUF = data->value; //Escreve endereço do dispositivo sempre depois de um START ou RESTART
		data->controlbit = 0;
	}
	else if(data->controlbit && data->bitselect && data->bitselect != I2C_BTSEL_RESTART){
		if(SSPSTATbits.BF || SSPSTATbits.R_NOT_W){
			if(data->use_bytestream){
				char keepgoing = 1;
				SSPCON2bits.ACKDT = data->ret.rdstream(msg->tag, SSPCON2bits.ACKDT, SSPBUF, &keepgoing);
				__I2C_CUR_STATUS.bytestreaming = keepgoing ? 1 : 0;	
			}
			else{
				data->value--;
				SSPCON2bits.ACKDT = data->ret.callback(msg->tag, SSPCON2bits.ACKDT, SSPBUF);
			} 

			SSPCON2bits.ACKEN = 1;
		}
		else if(SSPSTATbits.P){
			if(!msg->use_datastream && msg->func.on_complete) msg->func.on_complete(msg->tag, 0);

			__I2C_CUR_STATUS.databuf_idx = -1;

			__i2c_loopstep();
		}
		else if(data->bitselect == I2C_BTSEL_READ && !SSPCON2bits.ACKEN){
			__i2c_loopstep();
		}
	}
	else{
		if(!data->use_bytestream && data->ret.on_complete) data->ret.on_complete(msg->tag, SSPCON2bits.ACKSTAT);

		__i2c_loopstep();
	}
}

void __i2c_loopstep(){
	if(__I2C_CUR_STATUS.databuf_idx != -1){
		if(!(__I2C_CUR_STATUS.databuf_idx && __I2C_CUR_STATUS.last_data.controlbit && (__I2C_CUR_STATUS.last_data.bitselect == I2C_BTSEL_READ) && __I2C_CUR_STATUS.last_data.value) && !__I2C_CUR_STATUS.bytestreaming){
			if(__I2C_CUR_STATUS.last_msg->use_datastream && __I2C_CUR_STATUS.databuf_idx == 1){
				char keepgoing = 1;
				__I2C_CUR_STATUS.last_data = __I2C_CUR_STATUS.last_msg->func.stream(__I2C_CUR_STATUS.last_msg->tag, SSPCON2bits.ACKSTAT, &keepgoing);
				if(!keepgoing) __I2C_CUR_STATUS.last_msg->use_datastream = 0;
			}
			else{
				__I2C_CUR_STATUS.last_data = __I2C_CUR_STATUS.last_msg->databuf[__I2C_CUR_STATUS.databuf_idx++];
			}
		}

		if(__I2C_CUR_STATUS.last_data.controlbit){
			SSPCON2 |= bitselection_table[__I2C_CUR_STATUS.last_data.bitselect];			
		}
		else if(__I2C_CUR_STATUS.last_data.use_bytestream){
			char keepgoing = 1;
			SSPBUF = __I2C_CUR_STATUS.last_data.ret.wrstream(__I2C_CUR_STATUS.last_msg->tag, SSPCON2bits.ACKSTAT, &keepgoing);
			__I2C_CUR_STATUS.bytestreaming = keepgoing ? 1 : 0;			
		}
		else{
			SSPBUF = __I2C_CUR_STATUS.last_data.value;
		}
	}
	else if(__I2C_CUR_STATUS.bufsize){
		__I2C_CUR_STATUS.last_msg = &__I2C_CUR_STATUS.msgbuf[__I2C_CUR_STATUS.bufidx++];
		__I2C_CUR_STATUS.databuf_idx = 0;
		__I2C_CUR_STATUS.bufsize--;

		__i2c_loopstep();
	}
	else if(__I2C_CUR_STATUS.msgstreaming){
		char keepgoing = 1;
		__I2C_CUR_STATUS.last_msg = __I2C_CUR_STATUS.msgstream(&keepgoing);
		__I2C_CUR_STATUS.msgstreaming = keepgoing ? 1 : 0;
	}
	else{
		__I2C_CUR_STATUS.transmiting = 0;
		if(__I2C_CUR_STATUS.on_complete) __I2C_CUR_STATUS.on_complete(__I2C_CUR_STATUS.last_msg->tag, 0);
	}
}

