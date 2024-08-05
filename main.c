
/*
Программа будет считывать значение температуры с датчика в 9-ти битном разрешении 
с интервалом 300 мс. И полученные данные отправлять в USART1 для индикации в терминале. 
в USART1 отправлять через printf(). Так представление цифр будет верное.
*/


#include <stdio.h>

#include "stm32f4xx.h"

#define SENSOR_CHECK_TIME_US	1000000	// 1_000_000 us = 1 s

// --------- 1 wire подстановки для лучшей читабельности кода -------
#define release_1wire()		(GPIOE -> BSRR |= GPIO_BSRR_BS2)	// для этого конфигурировать выход в режиме open-drain
#define pull_low_1wire()	(GPIOE -> BSRR |= GPIO_BSRR_BR2)
#define rx_mode_1wire()		(GPIOE -> MODER &= ~(GPIO_MODER_MODE2_0))
#define tx_mode_1wire()		(GPIOE -> MODER	|= GPIO_MODER_MODE2_0)
#define check_1wire()		(GPIOE -> IDR & GPIO_IDR_ID2)

//-------- 1 wire ROM commands -----------------
#define READ_ROM		0x33
#define MATCH_ROM		0x55
#define SKIP_ROM		0xCC
#define SEARCH_ROM		0xF0
#define ALARM_SEARCH	0xEC

//------- 1-wire memory command ---------------
#define READ_SCRATCH	0xBE
#define WRITE_SCRATCH	0x4E
#define COPY_SCRATCH	0x48
#define CONVERT_T		0x44
#define RECALL_E2		0xB8
#define READ_PWR		0xB4

//--------- параметры вычисления CRC для DS18B20 --------
#define CRC_POLYNOM		(char)0x31	// BIN = 1_0011_0001 берем младшие 8 бит
#define CRC_LEN_8_BITS		8



uint32_t us_count = 0;
uint32_t delay_us_count = 0;



void RCC_Init(void);



void GPIO_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	
	//-------- GPIO for buttons -------------------
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD10_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD11_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD12_0;
	   
	//-------- GPIO settings for LED1 LED2 LED3 --------
	GPIOE -> MODER |=GPIO_MODER_MODE13_0;
	GPIOE -> MODER |=GPIO_MODER_MODE14_0;
	GPIOE -> MODER |=GPIO_MODER_MODE15_0;

	//--------- GPIO settings for 1-WIRE PE2-pin ----------------
	GPIOE -> MODER	|= GPIO_MODER_MODE2_0;	// PE2 output mode
	GPIOE -> OTYPER |= GPIO_OTYPER_OT2;		// PE2 output open-drain
	 
}






// настройка USART1 для передачи принятых данных в ПК по USART1
void USART1_Init(void){	
	
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;							// включение тактирования GPIOA: PA9 = TX, PA10 = RX
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;							// включение тактирования USART1 от шины APB2

	
	GPIOA -> MODER  |= GPIO_MODER_MODE9_1;							// Альтернативная функция для PA9 (USART1 - TX)
	GPIOA -> AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);					// AF7 для PA9
	GPIOA -> MODER  |= GPIO_MODER_MODE10_1;                         // Альтернативная функция для PA10 (USART1 - RX)
	GPIOA -> AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);				// AF7 для PA10

	
	/* Расчет скорости передачи данных:
		(84МГц/115200)/16 = 45.57; 
		Целая часть = 45 = 0x2D; 
		Дробная часть = 0.57*16 = 9 = 0x09 
	*/

	USART1 -> BRR |= 0x2D9;	// 115200
	
	/* Включение приемника и передатчика */
	USART1 -> CR1 |= USART_CR1_TE | USART_CR1_RE; 
	USART1 -> CR1 &= ~(USART_CR1_M) | ~(USART_CR1_PCE);              // 8-бит, без контроля четности
	USART1 -> CR2 &= ~(USART_CR2_STOP);                              // 1 стоповый бит
	USART1 -> CR1 |= USART_CR1_UE;                                   // Включение USART1

}




// перенаправление printf() в USART1 для удобства отображения температуры дробной
// retarget the C library printf function to the USART 
//retarget the C library printf function to the USART 
int __SEGGER_RTL_X_file_write(__SEGGER_RTL_FILE *__stream, const char *__s, unsigned __len) {
  
	// Send string over USART1 in pending mode 
	for (; __len != 0; --__len) {
		USART1->DR = * __s++;
		while (RESET == READ_BIT(USART1->SR, USART_SR_TXE));
	} 
	
	return 0;
}





void Delay_us(uint32_t us_number){		// фунукция задержки на 1 мкс
	delay_us_count = 0;
	while(delay_us_count < us_number){};
}




char Start_1wire(void){	// функция начала транзакции 1-wire 
	tx_mode_1wire();
	pull_low_1wire();	// Master reset pulse
	Delay_us(500);		
	release_1wire();
	rx_mode_1wire();	
	Delay_us(100);		// wait 100 us = 60 us pause + 40 us presence pulse
	if(check_1wire()){
		return 1;	// no presence pulse from 1-wire device
	}
	else{
	Delay_us(200);
		return 0;	// received presence pulse from 1-wire device
	}
}




void WriteByte_1wire(char byte_value){
	char write_bit_code = 0;
	char tmp = 0;
	tx_mode_1wire();
	for(char i = 0; i < 8; i++){
		write_bit_code = 0;
		tmp = (1 << i);
		write_bit_code = (byte_value & tmp);
		pull_low_1wire();
		Delay_us(5);
		if(write_bit_code != 0){
			release_1wire();
		}
		Delay_us(55);		// write bit slot time = 60 us
		release_1wire();
		Delay_us(2);	// пауза между битами 2 мкс
	}
	rx_mode_1wire();
}



char ReadByte_1wire(void){
	char rx_byte = 0;
	for(char i = 0; i < 8; i++){
		tx_mode_1wire();
		pull_low_1wire();
		Delay_us(2);
		release_1wire();
		rx_mode_1wire();
		Delay_us(12);
		if(check_1wire()){	// if received one
			rx_byte |= (1 << i);
		}
		Delay_us(60-14);	// read bit slot time = 60 us
		tx_mode_1wire();
		release_1wire();
		Delay_us(2);		// пауза между битами 2 мкс
	}
	return  rx_byte;
}



/*
	Алгоритм вычислени CRC:
	========================
	Циклически сдвигаем CRC и вычисляем бит shift_in_bit = CRC[7] XOR data_bit_in
	Если shift_in_bit == 1, то после сдвига выполняем еще CRC XOR CRC_POLY
	Пока не кончатся биты в последовательности данных
	data_bit_in - это младший бит в байте. 
	В CRC в младший бит задвигаются байты входных данных начиная с младшего бита.
*/
unsigned char CRC_Calc(unsigned char mass[], unsigned char mass_size, unsigned char POLY){
	unsigned char crc = 0 , crc_out = 0;
	unsigned char in_data;
	unsigned char in_bits;
	for(char j = 0; j < mass_size; j++){
		in_data = mass[j];
		for(char i = 0; i < 8; i++){
			
			if(((crc & 0x80) >> 7) != (in_data & 0x01)){
				crc = crc << 1;
				crc = crc ^ POLY;
			}
			else{
				crc = crc << 1;
			}
			in_data = in_data >> 1;
		}
	}
	
	for(char i = 0; i < 8; i++){	// разворачиваем CRC биты в правильном порядке
		if(crc & (1 << i)) crc_out |= (1 << (7-i));
	}

	return crc_out;
	
}	
	
	



char Read_ROM64(char *family_code, char ser_num[], char *crc){
	unsigned char tmp_array[8];
	unsigned char crc_value = 0;
	char err_code = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(READ_ROM);
		Delay_us(100);
		*family_code = ReadByte_1wire();
		tmp_array[0] = *family_code;
		for(char i = 0; i < 6; i++){
			ser_num[i] = ReadByte_1wire();
			tmp_array[i+1] = ser_num[i];
		}
		*crc = ReadByte_1wire();
		tmp_array[7] = *crc;
		printf("================= \n");
		printf("==== READ ROM 64 bits .... \n");
		printf("==== SCRATCH = ");
		for (char i = 0; i < 8; i++){

			printf("0x%X ", tmp_array[i]);

		}
		printf("\n==== CRC Rx = 0x%X \n", tmp_array[7]);

		crc_value = CRC_Calc(tmp_array, 7, CRC_POLYNOM);
		printf("==== CRC calculated = 0x%X \n" , crc_value);
		
		return 0;
	}
	else{
		return 1;				// error. 1-wire device are not found
	}

}

char ReadScratchpad(char scratch_array[]){
	char err_code = 0;
	uint16_t temp = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(SKIP_ROM);
		Delay_us(100);
		WriteByte_1wire(READ_SCRATCH);
		Delay_us(100);
		for(char i = 0; i < 9; i++){	// read all 9 bytes from scratchpad
			scratch_array[i] = ReadByte_1wire();
			Delay_us(100);
		}						
		return 0;
	}
	else{
		return 1;
	}
}





char Convert_Temperature(void){
	char err_code = 0;
	uint16_t temp = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(SKIP_ROM);
		Delay_us(100);
		WriteByte_1wire(CONVERT_T);
		Delay_us(100);
		return 0;
	}
	else{
		return 1;
	}
}







void SysTick_Handler(void){		// прервание от Systick таймера, выполняющееся с периодом 1 мкс
	us_count++;			
	delay_us_count++;
}







int main(void){
	char error_1wire = 0;		// 0 = OK, 1 = ERROR;
	char family_byte = 0;
	char ser_number[6] = {};
	char crc_rx = 0;
	char scratch_mem[9] = {};
	uint16_t temper;
	uint16_t temper_fract;
	float temper_float;

	RCC_Init();

	GPIO_Init();

	USART1_Init();

	SysTick_Config(84);	// SysTick interrupt 1 us

	
	//---- turn off leds ---------- 
	GPIOE -> BSRR |= GPIO_BSRR_BS13;
	GPIOE -> BSRR |= GPIO_BSRR_BS14;
	GPIOE -> BSRR |= GPIO_BSRR_BS15;

	release_1wire();

	
	while(1){
		error_1wire = Read_ROM64(&family_byte, ser_number, &crc_rx);
		if( !error_1wire ){
			
			printf("+++ DS18B20 found +++ \n");
			printf("+++ FAMILY_CODE = %X \n", family_byte);
			printf("+++ SERIAL NUMBER = ");

			for(char i = 0; i < 6; i++){ 
				printf("%X ", ser_number[i]);
			}
			printf("\n");
			printf("+++ CRC RX = %X \n", crc_rx);

		}
		else{
			printf("---- ERROR: 1-Wire DS18B20 not found \n");
		}

		Convert_Temperature(); // convert_t

		while(us_count < SENSOR_CHECK_TIME_US){};
		us_count = 0;
		
		error_1wire = ReadScratchpad(scratch_mem);
		if(!error_1wire){

			if(scratch_mem[1] < 0x04){	// если положительные температуры
				temper = ((scratch_mem[1] << 8 ) & 0x0700) + scratch_mem[0];
				temper_fract = ((temper & 0x000F)*1000) >> 4;
				temper_float = (float)temper / 16;

				printf("=== Temper = %d.%d \n", (temper >> 4), temper_fract);
				
			}
			else{	// если отрицательные температуры (ПРОВЕРИТЬ ВЫЧИСЛЕНИЯ!!)
				temper = 0xFFFF - (((scratch_mem[1] << 8 ) & 0x0700) + scratch_mem[0]) + 1;
				temper_fract = (((0x000F - (temper & 0x000F))*1000) >> 4);
				
				printf("=== Temper = -%d.%d \n", (temper >> 4), temper_fract);
			}
			// Вычисления дробной температуры. Возможно одинаковый алгоритм для + и - температур. 
			//ПРОВЕРИТЬ!
			temper_float = (float)temper / 16;
			printf("=== Temper float = %f \n", temper_float);
		}
		
	}	// while(1)

	return 0;
}	// main()
