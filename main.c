
/*
Программа будет считывать значение температуры с датчика в 9-ти битном разрешении 
с интервалом 300 мс. И полученные данные отправлять в USART1 для индикации в терминале. 
в USART1 отправлять через printf(). Так представление цифр будет верное.
*/


#include <stdio.h>

#include "stm32f4xx.h"

//#define SENSOR_CHECK_TIME_US	1000000	// 1_000_000 us = 1 s
#define SENSOR_CHECK_TIME_US	300000	// 300_000 us = 300 ms

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

//-------- 1-wire ERROR codes ---------------
#define OK_1WIRE		0
#define NO_DEVICE_1WIRE	1
#define CRC_ERR_1WIRE	2

//--------- параметры вычисления CRC для DS18B20 --------
#define CRC_POLYNOM		(uint8_t)0x31	// BIN = 1_0011_0001 берем младшие 8 бит
#define CRC_LEN_8_BITS		8

//------- константы для сканирования шины 1-Wire --------
#define ROM64_ZERO_BITS_CONDITION	0x01
#define ROM64_ONE_BITS_CONDITION	0x02
#define ROM64_DIFF_BITS_CONDITION	0x00
#define ROM64_NO_DEVICE_CONDITION	0x03
#define ROM64_BITS_CONDITION_MASK	0x03

#define MAX_1WIRE_DEVICES_NUMBER	16

#define ROM64_BYTE_LEN				8
#define ROM64_BIT_LEN				64
#define BYTE_LEN					8







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




uint8_t Start_1wire(void){	// функция начала транзакции 1-wire 
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


void WriteBit(uint8_t bit){	
	tx_mode_1wire();
	pull_low_1wire();
	Delay_us(5);
	if(bit != 0){
		release_1wire();
	}
	Delay_us(55);		// write bit slot time = 60 us
	rx_mode_1wire();
}




uint8_t ReadBit(void){	
	uint8_t bit = 0;
	tx_mode_1wire();
	pull_low_1wire();
	Delay_us(2);
	release_1wire();
	rx_mode_1wire();
	Delay_us(12);
	
	if(check_1wire()) bit = 1;	// if received one
	else bit = 0;				// if receive zero

	Delay_us(60-14);	// read bit slot time = 60 us
	tx_mode_1wire();
	release_1wire();
	Delay_us(2);		// пауза между битами 2 мкс
	return bit;
}






void WriteByte_1wire(uint8_t byte_value){
	uint8_t write_bit_code = 0;
	uint8_t tmp = 0;
	tx_mode_1wire();
	for(uint8_t i = 0; i < 8; i++){
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



uint8_t ReadByte_1wire(void){
	uint8_t rx_byte = 0;
	for(uint8_t i = 0; i < 8; i++){
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
uint8_t CRC_Calc(uint8_t mass[], uint8_t mass_size, uint8_t POLY){
	uint8_t crc = 0 , crc_out = 0;
	uint8_t in_data;
	uint8_t in_bits;
	for(uint8_t j = 0; j < mass_size; j++){
		in_data = mass[j];
		for(uint8_t i = 0; i < 8; i++){
			
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
	
	for(uint8_t i = 0; i < 8; i++){	// разворачиваем CRC биты в правильном порядке
		if(crc & (1 << i)) crc_out |= (1 << (7-i));
	}

	return crc_out;
	
}	
	
	



uint8_t Read_ROM64(uint8_t *family_code, uint8_t ser_num[], uint8_t *crc){
	uint8_t tmp_array[ROM64_BYTE_LEN];
	uint8_t crc_calculated = 0;
	uint8_t err_code = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(READ_ROM);
		Delay_us(100);
		*family_code = ReadByte_1wire();
		tmp_array[0] = *family_code;
		for(uint8_t i = 0; i < 6; i++){
			ser_num[i] = ReadByte_1wire();
			tmp_array[i+1] = ser_num[i];
		}
		*crc = ReadByte_1wire();
		tmp_array[7] = *crc;
		printf("================= \n");
		printf("==== READ ROM 64 bits .... \n");
		printf("==== SCRATCH = ");
		for (uint8_t i = 0; i < ROM64_BYTE_LEN; i++){

			printf("0x%X ", tmp_array[i]);

		}
		printf("\n==== CRC Rx = 0x%X \n", tmp_array[7]);

		crc_calculated = CRC_Calc(tmp_array, 7, CRC_POLYNOM);
		printf("==== CRC calculated = 0x%X \n" , crc_calculated);
		
		if(crc_calculated == tmp_array[7]) return OK_1WIRE;
		else return CRC_ERR_1WIRE;	// error ROM64 read  
	}
	else{
		return NO_DEVICE_1WIRE;			// error. 1-wire device are not found
	}

}






// функция сканирования шины 1wire для выделения ROM64 из нескольких устройств
uint8_t ScanROM(uint8_t ROM64_array_prev[],	// массив uint8_t ROM_64[8] с предыдущим значением
				uint8_t ROM64_array[],		//  массив uint8_t ROM_64[8] с новым значением
				uint8_t branches[]			// массив с разночтениями: 1 - разночтение в позиции бита, 0 - нет разночтений.
				){
	 
	uint8_t rx_bits = 0;
	uint8_t rx_byte_val = 0; 
	uint8_t byte_number = 0; 
	uint8_t bit_num = 0;
	uint8_t byte_branches = 0;
	uint8_t bit_mode_select = 0;

	if(!Start_1wire()){	
		WriteByte_1wire(SEARCH_ROM);
		printf("======= Searching ROM.... \n");
		printf("========================== \n");
		Delay_us(100);
		
		for(uint8_t i = 0; i < ROM64_BIT_LEN; i++){
			byte_number = i / BYTE_LEN;
			bit_num = i % BYTE_LEN;
			rx_bits = 0x00;		// clear rx_bits

			if(bit_num == 0) {
				byte_branches = 0;	// clear byte_branches for next byte
				rx_byte_val = 0;	// clear rx byte value
			}

			if(ReadBit()) rx_bits |= (1 << 1);	// read bit value
			
			if(ReadBit()) rx_bits |= (1 << 0);	// read bit NOT_value
			
			bit_mode_select = rx_bits & ROM64_BITS_CONDITION_MASK;

			switch(bit_mode_select){
			case ROM64_ZERO_BITS_CONDITION:
				WriteBit(0);
				break;

			case ROM64_ONE_BITS_CONDITION:
				rx_byte_val |= (1 << bit_num);
				ROM64_array[byte_number] = rx_byte_val;
				WriteBit(1);
				break;

			case ROM64_DIFF_BITS_CONDITION:
				byte_branches |= (1 << bit_num);
				branches[byte_number] = byte_branches;
				if(ROM64_array_prev[byte_number] & (1 << bit_num)){	// send bit value from ROM64_array_prev in diffenent readed bit
					WriteBit(1);	
				}
				else{
					WriteBit(0);	
				}
				break;

			case ROM64_NO_DEVICE_CONDITION:
				return NO_DEVICE_1WIRE;
				//NULL;
				break;
		
			
			}	// switch (ROM64_BITS_CONDITION_MASK)
		
		/******** FOR DEBUG ONLY ********
		printf("bit_num = %d, \t byte_num = %d, \t rx_bits = 0x%X, \t rx_byte_val = 0x%X \n", 
				bit_num,		byte_number,		rx_bits,		rx_byte_val);

		********************************/

		}	// for ROM64_BITS_NUMBER

		return OK_1WIRE;
	}
}

/*
	функция ищет в массиве branches[] единицы, начиная со старших байтов и старших битов
	если есть единица, то проверяется значение бита в массиве ROM64 в этой же позиции.
	если в ROM64 стоит 0, то в ROM_prev[] в эту позицию записывается 1. 
	И в ROM_prev[] копируются все биты из ROM64[] до этой позиции.
	После этой позиции в ROM_prev[] записываются нули.
*/ 
void BranchesScan( uint8_t branches[], uint8_t ROM64[], uint8_t ROM_prev[] ){}



/*
	Функция поиска "правильной" единицы в массиве branches[] и выдает номер бита и байта ее позиции
*/
uint8_t FindOnesBranches( uint8_t branches[], 
						uint8_t ROM64[], 
						uint8_t ROM_prev[], 
						uint8_t *bit_num,
						uint8_t *byte_num ){
	uint8_t branches_byte;
	uint8_t bit = 0;
	uint8_t byte = 0;

	if(byte < ROM64_BYTE_LEN) {
		
		branches_byte = branches[ROM64_BYTE_LEN - byte];

		if(bit < BYTE_LEN ){
		
			if(branches_byte & (1 << (BYTE_LEN - bit))){
				if(ROM64[ROM64_BYTE_LEN - byte] & (1 << (BYTE_LEN - bit))){	// 
					///////!! дописать код здесь!!!
				}
				else{	// это новая ветка.
					*bit_num = (BYTE_LEN - bit);
					*byte_num = (ROM64_BYTE_LEN - byte);
					return 0;
					
				}
			}
			bit++;
		}	
		byte++;
		bit = 0;
	}



}




/* Функция сканирования шины 1-wire для поиска всех устройств и их ROM64
 
	первым находит устройство с минимальным значением кода ROM64
	сортировка устройств по возрастанию кода ROM64 
*/

uint8_t Scan_1Wire(uint8_t ROMs_array[MAX_1WIRE_DEVICES_NUMBER][8]){	// двумерный массив с найденными ROM64 устройств
	
	uint8_t ROM64_Rx[MAX_1WIRE_DEVICES_NUMBER][ROM64_BYTE_LEN] = {};	// нужно хранить всю последовательность ROM-кодов
	uint8_t branch_bits[MAX_1WIRE_DEVICES_NUMBER][ROM64_BYTE_LEN] = {};	// нужно хранить всю последовательность ветвлений в битах
	uint8_t ROM64_Prev[ROM64_BYTE_LEN] = {};
	
	uint8_t err = 0;
	
	for (uint8_t i = 0; i < ROM64_BYTE_LEN; i++) ROM64_Prev[i] = 0;	// clear ROM64_Prev

	/***** Алгоритм сканирования шины 1-Wire ***********
	1. ScanROM() - получаем массивы ROM_rx[] и branch_bits[]
	2. поиск единицы в branch_bits[] начиная со старшего байта и старшего бита
		2.1 если нет единиц, то на шине было всего одно устройство и его ROM64 мы теперь знаем 
		2.2.если найдена единица в branch_bits[], то проверяем значение бита в этой же позиции в массиве ROM64_rx[].
			2.2.1. если в этой позиции 0, то это точка ветвления в 1 при следующем запуске ScanROM().
					Копируем с массив ROM64_Prev[] все биты до этой позиции (младше этой позиции), 
					а в эту позицию ставим 1. И в следующие позиции пишем нули.
			2.2.2. если в этой позиции 1. То это отработанный узел. 
				Ищем следующую единицу в branch_bits[] в меньших значащих битах. Возврат к п. 2.2.
	3. В массиве ROM_rx 

	*****************************************************/
	for(uint8_t dev_num = 0; dev_num < MAX_1WIRE_DEVICES_NUMBER; dev_num++){
		err = ScanROM(ROM64_Prev, ROM64_Rx[dev_num], branch_bits[dev_num]);
		if(! err) {
			// ------- поиск единиц в branch_bits[dev_num] и формирование нового ROM64_Prev[] ------------

		}
		else{
			printf("---- ERROR: 1-Wire devices not found \n");
		}
	}





	/*******************************************
	if(!ScanROM(ROM64_Prev, ROM64_Rx, branch_bits)){
		printf("ROM64 RX = ");
		for(uint8_t i = 0; i < ROM64_BYTE_LEN; i++){
			printf("0x%X \t", ROM64_Rx[i]);
		}
		printf("\n");

		printf("Branches = ");
		for(uint8_t i = 0; i < ROM64_BYTE_LEN; i++){
			printf("0x%X \t", branch_bits[i]);
		}
		printf("\n");

		

	}
	else{
		printf("---- ERROR: 1-Wire devices not found \n");
	}
	**********************************************/
}




/******* BAK function Scan_1Wire() *****************

uint8_t Scan_1Wire(uint8_t ROMs_array[MAX_1WIRE_DEVICES_NUMBER][8]){	// двумерный массив с найденными ROM64 устройств
	
	uint8_t ROM64_Rx[ROM64_BYTE_LEN] = {};
	uint8_t ROM64_Prev[ROM64_BYTE_LEN] = {};
	uint8_t branch_bits[ROM64_BYTE_LEN] = {};
	
	for (uint8_t i = 0; i < ROM64_BYTE_LEN; i++) ROM64_Prev[i] = 0;	// clear ROM64_Prev

	if(!ScanROM(ROM64_Prev, ROM64_Rx, branch_bits)){
		printf("ROM64 RX = ");
		for(uint8_t i = 0; i < ROM64_BYTE_LEN; i++){
			printf("0x%X \t", ROM64_Rx[i]);
		}
		printf("\n");

		printf("Branches = ");
		for(uint8_t i = 0; i < ROM64_BYTE_LEN; i++){
			printf("0x%X \t", branch_bits[i]);
		}
		printf("\n");

	}
	else{
		printf("---- ERROR: 1-Wire devices not found \n");
	}
}

*********************************/


uint8_t ReadScratchpad(uint8_t scratch_array[]){
	uint8_t err_code = 0;
	uint16_t temp = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(SKIP_ROM);
		Delay_us(100);
		WriteByte_1wire(READ_SCRATCH);
		Delay_us(100);
		for(uint8_t i = 0; i < 9; i++){	// read all 9 bytes from scratchpad
			scratch_array[i] = ReadByte_1wire();
			Delay_us(100);
		}						
		return OK_1WIRE;
	}
	else{
		return NO_DEVICE_1WIRE;
	}
}





uint8_t Convert_Temperature(void){
	uint8_t err_code = 0;
	uint16_t temp = 0;
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(SKIP_ROM);
		Delay_us(100);
		WriteByte_1wire(CONVERT_T);
		Delay_us(100);
		return OK_1WIRE;
	}
	else{
		return NO_DEVICE_1WIRE;
	}
}


uint8_t WriteScratch(uint8_t tx_array[]){	// write only 3 byties from array [0 1 2 ] will be writed
	if(!Start_1wire()){			// 1-wire device found
		WriteByte_1wire(SKIP_ROM);
		Delay_us(100);
		WriteByte_1wire(WRITE_SCRATCH);
		Delay_us(100);
		for(uint8_t i = 0; i < 3; i++){		// write only 3 byties from tx_array 
			WriteByte_1wire(tx_array[i]);
			Delay_us(100);
		}	
		Start_1wire();		// final reset pulse					
		return OK_1WIRE;
	}
	else{
		return NO_DEVICE_1WIRE;
	}
}





void SysTick_Handler(void){		// прервание от Systick таймера, выполняющееся с периодом 1 мкс
	us_count++;			
	delay_us_count++;
}







int main(void){
	uint8_t error_1wire = 0;		// 0 = OK, 1 = ERROR;
	uint8_t family_byte = 0;
	uint8_t ser_number[6] = {};
	uint8_t crc_rx = 0;
	uint8_t scratch_mem[9] = {};
	uint16_t temper;
	uint16_t temper_fract;
	float temper_float;
	uint8_t ROM_1wire_devices[MAX_1WIRE_DEVICES_NUMBER][8] = {};	// двуменрый массив на 16 ROM64 
	

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
		//------ Scan 1-wire bus. Detect Device ROM64 ------------
		if(!Scan_1Wire(ROM_1wire_devices)){
			printf("=== 1-Wire bus scanned ==== \n");
		}
		else {
			printf("---- ERROR: 1-Wire bus have errors \n");
		}

		
		
		/*************** Scan ROM test *****************

		if(!ScanROM(ROM64_Prev, ROM64_Rx, branch_bits)){
			printf("ROM64 RX = ");
			for(uint8_t i = 0; i < 8; i++){
				printf("0x%X \t", ROM64_Rx[i]);
			}
			printf("\n");

			printf("Branches = ");
			for(uint8_t i = 0; i < 8; i++){
				printf("0x%X \t", branch_bits[i]);
			}
			printf("\n");

		}
		else{
			printf("---- ERROR: 1-Wire devices not found \n");
		}
		******************************************************/




		/********** ReadROM test **********************

		error_1wire = Read_ROM64(&family_byte, ser_number, &crc_rx);
		if( error_1wire == OK_1WIRE ){
		
			
			printf("+++ DS18B20 found +++ \n");
			printf("+++ FAMILY_CODE = %X \n", family_byte);
			printf("+++ SERIAL NUMBER = ");

			for(uint8_t i = 0; i < 6; i++){ 
				printf("%X ", ser_number[i]);
			}
			printf("\n");
			
		}
		else{
			printf("---- ERROR: 1-Wire DS18B20 not found \n");
		}
		***********************************************/



		
		/******************************
		//------- CONFIG settings for DS18B20 ----------
		scratch_mem[0] = 0x64;			// TH = 0x64 = 100	 
		scratch_mem[1] = 0x0A;			// TL = 0x0A = 10
		scratch_mem[2] = 0x1F;			// CONFIG = 0x1F; 9-bit temperature format
		
		//------- config, start temper conversion, read temperature --------
		WriteScratch(scratch_mem);	// write config scratchpad 

		Convert_Temperature(); // convert_t

		while(us_count < SENSOR_CHECK_TIME_US){};
		us_count = 0;
		
		error_1wire = ReadScratchpad(scratch_mem);
		if(error_1wire == OK_1WIRE){

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

		*************************/
		
	}	// while(1)

	return 0;
}	// main()
