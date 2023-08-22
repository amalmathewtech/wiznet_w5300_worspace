#ifndef _MAIN_H_
#define _MAIN_H_


/* C/C++ Macro & Bit Operations  */
#define SET_BIT(byte,nbit)   ((byte) |=  (1<<(nbit)))
#define CLEAR_BIT(byte,nbit) ((byte) &= ~(1<<(nbit)))
#define FLIP_BIT(byte,nbit)  ((byte) ^=  (1<<(nbit)))
#define CHECK_BIT(byte,nbit) ((byte) &   (1<<(nbit)))


#define HIGH 1
#define LOW  0



/* DATA Pins */

#define DATA0 2
#define DATA1 3
#define DATA2 4
#define DATA3 5
#define DATA4 6
#define DATA5 7
#define DATA6 8
#define DATA7 9



#define FIRST_DATA_PIN 2  // DATA0 - GP2
#define LAST_DATA_PIN  9  // DATA7 - GP9

/* Address Pins */


#define ADDR0 10
#define ADDR1 11
#define ADDR2 12

#define FIRST_ADDR_PIN 10
#define LAST_ADDR_PIN  12

/* Read Enable , WRite Enable and Chip Select Pin */
#define RD 13
#define WR 14
#define CS_PIN 15

#define Read_Enable_Pin      RD
#define Write_Enable_Pin     WR 
#define Chip_Select_Pin      CS_PIN



void W5300_Read_Enable(void);
void W5300_Read_Disable(void);
void W5300_Write_Enable(void);
void W5300_Write_Disable(void);
void W5300_Chip_Select_Enable(void);
void W5300_Chip_Select_Disable(void);


void Config_Data_Bus_Output(void);
void Config_Data_Bus_Input(void);
void W5300_WriteAddress(uint8_t address);

void W5300_Write_Data_D0_D7(uint8_t data);
uint8_t W5300_Read_Data_D0_D7(void);

void W5300_SW_Reset(void);
void W5300_Config_Indirect_Mode(void);


uint16_t W5300_Bus_Read_Indirect(uint16_t address);
void W5300_Bus_Write_Indirect(uint16_t address,uint16_t data);

void W5300_check(void);

int8_t socket_connect(uint8_t sn, uint8_t * addr, uint16_t port);
void send_tcp(uint8_t sn, uint8_t * buf, uint16_t len);



#endif