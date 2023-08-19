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
#define CS 15

#define Read_Enable_Pin      RD
#define Write_Enable_Pin     WR 
#define Chip_Select_Pin      CS 



void W5300_Read_Enable(void);
void W5300_Read_Disable(void);
void W5300_Write_Enable(void);
void W5300_Write_Disable(void);
void W5300_Chip_Select_Enable(void);
void W5300_Chip_Select_Disable(void);


void Config_Data_Bus_Output(void);
void Config_Data_Bus_Input(void);
void W5300_WriteAddress(__uint8_t address);

void W5300_Write_Data_D0_D7(__uint8_t data);
__uint8_t W5300_Read_Data_D0_D7(void);

void W5300_SW_Reset(void);
void W5300_Config_Indirect_Mode(void);


__uint16_t W5300_Bus_Read_Indirect(__uint16_t address);
void W5300_Bus_Write_Indirect(__uint16_t address,__uint16_t data);



#endif