#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "main.h"

/*
  
  Host Interface : Indirect Address Mode with 8-bit Data Bus

  Hardware Components:
  1.W5300-TOE-Shield  - (refer. https://docs.wiznet.io/Product/iEthernet/W5300/w5300-TOE-Shield)
  2.Raspberry Pi Pico - (pinout - https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf)

  * DATA0 to DATA7 connected to GP2 to GP9
  * ADDR0 to ADDR1 connected to GP10 to GP12
  * /RD - GP13
  * /WR - GP14
  * /CS - GP15
  

   SPDX-License-Identifier: BSD-3-Clause

*/

void W5300_Read_Enable(void){
    // RD Pin is Active Low
    gpio_put(Read_Enable_Pin, LOW);
    // Disable WR pin
    gpio_put(Write_Enable_Pin, HIGH);
}
    
void W5300_Read_Disable(void){
    gpio_put(Read_Enable_Pin, HIGH);
}
    
void W5300_Write_Enable(void){
    // WR Pin is Active Low
    gpio_put(Write_Enable_Pin, LOW);
    // Disable RD pin
    gpio_put(Read_Enable_Pin, HIGH);
}
    
void W5300_Write_Disable(void){
    gpio_put(Write_Enable_Pin, HIGH);
}
    
void W5300_Chip_Select_Enable(void){
    // CS Pin is Active Low
    gpio_put(Chip_Select_Pin, LOW);
}
    
void W5300_Chip_Select_Disable(void){
    gpio_put(Chip_Select_Pin, HIGH);
}

/* As DATA Pins (DATA0 to DATA7 ) will be used as Input or Output based 
   on READ/WRITE operation following functions are used to set 
   DATA0 to DATA7 as either inputs or outputs .
    
    * Config_Data_Bus_Output()  - to configure DATA0 to DATA7 as outputs
    * Config_Data_Bus_Input()   - to configure DATA0 to DATA7 as inputs
*/
void Config_Data_Bus_Output(void){
    for (int data_gpio = FIRST_DATA_PIN; data_gpio <= LAST_DATA_PIN ; data_gpio++) {
        gpio_init(data_gpio);
        gpio_set_dir(data_gpio, GPIO_OUT);
    }
} 
    
void Config_Data_Bus_Input(void){
    for (int data_gpio = FIRST_DATA_PIN; data_gpio <= LAST_DATA_PIN ; data_gpio++) {
        gpio_init(data_gpio);
        gpio_set_dir(data_gpio, GPIO_IN);
    }

}

void W5300_WriteAddress(__uint8_t address){
    gpio_put(ADDR0,((address & 0b00000001) >>0));
    gpio_put(ADDR1,((address & 0b00000010) >>1));
    gpio_put(ADDR2,((address & 0b00000100) >>2));
}

void W5300_Write_Data_D0_D7(__uint8_t data){
    Config_Data_Bus_Output();
    gpio_put(DATA0,((data & 0b00000001) >>0));
    gpio_put(DATA1,((data & 0b00000010) >>1));
    gpio_put(DATA2,((data & 0b00000100) >>2));
    gpio_put(DATA3,((data & 0b00001000) >>3));
    gpio_put(DATA4,((data & 0b00010000) >>4));
    gpio_put(DATA5,((data & 0b00100000) >>5));
    gpio_put(DATA6,((data & 0b01000000) >>6));
    gpio_put(DATA7,((data & 0b10000000) >>7));

}

__uint8_t W5300_Read_Data_D0_D7(void){
  
    __uint8_t d0_to_d7 = 0b00000000; 
    // DATA0
    if (gpio_get(DATA0) == 1){
        SET_BIT(d0_to_d7,0);
    }
    else{
        CLEAR_BIT(d0_to_d7,0);
    }

    // DATA1   
    if (gpio_get(DATA1) == 1){
        SET_BIT(d0_to_d7,1);
    }
    else{
        CLEAR_BIT(d0_to_d7,1);
    }
    // DATA2  
    if (gpio_get(DATA2) == 1){
        SET_BIT(d0_to_d7,2);
    }
    else{
       CLEAR_BIT(d0_to_d7,2);
    }
    // DATA3    
    if (gpio_get(DATA3) == 1){
        SET_BIT(d0_to_d7,3);
    }
    else{
        CLEAR_BIT(d0_to_d7,3);
    }
    // DATA4    
    if (gpio_get(DATA4) == 1){
       SET_BIT(d0_to_d7,4);
    }
    else{
        CLEAR_BIT(d0_to_d7,4);
    }
    // DATA5   
    if (gpio_get(DATA5) == 1){
        SET_BIT(d0_to_d7,5);
    }
    else{
        CLEAR_BIT(d0_to_d7,5);
    }
    // DATA6   
   if (gpio_get(DATA6) == 1){
       SET_BIT(d0_to_d7,6);
   }
    else{
        CLEAR_BIT(d0_to_d7,6);
    }
    // DATA7   
    if (gpio_get(DATA7) == 1){
        SET_BIT(d0_to_d7,7);
    }
    else{
        CLEAR_BIT(d0_to_d7,7);
    }

    // returns read 8 bit data
    return d0_to_d7;  

}

// W5300 S/W Reset
void W5300_SW_Reset(void){
    // 'RST' bit (7th bit ) of MR1 to 1
    // MR1 - High address register of MR(Address offset - 0x001) , Least Significant byte
    // 0x80 - 0b10000000
    W5300_Write_Data_D0_D7(0x80);
    // Address offset of MR1  - 0x01(0b001) 
    W5300_WriteAddress(0x01);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1);
    W5300_Chip_Select_Disable();
    sleep_us(1);
}

void W5300_Config_Indirect_Mode(void){
    // 'IND' bit (0th bit) of MR1 to 1
    // 0x01 -  0b00000001
    W5300_Write_Data_D0_D7(0x01);
    // Address offset of MR1  - 0x01(0b001)
    W5300_WriteAddress(0x01);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1);
    W5300_Chip_Select_Disable();
    sleep_us(1);
}

__uint16_t W5300_Bus_Read_Indirect(__uint16_t address){
    // 16 bits(1 Word) of data to be returned 
    __uint16_t read_word = 0;
    __uint8_t MS_Byte = 0;  // 8 - 15 bits
    __uint8_t LS_Byte = 0;  // 0 to 7 bits
    
    /*
    When W5300 operates as indirect address mode , Target 
    host system can access indirectly COMMON and Socket registers using "ONLY" 
    MR,IDM_AR, IDM_DR
    
    more more info. refer 4.2(Indirect Mode Register " section of datasheet
    */
    // The following are Indirect Mode Address Register 
    // Address offset of IDM_AR0  - 0x02  - MSB
    // Address offset of IDM_AR1  - 0x03  - LSB
    
   
    W5300_Write_Data_D0_D7(((address >> 8) & 0xFF ));  // MSB
    // Address offset of IDM_AR0  - 0x02  - MSB
    W5300_WriteAddress(0x02);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(5);
    W5300_Chip_Select_Disable();
    sleep_us(5);
     
     
    W5300_Write_Data_D0_D7((address & 0xFF ));        // LSB
    // Address offset of IDM_AR1  - 0x03  - LSB
    W5300_WriteAddress(0x03);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(5);
    W5300_Chip_Select_Disable();
    sleep_us(5);
    
    
    // Configure DATA0 to DATA7 as input
    Config_Data_Bus_Input();
    // The following are Indirect Mode Data Register 
    // Address offset of IDM_DR0  - 0x04  - MSB
    // Address offset of IDM_DR1  - 0x05  - LSB
    sleep_us(100);
    W5300_WriteAddress(0x04); // Address offset of IDM_DR0  - 0x04  - MSB
    W5300_Read_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(5);
    MS_Byte = W5300_Read_Data_D0_D7();
    W5300_Chip_Select_Disable();
    sleep_us(5);
    
    W5300_WriteAddress(0x05); // Address offset of IDM_DR1  - 0x05  - LSB
    W5300_Read_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(5);
    LS_Byte = W5300_Read_Data_D0_D7();
    W5300_Chip_Select_Disable();
    sleep_us(5);
    
    read_word = ((MS_Byte<<8)| LS_Byte);
    
    return read_word;
    
}

void W5300_Bus_Write_Indirect(__uint16_t address,__uint16_t data){
    // The following are Indirect Mode Address Register 
    // Address offset of IDM_AR0  - 0x02  - MSB
    // Address offset of IDM_AR1  - 0x03  - LSB
    
   
    W5300_Write_Data_D0_D7(((address >> 8) & 0xFF ));  // MSB
    // Address offset of IDM_AR0  - 0x02  - MSB
    W5300_WriteAddress(0x02);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1);
    W5300_Chip_Select_Disable();
    sleep_us(1);
    
    W5300_Write_Data_D0_D7((address & 0xFF ));        // LSB
    // Address offset of IDM_AR1  - 0x03  - LSB
    W5300_WriteAddress(0x03);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1); 
    W5300_Chip_Select_Disable();
    sleep_us(1);
    
    // The following are Indirect Mode Data Register 
    // Address offset of IDM_DR0  - 0x04  - MSB
    // Address offset of IDM_DR1  - 0x05  - LSB
    
    W5300_Write_Data_D0_D7(((data >> 8) & 0xFF ));  // MSB
    // Address offset of IDM_DR0  - 0x04  - MSB
    W5300_WriteAddress(0x04);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1);
    W5300_Chip_Select_Disable();
    sleep_us(1);
    
    W5300_Write_Data_D0_D7((data & 0xFF ));        // LSB
    // Address offset of IDM_DR1  - 0x05  - LSB
    W5300_WriteAddress(0x05);
    W5300_Write_Enable();
    W5300_Chip_Select_Enable();
    sleep_us(1);
    W5300_Chip_Select_Disable();
    sleep_us(1);

}

void W5300_Init(void){
    __uint16_t val = 0;  
    // Disable RD,WR,CS pin - Make these pin to 'HIGH' Logic Level
    W5300_Read_Disable();
    W5300_Write_Disable();
    W5300_Chip_Select_Disable();
    sleep_ms(100);
    W5300_SW_Reset();
    sleep_ms(500);  // with smaller time delay, I had issues during initilization 
    W5300_Config_Indirect_Mode();
    sleep_ms(500); // with smaller time delay, I had issues during initilization
    printf("Reading from 0xFE address \n");
    printf("..........................\n");
    val = W5300_Bus_Read_Indirect(0xFE);
    printf("Response is 0x%x  \n",val);
    if(val == 0x5300){  // If W5300 ID is returned ,  , ie; 0x5300
        printf("W5300  initialization success !! \n");
        printf("Writing 0xC000 to address 0x20A \n");
        W5300_Bus_Write_Indirect(0x20A,49152);
        val = W5300_Bus_Read_Indirect(0x20A);
        printf("Response is 0x%x \n",val);
        printf("READ & WRITE Success !!!\n");
    }
    else{
        printf("W5300  initialization  fail \n");
    }
    
}


int main() {
    stdio_init_all(); // Initialize all of the present standard stdio types that are linked into the binary. 
    printf("Lets begin \n");
    // We could use gpio_set_dir_out_masked() here

    /* configure GP10 to GP12 as outputs*/
    for (int addr_gpio = FIRST_ADDR_PIN; addr_gpio <= LAST_ADDR_PIN ; addr_gpio++) {
        gpio_init(addr_gpio);
        gpio_set_dir(addr_gpio, GPIO_OUT);
    }
    /* configure Read Enable, Write Enable and Chip Select as outputs */
    gpio_init(Read_Enable_Pin);
    gpio_set_dir(Read_Enable_Pin, GPIO_OUT);

    gpio_init(Write_Enable_Pin);
    gpio_set_dir(Write_Enable_Pin, GPIO_OUT);

    gpio_init(Chip_Select_Pin);
    gpio_set_dir(Chip_Select_Pin, GPIO_OUT);

    W5300_Init();
    for(;;){


    }





    

}


