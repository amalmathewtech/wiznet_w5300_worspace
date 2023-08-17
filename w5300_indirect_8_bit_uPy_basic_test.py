'''
Test Code for W5300 Indirect Address Mode with 8-bit Data Bus

Tutorial can be found at : https://tinkererway.com/2023/08/17/wiznets-w5300-indirect-address-mode-with-8-bit-data-bus/

'''

from machine import Pin
import utime


# GP2 to GP9
eight_bit_data_bus_pins = range(2, 10)


ADDR0 = Pin(10, Pin.OUT)
ADDR1 = Pin(11, Pin.OUT)
ADDR2 = Pin(12, Pin.OUT)

RD = Pin(13, Pin.OUT)
WR = Pin(14, Pin.OUT)
CS = Pin(15, Pin.OUT)

def Get_Bit(value, n):
    return ((value >> n & 1) != 0)

def Set_Bit(value, n):
    return value | (1 << n)

def Clear_Bit(value, n):
    return value & ~(1 << n)


def Config_Data_Bus_Output():
    global data_bus_pins_as_ouputs
    # >> type(data_bus_pins_as_ouputs) = list
    # configures GP2 to GP9 as Outputs
    data_bus_pins_as_ouputs = [Pin(pin, Pin.OUT) for pin in eight_bit_data_bus_pins]
    
    
def Config_Data_Bus_Input():
    global data_bus_pins_as_inputs
    # >> type(data_bus_pins_as_inputs) = list
    # configures GP2 to GP9 as inputs
    data_bus_pins_as_inputs = [Pin(pin, Pin.IN) for pin in eight_bit_data_bus_pins]
    
def W5300_WriteAddress(address):
    ADDR0.value((address & 0b00000001) >>0)
    ADDR1.value((address & 0b00000010) >>1)
    ADDR2.value((address & 0b00000100) >>2)
    
def W5300_Read_Enable():
    # RD Pin is Active Low
    RD.value(0)
    # Disable WR pin
    WR.value(1)
    
def W5300_Read_Disable():
    RD.value(1)
    
def W5300_Write_Enable():
    # WR Pin is Active Low
    WR.value(0)
    # Disable RD pin
    RD.value(1)
    
def W5300_Write_Disable():
    WR.value(1)
    
def W5300_Chip_Select_Enable():
    # CS Pin is Active Low
    CS.value(0)
    
def W5300_Chip_Select_Disable():
    CS.value(1)
    
def W5300_Write_Data_D0_D7(data):
    Config_Data_Bus_Output()
    data_bus_pins_as_ouputs[0].value((data & 0b00000001) >>0)
    data_bus_pins_as_ouputs[1].value((data & 0b00000010) >>1)
    data_bus_pins_as_ouputs[2].value((data & 0b00000100) >>2)
    data_bus_pins_as_ouputs[3].value((data & 0b00001000) >>3)
    data_bus_pins_as_ouputs[4].value((data & 0b00010000) >>4)
    data_bus_pins_as_ouputs[5].value((data & 0b00100000) >>5)
    data_bus_pins_as_ouputs[6].value((data & 0b01000000) >>6)
    data_bus_pins_as_ouputs[7].value((data & 0b10000000) >>7)
    
def W5300_Read_Data_D0_D7():
  
    d0_to_d7 = 0b00000000
    # DATA0
    if data_bus_pins_as_inputs[0].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,0)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,0)
    # DATA1   
    if data_bus_pins_as_inputs[1].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,1)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,1)
    # DATA2  
    if data_bus_pins_as_inputs[2].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,2)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,2)
    # DATA3    
    if data_bus_pins_as_inputs[3].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,3)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,3)
    # DATA4    
    if data_bus_pins_as_inputs[4].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,4)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,4)
    # DATA5   
    if data_bus_pins_as_inputs[5].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,5)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,5)
    # DATA6   
    if data_bus_pins_as_inputs[6].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,6)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,6)
    # DATA7   
    if data_bus_pins_as_inputs[7].value() == 1:
        d0_to_d7 = Set_Bit(d0_to_d7,7)
    else:
        d0_to_d7 = Clear_Bit(d0_to_d7,7)
    
    return d0_to_d7  # returns read 8 bit data
    
    

# W5300 S/W Reset
def W5300_SW_Reset():
    # 'RST' bit (7th bit ) of MR1 to 1
    # MR1 - High address register of MR(Address offset - 0x001) , Least Significant byte
    # 0x80 - 0b10000000
    W5300_Write_Data_D0_D7(0x80)
    # Address offset of MR1  - 0x01(0b001) 
    W5300_WriteAddress(0x01)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1)
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
def W5300_Config_Indirect_Mode():
    # 'IND' bit (0th bit) of MR1 to 1
    # 0x01 -  0b00000001
    W5300_Write_Data_D0_D7(0x01)
    # Address offset of MR1  - 0x01(0b001)
    W5300_WriteAddress(0x01)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    

def W5300_Init():
    # Disable RD,WR,CS pin - Make these pin to 'HIGH' Logic Level
    W5300_Read_Disable()
    W5300_Write_Disable()
    W5300_Chip_Select_Disable()
    utime.sleep_ms(100)
    W5300_SW_Reset()
    utime.sleep_ms(100)
    W5300_Config_Indirect_Mode()
    utime.sleep_ms(100)
    print("Reading from 0xFE address ")
    val = W5300_Bus_Read_Indirect(0xFE)
    print("response is "+ hex(val))
    print("Writing 0xC000 to address0x20A ")
    W5300_Bus_Write_Indirect(0x20A,49152)
    val = W5300_Bus_Read_Indirect(0x20A)
    print("response is "+ hex(val))
    
    
	
def W5300_Bus_Read_Indirect(address):
    # 16 bits(1 Word) of data to be returned 
    global read_word
    read_word = 0
    MS_Byte = 0 # 8 - 15 bits
    LS_Byte = 0 # 0 to 7 bits
    
    '''
    When W5300 operates as indirect address mode , Target 
    host system can access indirectly COMMON and Socket registers using "ONLY" 
    MR,IDM_AR, IDM_DR
    
    more more info. refer 4.2(Indirect Mode Register " section of datasheet
    '''
    # The following are Indirect Mode Address Register 
    # Address offset of IDM_AR0  - 0x02  - MSB
    # Address offset of IDM_AR1  - 0x03  - LSB
    
   
    W5300_Write_Data_D0_D7(((address >> 8) & 0xFF ))  # MSB
    # Address offset of IDM_AR0  - 0x02  - MSB
    W5300_WriteAddress(0x02)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
     
     
    W5300_Write_Data_D0_D7((address & 0xFF ))        # LSB
    # Address offset of IDM_AR1  - 0x03  - LSB
    W5300_WriteAddress(0x03)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    
    # Configure DATA0 to DATA7 as input
    Config_Data_Bus_Input()
    # The following are Indirect Mode Data Register 
    # Address offset of IDM_DR0  - 0x04  - MSB
    # Address offset of IDM_DR1  - 0x05  - LSB
    
    W5300_WriteAddress(0x04) #Address offset of IDM_DR0  - 0x04  - MSB
    W5300_Read_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    MS_Byte = W5300_Read_Data_D0_D7()
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    W5300_WriteAddress(0x05) # Address offset of IDM_DR1  - 0x05  - LSB
    W5300_Read_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    LS_Byte = W5300_Read_Data_D0_D7()
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    read_word = ((MS_Byte<<8)| LS_Byte)
    
    return read_word
    


def W5300_Bus_Write_Indirect(address, data):
    # The following are Indirect Mode Address Register 
    # Address offset of IDM_AR0  - 0x02  - MSB
    # Address offset of IDM_AR1  - 0x03  - LSB
    
   
    W5300_Write_Data_D0_D7(((address >> 8) & 0xFF ))  # MSB
    # Address offset of IDM_AR0  - 0x02  - MSB
    W5300_WriteAddress(0x02)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1)  
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    W5300_Write_Data_D0_D7((address & 0xFF ))        # LSB
    # Address offset of IDM_AR1  - 0x03  - LSB
    W5300_WriteAddress(0x03)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1)  
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    # The following are Indirect Mode Data Register 
    # Address offset of IDM_DR0  - 0x04  - MSB
    # Address offset of IDM_DR1  - 0x05  - LSB
    
    W5300_Write_Data_D0_D7(((data >> 8) & 0xFF ))  # MSB
    # Address offset of IDM_DR0  - 0x04  - MSB
    W5300_WriteAddress(0x04)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    
    W5300_Write_Data_D0_D7((data & 0xFF ))        # LSB
    # Address offset of IDM_DR1  - 0x05  - LSB
    W5300_WriteAddress(0x05)
    W5300_Write_Enable()
    W5300_Chip_Select_Enable()
    utime.sleep_us(1) 
    W5300_Chip_Select_Disable()
    utime.sleep_us(1)
    

W5300_Init()   
    
    
    






 
    
    
    
    
    
    
    
        

    
    
    
    
    

    


    
    


    

    

    
    




