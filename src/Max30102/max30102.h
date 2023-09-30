#ifndef MAX30102_H_
#define MAX30102_H_

#include <zephyr/types.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

// Things what i add
#include <stdio.h>
#include <zephyr/drivers/i2c.h>

#define SYS_TIME_SLEEP_MS 10U

#define MAX30102_MEASUREMENT_SECONDS 5U
#define MAX30102_SAMPLES_PER_SECOND	(uint8_t)3200 // 50, 100, 200, 400, 800, 100, 1600, 3200 sample rating
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES 17U // used for IRQ

#define OMIT_FIFO 1  // enable = 1   disable = 0 
#define RESET_DURING_INIT 0  // enable = 1   disable = 0 
// #define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)


#define MAX30102_ADDRESS                   0x57 // Address MAX30102
#define MAX30102_IRQ_STATUS_1              0x00
#define MAX30102_IRQ_STATUS_2              0x01
#define MAX30102_IRQ_ENABLE_1              0x02
#define MAX30102_IRQ_ENABLE_2              0x03
#define MAX30102_FIFO_WRITE_PTR            0x04
#define MAX30102_OVERFLOW_COUNT            0x05
#define MAX30102_FIFO_READ_PTR             0x06
#define MAX30102_REG_FIFO_DATA             0x07
#define MAX30102_FIFO_CONFIG               0x08
#define MAX30102_MODE_CONFIG               0x09
#define MAX30102_SPO2_CONFIG               0x0A
#define MAX30102_LED1_PULSE_AMPLITUDE      0x0C
#define MAX30102_LED2_PULSE_AMPLITUDE      0x0D
#define MAX30102_PARTID                    0xFF//0x15
#define MAX30102_RESET                     0x40


#define FIFO_CONF_SMP_AVE_BIT 			    7 // FIFO Configuration (0x08)   
#define FIFO_CONF_SMP_AVE_LENGHT 		    3
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 	    4
#define FIFO_CONF_FIFO_A_FULL_BIT 		    3
#define FIFO_CONF_FIFO_A_FULL_LENGHT 	    4

#define FIFO_SMP_AVE_1	                    0
#define FIFO_SMP_AVE_2	                    1
#define FIFO_SMP_AVE_4	                    2
#define FIFO_SMP_AVE_8	                    3
#define FIFO_SMP_AVE_16	                    4
#define FIFO_SMP_AVE_32	                    5


                                              // Mode Config (0x09)
#define MODE_HEART_RATE						2 // red only	
#define MODE_SpO2		                	3 // red & IR
#define MODE_Multi_LED		 				7 // red & IR


#define SPO2_CONF_LED_PW_BIT		        1 //SpO2 Config (0x0A)
#define SPO2_CONF_LED_PW_LENGTH		        2
#define SPO2_CONF_SR_LENGTH			        3
#define SPO2_CONF_SR_BIT			        4
#define SPO2_CONF_ADC_RGE_LENGTH	        2
#define SPO2_CONF_ADC_RGE_BIT		        6


#define	SPO2_ADC_RGE_2048	                0
#define	SPO2_ADC_RGE_4096	                1
#define	SPO2_ADC_RGE_8192	                2
#define	SPO2_ADC_RGE_16384	                3

#define MODE_HEART_RATE	                    2
#define MODE_SPO2		                    3
#define MODE_MULTI  	                    7

#define	PULSE_WIDTH_69			            0
#define	PULSE_WIDTH_118		                1
#define	PULSE_WIDTH_215		                2
#define	PULSE_WIDTH_411		                3

#define IR_LED_CURRENT_LOW		            0x07
#define RED_LED_CURRENT_LOW	                0x07
#define IR_LED_CURRENT_HIGH	                0xAF
#define RED_LED_CURRENT_HIGH	            0xAF

#define TEMP_INTEGER                        0x1F
#define TEMP_FRACTION                       0x20 //TFRAC[3:0]
#define TEMP_ENABLE                         0x21
#define TEMP_FRACTION_STEP                  0.0625

#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c0_dev = DEVICE_DT_GET(I2C_NODE);

typedef enum MAX30102_STATUS
{
	MAX30102_OK = 0,
	MAX30102_ERROR = 1
} MAX30102_STATUS;

typedef struct
{
	uint8_t IRQ_Status_Flags[1];
	uint8_t Temp_Reg;
	uint32_t Raw_HR_Data;
	uint32_t Raw_Data_Red_LED;
	uint32_t Raw_Data_IR_LED;
	float Data_Temp;

} MAX30102_Data;



MAX30102_STATUS MAX30102_Init(const struct device *i2c0_dev);
MAX30102_STATUS MAX30102_Read_Reg(const struct device *i2c0_dev, uint8_t reg_addr, uint8_t Tmp);
MAX30102_STATUS MAX30102_Write_Reg(const struct device *i2c0_dev, uint8_t Reg_Addr, uint8_t value);
MAX30102_STATUS MAX30102_Write_Reg_Bit(const struct device *i2c0_dev, uint8_t Register, uint8_t Bit, uint8_t Value);

MAX30102_STATUS MAX30102_Reset(const struct device *i2c0_dev);
MAX30102_STATUS MAX30102_Read_and_Clear_IRQ_FLAG(const struct device *i2c0_dev);
MAX30102_STATUS MAX30102_Set_Mode(const struct device *i2c0_dev,uint8_t Mode);
MAX30102_STATUS MAX30102_Sleep_Mode(const struct device *i2c0_dev,uint8_t Enable); // POWER SAVE MODE

MAX30102_STATUS MAX30102_Read_Fifo(MAX30102_Data *data);
MAX30102_STATUS MAX30102_Fifo_Write_Pointer(uint16_t Value_to_write);
MAX30102_STATUS MAX30102_Fifo_Overflow_Counter(uint16_t Value_to_write);
MAX30102_STATUS MAX30102_Fifo_Read_Pointer(uint16_t Value_to_write);
MAX30102_STATUS MAX30102_Fifo_Sample_Averaging(uint16_t Value_to_write);
MAX30102_STATUS MAX30102_Fifo_Rollover_Enable(uint8_t Flag_to_write);
MAX30102_STATUS MAX30102_Fifo_Almost_Full_Value(uint8_t Value);

// SpO2
MAX30102_STATUS MAX30102_Adc_Range(uint8_t Value);
MAX30102_STATUS MAX30102_Sample_Rate(uint8_t Value);

// LED
MAX30102_STATUS MAX30102_LED_Pulse_Width(uint8_t Value);
MAX30102_STATUS MAX30102_LED_1_Pulse_Amplitude(uint16_t Value);
MAX30102_STATUS MAX30102_LED_2_Pulse_Amplitude(uint16_t Value);

// Temperature
MAX30102_STATUS MAX30102_Get_Temp(MAX30102_Data *data);















#endif /* MAX30102_H_ */