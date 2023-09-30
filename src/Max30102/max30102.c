#include "max30102.h"


MAX30102_STATUS MAX30102_Who_Im(const struct device *i2c0_dev)
{
	uint8_t RegToRead[1] = {0};
	if (0 != i2c_burst_read(i2c0_dev, MAX30102_ADDRESS, MAX30102_PARTID, RegToRead, 1))
	{
		printk("MAX30102 connected! Part ID: 0x%x\n", RegToRead[0]);
		return MAX30102_OK;
	}
	else
	{
		printk("MAX30102 disconnected!");

		if (RegToRead[0] != 0x15)
			printk("MAX30102 wrong ID! Part ID: %dd\n", RegToRead[0]);

		return MAX30102_ERROR;
	}
}

MAX30102_STATUS MAX30102_Init(const struct device *i2c0_dev) // Who Im?
{

#if RESET_DURING_INIT
	if (MAX30102_OK != MAX30102_Reset(i2c0_dev)) // resets the MAX30102
		return MAX30102_ERROR;
#endif

	// FIFO config
	if (MAX30102_OK != MAX30102_Fifo_Write_Pointer(0x00)) // PoR 0X00 default
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Fifo_Overflow_Counter(0x00))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Fifo_Read_Pointer(0x00))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Fifo_Sample_Averaging(FIFO_SMP_AVE_1))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Fifo_Rollover_Enable(1))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Fifo_Almost_Full_Value(MAX30102_FIFO_ALMOST_FULL_SAMPLES))
		return MAX30102_ERROR;

	// Mode config
	if (MAX30102_OK != MAX30102_Set_Mode(i2c0_dev,MODE_SpO2))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Adc_Range(SPO2_ADC_RGE_4096))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_Sample_Rate(MAX30102_SAMPLES_PER_SECOND))
		return MAX30102_ERROR;

	// LED Config
	if (MAX30102_OK != MAX30102_LED_Pulse_Width(PULSE_WIDTH_69))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_LED_1_Pulse_Amplitude(IR_LED_CURRENT_HIGH))
		return MAX30102_ERROR;
	if (MAX30102_OK != MAX30102_LED_2_Pulse_Amplitude(RED_LED_CURRENT_HIGH))
		return MAX30102_ERROR;

	// Temp config
	// if (MAX30102_OK != MAX30102_Sample_Rate(MAX30102_SAMPLES_PER_SECOND))
	// 	return MAX30102_ERROR;
	// if (MAX30102_OK != MAX30102_Sample_Rate(MAX30102_SAMPLES_PER_SECOND))
	// 	return MAX30102_ERROR;

	// IRQ config
	//  if (MAX30102_OK != MAX30102_Set_IRQ_Almost_Full_Enabled(1))
	//  	return MAX30102_ERROR;
	//  if (MAX30102_OK != MAX30102_Set_IRQ_Fifo_Data_Ready_Enabled(1))
	//  return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_Read_Reg(const struct device *i2c0_dev, uint8_t Reg_Addr, uint8_t Tmp)
{
	int err = 0;
	// uint8_t buf[1] = {0x00};

	err = i2c_burst_read(i2c0_dev, MAX30102_ADDRESS, Reg_Addr, &Tmp, 1);
	if (err)
	{
		printk("MAX30102: Error!\n");
		return MAX30102_ERROR;
	}
	else
	{
		// Tmp = buf[0];
		return MAX30102_OK;
	}
}
MAX30102_STATUS MAX30102_Write_Reg(const struct device *i2c0_dev, uint8_t Reg_Addr, uint8_t Value)
{
	int err = i2c_burst_write(i2c0_dev, MAX30102_ADDRESS, Reg_Addr, &Value, 1);
	if (err)
	{
		printk("MAX30102: Error!\n");
		return MAX30102_ERROR;
	}
	else
	{
		return MAX30102_OK;
	}
}
MAX30102_STATUS MAX30102_Write_Reg_Bit(const struct device *i2c0_dev, uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, Register, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(1 << Bit);
	Tmp |= (Value & 0x01) << Bit;
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, Register, Tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS MAX30102_Reset(const struct device *i2c0_dev) // Reset MAX30102
{
	uint8_t RegToWrite[1] = {MAX30102_RESET};
	uint8_t err = i2c_reg_write_byte(i2c0_dev, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, RegToWrite[0]);

	if (err)
	{
		printk("Reset error!\n");
		return MAX30102_ERROR;
	}
	else
	{
		printk("Reset done!\n");
		return MAX30102_OK;
	}
}
MAX30102_STATUS MAX30102_Clear_IRQ_FLAG(const struct device *i2c0_dev, MAX30102_Data *Data) // Reads the interrupt status registers, clearing is automaticly
{
	Data->IRQ_Status_Flags[1] = 0x0;
	int err = i2c_burst_read(i2c0_dev, MAX30102_ADDRESS, MAX30102_IRQ_STATUS_1, Data->IRQ_Status_Flags, 1);

	if (err)
	{
		printk("MAX30102: Interrupt Status 1: Error!\n");
		return MAX30102_ERROR;
	}
	else
	{
		printk("MAX30102: Interrupt Status 1: Ok (%d)\n", Data->IRQ_Status_Flags[0]);
		return MAX30102_OK;
	}
}
MAX30102_STATUS MAX30102_Set_Mode(const struct device *i2c0_dev,uint8_t Mode)
{
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_MODE_CONFIG, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(0x07);
	Tmp |= (Mode & 0x07);
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_MODE_CONFIG, Tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_Sleep_Mode(const struct device *i2c0_dev, uint8_t Enable) // Power save mode
{
	uint8_t Tmp = 0;
	
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_MODE_CONFIG, Tmp))
		return MAX30102_ERROR;

	if (Enable == 0)
	{
		Tmp &= 0b00000111;
		if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_MODE_CONFIG, Tmp))
			return MAX30102_ERROR;
	}
	else if (Enable == 1)
	{
		Tmp |= 0b10000000;
		if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_MODE_CONFIG, Tmp))
			return MAX30102_ERROR;
	}

	return MAX30102_OK;
}

// FIFO
MAX30102_STATUS MAX30102_Read_Fifo(MAX30102_Data *Data)
{

	uint32_t Tmp = 0;
	Data->Raw_Data_Red_LED = 0;
	Data->Raw_Data_IR_LED = 0;
	unsigned char Tmp_Buff_I2C[6];

#if OMIT_FIFO // USE FOR OMIT FIFO AND GET GOING SAMPLES
	if (MAX30102_OK != MAX30102_Fifo_Write_Pointer(0x00))
		return MAX30102_ERROR;
#endif

	if (0 != i2c_burst_read(i2c0_dev, MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA, Tmp_Buff_I2C, 6))
	{
		printk("MAX30102: Error!\n");
		return MAX30102_ERROR;
	}
	Tmp = Tmp_Buff_I2C[0];
	Tmp <<= 16;
	Data->Raw_Data_Red_LED += Tmp;
	Tmp = Tmp_Buff_I2C[1];
	Tmp <<= 8;
	Data->Raw_Data_Red_LED += Tmp;
	Tmp = Tmp_Buff_I2C[2];
	Data->Raw_Data_Red_LED += Tmp;

	Tmp = Tmp_Buff_I2C[3];
	Tmp <<= 16;
	Data->Raw_Data_IR_LED += Tmp;
	Tmp = Tmp_Buff_I2C[4];
	Tmp <<= 8;
	Data->Raw_Data_IR_LED += Tmp;
	Tmp = Tmp_Buff_I2C[5];
	Data->Raw_Data_IR_LED += Tmp;

	Data->Raw_Data_Red_LED &= 0x03FFFF;
	Data->Raw_Data_IR_LED &= 0x03FFFF;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_Fifo_Write_Pointer(uint16_t Value)
{
	return MAX30102_Write_Reg(i2c0_dev, MAX30102_FIFO_WRITE_PTR, (Value & 0x1F)); // FIFO_WR_PTR[4:0]
}
MAX30102_STATUS MAX30102_Fifo_Overflow_Counter(uint16_t Value)
{
	return MAX30102_Write_Reg(i2c0_dev, MAX30102_OVERFLOW_COUNT, (Value & 0x1F)); // FIFO_WR_PTR[4:0]
}
MAX30102_STATUS MAX30102_Fifo_Read_Pointer(uint16_t Value)
{
	return MAX30102_Write_Reg(i2c0_dev, MAX30102_FIFO_READ_PTR, (Value & 0x1F)); // FIFO_WR_PTR[4:0]
};
MAX30102_STATUS MAX30102_Fifo_Sample_Averaging(uint16_t Value)
{
	uint8_t Tmp = 0;
	MAX30102_Read_Reg(i2c0_dev, MAX30102_FIFO_CONFIG, Tmp);

	Tmp &= ~(0x07);
	Tmp |= (Value & 0x07) << 5;
	return MAX30102_Write_Reg(i2c0_dev, MAX30102_FIFO_CONFIG, Tmp);
}
MAX30102_STATUS MAX30102_Fifo_Rollover_Enable(uint8_t Flag)
{
	// ON = 1
	// OFF = 0
	return MAX30102_Write_Reg_Bit(i2c0_dev, MAX30102_FIFO_CONFIG, FIFO_CONF_FIFO_ROLLOVER_EN_BIT, (Flag & 0x01));
}
MAX30102_STATUS MAX30102_Fifo_Almost_Full_Value(uint8_t Value)
{
	if (Value < 17)
		Value = 17;
	if (Value > 32)
		Value = 32;
	Value = 32 - Value;
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_FIFO_CONFIG, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(0x0F);
	Tmp |= (Value & 0x0F);
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_FIFO_CONFIG, Tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS MAX30102_Adc_Range(uint8_t Value)
{
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(0x03);
	Tmp |= ((Value & 0x03) << 5);
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Value))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_Sample_Rate(uint8_t Value)
{
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(0x07);
	Tmp |= ((Value & 0x07) << 2);
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Value))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_LED_Pulse_Width(uint8_t Value)
{
	uint8_t Tmp = 0;
	if (MAX30102_OK != MAX30102_Read_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Tmp))
		return MAX30102_ERROR;
	Tmp &= ~(0x03);
	Tmp |= (Value & 0x03);
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_SPO2_CONFIG, Value))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_LED_1_Pulse_Amplitude(uint16_t Value) //	LED Current = Value * 0.2 mA
{
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_LED1_PULSE_AMPLITUDE, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}
MAX30102_STATUS MAX30102_LED_2_Pulse_Amplitude(uint16_t Value) //	LED Current = Value * 0.2 mA
{
	if (MAX30102_OK != MAX30102_Write_Reg(i2c0_dev, MAX30102_LED2_PULSE_AMPLITUDE, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS MAX30102_Get_Temp(MAX30102_Data *Data) // Don't read register, return 0
{
	uint16_t Tmp_Integer = 0;
	uint16_t Tmp_Fraction = 0;
	// MAX30102_Temp_Init_Conversion();
	MAX30102_Write_Reg_Bit(i2c0_dev, TEMP_ENABLE, 0, 1);
	MAX30102_Read_Reg(i2c0_dev, TEMP_INTEGER, Tmp_Integer);
	MAX30102_Read_Reg(i2c0_dev, TEMP_FRACTION, Tmp_Fraction);
	Tmp_Fraction = Tmp_Fraction * TEMP_FRACTION_STEP;
	Data->Data_Temp = (Tmp_Integer + Tmp_Fraction);
	return MAX30102_OK;
}

// Timer