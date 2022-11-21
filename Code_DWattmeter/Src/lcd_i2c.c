#include "i2c.h"
#include "lcd_i2c.h"
#include <stdio.h>

void Lcd_Send_Cmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xF0);
	data_l = ((cmd<<4) & 0xF0);
	data_t[0] = data_u|0x0C;
	data_t[1] = data_u|0x08;
	data_t[2] = data_l|0x0C;
	data_t[3] = data_l|0x08;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS,(uint8_t*) data_t, 4, 100);
}

void Lcd_Send_Char(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xF0);
	data_l = ((data<<4) & 0xF0);
	data_t[0] = data_u|0x0D;
	data_t[1] = data_u|0x09;
	data_t[2] = data_l|0x0D;
	data_t[3] = data_l|0x09;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS,(uint8_t*) data_t, 4, 100);
}

void Lcd_Init(void)
{
	HAL_Delay(60);
	Lcd_Send_Cmd(0x02);
	Lcd_Send_Cmd(0x28);
	Lcd_Send_Cmd(0x0C);
	Lcd_Send_Cmd(0x80);
	Lcd_Send_Cmd(0x01);
}

void Lcd_Clear(void)
{
	Lcd_Send_Cmd(0x01);
	HAL_Delay(2);
}

void Lcd_Set_Cursor(int row, int col)
{
	uint8_t address;
	switch(row)
	{
		case 1:
			address = 0x00;
			break;
		case 2:
			address = 0x40;
			break;
		case 3:
			address = 0x14;
			break;
		case 4:
			address = 0x54;
			break;
	}
	address += col - 1;
	Lcd_Send_Cmd(0x80 | address);

}

void Lcd_Send_String(char *str)
{
	while(*str) Lcd_Send_Char(*str++);
}

// void Lcd_Send_Float(float num)
// {
// 	char buf[10];
// 	int num_int = (int)(num);
// 	int num_dec1 = (int)((num - num_int) * 10);
// 	int num_dec2 = (int)((num - num_int - num_dec1 / 10.0) * 100);
// 	sprintf(buf, "%d.%d%d", num_int, num_dec1, num_dec2);
// 	Lcd_Send_String(buf);
// }

void Lcd_Send_Float(float num, int decimals)
{
	char buf[10];
	int num_int = (int)(num);
	if (decimals == 0)
	{
		sprintf(buf, "%d", num_int);
	}
	else if (decimals == 1)
	{
		int num_dec1 = (int)((num - num_int) * 10);
		sprintf(buf, "%d.%d", num_int, num_dec1);
	}
	else if (decimals == 2)
	{
		int num_dec1 = (int)((num - num_int) * 10);
		int num_dec2 = (int)((num - num_int - num_dec1 / 10.0) * 100);
		sprintf(buf, "%d.%d%d", num_int, num_dec1, num_dec2);
	}
	Lcd_Send_String(buf);
}

void Lcd_Shift_Right(void)
{
	Lcd_Send_Cmd(0x1C);
}

void Lcd_Shift_Left(void)
{
	Lcd_Send_Cmd(0x18);
}

void Lcd_Blink(void)
{
	Lcd_Send_Cmd(0x0F);
}

void Lcd_NoBlink(void)
{
	Lcd_Send_Cmd(0x0C);
}

void Lcd_CGRAM_CreateChar(unsigned char pos, const char*msg)
{
    if(pos < 8)
    {
        Lcd_Send_Cmd(0x40 + (pos*8));
        for(unsigned char i=0; i<8; i++)
        {
            Lcd_Send_Char(msg[i]);
        }
    }
}

void Lcd_CGRAM_WriteChar(char pos)
{
    Lcd_Send_Char(pos);
}