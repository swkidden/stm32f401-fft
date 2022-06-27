#ifndef __OLED_H
#define __OLED_H	 

//#include "i2c.h"
#include "spi.h" 
#define u8 uint8_t
#define u32 uint32_t

#define OLED0561_ADD	0x78  // OLED��I2C��ַ����ֹ�޸ģ�
#define COM				0x00  // OLED ָ���ֹ�޸ģ�
#define DAT 			0x40  // OLED ���ݣ���ֹ�޸ģ�
//#define SSD1306_USE_I2C
#define SSD1306_USE_SPI
#define SSD1306_SPI_PORT        hspi1
#define SSD1306_DC_Port         GPIOA
#define SSD1306_DC_Pin          GPIO_PIN_4
#define SSD1306_Reset_Port      GPIOA
#define SSD1306_Reset_Pin       GPIO_PIN_5
void WriteCmd(unsigned char I2C_Command);//д����
void WriteDat(unsigned char I2C_Data);//д����
void OLED_Init(void);//��ʼ��
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);//ȫ�����
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//��ʾ�ַ���
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);//��ʾ����
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//��ʾͼƬ

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
u32 oled_pow(u8 m,u8 n);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);//size2(16|12)
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void gui_draw_hline(uint16_t y,uint16_t x0,uint16_t x1);
void gui_draw_vline(uint16_t x,uint16_t y0,uint16_t y1);
void gui_draw_axis(void);
void OLED_Refresh_Gram(void);
void Gram_clear(void);
#endif
