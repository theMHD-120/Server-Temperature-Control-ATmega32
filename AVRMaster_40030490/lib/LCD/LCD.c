#include <avr/io.h>
#include <util/delay.h>

#define LCD_DATA PORTA
#define ctrl PORTC
// #define en PINC.2
// #define rw PINC.1
// #define rs PINC.0

void init_LCD(void);
void LCD_cmd(unsigned char cmd);
void LCD_write(unsigned char data);

void init_LCD(void)
{
    LCD_cmd(0x38); // 8-bit mode
    _delay_ms(1);
    LCD_cmd(0x01); // clear the screen
    _delay_ms(1);
    LCD_cmd(0x0E); // turn on the cursor
    _delay_ms(1);
    LCD_cmd(0x80); // move cursor to the first place of the first row
    _delay_ms(1);
    return;
}

void LCD_cmd(unsigned char cmd)
{
    LCD_DATA = cmd;
    ctrl = 0x04; // Register Select = 0, Read/Write = 0, Enable = 1
    _delay_ms(1);
    ctrl = 0x00; // Enable = 0
    _delay_ms(30);
    return;
}

void LCD_write(unsigned char data)
{
    LCD_DATA = data;
    ctrl = 0x05; // Register Select = 1, Read/Write = 0, Enable = 1
    _delay_ms(1);
    ctrl = 0x01; // Enable = 0
    _delay_ms(15);
    return;
}