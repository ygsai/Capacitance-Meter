#ifndef LCD_H
#define LCD_H
#define F_CPU 8000000UL

void ms_spin(unsigned short ms);

void lcd_init(void);
void lcd_string(char *p, unsigned char pos); 
void lcd_clear(void);
void hex2ascii(char *target, long value, char pointplace);
char long2ascii(char *target, unsigned long value);

#endif
