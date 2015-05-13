#ifndef HT1632_H_
#define HT1632_H_

#define HT1632_READ  0x6
#define HT1632_WRITE 0x5
#define HT1632_COMMAND 0x4

#define HT1632_SYS_DIS 0x00
#define HT1632_SYS_EN 0x01
#define HT1632_LED_OFF 0x02
#define HT1632_LED_ON 0x03
#define HT1632_BLINK_OFF 0x08
#define HT1632_BLINK_ON 0x09
#define HT1632_SLAVE_MODE 0x10
#define HT1632_MASTER_MODE 0x14
#define HT1632_INT_RC 0x18
#define HT1632_EXT_CLK 0x1C
#define HT1632_PWM_CONTROL 0xA0

#define HT1632_COMMON_8NMOS  0x20
#define HT1632_COMMON_16NMOS  0x24
#define HT1632_COMMON_8PMOS  0x28
#define HT1632_COMMON_16PMOS  0x2C

#define PIN_MOSI 5
#define PIN_MISO 6
#define PIN_SCK 7
#define PIN_SSBAR 4 

/*
void initializeBoard( int8_t data, int8_t wr, int8_t cs, int8_t rd=-1);

void begin(uint8_t type);

void clrPixel(uint16_t i);
void setPixel(uint16_t i);

void blink(boolean state);
void setBrightness(uint8_t pwm);

void clearScreen();
void fillScreen();
void writeScreen();
void dumpScreen();

int8_t WIDTH, HEIGHT;
int8_t _data, _cs, _wr, _rd;

void sendcommand(uint8_t c);
void writedata(uint16_t d, uint8_t bits);
void writeRAM(uint8_t addr, uint8_t data);


struct HT1632LEDMatrixSruct {
	uint8_t led[48];     // 16 * 24 / 8
	uint8_t _width, _height;
	uint8_t cursor_x, cursor_y, textsize, textcolor;
};
*/

#endif //HT1632_H_