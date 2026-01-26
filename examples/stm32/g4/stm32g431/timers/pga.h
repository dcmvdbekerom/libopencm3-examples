#ifndef PGA_H_
#define PGA_H_


#define PGA_CMD_READ  0x6A
#define PGA_CMD_WRITE 0x2A
#define PGA_CMD_NOP_WRITE 0x00
#define PGA_CMD_SDN_WRITE 0xE1

#define PGA_SDN_DIS 0x00
#define PGA_SDN_EN  0xF1

enum pga_gain {
	PGA_GAIN_1X   = 0x0,
	PGA_GAIN_2X   = 0x1,
	PGA_GAIN_5X   = 0x2,
	PGA_GAIN_10X  = 0x3,
	PGA_GAIN_20X  = 0x4,
	PGA_GAIN_50X  = 0x5,
	PGA_GAIN_100X = 0x6,
	PGA_GAIN_200X = 0x7,

};


enum pga_mux {
	PGA_MUX_VCAL_CH0 = 0x0,
	PGA_MUX_CH1 = 0x1,
	PGA_MUX_CH2 = 0x2,
	PGA_MUX_CH3 = 0x3,
	PGA_MUX_CH4 = 0x4,
	PGA_MUX_CH5 = 0x5,
	PGA_MUX_CH6 = 0x6,
	PGA_MUX_CH7 = 0x7,
	PGA_MUX_CH8 = 0x8,
	PGA_MUX_CH9 = 0x9,
	PGA_MUX_CAL1 = 0xC,
	PGA_MUX_CAL2 = 0xD,
	PGA_MUX_CAL3 = 0xE,
	PGA_MUX_CAL4 = 0xF,

};






void pga_write(enum pga_mux mux, enum pga_gain gain);


#endif //PGA_H_

