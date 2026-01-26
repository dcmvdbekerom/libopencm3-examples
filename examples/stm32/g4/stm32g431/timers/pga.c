
#include "pga.h"


void pga_write(enum pga_mux mux, enum pga_gain gain){
	uint16_t cmd = (PGA_CMD_WRITE << 8) | (mux << 4) | gain; 
	spi_send(cmd);
}