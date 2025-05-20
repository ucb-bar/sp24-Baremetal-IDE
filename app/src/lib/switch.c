#include <stdint.h>
#include "switch.h"



int SW_REMAP[8] = {5, 4, 7, 6, 1, 0, 2, 3};
uint8_t vals;

uint8_t send_cmd[20] = {0x00};
uint8_t data_sw[2] = {0, 0};
uint8_t I2C_ADDR = 0x20;

void switches_init() {
  I2C_InitType I2C_init_config;
  I2C_init_config.clock = 40000;
  i2c_init(I2C0, &I2C_init_config);


}

uint8_t i2c_read_io() {
  Status trans = i2c_master_transmit(I2C0, I2C_ADDR, send_cmd, sizeof(send_cmd), 1000 );
  Status rcv = i2c_master_receive(I2C0, I2C_ADDR, data_sw, sizeof(data_sw), 1000 );
  // printf("status transmit: %d recieve %d\r\n", trans, rcv);

  if (trans || rcv) {
    return 0xFF;
  }
  return data_sw[0];
}

uint8_t get_switches() {
  return i2c_read_io();
}
  

uint8_t read_switch(int num) {
  uint8_t sw_val = i2c_read_io();
  return ((0x01) & (sw_val >> SW_REMAP[num]));
}

