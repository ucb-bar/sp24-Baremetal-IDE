#include <stdint.h>
#include "switch.h"
#include "joints.h"
#include "home.h"

int EN_HOME[8] = {1, 1, 1, 1, 1, 1, 1, 1};

void home_motors() {
  motor_set_state(0);
  motor_set_en(1);

  for(int i=0; i<8; i++) {
    if (EN_HOME[i]) {
      home_joint(i);
    }
  }

  reset_encoders();
  motor_set_state(1);
}


void home_joint(int num) {
  printf("Homing motor [%d]\r\n", num);

  int counter = 0;
  while (counter < 10) {
    // printf("[%d, %d, %d, %d, %d, %d, %d, %d]\n", read_switch(0), read_switch(1), read_switch(2), read_switch(3), read_switch(4), read_switch(5), read_switch(6), read_switch(7));
    set_motor_speed(num, -1200);
    if (read_switch(num)) {
      counter++;
    } else {
      counter = 0;
    }
  }
  set_motor_speed(num, 0);
  reset_encoder(num);
  msleep(300);

  printf("Finished Homing: [%d]\r\n", num);
}