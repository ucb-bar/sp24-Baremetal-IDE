#include <stdint.h>
#include "switch.h"
#include "joints.h"
#include "home.h"

int EN_HOME[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int switch_map[8] = {1, 5, 0, 4, 3, 6, 2, 7};

void home_motors() {
  motor_set_state(0);
  motor_set_en(1);

  for(int i=0; i<8; i++) {
    if (EN_HOME[i]) {
      home_joint(i);
    }
  }

  reset_encoders();
}


void home_joint(int num) {
  printf("Homing motor [%d]\n", num);

  int counter = 0;
  while (counter < 10) {
    set_motor_speed(num, -800);
    if (read_switch(num)) {
      counter++;
    } else {
      counter = 0;
    }
  }
  set_motor_speed(num, 0);
  reset_encoder(num);
  msleep(300);
  

  printf("Finished Homing: [%d]\n", num);
}