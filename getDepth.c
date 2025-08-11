#include "grbl.h"


#define L_WATER_PORT PORTD
#define L_WATER_DDR DDRD
#define L_WATER_PIN PIND
#define L_WATER_BIT 1
#define R_WATER_PORT PORTD
#define R_WATER_DDR DDRD
#define R_WATER_PIN PIND
#define R_WATER_BIT 0


void getDepth_init() {
  L_WATER_DDR &= ~((1 << L_WATER_BIT)); // 设置为输入引脚
  L_WATER_PORT |= ((1 << L_WATER_BIT)); // 启用内部上拉电阻。正常高操作。
  R_WATER_DDR &= ~((1 << R_WATER_BIT)); // 设置为输入引脚
  R_WATER_PORT |= ((1 << R_WATER_BIT)); // 启用内部上拉电阻。正常高操作。
}

void get_L_Depth() {
  sys.lWaterStatus = L_WATER_PIN & (1 << L_WATER_BIT);
}

void get_R_Depth() {
  sys.lWaterStatus = R_WATER_PIN & (1 << R_WATER_BIT);
}