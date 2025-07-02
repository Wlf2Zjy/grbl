#include "grbl.h"


void sendByte(uint8_t b, uint8_t flag) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & (1 << (7 - i))) {
      // 1 bit: 高电平 ~0.8us，低电平 ~0.45us
      if(flag == 1){
        TOOL_LED_PORT |= (1 << TOOL_LED_BIT);
      }else if (flag == 2)
      {
        STATUS_LED_PORT |= (1 << STATUS_LED_BIT);
      }
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
      if(flag == 1){
         TOOL_LED_PORT &= ~(1 << TOOL_LED_BIT);
      }else if (flag == 2)
      {
        STATUS_LED_PORT &= ~(1 << STATUS_LED_BIT);
      }
      asm volatile (
        "nop\n nop\n"
      );
    } else {
      // 0 bit: 高电平 ~0.4us，低电平 ~0.85us
      if(flag == 1){
        TOOL_LED_PORT |= (1 << TOOL_LED_BIT);
      }else if (flag == 2)
      {
        STATUS_LED_PORT |= (1 << STATUS_LED_BIT);
      }
      asm volatile (
        "nop\n nop\n");
      if(flag == 1){
        TOOL_LED_PORT &= ~(1 << TOOL_LED_BIT);
      }else if (flag == 2)
      {
        STATUS_LED_PORT &= ~(1 << STATUS_LED_BIT);
      }
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
    }
  }
}

void sendColor(uint8_t r, uint8_t g, uint8_t b, uint8_t flag) {
  sendByte(g, flag);
  sendByte(r, flag);
  sendByte(b, flag);
}

void led_init(){
  TOOL_LED_DDR |= (1 << TOOL_LED_ENABLE_BIT); // 将其配置为输出引脚。
  TOOL_LED_DDR |= (1 << TOOL_LED_BIT); // 将其配置为输出引脚。
  STATUS_LED_DDR |= (1 << STATUS_LED_ENABLE_BIT); // 将其配置为输出引脚。
  STATUS_LED_DDR |= (1 << STATUS_LED_BIT); // 将其配置为输出引脚。
}

void control_tool_led()
{
  TOOL_LED_PORT &= ~(1 << TOOL_LED_ENABLE_BIT); // 失能
  TOOL_LED_PORT |= (1 << TOOL_LED_ENABLE_BIT); // 使能

  cli(); // 临时关闭中断（发送期间必须）
  sendColor(0, 0, 0, 1);  // 显示绿色
  
  sendColor(255, 0, 0, 1);  // 显示红色
  sendColor(0, 255, 0, 1);  // 显示绿色
  sendColor(255, 0, 0, 1);  // 显示红色
  sendColor(0, 255, 0, 1);  // 显示绿色
  sendColor(255, 0, 0, 1);  // 显示红色

  // sendColor(0, 0, 255, 1);  // 显示绿色
  // sendColor(0, 255, 0, 1);  // 显示绿色
  // sendColor(0, 0, 255, 1);  // 显示绿色
  // sendColor(0, 255, 0, 1);  // 显示绿色
  // sendColor(0, 0, 255, 1);  // 显示绿色

  // sendColor(255, 0, 0, 1);  // 显示绿色
  sei(); // 恢复中断
  _delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}

void control_status_led()
{
  STATUS_LED_PORT &= ~(1 << STATUS_LED_ENABLE_BIT); // 失能
  STATUS_LED_PORT |= (1 << STATUS_LED_ENABLE_BIT); // 使能

  cli(); // 临时关闭中断（发送期间必须）
  for (uint8_t i = 0; i < 15; i++)
  {
    // sendColor(222, 49, 99, 2);  // 显示绿色
    sendColor(0, 255, 0, 2);  // 显示绿色
  }
  sei(); // 恢复中断
  _delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}

void control_led(uint8_t color)
{
  STATUS_LED_PORT &= ~(1 << STATUS_LED_ENABLE_BIT); // 失能
  STATUS_LED_PORT |= (1 << STATUS_LED_ENABLE_BIT); // 使能

  cli(); // 临时关闭中断（发送期间必须）
  for (uint8_t i = 0; i < 15; i++)
  {
    if(color==1){
      sendColor(255, 0, 0, 2);  // 显示红色
    }else if (color == 2)
    {
      sendColor(0, 255, 0, 2);  // 显示绿色
    }else if (color == 3)
    {
      sendColor(0, 0, 255, 2);  // 显示蓝色
    }
  }
  sei(); // 恢复中断
  _delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}