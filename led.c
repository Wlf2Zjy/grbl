#include "grbl.h"

uint8_t toolLed[5] = {LED_WHITE, LED_WHITE, LED_WHITE, LED_WHITE, LED_WHITE};


void sendToolByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & (1 << (7 - i))) {
      // 1 bit: 高电平 ~0.8us，低电平 ~0.45us
      TOOL_LED_PORT |= (1 << TOOL_LED_BIT);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
      TOOL_LED_PORT &= ~(1 << TOOL_LED_BIT);
      asm volatile (
        "nop\n nop\n"
      );
    } else {
      // 0 bit: 高电平 ~0.4us，低电平 ~0.85us
      TOOL_LED_PORT |= (1 << TOOL_LED_BIT);

      asm volatile (
        "nop\n nop\n");
      TOOL_LED_PORT &= ~(1 << TOOL_LED_BIT);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
    }
  }
}

void sendStatusByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & (1 << (7 - i))) {
      // 1 bit: 高电平 ~0.8us，低电平 ~0.45us
      STATUS_LED_PORT |= (1 << STATUS_LED_BIT);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
      STATUS_LED_PORT &= ~(1 << STATUS_LED_BIT);
      asm volatile (
        "nop\n nop\n"
      );
    } else {
      // 0 bit: 高电平 ~0.4us，低电平 ~0.85us
      STATUS_LED_PORT |= (1 << STATUS_LED_BIT);
      asm volatile (
        "nop\n nop\n");
      STATUS_LED_PORT &= ~(1 << STATUS_LED_BIT);
      asm volatile (
        "nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"
        "nop\n nop\n"
      );
    }
  }
}

void sendToolColor(uint8_t r, uint8_t g, uint8_t b) {
  sendToolByte(g);
  sendToolByte(r);
  sendToolByte(b);
}

void sendStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  sendStatusByte(g);
  sendStatusByte(r);
  sendStatusByte(b);
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
  sendColor(0, 255, 0, 1);  // 显示绿色
  
  sendColor(0, 255, 0, 1);  // 显示红色
  sendColor(0, 255, 0, 1);  // 显示绿色
  sendColor(0, 255, 0, 1);  // 显示红色
  sendColor(0, 255, 0, 1);  // 显示绿色
  sendColor(0, 255, 0, 1);  // 显示红色
  
  sei(); // 恢复中断
  delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
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
  delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}

void control_led(uint8_t color)
{
  STATUS_LED_PORT &= ~(1 << STATUS_LED_ENABLE_BIT); // 失能
  STATUS_LED_PORT |= (1 << STATUS_LED_ENABLE_BIT); // 使能

  cli(); // 临时关闭中断（发送期间必须）
  for (uint8_t i = 0; i < 60; i++)
  {
    if(color==1){
      sendStatusColor(255, 0, 0);  // 显示红色
    }else if (color == 2)
    {
      sendStatusColor(0, 255, 0);  // 显示绿色
    }else if (color == 3)
    {
      sendStatusColor(0, 0, 255);  // 显示蓝色
    }
  }
  sei(); // 恢复中断
  delay_us(60);  // 至少 50µs 的低电平时间，让 WS2812B 更新
}

// 设置所有5个刀位的LED颜色
void set_tool_leds(LedColor color1, LedColor color2, LedColor color3, 
                LedColor color4, LedColor color5) 
{
  // 禁用LED控制器
  TOOL_LED_PORT &= ~(1 << TOOL_LED_ENABLE_BIT);
  // 启用LED控制器
  TOOL_LED_PORT |= (1 << TOOL_LED_ENABLE_BIT);
  
  cli(); // 临时关闭中断
  
  // 设置第1个灯
  switch(color1) {
      case LED_WHITE: sendToolColor(255, 255, 255); break;
      case LED_RED:   sendToolColor(255, 0, 0);     break;
      case LED_GREEN: sendToolColor(0, 255, 0);     break;
      case LED_BLUE:  sendToolColor(0, 0, 255);     break;
      default:       sendToolColor(0, 0, 0);        break;
  }
  
  // 设置第2个灯
  switch(color2) {
      case LED_WHITE: sendToolColor(255, 255, 255); break;
      case LED_RED:   sendToolColor(255, 0, 0);     break;
      case LED_GREEN: sendToolColor(0, 255, 0);     break;
      case LED_BLUE:  sendToolColor(0, 0, 255);     break;
      default:       sendToolColor(0, 0, 0);        break;
  }
  
  // 设置第3个灯
  switch(color3) {
      case LED_WHITE: sendToolColor(255, 255, 255); break;
      case LED_RED:   sendToolColor(255, 0, 0);     break;
      case LED_GREEN: sendToolColor(0, 255, 0);     break;
      case LED_BLUE:  sendToolColor(0, 0, 255);     break;
      default:       sendToolColor(0, 0, 0);        break;
  }
  
  // 设置第4个灯
  switch(color4) {
      case LED_WHITE: sendToolColor(255, 255, 255); break;
      case LED_RED:   sendToolColor(255, 0, 0);     break;
      case LED_GREEN: sendToolColor(0, 255, 0);     break;
      case LED_BLUE:  sendToolColor(0, 0, 255);     break;
      default:       sendToolColor(0, 0, 0);        break;
  }
  
  // 设置第5个灯
  switch(color5) {
      case LED_WHITE: sendToolColor(255, 255, 255); break;
      case LED_RED:   sendToolColor(255, 0, 0);     break;
      case LED_GREEN: sendToolColor(0, 255, 0);     break;
      case LED_BLUE:  sendToolColor(0, 0, 255);     break;
      default:       sendToolColor(0, 0, 0);        break;
  }
  
  sei(); // 恢复中断
  delay_us(60);  // WS2812B更新需要的时间
}