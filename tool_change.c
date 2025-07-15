#include "grbl.h"

uint8_t tool_status = 0; // 0 松刀，1紧刀
char x_char[20], y_char[20], z_char[20], command[80];
void set_tool_length();
void tool_length_zero();

void tool_control_init()
{
  // 换刀检测
  DDRF &= ~(1 << 6); // 设置为输入引脚
  PORTF |= (1 << 6); // 启用内部上拉电阻。正常高操作。
  // PORTF &= ~(1 << 6); // 正常低操作。需要外部下拉。
}

void blowToolSlag(){
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  spindle_r_fan_control(1);
  gc_execute_line("G90G53G0X-5Y-5");
  gc_execute_line("G90G53G0X-5Y-200");
  protocol_buffer_synchronize();
  spindle_r_fan_control(0);
}

void blowAllSlag(){
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  spindle_r_fan_control(1);
  gc_execute_line("G90G53G0X-5Y-5");
  gc_execute_line("G90G53G0X-5Y-200");
  gc_execute_line("G90G53G0X-50Y-200");
  gc_execute_line("G90G53G0X-50Y-5");
  
  gc_execute_line("G90G53G0X-100Y-5");
  gc_execute_line("G90G53G0X-100Y-200");
  gc_execute_line("G90G53G0X-150Y-200");
  gc_execute_line("G90G53G0X-150Y-5");

  gc_execute_line("G90G53G0X-200Y-5");
  gc_execute_line("G90G53G0X-200Y-200");
  gc_execute_line("G90G53G0X-10Y-10");
  protocol_buffer_synchronize();
  spindle_r_fan_control(0);
}

void return_tool()
{
  for (uint8_t i = 0; i < TOOL_NUM - 1; i++)
  {
    if (i == settings.tool - 1)
    {
      toolLed[i] = LED_BLUE;
    }
  }
  set_tool_leds(toolLed[0], toolLed[1], toolLed[2], toolLed[3], toolLed[4]);

  printPgmString(PSTR("beforeTool:"));
  printInteger(settings.tool);
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  gc_execute_line("M4S2300");
  protocol_buffer_synchronize();
  if (settings.tool != 0)
  {
    // 移动到要还刀的xy位置
    float2string(settings.tool_x[settings.tool - 1], x_char, 3);
    float2string(settings.tool_y[settings.tool - 1], y_char, 3);
    sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
    gc_execute_line(command);
    // 下降到还刀位置
    float2string(settings.tool_z[settings.tool - 1], z_char, 3);
    sprintf(command, "G90G53G01Z%sF1500", z_char);
    gc_execute_line(command);
    // 松刀
    // 抬刀
    gc_execute_line("G90G53G0Z-5");
    protocol_buffer_synchronize();
    gc_execute_line("M5");
    // 向下压刀
    // gc_execute_line("G91G0X-6");
    // gc_execute_line("G91G1Z-78F2000");
    // gc_execute_line("G90G53G0Z-5");

    if(sys.isRunGcode){
      // RFID运动到刀旁边
      unsigned long nowTime = getTime(); // 记录开始时间
      unsigned long useTime = nowTime - sys.startTime;
      uint16_t minutes = useTime / 60;

      char y_char[20], command[80];
      uint8_t return_data[8];
      gc_execute_line("G90G53G0Z-5");
      protocol_buffer_synchronize();
      set_rfid(0);
      memset(return_data, 0, 8);
      // 移动刀位置
      float2string(settings.tool_y[settings.tool - 1] - 30, y_char, 3);
      sprintf(command, "G90G53G0Y%s", y_char);
      gc_execute_line(command);
      protocol_buffer_synchronize();
      rfid_write(settings.tool, minutes);
      set_rfid(1);
    }
    set_tool_leds(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
  }
}

void getToolStatus(){
  printPgmString(PSTR("[toolStatus: "));
  // uint8_t status = PINF & (1 << 6);
  print_uint8_base10((PINF & (1 << 6)) ? 1 : 0);
  printPgmString(PSTR("]"));
  printPgmString(PSTR("\r\n"));
  return 0;
}

void get_tool(uint8_t tool_number)
{
  for (uint8_t i = 0; i < TOOL_NUM - 1; i++)
  {
    if (i == tool_number - 1)
    {
      toolLed[i] = LED_BLUE;
    }
  }
  set_tool_leds(toolLed[0], toolLed[1], toolLed[2], toolLed[3], toolLed[4]);

  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  gc_execute_line("M3S2200");
  // 移动取刀位置
  float2string(settings.tool_x[tool_number - 1], x_char, 3);
  float2string(settings.tool_y[tool_number - 1], y_char, 3);
  sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
  gc_execute_line(command);
  // 下降到取刀位置
  float2string(settings.tool_z[tool_number - 1], z_char, 3);
  sprintf(command, "G90G53G01Z%sF1500", z_char);
  gc_execute_line(command);
  protocol_buffer_synchronize();
  delay_ms(500);
  // 紧刀
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  protocol_buffer_synchronize();
  gc_execute_line("M5");

  if(sys.isRunGcode){
    sys.startTime = getTime(); // 记录开始时间
  }
  set_tool_leds(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
}

void change_tool(uint8_t tool_number)
{
  uint8_t beforeTool = settings.tool;
  blowToolSlag();
  // 开门
  set_flip(1);
  if (tool_number == 0)
  {
    return_tool();
  }
  else
  {
    return_tool();
    get_tool(tool_number);
    set_tool_length();
  }
  protocol_buffer_synchronize();
  set_flip(0);
  // 将换完刀后刀号保存
  settings.tool = tool_number;
  write_global_settings(); // 将更新后的刀号写入eeprom
  printPgmString(PSTR("nowTool:"));
  printInteger(tool_number);
  printPgmString(PSTR("\r\n"));
}


// 校准刀具长度
void tool_length_zero()
{
  printPgmString(PSTR("Start tool setting"));
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  // 移动到对刀的xy位置
  float2string(settings.tool_x[TOOL_NUM - 1], x_char, 3);
  float2string(settings.tool_y[TOOL_NUM - 1], y_char, 3);
  sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
  gc_execute_line(command);
  // 下降到对刀z位置
  float2string(settings.tool_z[TOOL_NUM - 1], z_char, 3);
  sprintf(command, "G90G53G0Z%s", z_char);
  gc_execute_line(command);
  gc_execute_line("G21G91G38.2Z-100F200");
  gc_execute_line("G0Z1");
  gc_execute_line("G38.2Z-2F30");

  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, sys_position);
  settings.tool_zpos = print_position[2];
  settings.tool_length = 0;
  gc_state.tool_length_offset = 0;
  write_global_settings(); // 将更新后的刀长写入eeprom
  // report_probe_parameters();
  gc_execute_line("G90G53G0Z-5"); // 抬刀
}

// 设置刀补
void set_tool_length()
{
  printPgmString(PSTR("Start tool setting"));
  printPgmString(PSTR("\r\n"));
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
  // 移动到对刀的xy位置
  float2string(settings.tool_x[TOOL_NUM - 1], x_char, 3);
  float2string(settings.tool_y[TOOL_NUM - 1], y_char, 3);
  sprintf(command, "G90G53G0X%sY%s", x_char, y_char);
  gc_execute_line(command);
  // 下降到对刀z位置
  float2string(settings.tool_z[TOOL_NUM - 1], z_char, 3);
  sprintf(command, "G90G53G0Z%s", z_char);
  gc_execute_line(command);
  gc_execute_line("G21G91G38.2Z-100F200");
  gc_execute_line("G0Z1");
  gc_execute_line("G38.2Z-2F30");
  float print_position[N_AXIS];
  system_convert_array_steps_to_mpos(print_position, sys_position);
  // 现在z轴位置- 之前刀z轴位置 + 之前刀长
  gc_state.tool_length_offset = print_position[2] - settings.tool_zpos + settings.tool_length;
  settings.tool_length = gc_state.tool_length_offset;
  settings.tool_zpos = print_position[2];
  // write_global_settings(); // 将更新后的刀长写入eeprom
  // report_probe_parameters();
  // 抬刀
  gc_execute_line("G90G53G0Z-5");
}