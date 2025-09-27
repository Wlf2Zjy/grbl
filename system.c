/*
  system.c - 处理系统级命令和实时过程
  Grbl 的一部分

  版权所有 (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl 是自由软件：您可以在 GNU 通用公共许可证的条款下重新分发和/或修改
  该许可证由自由软件基金会发布，版本 3，或
  （根据您的选择）任何更高版本。

  Grbl 的发行是希望它能对您有用，
  但不提供任何担保；甚至没有对
  适销性或特定用途的适用性的默示担保。有关更多细节，请参阅
  GNU 通用公共许可证。

  您应该已收到 GNU 通用公共许可证的副本
  与 Grbl 一起。如果没有，请查看 <http://www.gnu.org/licenses/>。
*/

#include "grbl.h"

void system_init()
{
  CONTROL_DDR &= ~(CONTROL_MASK);  // 配置为输入引脚
                                   // #ifdef DISABLE_CONTROL_PIN_PULL_UP
  CONTROL_PORT &= ~(CONTROL_MASK); // 常规低操作。需要外部下拉。
  // CONTROL_PORT &= ~((1 << X_ALARM_BIT) | (1 << Y_ALARM_BIT) | (1 << Z_ALARM_BIT));
  //   CONTROL_PORT |= ((1 << CONTROL_SAFETY_DOOR_BIT) | (1 << STOP_ALARM_BIT));
  // #else
    // CONTROL_PORT |= (1<< A_LIMIT_BIT); // 启用内部上拉电阻。常规高操作。
  // #endif
  CONTROL_PCMSK |= CONTROL_MASK; // 启用引脚变化中断的特定引脚
  PCICR |= (1 << CONTROL_INT);   // 启用引脚变化中断
}

// 返回控制引脚状态作为 uint8 位域。每个位表示输入引脚状态，其中
// 被触发为 1，未被触发为 0。应用反转掩码。位域组织由
// 头文件中的 CONTROL_PIN_INDEX 定义。
uint8_t system_control_get_state()
{
  // print_uint8_base2_ndigit(CONTROL_PIN, 8);
  uint8_t control_state = 0;
  uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
#ifdef INVERT_CONTROL_PIN_MASK
  pin ^= INVERT_CONTROL_PIN_MASK;
#endif
  if (pin)
  {
    if (bit_isfalse(pin, (1 << CONTROL_SAFETY_DOOR_BIT)))
    {
      control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR;
    }
  }
  // return (control_state);
  return 0;
}

// 引脚变化中断，用于引脚输出命令，即循环开始、进给保持和重置。
// 仅设置实时命令执行变量，以便在主程序准备好时执行这些命令。
// 这与从输入串行数据流直接提取的字符型实时命令的工作方式相同。
// ISR(CONTROL_INT_vect)
// {
//   // print_uint8_base2_ndigit(CONTROL_PIN, 8);
//   // printString("111\n");
//   uint8_t pin = system_control_get_state();
//   if (pin)
//   {
//     // if (bit_istrue(pin, CONTROL_PIN_INDEX_RESET))
//     // {
//     //   mc_reset();
//     // }
//     if (bit_istrue(pin, CONTROL_PIN_INDEX_SAFETY_DOOR))
//     {
//       print_uint8_base2_ndigit(CONTROL_PIN, 8);
//       printString("\n");
//       bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
//     }
//   }
// }

extern volatile uint8_t time_ms;
extern volatile uint8_t debounce_counter;
extern volatile uint8_t last_pin_state;
extern volatile bool pin_state;

// 控制中断服务
ISR(CONTROL_INT_vect)
{
  uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
  // pin ^= CONTROL_MASK;
  if (pin != last_pin_state)
  {
    last_pin_state = pin;
    debounce_counter = time_ms; // 重置消抖计数器
    pin_state = true;
  }
  // 只有当消抖计数器为0时才处理稳定输入
  // print_uint8_base2_ndigit(pin, 8);
  // printString("A\r\n");
  if (debounce_counter == 0 && sys.state != STATE_ALARM && pin_state) {
      pin_state = false;
      handle_stable_input(pin);
  }
}

// 处理稳定输入的函数
void handle_stable_input(uint8_t pin)
{
  sys.doorStatus = pin & (1 << CONTROL_SAFETY_DOOR_BIT);
  if (!sys.doorStatus)
  {
    light_control(1);
  }
  sys.drawerStatus = pin & (1 << DRAWER_DETECT_BIT);
  if (!(sys_rt_exec_alarm))
  {
    // uint8_t stopStatus = ((pin & (1 << 4 | 1 << 5 | 1 << 6)) || (~pin & (1 << 7)));  // 前三位屏蔽
    // uint8_t stopStatus = (~pin & (1 << 4 | 1 << 5 | 1 << 6 | 1 << 7));
    uint8_t stopStatus = (~pin & 1 << 7);
    // sys.ALimit = (pin & 1 << A_LIMIT_BIT);
    // print_uint8_base2_ndigit(stopStatus, 8);
    // printString("S\r\n");
    // 检查限位引脚状态
    if (stopStatus)
    {
      mc_reset();
      // printString("急停\r\n");
      system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
    }
  }
}

// 返回安全门是否开启（T）或关闭（F），基于引脚状态。
uint8_t system_check_safety_door_ajar()
{
  // print_uint8_base10((system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR));
  // print_uint8_base2_ndigit(system_control_get_state(), 8);
  return (system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
}

// 执行用户启动脚本（如果已存储）。
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n = 0; n < N_STARTUP_LINE; n++)
  {
    if (!(settings_read_startup_line(n, line)))
    {
      line[0] = 0;
      report_execute_startup_message(line, STATUS_SETTING_READ_FAIL);
    }
    else
    {
      if (line[0] != 0)
      {
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line, status_code);
      }
    }
  }
}

void print_tool_info(uint8_t *data)
{
  float diameter;
  uint32_t int_rep;
  float pitch;
  // 提取各字段
  uint8_t tool_type = data[0];
  uint8_t angle = data[1];
  uint8_t bladeNum = data[10];
  uint8_t bladeLength = data[11];
  uint8_t handleDiameter = data[12];
  uint8_t allLength = data[13];
  uint16_t useTime = (data[14] << 8) | data[15];

  memcpy(&diameter, &data[2], 4);
  memcpy(&pitch, &data[6], 4);
  // pitch = (data[9] << 24) | (data[8] << 16) | (data[7] << 8) | data[6];
  // // 打印解析后的信息
  printPgmString(PSTR("["));
  switch (tool_type)
  {
  case 1:
    printPgmString(PSTR("'endmill',"));
    break;
  case 2:
    printPgmString(PSTR("'ball',"));
    break;
  case 3:
    printPgmString(PSTR("'cone',"));
    break;
  case 4:
    printPgmString(PSTR("'drill',"));
    break;
  case 5:
    printPgmString(PSTR("'thread',"));
    break;
  case 6:
    printPgmString(PSTR("'chamfer',"));
    break;
  default:
    printPgmString(PSTR("null]"));
    return;
  }
  print_uint8_base10(angle);
  printPgmString(PSTR(","));
  printFloat(diameter, 3);
  printPgmString(PSTR(","));
  printFloat(pitch, 3);
  printPgmString(PSTR(","));
  print_uint8_base10(bladeNum);
  printPgmString(PSTR(","));
  print_uint8_base10(bladeLength);
  printPgmString(PSTR(","));
  print_uint8_base10(handleDiameter);
  printPgmString(PSTR(","));
  print_uint8_base10(allLength);
  printPgmString(PSTR(","));
  print_uint32_base10(useTime);
  printPgmString(PSTR("]"));
}

// 指导并执行来自 protocol_process 的一行格式化输入。虽然主要是
// 输入流 g-code 块，但这也执行 Grbl 内部命令，例如
// 设置、启动归位循环和切换开关状态。这与
// 实时命令模块不同，因为它取决于 Grbl 准备执行下一行的时机，
// 因此对于像块删除这样的开关，仅影响后续处理的行，而不一定是实时
// 在循环过程中，因为已经在缓冲区中存储了运动。不过，这种“延迟”
// 不应该成为问题，因为这些命令在循环过程中通常不使用。

uint8_t system_execute_line(char *line)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // 辅助变量
  float parameter, value;
  uint8_t return_data[8];
  memset(return_data, 0, 8);
  switch (line[char_counter])
  {
  case 0:
    report_grbl_help();
    break;  // 显示 Grbl 帮助
  case 'A': // rfid相关
    if (line[2] == 0)
    {
      printPgmString(PSTR("{'toolData':["));
      for (uint8_t i = 0; i < 5; i++)
      {
        print_tool_info(settings.tool_data[i]);
        if (i < 4)
        {
          printPgmString(PSTR(","));
        }
      }
      printPgmString(PSTR("]}\r\n"));
      break;
    }
    if (line[3] == 0)
    {
      switch (line[2])
      {
        unsigned long nowTime, useTime, seconds;
        uint16_t minutes;
      case 'R':
        // for (uint8_t i = 0; i < 5; i++) {
        //   uint8_t rt_exec = sys_rt_exec_state;
        //   if(rt_exec & EXEC_RESET){
        //     break;
        //   }
        //   printString("{'tool");
        //   print_uint8_base10(i+1);
        //   printString("':");
        //   print_tool_info(settings.tool_data[i]);
        //   printString("}\r\n");
        //   delay_ms(3000);
        // }
        read_all_rfid();
        break;
      case 'T':
        nowTime = getTime(); // 记录开始时间
        useTime = nowTime - sys.startTime;
        minutes = useTime / 60;
        sys.startTime = nowTime;
        print_uint32_base10(minutes);
        printString("\r\n");
        print_uint32_base10(nowTime);
        printString("\r\n");
        rfid_write(1, minutes);
        break;
      case 'A':
        getToolStatus();
        break;
      default:
        return (STATUS_INVALID_STATEMENT);
      }
      break;
    }
    if (line[4] == 0)
    {
      uint8_t tool_index = line[3] - '0';
      if (tool_index >= TOOL_NUM)
        return STATUS_INVALID_STATEMENT;
      switch (line[2])
      {
      case 'R':
        rfid_read(return_data);
        memcpy(settings.tool_data[tool_index - 1], return_data, 16);
        write_global_settings();
        break;
      case 'G':
        serial_write_bytes(settings.tool_data[tool_index - 1], 16);
        break;
      default:
        return (STATUS_INVALID_STATEMENT);
      }
      break;
    }
  case 'V':
    report_version();
    break;
  case 'P':
    report_probe_offset();
    break;
  case 'S':
    if (line[3] == 0){
      switch (line[2])
      {
      case 'R':
        set_rfid(index); // $SR
        break;
      default:
        break;  
      }
    }else if (line[4] == 0)
    {
      // serial_write_bytes(line, 4);
      uint8_t index = line[3] - '0';
      switch (line[2])
      {
      case 'R':
        set_rfid(index); // $SR
        break;
      case 'P':
        set_probe(index); // $SP
        break;
      case 'T':
        set_flip(index); // $SP
        break;
      }
    }
    break;
  case 'B':
    if (line[3] == 0)
    {
      switch (line[2])
      {
        case 'A':
        sys.isRunGcode = true;
        // 开始运行gcode
        control_led(3);
        sys.startTime = getTime();
        spindle_fan_control(1);
        sys.handwheel_mode = 0;
        UCSR3B &= ~(1<<RXEN3 |1<<RXCIE3);  // 关闭串口3接收中断
        printString("finish");
        break;
      case 'S':
        break;
      case 'E':
        // 运行gcode结束
        control_led(2);
        spindle_fan_control(0);
        blowAllSlag();
        // 如果刀号一样跳过换刀
        if (settings.tool != 0)
        {
          change_tool(0);
        }
        sys.isRunGcode = false;
        report_realtime_status();
        // Gcode运行结束
        break;
      }
    }
    break;
  case 'E':
    // tool_length_zero();
    // 开门
    set_flip(1);
    set_tool_length();
    // 开门
    set_flip(0);
    break;
  case 'W':
    if (line[2] == 0)
    {
      // 开门
      set_flip(1);
      tool_length_zero();
      // 关门
      set_flip(0);
    }else if (line[3] == 0)
    {
      switch (line[2])
      {
        protocol_buffer_synchronize();
        break;
      }
    }
    break;
  case 'L':
    if (line[2] == 0)
    {
      sys.isOpenLaser = true;
      laserScaning();
      sys.isOpenLaser = false;
      sys.laserDistance = 0;
    }else if (line[3] == 0)
    {
      uint8_t index = line[2] - '0';
      sys.isOpenLaser = index;
      sys.laserDistance = 0;
    }
    break;
  case 'F':
    if (line[4] == 0)
    {
      uint8_t index = line[3] - '0';
      switch (line[2])
      {
      case 'A':
        air_fan_control(index);
        break;
      case 'B':
        spindle_fan_control(1);
        break;
      case 'C':
        spindle_fan_control(2);
        break;
      case 'D':
        blow_fan_control(index);
        break;
      case 'E':
        suction_cup_control(index);
        break;
      case 'F':
        light_control(index);
        break;
      case 'G':
        spray_control(index);
        break;
      case 'H':
        l_water_control(index);
        break;
      case 'I':
        r_water_control(index);
        break;
      case 'J':
        outline_control(index);
        break;
      case 'K':
        camera_control(index);
        break;
      case 'L':
        rfid_ele_control(index);
        break;
      case 'N':
        set_flip(index);
        break;
      case 'M':
        spindle_fan_control(0);
        break;
      case 'O':
        sys.handwheel_mode = index;
        if(index){
          UCSR3B |= (1<<RXEN3 |1<<RXCIE3);  // 打开串口3接收中断
        }else{
          UCSR3B &= ~(1<<RXEN3 |1<<RXCIE3);  // 关闭串口3接收中断
        }
        break;
      default:
        return (STATUS_INVALID_STATEMENT);
      }
    }
    break;
  case 'T':
    report_tool();
    break;
  case 'J': // 手动移动
    // 仅在 IDLE 或 JOG 状态下执行。
    if (sys.state != STATE_IDLE && sys.state != STATE_JOG)
    {
      return (STATUS_IDLE_ERROR);
    }
    if (line[2] != '=')
    {
      return (STATUS_INVALID_STATEMENT);
    }
    return (gc_execute_line(line)); // 注意：$J= 在 g-code 解析器中被忽略，并用于检测手动移动。
    break;
  case '$':
  case 'G':
  case 'C':
  case 'X':
    if (line[2] != 0)
    {
      return (STATUS_INVALID_STATEMENT);
    }
    switch (line[1])
    {
    case '$': // 打印 Grbl 设置
      if (sys.state & (STATE_CYCLE | STATE_HOLD))
      {
        return (STATUS_IDLE_ERROR);
      } // 在循环期间阻止。打印时间过长。
      else
      {
        report_grbl_settings();
      }
      break;
    case 'G': // 打印 gcode 解析器状态
      // TODO: 将此移动到实时命令，以便 GUI 在暂停状态时请求此数据。
      report_gcode_modes();
      break;
    case 'C': // 设置检查 g-code 模式 [IDLE/CHECK]
      // 切换关闭时执行重置。检查 g-code 模式应仅在 Grbl 空闲且准备好时工作，无论警报锁定状态如何。这主要是为了保持简单和一致。
      if (sys.state == STATE_CHECK_MODE)
      {
        mc_reset();
        // printString("gcode检测\r\n");
        report_feedback_message(MESSAGE_DISABLED);
      }
      else
      {
        if (sys.state)
        {
          return (STATUS_IDLE_ERROR);
        } // 需要无警报模式。
        sys.state = STATE_CHECK_MODE;
        report_feedback_message(MESSAGE_ENABLED);
      }
      break;
    case 'X': // 禁用警报锁定 [ALARM]
      if (sys.state == STATE_ALARM)
      {
        // 如果安全门未关闭，则阻止。
        if (system_check_safety_door_ajar())
        {
          return (STATUS_CHECK_DOOR);
        }
        report_feedback_message(MESSAGE_ALARM_UNLOCK);
        sys.state = STATE_IDLE;
        control_led(2);
        // 不运行启动脚本。防止启动中的存储移动造成事故。
      } // 否则无效。
      break;
    }
    break;
  default:
    // 阻止任何需要状态为 IDLE/ALARM 的系统命令。（即 EEPROM，归位）
    if (!(sys.state == STATE_IDLE || sys.state == STATE_ALARM))
    {
      return (STATUS_IDLE_ERROR);
    }
    switch (line[1])
    {
    case '#': // 打印 Grbl NGC 参数
      if (line[2] != 0)
      {
        return (STATUS_INVALID_STATEMENT);
      }
      else
      {
        report_ngc_parameters();
      }
      break;
    case 'H': // 执行归位循环 [IDLE/ALARM]
      if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE))
      {
        return (STATUS_SETTING_DISABLED);
      }
      if (system_check_safety_door_ajar())
      {
        return (STATUS_CHECK_DOOR);
      } // 如果安全门未关闭，则阻止。
      sys.state = STATE_HOMING; // 设置系统状态变量
      report_realtime_status();
      if (line[2] == 0)
      {
        // 回零
        set_probe(1);
        mc_homing_cycle(HOMING_CYCLE_ALL);
        set_rfid(1);
        set_flip(0);
        sys.isHomed = 1;
#ifdef HOMING_SINGLE_AXIS_COMMANDS
      }
      else if (line[3] == 0)
      {
        switch (line[2])
        {
        case 'X':
          mc_homing_cycle(HOMING_CYCLE_X);
          break;
        case 'Y':
          mc_homing_cycle(HOMING_CYCLE_Y);
          break;
        case 'Z':
          mc_homing_cycle(HOMING_CYCLE_Z);
          break;
        case 'A':
          a_go_home();
          // mc_homing_cycle(HOMING_CYCLE_A);
          break;
        default:
          return (STATUS_INVALID_STATEMENT);
        }
#endif
      }
      else
      {
        return (STATUS_INVALID_STATEMENT);
      }
      if (!sys.abort)
      {                         // 在成功归位后执行启动脚本。
        sys.state = STATE_IDLE; // 完成时设置为 IDLE。
        st_go_idle();           // 在返回之前将步进电机设置为设置的空闲状态。
        if (line[2] == 0)
        {
          system_execute_startup(line);
        }
      }
      break;
    case 'S': // 将 Grbl 置于睡眠状态 [IDLE/ALARM]
      if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0))
      {
        return (STATUS_INVALID_STATEMENT);
      }
      system_set_exec_state_flag(EXEC_SLEEP); // 立即设置为执行睡眠模式
      break;
    case 'I': // 打印或存储构建信息 [IDLE/ALARM]
      if (line[++char_counter] == 0)
      {
        settings_read_build_info(line);
        report_build_info(line);
#ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
      }
      else
      { // 存储启动行 [IDLE/ALARM]
        if (line[char_counter++] != '=')
        {
          return (STATUS_INVALID_STATEMENT);
        }
        helper_var = char_counter; // 将辅助变量设置为用户信息行的开始计数器。
        do
        {
          line[char_counter - helper_var] = line[char_counter];
        } while (line[char_counter++] != 0);
        settings_store_build_info(line);
#endif
      }
      break;
    case 'R': // 恢复默认设置 [IDLE/ALARM]
      if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0))
      {
        return (STATUS_INVALID_STATEMENT);
      }
      switch (line[5])
      {
#ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
      case '$':
        settings_restore(SETTINGS_RESTORE_DEFAULTS);
        break;
#endif
#ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
      case '#':
        settings_restore(SETTINGS_RESTORE_PARAMETERS);
        break;
#endif
#ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
      case '*':
        settings_restore(SETTINGS_RESTORE_ALL);
        break;
#endif
      default:
        return (STATUS_INVALID_STATEMENT);
      }
      report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
      mc_reset(); // 强制重置以确保设置正确初始化。
      // printString("不知道\r\n");
      break;
    case 'N': // 启动行 [IDLE/ALARM]
      if (line[++char_counter] == 0)
      { // 打印启动行
        for (helper_var = 0; helper_var < N_STARTUP_LINE; helper_var++)
        {
          if (!(settings_read_startup_line(helper_var, line)))
          {
            report_status_message(STATUS_SETTING_READ_FAIL);
          }
          else
          {
            report_startup_line(helper_var, line);
          }
        }
        break;
      }
      else
      { // 存储启动行 [仅 IDLE] 防止在警报期间移动。
        if (sys.state != STATE_IDLE)
        {
          return (STATUS_IDLE_ERROR);
        } // 仅在空闲时存储。
        helper_var = true; // 将辅助变量设置为标记存储方法。
        // 没有 break。继续进入 default: 以读取剩余的命令字符。
      }
    default: // 存储设置方法 [IDLE/ALARM]
      if (!read_float(line, &char_counter, &parameter))
      {
        return (STATUS_BAD_NUMBER_FORMAT);
      }
      if (line[char_counter++] != '=')
      {
        return (STATUS_INVALID_STATEMENT);
      }
      if (helper_var)
      { // 存储启动行
        // 通过移动所有字符准备将 gcode 块发送到 gcode 解析器
        helper_var = char_counter; // 将辅助变量设置为 gcode 块的起始计数器
        do
        {
          line[char_counter - helper_var] = line[char_counter];
        } while (line[char_counter++] != 0);
        if (char_counter > EEPROM_LINE_SIZE)
        {
          return (STATUS_LINE_LENGTH_EXCEEDED);
        }
        // 执行 gcode 块以确保块有效。
        helper_var = gc_execute_line(line); // 将辅助变量设置为返回的状态代码。
        if (helper_var)
        {
          return (helper_var);
        }
        else
        {
          helper_var = trunc(parameter); // 将辅助变量设置为参数的整数值
          settings_store_startup_line(helper_var, line);
        }
      }
      else
      { // 存储全局设置。
         // 第一次调用strtok，传入原始字符串
        char *token;
        token = strtok(line, "$");
        // 后续调用strtok，传入NULL继续分割
        while (token != NULL) {
            char_counter = 0;
            read_float(token, &char_counter, &parameter);
            char_counter++;
            if (!read_float(token, &char_counter, &value))
            {
              return (STATUS_BAD_NUMBER_FORMAT);
            }
            settings_store_global_setting((uint8_t)parameter, value);
            token = strtok(NULL, "$");
        }
        write_global_settings();
      }
    }
  }
  return (STATUS_OK); // 如果 '$' 命令能到这里，则一切正常。
}

void system_flag_wco_change()
{
#ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
  protocol_buffer_synchronize(); // 在 WCO 变化期间强制同步缓冲区
#endif
  sys.report_wco_counter = 0; // 重置 WCO 计数器
}

// 返回轴 'idx' 的机器位置。必须传入一个 'step' 数组。
// 注意：如果电机步进和机器位置不在同一坐标系中，此函数
//   用作计算变换的中心位置。
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
#ifdef COREXY
  if (idx == X_AXIS)
  {
    pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx]; // 将 CoreXY 步进转换为 X 轴位置
  }
  else if (idx == Y_AXIS)
  {
    pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx]; // 将 CoreXY 步进转换为 Y 轴位置
  }
  else
  {
    pos = steps[idx] / settings.steps_per_mm[idx]; // 对于其他轴，直接计算位置
  }
#else
  pos = steps[idx] / settings.steps_per_mm[idx]; // 对于非 CoreXY 系统，直接计算位置
#endif
  return (pos); // 返回计算得到的位置
}

void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; idx++)
  {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx); // 将每个轴的步进转换为机器位置
  }
  return;
}

// 仅用于 CoreXY 计算。根据 CoreXY 电机步进返回 X 或 Y 轴的“步进”。
#ifdef COREXY
int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
{
  return ((steps[A_MOTOR] + steps[B_MOTOR]) / 2); // 计算 X 轴步进
}
int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
{
  return ((steps[A_MOTOR] - steps[B_MOTOR]) / 2); // 计算 Y 轴步进
}
#endif

// 检查并报告目标数组是否超过机器移动限制。
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; idx++)
  {
    // 允许通过将最大移动限制设置为零来禁用每个轴的软限制
    if (settings.max_travel[idx])
    {
#ifdef HOMING_FORCE_SET_ORIGIN
      // 当强制归位设置原点启用时，软限制检查需要考虑方向性。
      // 注意：最大移动限制存储为负值
      if (bit_istrue(settings.homing_dir_mask, bit(idx)))
      {
        if (target[idx] < settings.homing_pulloff - 0.1 || target[idx] > -settings.max_travel[idx])
        {
          return (true);
        }
      }
      else
      {
        if (target[idx] > settings.homing_pulloff - 0.1 || target[idx] < settings.max_travel[idx])
        {
          return (true);
        }
      }
#else
      // 注意：最大移动限制存储为负值
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx])
      {
        return (true);
      }
#endif
    }
  }
  return (false); // 没有超过限制
}

// 用于设置和清除 Grbl 实时执行标志的特殊处理程序。
void system_set_exec_state_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                       // 禁用中断
  sys_rt_exec_state |= (mask); // 设置执行状态标志
  SREG = sreg;                 // 恢复中断状态
}

void system_clear_exec_state_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                        // 禁用中断
  sys_rt_exec_state &= ~(mask); // 清除执行状态标志
  SREG = sreg;                  // 恢复中断状态
}

void system_set_exec_alarm(uint8_t code)
{
  uint8_t sreg = SREG;
  cli();                    // 禁用中断
  sys_rt_exec_alarm = code; // 设置执行警报
  SREG = sreg;              // 恢复中断状态
}

void system_clear_exec_alarm()
{
  uint8_t sreg = SREG;
  cli();                 // 禁用中断
  sys_rt_exec_alarm = 0; // 清除执行警报
  SREG = sreg;           // 恢复中断状态
}

void system_set_exec_motion_override_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                                 // 禁用中断
  sys_rt_exec_motion_override |= (mask); // 设置运动覆盖标志
  SREG = sreg;                           // 恢复中断状态
}

void system_set_exec_accessory_override_flag(uint8_t mask)
{
  uint8_t sreg = SREG;
  cli();                                    // 禁用中断
  sys_rt_exec_accessory_override |= (mask); // 设置附属设备覆盖标志
  SREG = sreg;                              // 恢复中断状态
}

void system_clear_exec_motion_overrides()
{
  uint8_t sreg = SREG;
  cli();                           // 禁用中断
  sys_rt_exec_motion_override = 0; // 清除运动覆盖
  SREG = sreg;                     // 恢复中断状态
}

void system_clear_exec_accessory_overrides()
{
  uint8_t sreg = SREG;
  cli();                              // 禁用中断
  sys_rt_exec_accessory_override = 0; // 清除附属设备覆盖
  SREG = sreg;                        // 恢复中断状态
}
