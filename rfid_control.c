#include "grbl.h"

void rfid_control_init()
{
    // 限位
    RFID_LIMIT_DDR &= ~((1 << RFID_LIMIT_BIT)); // 设置为输入引脚
    RFID_LIMIT_PORTD |= ((1 << RFID_LIMIT_BIT)); // 启用内部上拉电阻。正常高操作。
}

//  0上U 1下D
void set_rfid(uint8_t flag)
{
    protocol_buffer_synchronize();
    rfid_ele_control(1);
    limits_disable();
    uint8_t cycle_mask = 1 << B_AXIS;
    uint8_t idx = 4;
    if (sys.abort)
    {
        return;
    } // 如果已发出系统重置，则阻止。
    // 初始化用于回原点运动的计划数据结构。禁用主轴和冷却。
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE);
    pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

    // 初始化用于回原点计算的变量。
    uint8_t n_cycle = (2 * N_HOMING_LOCATE_CYCLE * 0 + 1);
    uint8_t step_pin[N_AXIS];
    float target[N_AXIS];
    float max_travel = 0.0;
    // 初始化步进引脚掩码
    step_pin[idx] = get_step_pin_mask(idx);
    if (bit_istrue(cycle_mask, bit(idx)))
    {
        // 基于 max_travel 设置目标。确保限位开关在搜索倍增器的影响下被激活。
        // 注意：settings.max_travel[] 存储为负值。
        max_travel = max(max_travel, (-3) * settings.max_travel[idx]);
    }

    float homing_rate = settings.homing_seek_rate;

    uint8_t limit_state, axislock, n_active_axis;
    system_convert_array_steps_to_mpos(target, sys_position);

    // 初始化并声明回原点例程所需的变量。
    axislock = 0;
    n_active_axis = 0;
    // 为活动轴设置目标位置并设置回原点速率的计算。
    if (bit_istrue(cycle_mask, bit(idx)))
    {
        n_active_axis++;
        sys_position[idx] = 0;
        // 根据循环掩码和回原点循环接近状态设置目标方向。
        // 注意：这样编译出来的代码比尝试过的任何其他实现都要小。
        if (flag)
        {
            target[idx] = max_travel;
        }
        else
        {
            target[idx] = settings.rfid_filp_value;
        }
        // 将轴锁应用于本循环中活动的步进端口引脚。
        axislock |= step_pin[idx];
    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] 调整以便每个轴都以回原点速率移动。
    sys.homing_axis_lock = axislock;

    // 执行回原点循环。计划器缓冲区应为空，以便启动回原点循环。
    pl_data->feed_rate = 50;          // 设置当前回原点速率。
    plan_buffer_line(target, pl_data); // 绕过 mc_line()。直接计划回原点运动。

    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // 设置为执行回原点运动并清除现有标志。
    st_prep_buffer();                                   // 准备并填充段缓冲区，来源于新计划的块。
    st_wake_up();                                       // 启动运动
    do
    {
        // 检查限位状态。当它们发生变化时锁定循环轴。
        if (flag)
        {   
            limit_state = ~RFID_LIMIT_PIN & (1 << RFID_LIMIT_BIT);
        }
        else
        {
            limit_state = 0;
        }
        if (axislock & step_pin[idx])
        {
            if (limit_state)
            {
                axislock &= ~(step_pin[idx]);
            }
        }

        sys.homing_axis_lock = axislock;
        st_prep_buffer(); // 检查并准备段缓冲区。注意：此操作应不超过 200 微秒。
        // 退出例程：在此循环中没有时间运行 protocol_execute_realtime()。
        if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP))
        {
            uint8_t rt_exec = sys_rt_exec_state;
            // 回原点失败条件：在循环中发出重置。
            if (rt_exec & EXEC_RESET)
            {
                system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET);
            }
            // 回原点失败条件：安全门已打开。
            if (rt_exec & EXEC_SAFETY_DOOR)
            {
                system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR);
            }
            if (sys_rt_exec_alarm)
            {
                mc_reset(); // 停止电机（如果正在运行）。
                // printString("rfid\r\n");
                protocol_execute_realtime();
                return;
            }
            else
            {
                // 拉离运动完成。禁用执行的 CYCLE_STOP。
                system_clear_exec_state_flag(EXEC_CYCLE_STOP);
                break;
            }
        }

    } while (STEP_MASK & axislock);

    st_reset();                               // 立即强制停止步进电机并重置步进段缓冲区。
    delay_ms(settings.homing_debounce_delay); // 延迟以允许瞬态动态衰减。

    // 在第一次循环后，回原点进入定位阶段。将搜索缩短至拉离距离。
    max_travel = settings.homing_pulloff * 3.0;
    homing_rate = settings.homing_feed_rate;

    sys_position[idx] = 0;
    sys.step_control = STEP_CONTROL_NORMAL_OP; // 将步进控制返回到正常操作。
    protocol_execute_realtime();               // 检查重置并设置系统中止。
    if (sys.abort)
    {
        return;
    } // 未完成。由 mc_alarm 设置的警报状态。

    // 归零循环完成！设置系统以正常运行。
    // -------------------------------------------------------------------------------------

    // 同步 G-code 解析器和规划器位置到归零位置。
    gc_sync_position();
    plan_sync_position();
    limits_init();
    protocol_buffer_synchronize();
    rfid_ele_control(0);
    // if(flag){
    //     gc_execute_line("G91G1B0.3F1000");
    // }
}


void rfid_read(uint8_t* return_data)
{
    uint8_t read_command[11] = {0xAA, 0x09, 0x20, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x55};
    clearSerial1BufferHard();
    for(uint8_t j=0; j < 15; j++){
        for(uint8_t i=0; i < sizeof(read_command); i++){
            serial1_write(read_command[i]);
        }
        delay_ms(100);
        if(serial1_read() != 0xAA) continue;
        uint8_t data_len = serial1_read();
        uint8_t read_data[data_len];
        serial1_read_bytes(read_data, data_len);
        if(read_data[data_len-1] == 0x55){
            // serial_write_bytes(&read_data[6], 8);
            memcpy(return_data, &read_data[6], 16);
            return;
        }
    }
}

void rfid_write(uint8_t toolNumber, uint16_t time)
{
    uint16_t allTime;
    uint8_t write_head[12] = {0xAA, 0x1B, 0x25, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x09, 0x40, 0x00};
    clearSerial1BufferHard();
    for(uint8_t j=0; j < 3; j++){
        // 帧头
        for(uint8_t i=0; i < sizeof(write_head); i++){
            serial1_write(write_head[i]);
        }
        // 数据位除时间
        for(uint8_t j=0; j < 14; j++){
            serial1_write(settings.tool_data[toolNumber-1][j]);
        }
        //时间
        allTime = ((settings.tool_data[toolNumber-1][14] << 8) | (settings.tool_data[toolNumber-1][15])) + time;
        serial1_write(allTime >> 8);
        serial1_write(allTime & 0x0F);
        //帧尾
        serial1_write(0x55);

        settings.tool_data[toolNumber-1][14] = allTime >> 8;
        settings.tool_data[toolNumber-1][15] = allTime & 0x0F;
        delay_ms(100);
        if(serial1_read() != 0xAA) continue;
        uint8_t data_len = serial1_read();
        uint8_t read_data[data_len];
        serial1_read_bytes(read_data, data_len);
        if(read_data[data_len-1] == 0x55){
            return;
        }
    }
}

// aa 02 12 55 停止循环
// aa 02 10 55 开始循环
typedef struct {
    float diameter; // 半径/直径 (4字节)
    uint8_t toolType;    // 刀类
    uint8_t tool_data[16];
    uint16_t count;      // 出现次数
} ToolStat;

#define MAX_TOOLS 5   // 最多存储 50 种刀具
ToolStat toolStats[MAX_TOOLS];
uint8_t toolCount = 0;
uint8_t data_len = 0;
uint8_t data_count = 0;
uint8_t read_rfid_data[20];
uint8_t read_rfid_status = 0;
// 0 读帧头 1 读长度 2 读数据 3 读帧尾


void add_tool(float diameter, uint8_t toolType, uint8_t* tool_data) {
    // 查找是否已存在
    for (uint8_t i = 0; i < toolCount; i++) {
        if (toolStats[i].diameter == diameter &&
            toolStats[i].toolType == toolType) {
            toolStats[i].count++;
            return;
        }
    }

    // 新增
    if (toolCount < MAX_TOOLS) {
        toolStats[toolCount].diameter = diameter;
        toolStats[toolCount].toolType = toolType;
        toolStats[toolCount].count = 1;
        memcpy(toolStats[toolCount].tool_data, tool_data, 16);
        toolCount++;
    }
}

ToolStat* get_max_tool() {
    if (toolCount == 0) return 0;

    ToolStat* maxTool = &toolStats[0];
    for (uint8_t i = 1; i < toolCount; i++) {
        if (toolStats[i].count > maxTool->count) {
            maxTool = &toolStats[i];
        }
    }
    return maxTool;
}

void rfid_read_loop(uint8_t* return_data) {
    toolCount = 0;
    memset(toolStats, 0, sizeof(toolStats));
    clearSerial1BufferHard();

    uint8_t loop_read_start[4] = {0xAA, 0x02, 0x10, 0x55};
    for(uint8_t i=0; i < sizeof(loop_read_start); i++){
        serial1_write(loop_read_start[i]);
    }
    for(uint16_t j=0; j < 500; j++){
        delay_ms(4);
        if(serial1_read() != 0xAA) continue;

        uint8_t data_len = serial1_read();
        uint8_t read_data[data_len];
        serial1_read_bytes(read_data, data_len);

        if(read_data[data_len-1] == 0x55 && data_len == 0x15){
            uint8_t toolType = read_data[4];
            float diameter;
            uint8_t tool_data[16];
            memcpy(&tool_data, &read_data[4], 16);
            memcpy(&diameter, &read_data[6], 4);

            add_tool(diameter, toolType, tool_data);
        }
        
    }

    uint8_t loop_read_stop[4] = {0xAA, 0x02, 0x12, 0x55};
    for(uint8_t i=0; i < sizeof(loop_read_stop); i++){
        serial1_write(loop_read_stop[i]);
    }

    // 输出统计结果
    ToolStat* maxTool = get_max_tool();
    if (maxTool) {
        if(maxTool->count > 10){
            memcpy(return_data, maxTool->tool_data, 16);
        }
    }
}


void rfid_read_loop3(uint8_t* return_data) {
    toolCount = 0;
    data_len = 0;
    data_count = 0;
    read_rfid_status = 0;
    memset(toolStats, 0, sizeof(toolStats));
    memset(read_rfid_data, 0, sizeof(read_rfid_data));
    clearSerial1BufferHard();

    uint8_t loop_read_start[4] = {0xAA, 0x02, 0x10, 0x55};
    for(uint8_t i=0; i < sizeof(loop_read_start); i++){
        serial1_write(loop_read_start[i]);
    }

    delay_ms(20);
    for(uint16_t j=0; j < 5000; j++){
        delay_ms(1);
        uint8_t data = serial1_read();
        // serial_write(data);
        if(data == 0xFF){
            if (j >0){
                j--;
            }
        }
        continue;
        // 0 读帧头 1 读长度 2 读数据 3 读帧尾
        switch (read_rfid_status)
        {
            case 0:
                if(data == 0xAA){
                    read_rfid_status == 1;
                }
                break;
            case 1:
                data_len = data;
                read_rfid_status = 2;
                break;
            case 2:
                read_rfid_data[data_count] = data;
                serial_write(data);
                data_count++;
                if(data_len - 1 == data_count){
                    read_rfid_status = 3;
                }
                break;
            case 3:
                if(data == 0x55){
                    read_rfid_status = 3;
                    uint8_t toolType = read_rfid_data[4];
                    float diameter;
                    uint8_t tool_data[16];
                    memcpy(&tool_data, &read_rfid_data[4], 16);
                    memcpy(&diameter, &read_rfid_data[6], 4);
        
                    add_tool(diameter, toolType, tool_data);
                }
                data_len = 0;
                data_count = 0;
                read_rfid_status = 0;
                memset(read_rfid_data, 0, sizeof(read_rfid_data));
                break;
            default:
                break;
        }
        
    }

    uint8_t loop_read_stop[4] = {0xAA, 0x02, 0x12, 0x55};
    for(uint8_t i=0; i < sizeof(loop_read_stop); i++){
        serial1_write(loop_read_stop[i]);
    }

    // 输出统计结果
    ToolStat* maxTool = get_max_tool();
    if (maxTool) {
        if(maxTool->count > 10){
            memcpy(return_data, maxTool->tool_data, 16);
        }
        print_uint8_base10(maxTool->toolType);
        printString("\r\n");
        printFloat(maxTool->diameter,3);
        printString("\r\n");
        print_uint32_base10(maxTool->count);
        printString("\r\n");
    }
}



void read_all_rfid(){
    char y_char[20], command[80];
    uint8_t return_data[16];
    gc_execute_line("G90G53G0Z-5");
    set_flip(0);
    protocol_buffer_synchronize();
    set_rfid(0);
    for (uint8_t i = 0; i < TOOL_NUM-1; i++)
    {
        toolLed[i] = LED_OFF;
    }
    set_tool_leds(toolLed[0], toolLed[1], toolLed[2], toolLed[3], toolLed[4]); 
    for (uint8_t i = 0; i < TOOL_NUM-1; i++)
    {
        memset(return_data, 0, 16);     
        // 移动刀位置
        float2string(settings.tool_y[i] - settings.rfid_offset, y_char, 3);
        sprintf(command, "G90G53G0Y%s", y_char);
        gc_execute_line(command);
        protocol_buffer_synchronize();
        rfid_read_loop(return_data);
        memcpy(settings.tool_data[i], return_data, 16);
        if (settings.tool_data[i][0] != 0){
            printPgmString(PSTR("{'tool"));
            print_uint8_base10(i + 1);
            printPgmString(PSTR("':"));
            print_tool_info(settings.tool_data[i]);
            printPgmString(PSTR("}\r\n"));
        }

        
        if (settings.tool_data[i][0] == 0){
                    // 移动刀位置
            float2string(settings.tool_y[i] - settings.rfid_offset + 1.5, y_char, 3);
            sprintf(command, "G90G53G0Y%s", y_char);
            gc_execute_line(command);
            protocol_buffer_synchronize();
            rfid_read_loop(return_data);
            memcpy(settings.tool_data[i], return_data, 16);
            if (settings.tool_data[i][0] != 0){
                printPgmString(PSTR("{'tool"));
                print_uint8_base10(i + 1);
                printPgmString(PSTR("':"));
                print_tool_info(settings.tool_data[i]);
                printPgmString(PSTR("}\r\n"));
            }
        }
        if (settings.tool_data[i][0] == 0){
            // 移动刀位置
            float2string(settings.tool_y[i] - settings.rfid_offset - 1.5, y_char, 3);
            sprintf(command, "G90G53G0Y%s", y_char);
            gc_execute_line(command);
            protocol_buffer_synchronize();
            rfid_read_loop(return_data);
            memcpy(settings.tool_data[i], return_data, 16);
            printPgmString(PSTR("{'tool"));
            print_uint8_base10(i + 1);
            printPgmString(PSTR("':"));
            print_tool_info(settings.tool_data[i]);
            printPgmString(PSTR("}\r\n"));
        }

        if (settings.tool_data[i][0] == 0){
            toolLed[i] = LED_OFF;
        }else{
            toolLed[i] = LED_GREEN;
        }
        set_tool_leds(toolLed[0], toolLed[1], toolLed[2], toolLed[3], toolLed[4]); 
    }
    set_rfid(1);
    write_global_settings();
}