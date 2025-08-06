#include "grbl.h"

void toolFlip_control_init()
{
    // 限位
    DDRL &= ~((1 << 4) | (1 << 5)); // 设置为输入引脚
    PORTL |= ((1 << 4) | (1 << 5)); // 启用内部上拉电阻。正常高操作。
}

//  0上V 1下P
void set_flip1(uint8_t flag)
{
    protocol_buffer_synchronize();
    limits_disable();
    uint8_t cycle_mask = 1 << D_AXIS;
    uint8_t idx = 6;
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
            target[idx] = -max_travel;
        }
        else
        {
            target[idx] = max_travel;
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
            limit_state = ~PINL & (1 << 4);
        }
        else
        {
            limit_state = ~PINL & (1 << 5);
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
}


void set_flip(uint8_t flag){
    uint8_t open_command[7] = {0xfe, 0xfe, 0x04, 0x01, 0x08, 0x00, 0xfa};   // 2048
    uint8_t close_command[7] = {0xfe, 0xfe, 0x04, 0x01, 0x0A, 0x28, 0xfa};  // 2600
    uint8_t read_command[5] = {0xfe, 0xfe, 0x02, 0x02, 0xfa}; 

    if(flag){
        for(uint8_t i=0; i < sizeof(open_command); i++){
            serial2_write(open_command[i]);
        }
        delay_ms(100);
        for(uint8_t j=0; j < 3; j++){
            
            for(uint8_t i=0; i < sizeof(read_command); i++){
                serial2_write(read_command[i]);
            }
            delay_ms(100);
            uint8_t head1 = serial2_read();
            uint8_t head2 = serial2_read();
            if(head1 != 0xfe) continue;
            if(head2 != 0xfe) continue;
            uint8_t data_len = serial2_read();
            uint8_t data1 = serial2_read();
            uint8_t data2 = serial2_read();
            uint8_t data3 = serial2_read();
            uint8_t data4 = serial2_read();

            serial_write(head1);
            serial_write(head2);
            serial_write(data_len);
            serial_write(data1);
            serial_write(data2);
            serial_write(data3);
            serial_write(data4);
            // uint8_t read_data[data_len];
            // serial1_read_bytes(read_data, data_len);
            // if(read_data[data_len-1] == 0xfa){
            //     serial_write_bytes(&read_data, sizeof(read_data));
            //     // memcpy(return_data, &read_data[6], 16);
            // }
        }
    }else{
        for(uint8_t i=0; i < sizeof(open_command); i++){
            serial2_write(close_command[i]);
        }
        delay_ms(100);
        for(uint8_t j=0; j < 3; j++){
            
            for(uint8_t i=0; i < sizeof(read_command); i++){
                serial2_write(read_command[i]);
            }
            delay_ms(100);
            uint8_t head1 = serial2_read();
            uint8_t head2 = serial2_read();
            if(head1 != 0xfe) continue;
            if(head2 != 0xfe) continue;
            uint8_t data_len = serial2_read();
            uint8_t data1 = serial2_read();
            uint8_t data2 = serial2_read();
            uint8_t data3 = serial2_read();
            uint8_t data4 = serial2_read();

            serial_write(head1);
            serial_write(head2);
            serial_write(data_len);
            serial_write(data1);
            serial_write(data2);
            serial_write(data3);
            serial_write(data4);
            // uint8_t read_data[data_len];
            // serial1_read_bytes(read_data, data_len);
            // if(read_data[data_len-1] == 0xfa){
            //     serial_write_bytes(&read_data, sizeof(read_data));
            //     // memcpy(return_data, &read_data[6], 16);
            // }
        }

    }

}