// ==================== 激光测距功能实现 ====================
#include "grbl.h"
#include <math.h>


laser_distance_t laser_distance = {0};

// 初始化激光测距ADC
void laser_distance_init(void)
{
    // 配置ADC寄存器
    // 配置PF4为输入并关闭上拉，禁用数字输入缓冲
    DDRF &= ~(1 << 4);
    PORTF &= ~(1 << 4);
    DIDR0 |= (1 << ADC4D);

    ADMUX_REG = 0;  // 清零MUX位
    ADMUX_REG |= (1 << REFS0);  // 使用AVCC作为参考电压(5V)
    ADMUX_REG |= (LASER_SENSOR_PIN & 0x07);  // 设置ADC通道(0-7)

    // 配置ADC控制和状态寄存器
    ADCSRA_REG |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 分频128(125kHz)
    ADCSRA_REG |= (1 << ADEN);  // 使能ADC

    // 初始化状态结构体
    laser_distance.distance_mm = 0.0;
    laser_distance.voltage = 0.0;
    laser_distance.adc_value = 0;
    laser_distance.is_valid = false;
    laser_distance.last_update_time = 0;
    laser_distance.sample_sum = 0;
    laser_distance.sample_count = 0;
    laser_distance.sampling_active = false;
}

// 直接寄存器方式读取ADC值
int laser_distance_read_adc(void)
{
    // 启动转换
    ADCSRA_REG |= (1 << ADSC);

    // 等待转换完成
    while (ADCSRA_REG & (1 << ADSC));

    // 读取ADC值 (ADCL必须先读)
    uint8_t low = ADCL_REG;
    uint8_t high = ADCH_REG;

    return (high << 8) | low;
}
// 获取距离值(mm)
float laser_distance_get_distance(void)
{
    return laser_distance.distance_mm;
}

// 获取电压值(V)
float laser_distance_get_voltage(void)
{
    return laser_distance.voltage;
}

// 获取ADC原始值
int laser_distance_get_adc_value(void)
{
    return laser_distance.adc_value;
}

// 检查数据是否有效
bool laser_distance_is_valid(void)
{
    return laser_distance.is_valid;
}

// 开始采样过程
void laser_distance_start_sampling(void)
{
    laser_distance.sample_sum = 0;
    laser_distance.sample_count = 0;
    laser_distance.sampling_active = true;
    laser_distance_report_status();
}
// 执行单次采样
void laser_distance_do_single_sample(void)
{
    if (laser_distance.sampling_active) {
        // 读取ADC值并累加
        int adc_value = laser_distance_read_adc();
        laser_distance.sample_sum += adc_value;
        laser_distance.sample_count++;
        
        // 如果完成10次采样，计算平均值并更新数据
        if (laser_distance.sample_count >= LASER_NUM_SAMPLES) {
            int sensorValue = laser_distance.sample_sum / LASER_NUM_SAMPLES;
            
            // 将ADC值转换为毫米值 (25.0-200.0mm)
            float displayValue = LASER_DISPLAY_MIN + (sensorValue * (LASER_DISPLAY_MAX - LASER_DISPLAY_MIN) / LASER_ADC_MAX);
            
            // 四舍五入保留一位小数
            displayValue = round(displayValue * 10) / 10.0;
            
            // 更新距离
            laser_distance.distance_mm = displayValue;
            laser_distance.is_valid = true;
            laser_distance.last_update_time = getTime();
            
            // 停止采样
            laser_distance.sampling_active = false;
        }
    }
}
// 更新激光测距数据（兼容性函数）
void laser_distance_update(void)
{
    // 如果当前没有在采样，开始新的采样
    if (!laser_distance.sampling_active) {
        laser_distance_start_sampling();
    }
}

// 报告激光测距状态
void laser_distance_report_status(void)
{
    if (laser_distance.is_valid) {
        sys.laserDistance = laser_distance.distance_mm;
        // 仅输出距离，单位mm
        // printFloat_CoordValue(laser_distance.distance_mm);
        // printString("\r\n");
    } else {
        // printString("0\r\n");
        sys.laserDistance = 0;
    }
}

void laserScaning(){
    protocol_buffer_synchronize();
    gc_execute_line("G90G53G0Z-50");
    protocol_buffer_synchronize();
    for(int i = 0; i <= 20; i++) {
        char command[50];
        char x_char[50];
        float x_pos = -10 * i;
        float next_x_pos = -10 * (i + 1);
        
        float2string(x_pos, x_char, 3);
        sprintf(command, "G90G53G0X%sY-200", x_char);
        gc_execute_line(command);
        sprintf(command, "G90G53G0X%sY-5", x_char);
        gc_execute_line(command);
        float2string(next_x_pos, x_char, 3);
        sprintf(command, "G90G53G0X%sY-5", next_x_pos);
        gc_execute_line(command);
    }
    protocol_buffer_synchronize();
  }