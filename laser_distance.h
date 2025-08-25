#ifndef laser_distance_h
#define laser_diatance_h

// 激光测距函数声明
void laser_distance_init(void);
int laser_distance_read_adc(void);
float laser_distance_get_distance(void);
float laser_distance_get_voltage(void);
int laser_distance_get_adc_value(void);
bool laser_distance_is_valid(void);
void laser_distance_update(void);
void laser_distance_report_status(void);
void laserScaning(void);

// 激光测距状态结构体
typedef struct {
    float distance_mm;            // 距离值(mm)
    float voltage;                // 电压值(V)
    int adc_value;                // ADC原始值
    bool is_valid;                // 数据是否有效
    uint32_t last_update_time;    // 最后更新时间
    long sample_sum;              // 采样累加值
    uint8_t sample_count;         // 当前采样计数
    bool sampling_active;         // 采样是否激活
} laser_distance_t;

// 全局变量声明
extern laser_distance_t laser_distance;
#endif
