#include "grbl.h"

volatile uint16_t tempConversionCounter = 0;
volatile uint32_t blowFanCounter = 0;
volatile uint32_t SpineFanCounter = 0;
volatile uint16_t waterCounter = 0;
volatile uint16_t readSpindleTempNum = 0;
volatile bool readFlag0 = true;
volatile bool tempConversionDone0 = false;
bool conversionStarted0 = false;
volatile bool readFlag1 = true;
volatile bool tempConversionDone1 = false;
bool conversionStarted1 = false;
volatile bool readFlag2 = true;
volatile bool tempConversionDone2 = false;
bool conversionStarted2 = false;
// 0主轴，1左风扇，2右风扇

// 全局变量用于消抖
volatile uint8_t time_ms = 20;  // 消抖时间
volatile uint8_t debounce_counter = 0;  // 当前时间
volatile uint8_t last_pin_state = 0;  // 最后状态
volatile bool pin_state = false;

all_temp temp_obj;

// 设置为输出
void onewire_output(uint8_t flag) {
    if(flag==0){
        SPINDLE_TEMP_DDR |= (1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_DDR |= (1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_DDR |= (1 << R_FAN_TEMP_BIT);
    }
}

// 设置为输入（释放总线）
void onewire_input(uint8_t flag) {
    if(flag==0){
        SPINDLE_TEMP_DDR &= ~(1 << SPINDLE_TEMP_BIT);
        SPINDLE_TEMP_PORT |= (1 << SPINDLE_TEMP_BIT); // 启用内部上拉
    }else if (flag==1)
    {
        L_FAN_TEMP_DDR &= ~(1 << L_FAN_TEMP_BIT);
        L_FAN_TEMP_PORT |= (1 << L_FAN_TEMP_BIT); // 启用内部上拉
    }else if (flag==2)
    {
        R_FAN_TEMP_DDR &= ~(1 << R_FAN_TEMP_BIT);
        R_FAN_TEMP_PORT |= (1 << R_FAN_TEMP_BIT); // 启用内部上拉
    }

}

// 写0或1
void onewire_write_bit(uint8_t flag, uint8_t bit) {
    onewire_output(flag);
    if (bit) {
        if(flag==0){
            SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
        }else if (flag==1)
        {
            L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
        }else if (flag==2)
        {
            R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
        }
        _delay_us(5);
        onewire_input(flag);
        _delay_us(55);
    } else {
        if(flag==0){
            SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
        }else if (flag==1)
        {
            L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
        }else if (flag==2)
        {
            R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
        }
        _delay_us(65);
        onewire_input(flag);
        _delay_us(5);
    }
}

// 读1位
uint8_t onewire_read_bit(uint8_t flag) {
    uint8_t bit = 0;
    onewire_output(flag);
    if(flag==0){
        SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
    }
    _delay_us(3);
    onewire_input(flag);
    _delay_us(10);
    if(flag==0){
        bit = (SPINDLE_TEMP_PIN & (1 << SPINDLE_TEMP_BIT)) ? 1 : 0;
    }else if (flag==1)
    {
        bit = (L_FAN_TEMP_PIN & (1 << L_FAN_TEMP_BIT)) ? 1 : 0;
    }else if (flag==2)
    {
        bit = (R_FAN_TEMP_PIN & (1 << R_FAN_TEMP_BIT)) ? 1 : 0;
    }
    _delay_us(50);
    return bit;
}

// 写1字节
void onewire_write_byte(uint8_t flag, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(flag, byte & 0x01);
        byte >>= 1;
    }
}

// 读1字节
uint8_t onewire_read_byte(uint8_t flag) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit(flag)) {
            byte |= 0x80;
        }
    }
    return byte;
}

// 复位并检测设备存在
uint8_t onewire_reset(uint8_t flag) {
    uint8_t presence = 0;
    onewire_output(flag);
    if(flag==0){
        SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
    }
    _delay_us(480);
    onewire_input(flag);
    _delay_us(70);
    if(flag==0){
        presence = (SPINDLE_TEMP_PIN & (1 << SPINDLE_TEMP_BIT)) ? 0 : 1;
    }else if (flag==1)
    {
        presence = (L_FAN_TEMP_PIN & (1 << L_FAN_TEMP_BIT)) ? 0 : 1;
    }else if (flag==2)
    {
        presence = (R_FAN_TEMP_PIN & (1 << R_FAN_TEMP_BIT)) ? 0 : 1;
    }
    _delay_us(410);
    return presence;
}

// // // 读取温度
// float ds18b20_read_temp(uint8_t flag) {
//     uint8_t temp_l, temp_h;
//     int16_t temp;
//     cli(); // 先关中断，避免配置时被打断
//     if (!onewire_reset(flag)) return -1;  // 无响应
//     onewire_write_byte(flag, 0xCC);  // Skip ROM
//     onewire_write_byte(flag, 0x44);  // Convert T
//     sei(); // 开启全局中断
//     _delay_ms(750);            // 等待转换

//     cli(); // 先关中断，避免配置时被打断
//     onewire_reset(flag);
//     onewire_write_byte(flag, 0xCC);  // Skip ROM
//     onewire_write_byte(flag, 0xBE);  // Read Scratchpad
//     temp_l = onewire_read_byte(flag);
//     temp_h = onewire_read_byte(flag);
//     temp = (temp_h << 8) | temp_l;
//     sei(); // 开启全局中断
//     // printFloat(temp * 0.0625, 3);
//     temp_obj.spindle_temp = temp * 0.0625;
//     return temp * 0.0625;  // 每位代表0.0625℃
// }


// void time2_init() {
//     // 配置 Timer2（8位定时器，CTC模式，64分频）
//     TCCR2A = (1 << WGM21);  // CTC模式
//     TCCR2B = (1 << CS22);   // 64分频（16MHz / 64 = 250kHz）
//     OCR2A = 249;            // 1ms = (250kHz / 250) - 1
//     TIMSK2 = (1 << OCIE2A); // 启用比较匹配中断
//     sei();                  // 启用全局中断
// }

// void time2_init() {
//     // 配置 Timer2（8位定时器，CTC模式，1024分频）
//     TCCR2A = (1 << WGM21);   // CTC模式
//     TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // 1024分频（16MHz / 1024 = 15.625kHz）
//     OCR2A = 1561;            // 100ms = (15.625kHz / 1562) - 1
//     TIMSK2 = (1 << OCIE2A);  // 启用比较匹配中断
//     sei();                   // 启用全局中断
// }

void timer5_init() {
    cli(); // 先关中断，避免配置时被打断

    TCCR5A = 0;              // 清除控制寄存器 A，设置为普通 CTC 模式
    TCCR5B = 0;
    TCNT5 = 0;               // 清零计数器
    OCR5A = 249;             // 计数到 249 触发中断（1ms）

    TCCR5B |= (1 << WGM52);  // CTC 模式，WGM bits: 4=1,5=0,6=0 => WGM52=1
    TCCR5B |= (1 << CS51) | (1 << CS50);  // 64 分频：CS52=0,CS51=1,CS50=1 (011)

    TIMSK5 |= (1 << OCIE5A); // 使能 Timer5 比较匹配 A 中断
    sei(); // 开启全局中断
}

ISR(TIMER5_COMPA_vect) {
    // 500ms延时等待
    if (tempConversionCounter < 5000) {  // 5000ms = 5000 * 1ms
        tempConversionCounter++;
        if(tempConversionCounter == 500){
            tempConversionDone0 = true;
            conversionStarted0 = true;
            readFlag0 = true;
        }else if (tempConversionCounter == 1000)
        {
            tempConversionDone0 = false;
            conversionStarted0 = false;
            readFlag0 = true;
        }
        else if (tempConversionCounter == 1500)
        {
            tempConversionDone1 = true;
            conversionStarted1 = true;
            readFlag1 = true;
        }
        else if (tempConversionCounter == 2000)
        {
            tempConversionDone1 = false;
            conversionStarted1 = false;
            readFlag1 = true;
        }
        else if (tempConversionCounter == 2500)
        {
            tempConversionDone2 = true;
            conversionStarted2 = true;
            readFlag2 = true;
        }
        else if (tempConversionCounter == 3000)
        {
            tempConversionDone2 = false;
            conversionStarted2 = false;
            readFlag2 = true;
        }
    } else {
        tempConversionCounter = 0;
    }

    // 风扇循环
    if (sys.isRunGcode){
        if (blowFanCounter >= (uint32_t)1000*60*15) {  // 确保一定会进入清零逻辑
            blowFanCounter = 0;
            blow_fan_control(0);
        } else {
            blowFanCounter++;
            if (blowFanCounter == (uint32_t)1000*60*14) {
                blow_fan_control(1);
            }
        }
        if(sys.spindleFanStatus != 0){
            if (SpineFanCounter < (uint32_t)1000*60*5) {
                SpineFanCounter++;
            } else {
                if(sys.spindleFanStatus == 1){
                    spindle_l_fan_control(0);
                    spindle_r_fan_control(1);
                    sys.spindleFanStatus == 2;
                }else if(sys.spindleFanStatus == 2){
                    spindle_r_fan_control(0);
                    spindle_l_fan_control(1);
                    sys.spindleFanStatus == 1;
                }
                SpineFanCounter = 0;
            }
        }
    }

    //读液位
    if(waterCounter < 5000){
        waterCounter++;
    }else{
        waterCounter = 0;
        sys.lWaterStatus = get_L_Depth();
        sys.rWaterStatus = get_R_Depth();
    }
    
    // 消抖
    if(pin_state){
        debounce_counter--;
        if (debounce_counter == 0) {
            uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
            pin ^= CONTROL_MASK;
            handle_stable_input(pin);
            pin_state = false;
            debounce_counter = time_ms;
        }
    }
}

float ds18b20_read_temp_timer2(uint8_t flag, bool conversionStarted, bool tempConversionDone) {
    if (!conversionStarted) {
        if (!onewire_reset(flag)) return 0;
        onewire_write_byte(flag, 0xCC);  // Skip ROM
        onewire_write_byte(flag, 0x44);  // Convert T
        conversionStarted = true;
        return 0;
    }

    if (tempConversionDone) {
        onewire_reset(flag);
        onewire_write_byte(flag, 0xCC);  // Skip ROM
        onewire_write_byte(flag, 0xBE);  // Read Scratchpad
        uint8_t temp_l = onewire_read_byte(flag);
        uint8_t temp_h = onewire_read_byte(flag);
        int16_t temp = (temp_h << 8) | temp_l;
        conversionStarted = false;
        tempConversionDone = false;
        if(flag==0 && temp * 0.0625 > 5 && temp * 0.0625 < 80){
            temp_obj.spindle_temp = temp * 0.0625;
        }else if (flag==1 && temp * 0.0625 > 5 && temp * 0.0625 < 80)
        {
            temp_obj.l_fan_temp = temp * 0.0625;
        }else if (flag==2 && temp * 0.0625 > 5 && temp * 0.0625 < 80)
        {
            temp_obj.r_fan_temp = temp * 0.0625;
        }
        
        // printFloat(temp * 0.0625, 3);
        return temp / 16.0;
    }
    return 0;  // 等待中
}
