// #include "grbl.h"

// #include <avr/io.h>
// #include <avr/interrupt.h>

// volatile unsigned long seconds_count = 0; // 记录秒数

// void time2Init() {
//     // 初始化 Timer2 (16MHz AVR)
//     TCCR2A = 0; // 清零控制寄存器 A
//     TCCR2B = 0; // 清零控制寄存器 B
    
//     // 设置预分频 = 1024
//     TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    
//     // 设置比较匹配值（1秒触发）
//     // 计算公式：OCR2A = (F_CPU / (预分频 * 目标频率)) - 1
//     // 16MHz / (1024 * 1Hz) - 1 = 15624
//     // 但 Timer2 是 8 位定时器（最大值 255），所以需要多次中断累计
//     OCR2A = 255; // 先设置最大比较值
    
//     // 启用比较匹配 A 中断
//     TIMSK2 |= (1 << OCIE2A);
    
//     sei(); // 启用全局中断
// }

// // Timer2 比较匹配中断（每 16ms 触发一次）
// ISR(TIMER2_COMPA_vect) {
//     static uint16_t overflow_count = 0;
//     overflow_count++;
    
//     // 计算 1 秒（16MHz / 1024 / 256 ≈ 61Hz，约 61 次中断 = 1秒）
//     if (overflow_count >= 61) {
//         overflow_count = 0;
//         seconds_count++;        
//     }
// }

// unsigned long getTime()
// {
// 	return seconds_count;
// }