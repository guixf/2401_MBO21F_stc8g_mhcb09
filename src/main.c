/**************************************************************************************
 * 程序文档说明（V2.0 最终版，使用定时器版本）
 * 1. 硬件平台：STC8G1K17单片机，晶振频率24MHz，MHCB09P和HLK2401电源受控型
 * 2. 编译环境：VSCode + SDCC/STC官方编译器
 * 3. 功能描述：
 *    - 基于2410s（P3.2）和PIR（P3.3）双人体传感器检测人员状态，P3.3上升沿中断唤醒掉电模式
 *    - 集成CH15通道LVD+ADC电压检测
 *    - P3.3中断防重复触发机制：唤醒后屏蔽中断，掉电前恢复中断
 *    - 核心计时逻辑：唤醒后P5.5置低→延时0.5s→检测电压 → 标记电压高/低（调试模式串口输出）
 *    - 精准控制HMBC09P芯片的Key1/Key2/Key3输出指定时长低脉冲，LED1→Key1、LED2→Key2、Relay3→Key3
 *    - 低功耗设计：无人员活动时进入掉电模式，关闭MHCB09P和HLK2401电源，仅P3.3上升沿中断可唤醒
 *    - 看门狗功能：防止程序跑飞，溢出时间约1秒，主循环定期喂狗，掉电模式自动休眠
 *    - 调试模式：电源常开（P5.5初始低）+ 串口1初始化（115200波特率）+ 串口输出电压值；
 *    - 非调试模式：电源按逻辑控制（初始高）+ 不初始化串口 + 不输出电压值
 * 4. IO口定义及模式：
 *    - 刷机/串口复用口：P3.1(TX1)、P3.0(RX1)（刷机时为下载口，运行时为串口1）
 *    - 输入口（高阻模式）：
 *      P3.2 - 2410s人体检测（下拉，默认低电平，高电平有人/低电平无人，下降沿触发中断）
 *      P3.3 - PIR红外传感器（默认低电平，高电平有人，上升沿触发中断）
 *      P3.4 - LED1状态（低电平亮）
 *      P3.5 - LED2状态（低电平亮）
 *      P1.3 - LED3状态（低电平亮）
 *      P3.6 - Relay1反馈（默认低电平，高电平打开）
 *      P3.7 - Relay2反馈（默认低电平，高电平打开）
 *      P1.4 - Relay3反馈（默认低电平，高电平打开）
 *    - 输出口（推挽输出模式，默认高电平）：
 *      P5.5 - 2410s/HMBC09P供电开关，低电平为打开电源
 *      P5.4 - HMBC09P Key1输出（LED1联动）
 *      P1.7 - HMBC09P Key2输出（LED2联动）
 *      P1.5 - HMBC09P Key3输出（Relay3电压联动）
 * 5. 核心逻辑（V2.0 最终版）：
 *    - P3.3上升沿触发中断 → 置位唤醒标志 + 屏蔽INT1中断 → 退出掉电模式 → 打开电源 → 延时0.5s→检测电压 → 标记电压高/低
 *    - 循环：喂狗 → 如果LED1关闭 → Key1输出0.05s低脉冲
 *    - 执行逻辑：
 *      ① 有人（P3.2高）+ LED2关闭 → Key2输出0.05s低脉冲 
 *      ③ 电压低+Relay3关闭 → Key3输出0.05s低脉冲；电压高+Relay3打开 → Key3输出0.05s低脉冲
 *      ④ 无人（P3.2低）+ LED2打开 → Key2输出0.05s低脉冲→ 如果led1打开则Key1输出0.05s低脉冲；检查P3.2、P3.3是否为低：
 *         - 满足：延时1s→关闭电源+恢复INT1→关闭看门狗→掉电→ 跳出当前循环；
 *         - 不满足：继续循环
 **************************************************************************************/
#include <STC8G.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>  // 串口打印需要的头文件

// ====================== 调试模式预定义开关（核心）======================
#define DEBUG_MODE  // 调试模式开关：电源常开+串口输出；注释则关闭调试模式

// 补充STC8G特殊功能寄存器定义
#define _P1ASF 0x9D
SFR(P1ASF, 0x9D);
#define _LVDCR 0xFD
SFR(LVDCR, 0xFD);
// 看门狗寄存器定义（STC8G1K17）
SFR(WDTCN, 0xE7); // 看门狗控制寄存器

/************************* 可配置参数区 *************************/
// 时间参数（ms）
#define DELAY_WAKEUP       500     // 唤醒后延时（0.5s）
#define DELAY_KEY_PULSE    50      // Key脉冲时长（0.05s）
#define DELAY_POWER_OFF    1000    // 掉电前延时（1s）
#define WDT_FEED_INTERVAL  500     // 看门狗喂狗间隔（0.5s，小于溢出时间）

// 电压参数
#define VOLTAGE_THRESHOLD  3000    // 电压阈值（3V，单位mV）
#define REF_VOLTAGE        1190    // 内部参考电压（1.19V，可校准）

// 硬件状态定义
#define LED_ON_LEVEL       0       // LED亮的电平（低电平亮）
#define RELAY3_OPEN_LEVEL  1       // Relay3打开的电平（高电平打开）
#define POWER_ON_LEVEL     0       // P5.5低电平=打开电源
#define POWER_OFF_LEVEL    1       // P5.5高电平=关闭电源

// 定时器参数（24MHz晶振，1ms中断一次）
#define FOSC               24000000
#define TIMER0_PRESCALER   1       // 1T模式
#define TIMER0_RELOAD      (65536 - (FOSC / 1000 / TIMER0_PRESCALER)) // 1ms重载值

// 串口参数（115200波特率，24MHz晶振）
#define BAUDRATE           115200

/************************* IO口定义 *************************/
// 输入口（高阻模式）
#define HUMAN_2410S_IN    P32     // 2410s人体检测
#define PIR_IN            P33     // PIR红外传感器
#define LED1_STATUS       P34     // LED1状态（低亮）
#define LED2_STATUS       P35     // LED2状态（低亮）
#define LED3_STATUS       P13     // LED3状态（低亮）
#define RELAY1_FEEDBACK   P36     // Relay1反馈（预留）
#define RELAY2_FEEDBACK   P37     // Relay2反馈（预留）
#define RELAY3_FEEDBACK   P14     // Relay3反馈

// 输出口（推挽模式）
#define POWER_CTRL        P55     // 电源控制（低电平开，高电平关）
#define KEY1_OUT          P54     // Key1输出（LED1联动）
#define KEY2_OUT          P17     // Key2输出（LED2联动）
#define KEY3_OUT          P15     // Key3输出（Relay3电压联动）

/************************* 全局变量 *************************/
// 系统状态变量
bool system_wakeup_flag = 0;      // 系统唤醒标志（P3.3中断触发）
bool voltage_low_flag = 0;        // 低电压标记（1=低于阈值）
bool voltage_high_flag = 0;       // 高电压标记（1=高于/等于阈值）
uint32_t wdt_feed_timer = 0;      // 看门狗喂狗计时计数器

// 定时器全局变量
volatile uint32_t timer_ms = 0;   // 毫秒计时计数器（定时器中断累加）

/************************* 函数声明 *************************/
// 系统初始化
void System_Init(void);          // 系统总初始化
void Timer0_Init(void);          // 定时器0初始化（1ms中断）
void UART1_Init(void);           // 串口1初始化（仅调试模式编译）
void LVD_ADC_Init(void);         // LVD+ADC初始化（CH15通道）
void WDT_Init(void);             // 看门狗初始化（溢出时间≈1秒）
void WDT_Feed(void);             // 看门狗喂狗
void WDT_Stop(void);             // 关闭看门狗

// 工具函数
void Timer_Delay_ms(uint16_t ms); // 阻塞式毫秒延时（基于定时器）
void UART1_SendChar(uint8_t ch);  // 串口发送单个字符
void UART1_SendString(char *str); // 串口发送字符串
void Print_Voltage(uint16_t volt);// 串口打印电压值（仅调试模式编译）

// 核心功能函数
void Enter_PowerDown_Mode(void); // 进入掉电模式
uint16_t Get_VCC_Voltage(void);  // 获取VCC电压（mV）
void Detect_Voltage_Status(void);// 检测电压状态并更新标记（调试模式串口输出）
void Output_Key1_Pulse(void);    // Key1输出0.05s低脉冲
void Output_Key2_Pulse(void);    // Key2输出0.05s低脉冲
void Output_Key3_Pulse(void);    // Key3输出0.05s低脉冲
bool Check_LED1_Status(void);    // 检测LED1状态（1=亮，0=灭）
bool Check_LED2_Status(void);    // 检测LED2状态（1=亮，0=灭）
bool Check_Relay3_Status(void);  // 检测Relay3状态（1=打开，0=关闭）
void Disable_INT1(void);         // 禁用INT1中断（防重复触发）
void Enable_INT1(void);          // 启用INT1中断（恢复唤醒）
bool Check_Exit_Condition(void); // 检查掉电条件（P3.2+P3.3均低）

/************************* 主函数（核心逻辑）*************************/
void main(void)
{
    // 1. 系统初始化：定时器+IO+中断+ADC/LVD+看门狗（调试模式额外初始化串口）
    Timer0_Init();
    System_Init();
    WDT_Init();                   // 初始化看门狗
    WDT_Feed();                   // 首次喂狗
    
    // 2. 初始进入掉电模式（低功耗）
    Enter_PowerDown_Mode();
    
    while(1)
    {
        // 仅P3.3中断唤醒时执行逻辑
        if(system_wakeup_flag)
        {
            system_wakeup_flag = 0; // 清除唤醒标志
            Disable_INT1();         // 屏蔽INT1中断，防止重复触发
            WDT_Init();             // 唤醒后重新初始化看门狗
            WDT_Feed();             // 唤醒后立即喂狗
            wdt_feed_timer = 0;     // 重置喂狗计时器
            
            // 唤醒后：打开电源（调试模式下始终保持打开，无需重复设置）
#ifndef DEBUG_MODE
            POWER_CTRL = POWER_ON_LEVEL;
#endif
            
            // 延时0.5秒（定时器精准实现）
            Timer_Delay_ms(DELAY_WAKEUP);
            
            // 检测电压并标记高低（调试模式串口输出电压值）
            Detect_Voltage_Status();
            
            // 核心循环：持续执行联动逻辑，直到满足掉电条件
            while(1)
            {
                // 看门狗喂狗逻辑：每500ms喂一次狗
                if(timer_ms - wdt_feed_timer >= WDT_FEED_INTERVAL)
                {
                    WDT_Feed();
                    wdt_feed_timer = timer_ms;
                }
                
                // 循环逻辑：LED1关闭 → Key1输出0.05s低脉冲
                if(Check_LED1_Status() == 0)
                {
                    Output_Key1_Pulse();
                }
                
                /************************* 执行逻辑①：有人+LED2关闭 → Key2脉冲 *************************/
                if(HUMAN_2410S_IN == 1 && Check_LED2_Status() == 0)
                {
                    Output_Key2_Pulse();
                }
                
                /************************* 执行逻辑③：电压联动Relay3 → Key3脉冲 *************************/
                // 电压低 + Relay3关闭 → Key3脉冲
                if(voltage_low_flag && Check_Relay3_Status() == 0)
                {
                    Output_Key3_Pulse();
                }
                // 电压高 + Relay3打开 → Key3脉冲
                else if(voltage_high_flag && Check_Relay3_Status() == 1)
                {
                    Output_Key3_Pulse();
                }
                
                /************************* 执行逻辑④：无人+LED2打开 → Key2脉冲+额外判断 *************************/
                if(HUMAN_2410S_IN == 0 && Check_LED2_Status() == 1)
                {
                    // 无人+LED2打开 → Key2输出0.05s低脉冲
                    Output_Key2_Pulse();
                    
                    // LED1打开 → Key1输出0.05s低脉冲
                    if(Check_LED1_Status() == 1)
                    {
                        Output_Key1_Pulse();
                    }
                    
                    // 检查掉电条件：P3.2（无人）+ P3.3（无PIR）均低
                    if(Check_Exit_Condition())
                    {
                        // 满足掉电条件：严格按文档顺序执行
                        // 1. 延时1秒
                        Timer_Delay_ms(DELAY_POWER_OFF);
                        
                        // 2. 关闭电源（仅非调试模式执行）
#ifndef DEBUG_MODE
                        POWER_CTRL = POWER_OFF_LEVEL;
#endif
                        
                        // 3. 恢复INT1中断，允许下次唤醒
                        Enable_INT1();
                        
                        // 4. 关闭看门狗 → 进入掉电模式（掉电模式下看门狗自动休眠）
                        WDT_Stop();
                        Enter_PowerDown_Mode();
                    }
                }
                
                // 不满足掉电条件 → 继续循环
            }
        }
    }
}

/************************* 函数实现 *************************/
// 系统初始化：IO/中断/ADC/LVD + 调试模式串口初始化
void System_Init(void)
{
    // 1. IO口模式配置
    // P3口：P3.2-P3.7 高阻输入；P3.0(RX1)/P3.1(TX1) 串口复用（调试模式）
    P3M0 &= ~0xFC;
    P3M1 |= 0xFC;
    
    // P1口：P12.-P1.4 高阻输入；P1.5/P1.7 推挽输出（PxM0=1, PxM1=0）
    P1M0 = (P1M0 & ~0x5c) | 0xa0; 
    P1M1 = (P1M1 & ~0xa0) | 0x5c; 

    
    // P5口：P5.4/P5.5 推挽输出（PxM0=1, PxM1=0）
    P5M0 |= 0x30;
    P5M1 &= ~0x30;
    
    // 2. 输出口初始化
    KEY1_OUT = 1;
    KEY2_OUT = 1;
    KEY3_OUT = 1;
    
    // 3. 电源初始化（预定义形式控制）
#ifdef DEBUG_MODE
    POWER_CTRL = POWER_ON_LEVEL;  // 调试模式：P5.5初始化为低（电源常开）
    UART1_Init();                 // 调试模式：初始化串口1（115200波特率）
#else
    POWER_CTRL = POWER_OFF_LEVEL; // 非调试模式：P5.5初始化为高（电源关闭）
#endif
    
    // 4. 中断配置
    IT0 = 1;  // INT0（P3.2）下降沿触发（预留）
    IT1 = 1;  // INT1（P3.3）上升沿触发（核心唤醒源）
    EX0 = 1;  // 开启INT0（预留）
    EX1 = 1;  // 开启INT1
    EA = 1;   // 开启总中断
    
    // 5. ADC+LVD初始化
    LVD_ADC_Init();
}

// 定时器0初始化：1T模式，1ms中断一次（24MHz晶振）
void Timer0_Init(void)
{
    TMOD &= 0xF0;               // 清除定时器0模式位
    TMOD |= 0x01;               // 定时器0模式1（16位）
    AUXR |= 0x80;               // 定时器0使用1T模式（STC8G特有）
    
    // 设置定时器重载值（1ms中断）
    TH0 = (uint8_t)(TIMER0_RELOAD >> 8);
    TL0 = (uint8_t)TIMER0_RELOAD;

    TF0 = 0;                     // 清除溢出标志
    ET0 = 1;                    // 开启定时器0中断
    TR0 = 1;                    // 启动定时器0
    EA = 1;                     // 开启总中断
}

// 串口1初始化（仅调试模式编译）：115200波特率，8N1，24MHz晶振
#ifdef DEBUG_MODE
void UART1_Init(void)
{
    SCON = 0x50;                // 8位数据，可变波特率
    AUXR |= 0x01;               // 串口1使用定时器2作为波特率发生器
    AUXR |= 0x04;               // 定时器2为1T模式
    T2L = 0xCC;                 // 波特率重载值低8位
    T2H = 0xFF;                 // 波特率重载值高8位
    AUXR |= 0x10;               // 启动定时器2
    ES = 1;                     // 开启串口1中断（可选，此处仅发送无需中断）
}

// 串口1发送单个字符
void UART1_SendChar(uint8_t ch)
{
    SBUF = ch;
    while(!TI);                 // 等待发送完成
    TI = 0;                     // 清除发送标志
}

// 串口1发送字符串
void UART1_SendString(char *str)
{
    while(*str != '\0')
    {
        UART1_SendChar(*str++);
    }
}

// 串口打印电压值（格式：VCC Voltage: XXXX mV\r\n）
void Print_Voltage(uint16_t volt)
{
    char buf[32];
    // 格式化电压字符串
    sprintf(buf, "VCC Voltage: %d mV\r\n", volt);
    // 发送字符串
    UART1_SendString(buf);
}
#endif

// LVD+ADC初始化（CH15通道：内部参考电压）
void LVD_ADC_Init(void)
{
    P1ASF = 0x00;               // P1口不作为ADC输入
    ADC_CONTR = 0x80;           // 开启ADC电源（ADON=1）
    ADC_RES = 0;                // 清空ADC结果寄存器
    ADC_RESL = 0;
    Timer_Delay_ms(2);          // ADC电源稳定延时（定时器实现）
    ADC_CONTR |= 0x0F;          // 选择CH15通道
    IE2 |= 0x80;                // 开启LVD中断允许位
    
    // LVD配置（3V阈值）
    LVDCR = 0x00;               // 清空配置
    LVDCR |= (0b100 << 1);      // LVD阈值3.0V
    LVDCR |= 0x01;              // 开启LVD检测
    PCON &= ~LVDF;              // 清除LVD中断标志
}

// 看门狗初始化：溢出时间≈1秒（STC8G1K17，24MHz晶振）
void WDT_Init(void)
{
    WDTCN = 0x00;               // 解锁看门狗寄存器
    WDTCN = 0x80;               // 启用看门狗（溢出时间≈1秒）
}

// 看门狗喂狗：重置看门狗计数器
void WDT_Feed(void)
{
    WDTCN = 0x00;               // 解锁看门狗寄存器
    WDTCN = 0xAA;               // 喂狗指令
}

// 关闭看门狗
void WDT_Stop(void)
{
    WDTCN = 0x00;               // 解锁看门狗寄存器
    WDTCN = 0xDE;               // 关闭看门狗
}

// 阻塞式毫秒延时函数（基于定时器0，精准无阻塞）
void Timer_Delay_ms(uint16_t ms)
{
    uint32_t start_ms = timer_ms; // 记录延时开始时间
    // 等待计时达到指定毫秒数（差值判断避免溢出）
    while((timer_ms - start_ms) < ms);
}

// 进入掉电模式（仅P3.3上升沿中断可唤醒）
void Enter_PowerDown_Mode(void)
{
    // 关闭定时器0，降低功耗
    TR0 = 0;
    ET0 = 0;
    
    // 关闭所有中断（仅保留INT1中断用于唤醒）
    EA = 0;
    EX0 = 0;
    IE2 &= ~0x80; // 关闭LVD中断
#ifdef DEBUG_MODE
    ES = 0;       // 调试模式：关闭串口中断
#endif
    EX1 = 1;      // 保留INT1中断
    
    // 置位PD位进入掉电模式，等待INT1中断唤醒
    PCON |= 0x02;
    NOP();
    NOP();
    
    // 唤醒后恢复定时器和中断
    ET0 = 1;
    TR0 = 1;
    EA = 1;
    EX0 = 1;
    IE2 |= 0x80;
#ifdef DEBUG_MODE
    ES = 1;       // 调试模式：恢复串口中断
#endif
}

// 获取VCC电压（单位：mV）
uint16_t Get_VCC_Voltage(void)
{
    uint16_t adc_val, voltage;
    
    ADC_CONTR |= 0x40;          // 启动ADC转换
    while(!(ADC_CONTR & 0x20)); // 等待转换完成
    ADC_CONTR &= ~0x20;         // 清除转换完成标志
    
    // 计算12位ADC值
    adc_val = (uint16_t)ADC_RES << 4;
    adc_val |= ADC_RESL & 0x0F;
    
    // 电压计算公式：VCC = 参考电压 * 4096 / ADC值（除零保护）
    if(adc_val != 0)
    {
        voltage = (uint32_t)REF_VOLTAGE * 4096 / adc_val;
    }
    else
    {
        voltage = 0; // 异常值处理
    }
    
    return voltage;
}

// 检测电压状态并更新高低标记（调试模式串口输出电压值）
void Detect_Voltage_Status(void)
{
    uint16_t volt = Get_VCC_Voltage();
    
    // 更新电压标记
    if(volt < VOLTAGE_THRESHOLD)
    {
        voltage_low_flag = 1;
        voltage_high_flag = 0;
    }
    else
    {
        voltage_low_flag = 0;
        voltage_high_flag = 1;
    }
    
    // 调试模式：串口输出电压值
#ifdef DEBUG_MODE
    Print_Voltage(volt);
#endif
}

// Key1输出0.05秒低脉冲（定时器延时）
void Output_Key1_Pulse(void)
{
    KEY1_OUT = 0;
    Timer_Delay_ms(DELAY_KEY_PULSE);
    KEY1_OUT = 1;
}

// Key2输出0.05秒低脉冲（定时器延时）
void Output_Key2_Pulse(void)
{
    KEY2_OUT = 0;
    Timer_Delay_ms(DELAY_KEY_PULSE);
    KEY2_OUT = 1;
}

// Key3输出0.05秒低脉冲（定时器延时）
void Output_Key3_Pulse(void)
{
    KEY3_OUT = 0;
    Timer_Delay_ms(DELAY_KEY_PULSE);
    KEY3_OUT = 1;
}

// 检测LED1状态（1=亮，0=灭）
bool Check_LED1_Status(void)
{
    return (LED1_STATUS == LED_ON_LEVEL) ? 1 : 0;
}

// 检测LED2状态（1=亮，0=灭）
bool Check_LED2_Status(void)
{
    return (LED2_STATUS == LED_ON_LEVEL) ? 1 : 0;
}

// 检测Relay3状态（1=打开，0=关闭）
bool Check_Relay3_Status(void)
{
    return (RELAY3_FEEDBACK == RELAY3_OPEN_LEVEL) ? 1 : 0;
}

// 禁用INT1中断（P3.3）- 防重复触发
void Disable_INT1(void)
{
    EX1 = 0;
}

// 启用INT1中断（P3.3）- 恢复唤醒能力
void Enable_INT1(void)
{
    EX1 = 1;
}

// 检查掉电条件：P3.2（无人）+ P3.3（无PIR）均低
bool Check_Exit_Condition(void)
{
    return (HUMAN_2410S_IN == 0 && PIR_IN == 0) ? true : false;
}

/************************* 中断服务函数 *************************/
// 定时器0中断服务函数（1ms一次）
void Timer0_ISR(void) __interrupt(1)
{
    // 重装定时器初值（模式1需要手动重装）
    TH0 = (uint8_t)(TIMER0_RELOAD >> 8);
    TL0 = (uint8_t)TIMER0_RELOAD;
    
    timer_ms++; // 毫秒计数器累加
}

// INT1中断（P3.3上升沿）- 核心唤醒源
void INT1_ISR(void) __interrupt(2)
{
    system_wakeup_flag = 1; // 置位唤醒标志
    PCON &= ~0x02;          // 清除掉电模式标志，退出掉电
}

// INT0中断（P3.2下降沿）- 预留扩展
void INT0_ISR(void) __interrupt(0)
{
    // 预留，暂无操作
}

// LVD中断服务函数 - 预留扩展
void LVD_ISR(void) __interrupt(26)
{
    if(PCON & LVDF)
    {
        voltage_low_flag = 1; // 标记低电压
        PCON &= ~LVDF;        // 清除中断标志
    }
}

// 串口1中断服务函数（仅调试模式编译，此处仅发送无需处理接收）
#ifdef DEBUG_MODE
void UART1_ISR(void) __interrupt(4)
{
    if(RI) // 接收中断（预留）
    {
        RI = 0; // 清除接收标志
    }
    if(TI) // 发送中断（自动清除，此处预留）
    {
        TI = 0;
    }
}
#endif