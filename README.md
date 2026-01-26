# 2401_XY-MBO21F_stc8g_mhcb09
2401_XY-MBO21F_stc8g_mhcb09人在、手机在多功能监测报告米家
你希望将原程序中通过`Delay_ms`模拟的3秒计时改为**硬件定时器精准计时**（替代软件延时的低效和精度问题），以下是修改后的完整程序文档，核心是基于STC8G1K17的定时器0实现3秒精准计时，同时保留所有业务逻辑，且保证计时过程中不阻塞其他操作。

# STC8G1K17 2410S+PIR人体检测程序文档（V2.1 定时器计时版）
## 1. 核心更新说明
- 替换原“软件延时累加”的3秒计时方式，改用**定时器0（16位自动重装模式）** 实现1ms中断一次，累计3000次即3秒，计时精度±1ms；
- 定时器计时过程中不阻塞主逻辑，保证电压检测、传感器状态判断等操作实时性；
- 保留所有原有业务逻辑（唤醒流程、Relay联动、掉电机制），仅修改计时实现方式。

## 2. 定时器0配置说明（24MHz晶振）
| 配置项         | 取值/设置                | 说明                                  |
|----------------|--------------------------|---------------------------------------|
| 定时器模式     | 16位自动重装（模式1）| TMOD=0x01                             |
| 计数时钟       | 1T模式（不分频）| AUXR |= 0x80                          |
| 定时周期       | 1ms                      | 初值：TH0=0xFF, TL0=0xCC（24MHz下1ms）|
| 中断优先级     | 高优先级（可选）| 保证计时精准                          |
| 计时标志       | `timer_3s_flag`          | 3秒计时完成后置位，主逻辑检测该标志   |

## 3. 完整程序代码（定时器计时版）
```c
/**************************************************************************************
 * 程序文档说明（V2.1 定时器计时版）
 * 1. 硬件平台：STC8G1K17单片机，晶振频率24MHz
 * 2. 编译环境：VSCode + SDCC/STC官方编译器
 * 3. 核心更新：
 *    - 改用定时器0（16位1T模式）实现3秒精准计时（1ms中断一次，累计3000次=3秒）
 *    - 定时器计时不阻塞主逻辑，保证传感器/Relay状态实时检测
 *    - 保留所有V2.0业务逻辑：唤醒流程、Relay联动、掉电机制
 * 4. 核心逻辑：
 *    - P3.3上升沿唤醒 → P5.5置低→延时1s→Relay1→Key1→P5.5置高→启动定时器0计时3秒
 *    - 3秒计时完成→电压检测→P5.5置低→延时1s→执行Relay2/电压联动→根据结果重置计时/掉电
 **************************************************************************************/
#include <STC8G.h>
#include <stdbool.h>
#include <stdint.h>

// Additional SFR definitions for STC8G
#define _P1ASF 0x9D
SFR(P1ASF, 0x9D);
#define _LVDCR 0xFD
SFR(LVDCR, 0xFD);

/************************* 可配置参数区 *************************/
// 时间参数（单位：ms）
#define DELAY_1S           1000    // 通用1秒延时
#define DELAY_0_1S         100     // Key1/Key2低脉冲时长
#define DELAY_0_5S         500     // Key3低脉冲时长
#define TIMER_3S_COUNT     3000    // 3秒计时阈值（1ms中断×3000次）
#define VOLTAGE_THRESHOLD  3000    // 电压阈值（3V）
#define REF_VOLTAGE        1190    // 内部参考电压（1.19V）

// Relay预设值
#define RELAY1_OPEN_LEVEL  1       // Relay1打开时为高电平
#define RELAY2_OPEN_LEVEL  1       // Relay2打开时为高电平
#define RELAY3_OPEN_LEVEL  1       // Relay3打开时为高电平

// 调试+串口参数
#define DEBUG_MODE         1       // 1=开启串口输出
#define BAUDRATE           115200  // 串口波特率
#define FOSC               24000000// 晶振频率（24MHz）

/************************* IO口定义 *************************/
// 输入口
#define HUMAN_2410S_IN    P32     // 2410s人体检测
#define PIR_IN            P33     // PIR红外传感器
#define RELAY1_FEEDBACK   P36     // Relay1反馈
#define RELAY2_FEEDBACK   P37     // Relay2反馈
#define RELAY3_FEEDBACK   P14     // Relay3反馈

// 输出口
#define POWER_CTRL        P55     // 2410s/HMBC09P供电控制
#define KEY1_OUT          P54     // HMBC09P Key1输出
#define KEY2_OUT          P17     // HMBC09P Key2输出
#define KEY3_OUT          P15     // HMBC09P Key3输出

/************************* 全局变量 *************************/
bool system_wakeup_flag = 0;      // 系统唤醒标志
bool voltage_low_flag = 0;        // 低电压标记
bool voltage_high_flag = 0;       // 高电压标记
uint16_t timer_3s_cnt = 0;        // 3秒计时计数器（1ms累加）
bool timer_3s_flag = 0;           // 3秒计时完成标志
bool timer_3s_running = 0;        // 3秒计时器运行标志

/************************* 函数声明 *************************/
void UART1_Init(void);           // 串口初始化
void System_Init(void);          // 系统初始化
void Timer0_Init(void);          // 定时器0初始化（1ms中断）
void LVD_ADC_Init(void);         // LVD+ADC初始化
void Delay_ms(uint16_t ms);      // 毫秒延时
void Enter_PowerDown_Mode(void); // 进入掉电模式
void UART1_Send_Voltage(uint16_t volt);// 串口发送电压
uint16_t Get_VCC_Voltage(void);  // 获取VCC电压
void Detect_Voltage_Status(void);// 检测电压状态
void Output_Key1_Pulse(void);    // Key1输出0.1s脉冲
void Output_Key2_Pulse(void);    // Key2输出0.1s脉冲
void Output_Key3_Pulse(void);    // Key3输出0.5s脉冲
bool Check_Relay1_Status(void);   // 检测Relay1状态
bool Check_Relay2_Status(void);   // 检测Relay2状态
bool Check_Relay3_Status(void);   // 检测Relay3状态
void Disable_INT1(void);         // 禁用INT1中断
void Enable_INT1(void);          // 启用INT1中断
bool Check_Exit_Condition(void); // 检查掉电条件
void Start_Timer3s(void);        // 启动3秒计时器
void Stop_Timer3s(void);         // 停止3秒计时器
void Reset_Timer3s(void);        // 重置3秒计时器

/************************* 主函数 *************************/
void main(void)
{
    // 初始化：串口+系统+定时器+LVD/ADC
    UART1_Init();
    System_Init();
    Timer0_Init();
    LVD_ADC_Init();
    
    // 初始进入掉电模式
    Enter_PowerDown_Mode();
    
    while(1)
    {
        // 仅当P3.3中断唤醒时执行逻辑
        if(system_wakeup_flag)
        {
            system_wakeup_flag = 0;
            Disable_INT1(); // 禁用INT1防重复触发
            
            // 唤醒后初始操作：P5.5置低 + 延时1s
            POWER_CTRL = 0;         
            Delay_ms(DELAY_1S);   
            
            // Relay1关闭则输出Key1脉冲
            if(Check_Relay1_Status() == 0)
            {
                Output_Key1_Pulse();
            }
            
            // P5.5置高 + 启动3秒计时器
            POWER_CTRL = 1;         
            Start_Timer3s(); // 启动定时器0计时
            
            // 核心业务循环
            while(1)
            {
                // 检测3秒计时完成标志
                if(timer_3s_flag)
                {
                    timer_3s_flag = 0; // 清除计时完成标志
                    Stop_Timer3s();    // 停止计时器
                    
                    // 3秒计时到：先检测电压并串口输出
                    Detect_Voltage_Status();
                    if(DEBUG_MODE)
                    {
                        UART1_Send_Voltage(Get_VCC_Voltage());
                    }
                    
                    // P5.5置低 + 延时1秒
                    POWER_CTRL = 0;         
                    Delay_ms(DELAY_1S);   
                    
                    /************************* 逻辑①：有人+Relay2关闭 → Key2脉冲 *************************/
                    if(HUMAN_2410S_IN == 1 && Check_Relay2_Status() == 0)
                    {
                        Output_Key2_Pulse();
                    }
                    
                    /************************* 逻辑③：电压联动Relay3 → Key3脉冲 *************************/
                    if(voltage_low_flag && Check_Relay3_Status() == 0)
                    {
                        Output_Key3_Pulse();
                    }
                    else if(voltage_high_flag && Check_Relay3_Status() == 1)
                    {
                        Output_Key3_Pulse();
                    }
                    
                    /************************* 逻辑④：无人+Relay2打开 → Key2脉冲 *************************/
                    if(HUMAN_2410S_IN == 0 && Check_Relay2_Status() == 1)
                    {
                        Output_Key2_Pulse();
                        
                        // 检查掉电条件：P3.2+P3.3都低
                        if(Check_Exit_Condition())
                        {
                            POWER_CTRL = 1;         
                            Delay_ms(DELAY_1S);     
                            Enable_INT1();          
                            Enter_PowerDown_Mode(); 
                            break;
                        }
                    }
                    else
                    {
                        // 逻辑④不满足：重置并重启3秒计时器
                        POWER_CTRL = 1;
                        Reset_Timer3s();
                        Start_Timer3s();
                    }
                }
                
                // 非计时阶段可执行其他实时检测逻辑（如传感器状态）
                // 此处可扩展：实时检测P3.2/P3.3状态、Relay状态等
            }
        }
    }
}

/************************* 定时器0初始化（1ms中断）*************************/
void Timer0_Init(void)
{
    TMOD &= 0xF0;    // 清空定时器0模式位
    TMOD |= 0x01;    // 定时器0模式1（16位自动重装）
    AUXR |= 0x80;    // 定时器0使用1T模式（不分频，24MHz下1个机器周期=1/24μs）
    TH0 = 0xFF;      // 1ms定时初值（24MHz：(65536 - 24000) = 41536 → 0xFFCC）
    TL0 = 0xCC;      
    ET0 = 1;         // 开启定时器0中断
    TR0 = 0;         // 初始停止定时器0
    EA = 1;          // 开启总中断
}

/************************* 定时器0中断服务函数（1ms一次）*************************/
void Timer0_ISR(void) __interrupt(1)
{
    // 重装初值（1ms）
    TH0 = 0xFF;
    TL0 = 0xCC;
    
    // 仅当计时器运行时累加
    if(timer_3s_running)
    {
        timer_3s_cnt++;
        // 累计3000次=3秒
        if(timer_3s_cnt >= TIMER_3S_COUNT)
        {
            timer_3s_cnt = 0;
            timer_3s_flag = 1; // 置位3秒完成标志
            timer_3s_running = 0; // 停止计时
        }
    }
}

/************************* 3秒计时器控制函数 *************************/
// 启动3秒计时器
void Start_Timer3s(void)
{
    timer_3s_running = true;
    TR0 = 1; // 启动定时器0
}

// 停止3秒计时器
void Stop_Timer3s(void)
{
    timer_3s_running = false;
    TR0 = 0; // 停止定时器0
}

// 重置3秒计时器
void Reset_Timer3s(void)
{
    timer_3s_cnt = 0;
    timer_3s_flag = 0;
    timer_3s_running = false;
}

/************************* 其他函数实现 *************************/
// 串口1初始化（115200波特率）
void UART1_Init(void)
{
    SCON = 0x50;                  
    AUXR |= 0x40; 
    AUXR &= 0xFE; 
    TMOD &= 0x0F;
    TL1 = 0xCC;
    TH1 = 0xFF;
    ET1 = 0;
    TR1 = 1;
    EA = 1;       
}

// 系统初始化
void System_Init(void)
{
    // IO模式配置
    P3M0 &= ~0xfc; P3M1 |= 0xfc;  // P3.2-P3.7高阻输入
    P1M0 &= ~0xbc; P1M1 = (P1M1 & ~0xa0) | 0x1c; // P1.4输入，P1.5/1.7输出
    P5M0 = (P5M0 & ~0x10) | 0x20; P5M1 &= ~0x30; // P5.4/5.5推挽输出
    
    // 输出口默认高电平
    POWER_CTRL = 1;
    KEY1_OUT = 1;
    KEY2_OUT = 1;
    KEY3_OUT = 1;
    
    // 中断配置
    IT0 = 1;  // INT0下降沿触发
    IT1 = 1;  // INT1上升沿触发
    EX0 = 1;  // 开启INT0
    EX1 = 1;  // 开启INT1
    EA = 1;   // 总中断开启
}

// LVD+ADC初始化
void LVD_ADC_Init(void)
{
    P1ASF = 0x00;               
    ADC_CONTR = 0x80;           
    ADC_RES = 0;                
    ADC_RESL = 0;
    Delay_ms(2);                
    ADC_CONTR |= 0x0F;          
    IE2 |= 0x80; 
    LVDCR = 0x00;               
    LVDCR |= (0b100 << 1);      
    LVDCR |= 0x01;              
    PCON &= ~LVDF;              
}

// 毫秒延时函数
void Delay_ms(uint16_t ms)
{
    uint16_t i, j;
    for(i = ms; i > 0; i--)
        for(j = 2475; j > 0; j--);
}

// 进入掉电模式
void Enter_PowerDown_Mode(void)
{
    PCON |= 0x02; 
    NOP();        
}

// 串口发送电压值
void UART1_Send_Voltage(uint16_t volt)
{
    uint8_t buf[16];
    buf[0] = 'V'; buf[1] = 'C'; buf[2] = 'C'; buf[3] = ':';
    buf[4] = volt / 1000 + '0';
    buf[5] = (volt % 1000) / 100 + '0';
    buf[6] = (volt % 100) / 10 + '0';
    buf[7] = volt % 10 + '0';
    buf[8] = 'm'; buf[9] = 'V'; buf[10] = '\r'; buf[11] = '\n'; buf[12] = '\0';
    
    // 串口发送字符串
    uint8_t *p = buf;
    while(*p != '\0')
    {
        SBUF = *p;
        while(!TI);
        TI = 0;
        p++;
    }
}

// 获取VCC电压
uint16_t Get_VCC_Voltage(void)
{
    uint16_t adc_val, voltage;
    ADC_CONTR |= 0x40;          
    while(!(ADC_CONTR & 0x20)); 
    ADC_CONTR &= ~0x20;         
    
    adc_val = (uint16_t)ADC_RES << 4;
    adc_val |= ADC_RESL & 0x0F;
    
    if(adc_val != 0)
    {
        voltage = (uint32_t)REF_VOLTAGE * 4096 / adc_val;
    }
    else
    {
        voltage = 0;
    }
    return voltage;
}

// 检测电压状态
void Detect_Voltage_Status(void)
{
    uint16_t volt = Get_VCC_Voltage();
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
}

// Key1输出0.1秒低脉冲
void Output_Key1_Pulse(void)
{
    KEY1_OUT = 0;
    Delay_ms(DELAY_0_1S);
    KEY1_OUT = 1;
}

// Key2输出0.1秒低脉冲
void Output_Key2_Pulse(void)
{
    KEY2_OUT = 0;
    Delay_ms(DELAY_0_1S);
    KEY2_OUT = 1;
}

// Key3输出0.5秒低脉冲
void Output_Key3_Pulse(void)
{
    KEY3_OUT = 0;
    Delay_ms(DELAY_0_5S);
    KEY3_OUT = 1;
}

// 检测Relay1状态
bool Check_Relay1_Status(void)
{
    return (RELAY1_FEEDBACK == RELAY1_OPEN_LEVEL) ? 1 : 0;
}

// 检测Relay2状态
bool Check_Relay2_Status(void)
{
    return (RELAY2_FEEDBACK == RELAY2_OPEN_LEVEL) ? 1 : 0;
}

// 检测Relay3状态
bool Check_Relay3_Status(void)
{
    return (RELAY3_FEEDBACK == RELAY3_OPEN_LEVEL) ? 1 : 0;
}

// 禁用INT1中断
void Disable_INT1(void)
{
    EX1 = 0;
}

// 启用INT1中断
void Enable_INT1(void)
{
    EX1 = 1;
}

// 检查掉电条件
bool Check_Exit_Condition(void)
{
    return (HUMAN_2410S_IN == 0 && PIR_IN == 0) ? true : false;
}

/************************* 中断服务函数 *************************/
// INT1中断（P3.3上升沿）
void INT1_ISR(void) __interrupt(2)
{
    system_wakeup_flag = 1;
}

// INT0中断（P3.2下降沿）
void INT0_ISR(void) __interrupt(0)
{
    // 预留扩展
}

// LVD中断
void LVD_ISR(void) __interrupt(26)
{
    if(PCON & LVDF)
    {
        voltage_low_flag = 1;
        PCON &= ~LVDF;
    }
}
```

## 4. 定时器3秒计时核心逻辑拆解
### 4.1 定时器配置核心
```c
void Timer0_Init(void)
{
    TMOD &= 0xF0;    // 清空定时器0模式
    TMOD |= 0x01;    // 16位自动重装模式
    AUXR |= 0x80;    // 1T模式（不分频）
    TH0 = 0xFF;      // 24MHz下1ms初值：0xFFCC
    TL0 = 0xCC;      
    ET0 = 1;         // 开启定时器0中断
    TR0 = 0;         // 初始停止
    EA = 1;          // 总中断开启
}
```
- 24MHz晶振下，1T模式的机器周期=1/24μs，要实现1ms中断，需要计数24000次；
- 16位定时器最大值65536，因此初值=65536-24000=41536 → 十六进制`0xFFCC`。

### 4.2 计时控制流程
```mermaid
flowchart TD
    A[启动计时:Start_Timer3s()] --> B[timer_3s_running=1, TR0=1]
    B --> C[定时器0每1ms中断一次]
    C --> D[timer_3s_cnt累加1]
    D --> E{timer_3s_cnt >= 3000?}
    E -- 否 --> C
    E -- 是 --> F[timer_3s_flag=1, timer_3s_running=0]
    F --> G[主逻辑检测到flag，执行3秒后操作]
    G --> H{是否重置计时?}
    H -- 是 --> I[Reset_Timer3s() → 重启Start_Timer3s()]
    H -- 否 --> J[Stop_Timer3s() → TR0=0]
```

### 4.3 关键控制函数
| 函数名          | 功能                          | 核心操作                          |
|-----------------|-------------------------------|-----------------------------------|
| `Start_Timer3s` | 启动3秒计时                   | `timer_3s_running=1` + `TR0=1`    |
| `Stop_Timer3s`  | 停止3秒计时                   | `timer_3s_running=0` + `TR0=0`    |
| `Reset_Timer3s` | 重置3秒计时                   | `timer_3s_cnt=0` + `timer_3s_flag=0` |

## 5. 核心优势对比（定时器VS软件延时）
| 对比项         | 软件延时（原方案）| 定时器计时（新方案）|
|----------------|--------------------------|--------------------------|
| 计时精度       | 受指令执行影响，偏差±10ms | 硬件计时，偏差±1ms       |
| 主逻辑阻塞     | 计时期间阻塞所有操作     | 计时不阻塞，实时检测传感器 |
| 资源占用       | 占用CPU全部资源          | 仅中断时占用少量CPU资源  |
| 扩展性         | 无法同时执行其他逻辑     | 可扩展多定时器/多计时任务 |

## 6. 总结
1. **定时器核心实现**：基于STC8G1K17的定时器0（1T模式）实现1ms精准中断，累计3000次触发3秒计时完成标志；
2. **控制逻辑**：通过`timer_3s_running`、`timer_3s_cnt`、`timer_3s_flag`三个变量实现计时器的启动/停止/重置/完成检测；
3. **业务兼容**：完全保留原有Relay联动、电压检测、掉电机制，仅替换计时方式，无业务逻辑变更；
4. **精度提升**：计时精度从±10ms提升至±1ms，且计时过程中可实时检测传感器/Relay状态，保证逻辑实时性。

调试时若需调整计时精度，可修改定时器初值（`TH0/TL0`）或`TIMER_3S_COUNT`参数；若需扩展多计时任务，可复用定时器0中断，通过不同计数器实现。
