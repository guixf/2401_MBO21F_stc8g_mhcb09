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
#define DELAY_0_1S         50     // Key1/Key2低脉冲时长
#define DELAY_0_5S         50     // Key3低脉冲时长
#define TIMER_3S_COUNT     3000    // 3秒计时阈值（1ms中断×3000次）
#define VOLTAGE_THRESHOLD  3000    // 电压阈值（3V）
#define REF_VOLTAGE        1190    // 内部参考电压（1.19V）

// Relay预设值
#define RELAY1_OPEN_LEVEL  1       // Relay1打开时为高电平
#define RELAY2_OPEN_LEVEL  0       // Relay2打开时为高电平
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
#define LED1              P34     // led1反馈
#define LED2              P35     // led2反馈
#define LED3              P13     // led3反馈
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
            Detect_Voltage_Status();                   // 先检测电压并串口输出
            if(DEBUG_MODE)
                    {
                        //UART1_Send_Voltage(Get_VCC_Voltage());
                    }

            // 核心业务循环
            while(1)
            {
                // 检测3秒计时完成标志
                if(timer_3s_flag)
                {
                    timer_3s_flag = 0; // 清除计时完成标志
                    Stop_Timer3s();    // 停止计时器
                    
 

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

                        Output_Key1_Pulse();
                        //UART1_Send_Voltage(Get_VCC_Voltage());
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
    return (LED2 == RELAY2_OPEN_LEVEL) ? 1 : 0;
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
    //return (HUMAN_2410S_IN == 0 && PIR_IN == 0) ? true : false;
    return (LED1==1 && LED2==1)? true : false;
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