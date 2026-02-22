/**************************************************************************************
 * 程序文档说明（V2.20 最终版）
 * 1. 硬件平台：STC8G1K17单片机，晶振频率24MHz
 * 2. 编译环境：VSCode + SDCC/STC官方编译器
 * 3. 功能描述：
 *    - 基于2410s（P3.2）和PIR（P3.3）双人体传感器检测人员状态，P3.3上升沿中断唤醒掉电模式
 *    - 集成CH15通道LVD+ADC电压检测，串口1（115200波特率）输出当前电压值，支持调试模式开关
 *    - 新增P3.3中断防重复触发机制：唤醒后屏蔽中断，掉电前恢复中断
 *    - 核心计时逻辑：唤醒后Relay1联动Key1→P5.5打开2401电源→延时1s→执行Relay2/电压联动→根据结果重置计时/进入掉电
 *    - 精准控制HMBC09P芯片的Key1/Key2/Key3输出指定时长低脉冲，Relay1→Key1、Relay2→Key2、Relay3→Key3
 *    - 低功耗设计：无人员活动时进入掉电模式，仅P3.3上升沿中断可唤醒
 *    - 预设调试模式：当定义调试模式时，串口输出状态变化，否则不初始化串口，不输出
 *    - 预设看门狗模式：定义看门狗模式时，打开看门狗功能，定期喂狗
 *    - 预设2401供电模式：定义时，掉电状态不供电，退出后打开供电，否则，保持常态供电
 *    - 预设p55供电时的输出电平：1为高电平打开供电，0为低电平打开供电
 *    - 低电压阈值（2.7V）
 * 4. IO口定义及模式：
 *    - 刷机/串口复用口：P3.1(TX1)、P3.0(RX1)（刷机时为下载口，运行时为串口1）
 *    - 输入口（高阻模式）：
 *      P3.2 - 2410s人体检测（下拉，默认低电平，高电平有人/低电平无人，下降沿触发中断）
 *      P3.3 - PIR红外传感器（默认低电平，高电平有人，上升沿触发中断）
 *      P3.4 - LED1状态（高电平亮）
 *      P3.5 - LED2状态（高电平亮）
 *      P1.3 - LED3状态（高电平亮）
 *      P3.6 - Relay1反馈（默认低电平，高电平打开）
 *      P3.7 - Relay2反馈（默认低电平，高电平打开）
 *      P1.4 - Relay3反馈（默认低电平，高电平打开）
 *    - 输出口（推挽输出模式，默认高电平）：
 *      P5.5 - 2410s供电开关
 *      P5.4 - HMBC09P Key1输出（Relay1联动）
 *      P1.7 - HMBC09P Key2输出（Relay2联动）
 *      P1.5 - HMBC09P Key3输出（Relay3电压联动）
 * 5. 核心逻辑（V2.0 最终版）：
 *    - P3.3上升沿触发中断 → 置位唤醒标志 + 屏蔽INT1中断（防重复触发）→ 退出掉电模式
 *    - 唤醒后：如果led1关闭 →Key1输出0.05s低脉冲→ P5.5打开2401的电源 → 延时1s→检测电压（CH15通道）→ 串口输出电压值 → 标记电压高/低→
 *    - 固定执行逻辑：
 *      ① 有人（P3.2高）+ led2关闭 → Key2输出0.05s低脉冲 
 *      ③ 电压低+Relay3关闭 → Key3输出0.05s低脉冲；电压高+Relay3打开 → Key3输出0.05s低脉冲
 *      ④ 无人（P3.2低）+ Relay2打开 → Key2输出0.05s低脉冲；检查P3.2、P3.3是否为低：
 *         - 满足：P3.3如为低→ Key1输出0.05s低脉冲；P5.5关闭2401供电+延时1s→恢复INT1→掉电；
 *         - 不满足： 继续循环
 * 编译环境：VSCode + SDCC/STC官方编译器
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
// 工作模式配置
#define DEBUG_MODE         1       // 1=开启串口输出；0=关闭串口
#define WDT_ENABLE         1       // 1=启用看门狗；0=禁用看门狗
#define POWER_SAVE_MODE    1       // 1=掉电时关闭2401供电；0=保持常态供电

#define POWER_CTRL_MODE    1       // 0=低电平开启电源；1=高电平开启电源

// 时间参数（单位：ms）
#define DELAY_1S           1000    // 1秒延时
#define DELAY_0_5S         500     // 0.5秒延时
#define DELAY_0_05S        50      // 脉冲低电平/高电平时长（0.05秒）
#define DELAY_0_2S         200     // 脉冲结束后保持高电平时间（0.2秒）
#define DELAY_10MS         10      // 10ms延时
#define VOLTAGE_THRESHOLD  2700    // 电压阈值（2.7V）
#define REF_VOLTAGE        1167    // 内部参考电压（1.19V）

// LED和继电器预设值
#define LED1_ON_LEVEL      1       // LED1亮时的电平（1=高电平亮）
#define LED2_ON_LEVEL      1       // LED2亮时的电平（1=高电平亮）
#define LED3_ON_LEVEL      1       // LED3亮时的电平（1=高电平亮）
#define RELAY1_OPEN_LEVEL  1       // Relay1打开时的电平
#define RELAY2_OPEN_LEVEL  1       // Relay2打开时的电平
#define RELAY3_OPEN_LEVEL  1       // Relay3打开时的电平

// 调试+串口参数
#define BAUDRATE           115200  // 串口波特率
#define FOSC               24000000// 晶振频率（24MHz）

/************************* IO口定义 *************************/
// 输入口
#define HUMAN_2410S_IN    P32     // 2410s人体检测（默认低电平，高电平有人）
#define PIR_IN            P33     // PIR红外传感器（默认低电平，高电平有人，上升沿触发中断）
#define LED1_STATUS       P34     // LED1状态输入
#define LED2_STATUS       P35     // LED2状态输入
#define LED3_STATUS       P13     // LED3状态输入
#define RELAY1_FEEDBACK   P36     // Relay1反馈
#define RELAY2_FEEDBACK   P37     // Relay2反馈
#define RELAY3_FEEDBACK   P14     // Relay3反馈

// 输出口
#define POWER_CTRL        P55     // 2410s/HMBC09P供电开关
#define KEY1_OUT          P54     // HMBC09P Key1输出（LED1联动）
#define KEY2_OUT          P17     // HMBC09P Key2输出（LED2联动）
#define KEY3_OUT          P15     // HMBC09P Key3输出（LED3电压联动）

/************************* 全局变量 *************************/
bool voltage_low_flag = 0;        // 低电压标记
bool voltage_high_flag = 0;       // 高电压标记
bool wakeup_flag = 0;             // 唤醒标记
bool int1_disabled = 0;           // INT1中断屏蔽标记
bool power_on_flag = 0;           // 2401电源开启标记
uint8_t loop_counter = 0;         // 循环计数器

/************************* 看门狗配置 *************************/
#define WDT_PS_2POW7    0x07       // 2^7 = 128 -> 约 12.8ms
#define WDT_PS_VALUE     WDT_PS_2POW7
#define WDT_ENABLE_BIT   0x20
#define WDT_CLEAR_BIT    0x10

#define WDT_SOFT_COUNT   500        // 10ms * 500 = 5000ms (5秒)
#define WDT_FEED_INTERVAL 20        // 每20次循环强制喂狗一次

static volatile uint16_t wdt_soft_counter = WDT_SOFT_COUNT;
static uint16_t wdt_loop_counter = 0;
static uint16_t wdt_feed_count = 0;
static bool wdt_initialized = 0;

/************************* 函数声明 *************************/
void System_Init(void);
void LVD_ADC_Init(void);
void Delay_ms_with_wdt(uint16_t ms);
void Enter_PowerDown(void);
void Enable_INT1(void);
void Disable_INT1(void);
uint16_t Get_VCC_Voltage(void);
void Detect_Voltage_Status(void);
void Output_Key1_Pulse(void);
void Output_Key2_Pulse(void);
void Output_Key3_Pulse(void);
bool Check_LED1_Status(void);
bool Check_LED2_Status(void);
bool Check_LED3_Status(void);
bool Check_Relay1_Status(void);
bool Check_Relay2_Status(void);
bool Check_Relay3_Status(void);
void Process_Wakeup_Sequence(void);
void Process_Fixed_Logic(void);
bool Check_Sleep_Condition(void);

/************************* 看门狗函数声明 *************************/
#if WDT_ENABLE == 1
void WDT_Init(void);
void WDT_Feed(void);
void WDT_Before_PowerDown(void);
void WDT_After_Wakeup(void);
#endif

/************************* 调试模式相关函数声明 *************************/
#if DEBUG_MODE == 1
void UART1_Init(void);
void UART1_Send_Voltage(uint16_t volt);
void UART1_Send_String(char *str);
void UART1_Send_Char(char c);
void UART1_Send_Num(uint16_t num);
void Debug_Output_Status(void);
#endif

/************************* 主函数 *************************/
void main(void)
{
    // 系统初始化
    System_Init();
    LVD_ADC_Init();
    
    // 看门狗初始化
    #if WDT_ENABLE == 1
    WDT_Init();
    #endif
    
    // 如果开启调试模式，初始化串口
    #if DEBUG_MODE == 1
    UART1_Init();
    Delay_ms_with_wdt(10);
    UART1_Send_String("\r\n\r\n=== System Initialized ===\r\n");
    UART1_Send_String("Version: V2.1 Final (IO Mode Optimized)\r\n");
    UART1_Send_String("Date: 2024\r\n");
    UART1_Send_String("==========================\r\n\r\n");
    #endif
    
    // 主循环
    while(1)
    {
        // 立即喂狗
        #if WDT_ENABLE == 1
        WDT_Feed();
        #endif
        
        // 检查唤醒标志
        if(wakeup_flag)
        {
            #if DEBUG_MODE == 1
            UART1_Send_String("\r\n=== Wakeup triggered ===\r\n");
            #endif
            
            // 唤醒后重新初始化看门狗
            #if WDT_ENABLE == 1
            WDT_After_Wakeup();
            #endif
            
            // 处理唤醒后的时序
            Process_Wakeup_Sequence();
            
            // 清除唤醒标志
            wakeup_flag = 0;
            
            #if DEBUG_MODE == 1
            UART1_Send_String("=== Wakeup processing complete ===\r\n\r\n");
            #endif
        }
        
        Process_Fixed_Logic();
        
        // 检查是否可以进入睡眠
        if(Check_Sleep_Condition())
        {
            #if DEBUG_MODE == 1
            UART1_Send_String("\r\n=== Entering power down mode ===\r\n");
            Delay_ms_with_wdt(10);
            #endif
            
            // 进入掉电模式前的看门狗处理
            #if WDT_ENABLE == 1
            WDT_Before_PowerDown();
            #endif
            
            // 进入掉电模式
            Enter_PowerDown();
        }
        
        // 小延时
        Delay_ms_with_wdt(DELAY_10MS);
    }
}

/************************* 唤醒处理函数 *************************/
void Process_Wakeup_Sequence(void)
{
    #if DEBUG_MODE == 1
    UART1_Send_String("Wakeup Sequence Start\r\n");
    #endif
    
    // 唤醒后：如果led1关闭 → Key1输出脉冲
    if(!Check_LED1_Status())
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("LED1 off -> Key1 pulse\r\n");
        #endif
        Output_Key1_Pulse();
    }
    
    // P5.5打开2401的电源
    #if POWER_SAVE_MODE == 1
    POWER_CTRL = POWER_CTRL_MODE ? 1 : 0;
    power_on_flag = 1;
    #if DEBUG_MODE == 1
    UART1_Send_String("Power ON for 2401\r\n");
    #endif
    #endif

    // 延时1s
    Delay_ms_with_wdt(DELAY_1S);
}

/************************* 固定执行逻辑 *************************/
void Process_Fixed_Logic(void)
{
    static uint16_t voltage_check_counter = 0;
    
    // ① 有人（P3.2高）+ led2关闭 → Key2输出脉冲
    if(HUMAN_2410S_IN == 1 && !Check_LED2_Status())
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("Human detected + LED2 off -> Key2 pulse\r\n");
        #endif
        Output_Key2_Pulse();
    }
    
    // ② 电压低+Relay3关闭 → Key3输出脉冲
    if(voltage_low_flag && !Check_LED3_Status())
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("Low voltage + LED3 off -> Key3 pulse\r\n");
        #endif
        Output_Key3_Pulse();
    }
    // 电压高+Relay3打开 → Key3输出脉冲
    else if(voltage_high_flag && Check_LED3_Status())
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("High voltage + LED3 on -> Key3 pulse\r\n");
        #endif
        Output_Key3_Pulse();
    }
    
    // ③ 无人（P3.2低）+ Relay2打开 → Key2输出脉冲
    if(HUMAN_2410S_IN == 0 && Check_LED2_Status() == 1)
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("No human + LED2 on -> Key2 pulse\r\n");
        #endif
        Output_Key2_Pulse();
    }
    
    // 定期电压检测
    voltage_check_counter++;
    if(voltage_check_counter >= 100)
    {
        voltage_check_counter = 0;
        Detect_Voltage_Status();
        
        #if DEBUG_MODE == 1
        Debug_Output_Status();
        #endif
    }
}

/************************* 睡眠条件检查 *************************/
bool Check_Sleep_Condition(void)
{
    if(HUMAN_2410S_IN == 0 && PIR_IN == 0)
    {
        #if DEBUG_MODE == 1
        UART1_Send_String("Sleep condition met: P3.2 low, P3.3 low\r\n");
        #endif
        
        if(PIR_IN == 0 && Check_LED1_Status() == 1)
        {
            #if DEBUG_MODE == 1
            UART1_Send_String("PIR low -> Key1 pulse before sleep\r\n");
            #endif
            Output_Key1_Pulse();
        }
        
        #if POWER_SAVE_MODE == 1
        POWER_CTRL = POWER_CTRL_MODE ? 0 : 1;
        power_on_flag = 0;
        #if DEBUG_MODE == 1
        UART1_Send_String("Power OFF for 2401\r\n");
        #endif
        #endif
        
        Delay_ms_with_wdt(DELAY_1S);
        
        if(int1_disabled)
        {
            Enable_INT1();
            int1_disabled = 0;
            #if DEBUG_MODE == 1
            UART1_Send_String("INT1 restored\r\n");
            #endif
        }
        
        return true;
    }
    
    return false;
}

/************************* 中断服务函数 *************************/
void INT1_ISR(void) __interrupt(2)
{
    IE1 = 0;                     // 清除中断标志
    if (PIR_IN == 1)             // 仅当引脚为高（上升沿后）才处理
    {
        wakeup_flag = 1;
        Disable_INT1();
        int1_disabled = 1;
    }
    // 若为低电平，直接返回，不唤醒
}

/************************* 掉电模式相关函数 *************************/
void Enter_PowerDown(void)
{
    ADC_CONTR &= ~0x80;
    
    EA = 1;
    if(!int1_disabled)
    {
        EX1 = 1;
    }
    IT1 = 1;
    IE1 = 0;
    
    Delay_ms_with_wdt(1);
    
    #if DEBUG_MODE == 1
    UART1_Send_String("Power Down Mode Entered\r\n");
    Delay_ms_with_wdt(1);
    #endif
    
    _nop_();
    _nop_();
    PCON |= 0x02;
    _nop_();
    _nop_();
}

void Enable_INT1(void)
{
    EX1 = 1;
    IE1 = 0;
}

void Disable_INT1(void)
{
    EX1 = 0;
}

/************************* 看门狗函数实现 *************************/
#if WDT_ENABLE == 1

void WDT_Init(void)
{
    WDT_CONTR = WDT_PS_VALUE | WDT_ENABLE_BIT;
    
    wdt_soft_counter = WDT_SOFT_COUNT;
    wdt_loop_counter = 0;
    wdt_feed_count = 0;
    wdt_initialized = 1;
    
    WDT_CONTR |= WDT_CLEAR_BIT;
    _nop_();
    _nop_();
    
    Delay_ms_with_wdt(10);
    
    #if DEBUG_MODE == 1
    UART1_Send_String("Watchdog Hardware Enabled\r\n");
    UART1_Send_String("Software Counter: 500 -> ~5s\r\n");
    #endif
}

void WDT_Feed(void)
{
    if(!wdt_initialized) return;
    
    wdt_loop_counter++;
    
    if(wdt_soft_counter > 0)
    {
        wdt_soft_counter--;
    }
    
    #if DEBUG_MODE == 1
    static uint16_t last_counter = 0;
    if(last_counter != wdt_soft_counter)
    {
        last_counter = wdt_soft_counter;
        if(wdt_soft_counter % 50 == 0 || wdt_soft_counter < 50)
        {
            UART1_Send_String("WDT: ");
            UART1_Send_Num(wdt_soft_counter);
            UART1_Send_String("/500\r\n");
        }
    }
    #endif
    
    if(wdt_soft_counter == 0 || wdt_loop_counter >= WDT_FEED_INTERVAL)
    {
        WDT_CONTR |= WDT_CLEAR_BIT;
        _nop_();
        _nop_();
        
        wdt_soft_counter = WDT_SOFT_COUNT;
        wdt_loop_counter = 0;
        wdt_feed_count++;
    }
}

void WDT_Before_PowerDown(void)
{
    wdt_initialized = 0;
    #if DEBUG_MODE == 1
    UART1_Send_String("WDT stopped for power down\r\n");
    #endif
}

void WDT_After_Wakeup(void)
{
    WDT_Init();
    #if DEBUG_MODE == 1
    UART1_Send_String("WDT reinitialized\r\n");
    #endif
}

#endif

/************************* 调试模式相关函数 *************************/
#if DEBUG_MODE == 1

void UART1_Send_Num(uint16_t num)
{
    if(num >= 100)
    {
        UART1_Send_Char(num / 100 + '0');
        UART1_Send_Char((num % 100) / 10 + '0');
        UART1_Send_Char(num % 10 + '0');
    }
    else if(num >= 10)
    {
        UART1_Send_Char(' ');
        UART1_Send_Char(num / 10 + '0');
        UART1_Send_Char(num % 10 + '0');
    }
    else
    {
        UART1_Send_Char(' ');
        UART1_Send_Char(' ');
        UART1_Send_Char(num + '0');
    }
}

void Debug_Output_Status(void)
{
    UART1_Send_String("\r\n=== Status ===\r\n");
    UART1_Send_String("Human: "); UART1_Send_String(HUMAN_2410S_IN ? "YES\r\n" : "NO\r\n");
    UART1_Send_String("PIR: "); UART1_Send_String(PIR_IN ? "HIGH\r\n" : "LOW\r\n");
    UART1_Send_String("LED1: "); UART1_Send_String(Check_LED1_Status() ? "ON\r\n" : "OFF\r\n");
    UART1_Send_String("LED2: "); UART1_Send_String(Check_LED2_Status() ? "ON\r\n" : "OFF\r\n");
    UART1_Send_String("Relay2: "); UART1_Send_String(Check_Relay2_Status() ? "ON\r\n" : "OFF\r\n");
    UART1_Send_String("Relay3: "); UART1_Send_String(Check_Relay3_Status() ? "ON\r\n" : "OFF\r\n");
    UART1_Send_String("Voltage: "); UART1_Send_String(voltage_low_flag ? "LOW\r\n" : "HIGH\r\n");
    UART1_Send_String("Power: "); UART1_Send_String(power_on_flag ? "ON\r\n" : "OFF\r\n");
    UART1_Send_String("WDT: "); UART1_Send_Num(wdt_soft_counter); UART1_Send_String("/500\r\n");
    UART1_Send_String("============\r\n\r\n");
}

void UART1_Send_Char(char c)
{
    SBUF = c;
    while(!TI);
    TI = 0;
}

void UART1_Send_String(char *str)
{
    while(*str)
    {
        UART1_Send_Char(*str++);
    }
}

void UART1_Send_Voltage(uint16_t volt)
{
    UART1_Send_String("VCC: ");
    UART1_Send_Char(volt / 1000 + '0');
    UART1_Send_Char('.');
    UART1_Send_Char((volt % 1000) / 100 + '0');
    UART1_Send_Char((volt % 100) / 10 + '0');
    UART1_Send_Char(volt % 10 + '0');
    UART1_Send_String("V\r\n");
}

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

#endif

/************************* 系统初始化（重构版） *************************/
void System_Init(void)
{
    // ---------- 将所有端口初始化为高阻输入 ----------
    // P1口全部高阻
    P1M0 = 0x00;
    P1M1 = 0xFF;
    // P3口全部高阻（后续单独配置串口引脚）
    P3M0 = 0x00;
    P3M1 = 0xFF;
    // P5口除P55外高阻，P55设为推挽输出（电源控制）
    P5M0 = 0x20;        // 仅P5.5 (bit5) 置1，其余为0
    P5M1 = 0x00;        // P5M1全0，与P5M0配合：P5.5推挽，其他高阻

    // ---------- 单独配置串口引脚 ----------
    // P3.0 (RxD) 保持高阻输入（已为高阻），P3.1 (TxD) 改为推挽输出
    P3M1 &= ~0x02;      // 清除P3.1对应的M1位
    P3M0 |= 0x02;       // 设置P3.1对应的M0位为1 → 推挽输出

    // ---------- 输出口默认电平 ----------
    #if POWER_SAVE_MODE == 1
    POWER_CTRL = POWER_CTRL_MODE ? 0 : 1;   // 默认关闭电源
    power_on_flag = 0;
    #else
    POWER_CTRL = POWER_CTRL_MODE ? 1 : 0;   // 默认开启电源
    power_on_flag = 1;
    #endif

    // 按键输出引脚初始为高阻，内部无上拉，外部需接上拉电阻或由外部电路决定电平
    // 此处无需额外赋值，因为高阻模式下写入无效

    // ---------- 中断配置 ----------
    IT1 = 0;            // INT1低电平触发（原为0，保持）
    EX1 = 1;            // 使能INT1中断
    IE1 = 0;
    PX1 = 1;            // 中断优先级高
    EA = 1;             // 总中断使能

    wakeup_flag = 0;
    int1_disabled = 0;
}

/************************* LVD与ADC初始化 *************************/
void LVD_ADC_Init(void)
{
    // ADC初始化（用于测量VCC）
    P1ASF = 0x00;              // 关闭P1口模拟输入功能
    ADC_CONTR = 0x80;          // 开启ADC电源
    ADC_RES = 0;
    ADC_RESL = 0;
    Delay_ms_with_wdt(2);
    ADC_CONTR |= 0x0F;          // 选择通道15（内部1.19V参考电压源）
    // 注意：不再配置LVD相关寄存器，也不再使能LVD中断
}

void Delay_ms_with_wdt(uint16_t ms)
{
    uint16_t i, j;
    for(i = 0; i < ms; i++)
    {
        for(j = 0; j < 2475; j++)
        {
            _nop_();
        }
        #if WDT_ENABLE == 1
        WDT_Feed();  // 每1ms喂狗一次
        #endif
    }
}

uint16_t Get_VCC_Voltage(void)
{
    uint16_t adc_val, voltage;
    if (!(ADC_CONTR & 0x80))
    {
        ADC_CONTR = 0x80 | 0x0F;
        Delay_ms_with_wdt(2);
    }
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
    #if DEBUG_MODE == 1
    UART1_Send_Voltage(volt);
    #endif
}

/************************* 脉冲输出函数（新时序） *************************/
void Output_Key1_Pulse(void)
{
    uint8_t p5m0_old, p5m1_old;

    // 保存P5口原模式（仅用于恢复）
    p5m0_old = P5M0;
    p5m1_old = P5M1;

    // 临时将P5.4设为推挽输出
    P5M0 |= 0x10;       // P5.4对应bit4
    P5M1 &= ~0x10;

    // 输出高电平
    KEY1_OUT = 1;
    Delay_ms_with_wdt(DELAY_0_05S);   // 50ms

    // 输出低电平
    KEY1_OUT = 0;
    Delay_ms_with_wdt(DELAY_0_05S);   // 50ms

    // 恢复高电平
    KEY1_OUT = 1;

    // 保持高电平200ms
    Delay_ms_with_wdt(DELAY_0_5S);

    // 恢复P5.4为高阻输入
    P5M0 = p5m0_old;
    P5M1 = p5m1_old;
}

void Output_Key2_Pulse(void)
{
    uint8_t p1m0_old, p1m1_old;

    p1m0_old = P1M0;
    p1m1_old = P1M1;

    // 临时将P1.7设为推挽输出
    P1M0 |= 0x80;       // P1.7对应bit7
    P1M1 &= ~0x80;

    KEY2_OUT = 1;
    Delay_ms_with_wdt(DELAY_0_05S);
    KEY2_OUT = 0;
    Delay_ms_with_wdt(DELAY_0_05S);
    KEY2_OUT = 1;
    Delay_ms_with_wdt(DELAY_0_5S);

    // 恢复P1口模式（全部高阻）
    P1M0 = p1m0_old;
    P1M1 = p1m1_old;
}

void Output_Key3_Pulse(void)
{
    uint8_t p1m0_old, p1m1_old;

    p1m0_old = P1M0;
    p1m1_old = P1M1;

    // 临时将P1.5设为推挽输出
    P1M0 |= 0x20;       // P1.5对应bit5
    P1M1 &= ~0x20;

    KEY3_OUT = 1;
    Delay_ms_with_wdt(DELAY_0_05S);
    KEY3_OUT = 0;
    Delay_ms_with_wdt(DELAY_0_05S);
    KEY3_OUT = 1;
    Delay_ms_with_wdt(DELAY_0_5S);

    // 恢复P1口模式（全部高阻）
    P1M0 = p1m0_old;
    P1M1 = p1m1_old;
}

/************************* 状态检测函数 *************************/
bool Check_LED1_Status(void)
{
    return (LED1_STATUS == LED1_ON_LEVEL);
}

bool Check_LED2_Status(void)
{
    return (LED2_STATUS == LED2_ON_LEVEL);
}

bool Check_LED3_Status(void)
{
    return (LED3_STATUS == LED3_ON_LEVEL);
}

bool Check_Relay1_Status(void)
{
    return (RELAY1_FEEDBACK == RELAY1_OPEN_LEVEL);
}

bool Check_Relay2_Status(void)
{
    return (RELAY2_FEEDBACK == RELAY2_OPEN_LEVEL);
}

bool Check_Relay3_Status(void)
{
    return (RELAY3_FEEDBACK == RELAY3_OPEN_LEVEL);
}

// ==================== 程序结束 ====================
