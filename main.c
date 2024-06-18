#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/hibernate.h"
#include "driverlib/eeprom.h"

#define SYSTICK_FREQUENCY       1000

#define PCA9557_I2CADDR         0x18
#define PCA9557_INPUT           0x00
#define	PCA9557_OUTPUT          0x01
#define PCA9557_POLINVERT       0x02
#define PCA9557_CONFIG          0x03

#define TCA6424_I2CADDR         0x22
#define TCA6424_INPUT_PORT0     0x00
#define TCA6424_INPUT_PORT1     0x01
#define TCA6424_INPUT_PORT2     0x02
#define TCA6424_OUTPUT_PORT0    0x04
#define TCA6424_OUTPUT_PORT1    0x05
#define TCA6424_OUTPUT_PORT2    0x06
#define TCA6424_CONFIG_PORT0    0x0c
#define TCA6424_CONFIG_PORT1    0x0d
#define TCA6424_CONFIG_PORT2    0x0e
#define TCA6424_DELAY           10000

#define BUTTON_UP               4
#define BUTTON_DOWN             3
#define BUTTON_LEFT             6
#define BUTTON_RIGHT            5
#define BUTTON_DISCARD          0 // same as BUTTON_1
#define BUTTON_1                0
#define BUTTON_2                1
#define BUTTON_3                2
#define BUTTON_BACK             7

#define KEY_CONFIG_PRESS        0x01
#define KEY_DELAY               10      // time interval for holding button, times of 20ms

#define MODE_DISPLAY            0x01
#define MODE_SETDATE            0x02
#define MODE_SETTIME            0x04
#define MODE_SETALARM           0x08

#define ERROR_SUCCESS           0x0000
#define ERROR_NOT_DIGIT         0x0100
#define ERROR_NO_DELIM          0x0200
#define ERROR_NOT_MATCH         0x0400
#define ERROR_PARTIAL           0x0800
#define ERROR_FORMAT            0x1000

#define ROM_MAGIC               0xbeefcafe
#define ROM_ADDRESS             0x0400

#define MAX(a, b)               (((a) > (b)) ? (a) : (b))

//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG(str) UART0StringPutNonBlocking((str))
#else
#define DEBUG(str)
#endif

typedef struct datetime {
    uint8_t month;
    uint8_t day;
    uint16_t year;
    uint32_t time;
} datetime_t;

typedef struct keystate {
    uint8_t config;
    uint8_t state;  // including previous state by shifting bit
    uint8_t timer;
    uint8_t flag;   // true if need to be handled
} keystate_t;

typedef uint16_t error_t;

void Setup(void);
void ProcDisplay(void);
void ProcSetDate(void);
void ProcSetTime(void);
void DisplayDatetime(uint8_t offset);
void DisplayDate(uint16_t year, uint8_t day, uint8_t month);
void DisplayTime(uint32_t time);
void DetectKey(void);
void ClearKeyFlags(void);
void ProcessCommand(void);
error_t ParseCommand(const char *pattern, const char *command, datetime_t *args);
error_t ParseIntegerUntil(const char *str, char delim, uint8_t *index, int *result);
void StringifyDate(uint16_t year, uint8_t month, uint8_t day, char *buffer);
void StringifyTime(uint32_t time, char *buffer);
char ToUpperCase(char x);
uint8_t GetDayOfMonth(uint16_t year, uint8_t month);
void Delay(uint32_t loop);
void ClearSystickCounter(void);

void GPIOInit(void);
void UART0Init(void);
void UART0StringPutNonBlocking(const char *message);
void UART0NumberPutNonBlocking(int64_t data);
void I2C0Init(void);
uint8_t I2C0WriteByte(uint8_t device, uint8_t reg, uint8_t data);
uint8_t I2C0ReadByte(uint8_t device, uint8_t reg);
void BuzzerInit(void);
void BuzzerStart(uint32_t freq);
void BuzzerStop(void);
void RTCInit(void);
void RTCStoreData(void);
void RTCLoadData(void);
void ROMInit(void);
void ROMStoreData(void);
void ROMLoadData(void);

void SysTick_Handler(void);
void UART0_Handler(void);

const uint8_t seg7[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x5c
};
const uint8_t student_id[] = {3, 1, 9, 1, 0, 7, 8, 1};
const uint8_t student_name[] = {0x39, 0x3e, 0x06, 0x00, 0xdb, 0xf6, 0x00, 0x00};
const uint8_t version[] = {0x3e, 0x00, 0x86, 0xbf, 0x3f, 0x00, 0x00, 0x00};
const char *help_message = "EST2506 课程大作业 V1.0.0 指令帮助\r\n"
    "UART串口波特率115200，数据帧8+0+1\r\n"
    "    CLOCK INIT          - 初始化时钟到默认状态，包括时间、日期、闹铃\r\n"
    "    CLOCK RESTART       - 重新启动时钟\r\n"
    "    CLOCK HIB           - 将处理器切入休眠状态\r\n"
    "    GET DATE            - 获取当前日期\r\n"
    "    GET TIME            - 获取当前时间\r\n"
    "    GET ALARM           - 获取闹铃时间\r\n"
    "    SET DATE <DATE>     - 设置当前日期，<DATE>为YYYY/MM/DD格式\r\n"
    "    SET TIME <TIME>     - 设置当前时间，<TIME>为HH:MM:SS格式\r\n"
    "    SET ALARM <TIME>    - 设置闹铃时间，<TIME>为HH:MM:SS格式\r\n"
    "    MUTE                - 关闭正在响铃的闹钟\r\n"
    "    ?                   - 输出帮助文本\r\n"
    "示例：\r\n"
    "    SET DATE 2024/06/18\r\n"
    "    SET ALARM 13:00:50";

uint32_t sys_clock_freq;

volatile uint16_t systick_20ms_counter = 0, systick_250ms_counter = 0, systick_500ms_counter = 0;
volatile uint16_t systick_1s_counter = 0;
volatile uint8_t systick_20ms_flag = 0, systick_250ms_flag = 0, systick_500ms_flag = 0;
volatile uint8_t systick_1s_flag = 0;

volatile uint8_t command[128];
volatile uint8_t command_ready = 0;

datetime_t datetime;
uint32_t alarm_time = 999;
volatile keystate_t keystate[8];

int8_t mode = MODE_DISPLAY;

uint8_t flow_offset = 0;
int8_t flow_speed = 1;
uint8_t focus_digit = 0;
uint8_t focus_flash = 0; // true to hide focus digit
int8_t setting_digit[8];

uint8_t alarming = 0;
uint8_t load_rom = 0;

int main(void) {
    sys_clock_freq = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480, 20000000);

    SysTickPeriodSet(sys_clock_freq / SYSTICK_FREQUENCY);
    SysTickEnable();
    SysTickIntEnable();
    
    if (!HibernateIsActive()) {
        load_rom = 1;
    }
    
    GPIOInit();
    UART0Init();
    I2C0Init();
    BuzzerInit();
    RTCInit();
    ROMInit();

    // Enable interrupt
    IntMasterEnable();
    
    datetime.year = 2000;
    datetime.month = 1;
    datetime.day = 1;
    datetime.time = 0;
    
    keystate[BUTTON_UP].config = keystate[BUTTON_DOWN].config &= KEY_CONFIG_PRESS;
    
    // Setup code
    Setup();
    
    // Main loop
    ClearSystickCounter();
    ClearKeyFlags();
    I2C0ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0); // solve glitch at first time
    while (1) {
        // Process systick counter
        if (systick_20ms_flag) {
            systick_20ms_flag = 0;
            DetectKey(); // detect key per 20ms
        }
        
        if (systick_250ms_flag) {
            systick_250ms_flag = 0;
            
            if (mode == MODE_DISPLAY) {
                // faster flow
                if (flow_speed == 2) {
                    flow_offset = (flow_offset + 1) % 16; // 16 is length of display datetime
                } else if (flow_speed == -2) {
                    flow_offset = (flow_offset - 1) % 16;
                }
            } else {
                // flash
                focus_flash = !focus_flash;
            }
            
            // stages of alarming
            if (alarming == 1) {
                BuzzerStart(880);
                alarming = 2;
            } else if (alarming == 2) {
                BuzzerStop();
                alarming = 1;
            } else {
                BuzzerStop();
            }
        }
        
        if (systick_500ms_flag) {
            systick_500ms_flag = 0;
            
            if (mode == MODE_DISPLAY) {
                // flow
                if (flow_speed == 1) {
                    flow_offset = (flow_offset + 1) % 16; // 16 is length of display datetime
                } else if (flow_speed == -1) {
                    flow_offset = (flow_offset - 1 + 16) % 16;
                }
            }
        }
        
        if (systick_1s_flag) {
            systick_1s_flag = 0;
            
            // next second
            ++datetime.time;
            if (datetime.time >= 86400) {
                uint8_t day_of_month = GetDayOfMonth(datetime.year, datetime.month);
                
                datetime.time -= 86400;
                ++datetime.day;
                
                if (datetime.day > day_of_month) {
                    datetime.day -= day_of_month;
                    ++datetime.month;
                    
                    if (datetime.month > 12) {
                        datetime.year = (datetime.year + 1) % 10000; // year should be 0-9999
                    }
                }
            }
            
            RTCStoreData();
            
            if (datetime.time == alarm_time) {
                alarming = 1;
            }
        }
        
        switch (mode) {
            case MODE_DISPLAY:
                ProcDisplay();
                break;
            case MODE_SETDATE:
                ProcSetDate();
                break;
            case MODE_SETTIME:
            case MODE_SETALARM:
                ProcSetTime();
                break;
        }
        
        I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~mode);
        
        // Process UART command
        if (command_ready) {
            ProcessCommand();
            command_ready = 0;
        }
    }
}

void Setup(void) {
    uint8_t i = 0, b = 0;
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff); // turn off all leds
    systick_500ms_counter = systick_500ms_flag = 0;
    while (!systick_500ms_flag); // delay for 500ms
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); // turn on all leds
    // Show student code
    systick_500ms_counter = systick_500ms_flag = 0; // reset systick counter
    while (!systick_500ms_flag) { // show for 500ms
        for (i = 0; i < 8; ++i) {
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[student_id[i]]);
            Delay(TCA6424_DELAY);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
        }
    }
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff); // turn off all leds
    systick_500ms_counter = systick_500ms_flag = 0;
    while (!systick_500ms_flag); // delay for 500ms
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); // turn on all leds
    // Show student name
    systick_500ms_counter = systick_500ms_flag = 0; // reset systick counter
    while (!systick_500ms_flag) { // show for 500ms
        for (i = 0; i < 8; ++i) {
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, student_name[i]);
            Delay(TCA6424_DELAY);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
        }
    }
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff); // turn off all leds
    systick_500ms_counter = systick_500ms_flag = 0;
    while (!systick_500ms_flag); // delay for 500ms
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); // turn on all leds
    // Show version
    systick_500ms_counter = systick_500ms_flag = 0; // reset systick counter
    while (!systick_500ms_flag) { // show for 500ms
        for (i = 0; i < 8; ++i) {
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, version[i]);
            Delay(TCA6424_DELAY);
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
        }
    }
    
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff); // turn off all leds
    systick_500ms_counter = systick_500ms_flag = 0;
    while (!systick_500ms_flag); // delay for 500ms
    
    // load data from rtc
    RTCLoadData();
}


void ProcDisplay(void) {
    if (keystate[BUTTON_LEFT].flag) {
        keystate[BUTTON_LEFT].flag = 0;
        if (flow_speed < 2) {
            ++flow_speed; // turn left or change speed
        }
    }
    
    if (keystate[BUTTON_RIGHT].flag) {
        keystate[BUTTON_RIGHT].flag = 0;
        if (flow_speed > -2) {
            --flow_speed; // turn right or change speed
        }
    }
    
    if (keystate[BUTTON_BACK].flag) {
        keystate[BUTTON_BACK].flag = 0;
        if (alarming) {
            alarming = 0;
            BuzzerStop();
        }
    }
    
    if (keystate[BUTTON_1].flag) {
        mode = MODE_SETDATE;
        focus_digit = 0;
        ClearKeyFlags();
        
        // Save date to display_digit for modification
        setting_digit[0] = datetime.year / 1000 % 10;
        setting_digit[1] = datetime.year / 100 % 10;
        setting_digit[2] = datetime.year / 10 % 10;
        setting_digit[3] = datetime.year % 10;
        setting_digit[4] = datetime.month / 10 % 10;
        setting_digit[5] = datetime.month % 10;
        setting_digit[6] = datetime.day / 10;
        setting_digit[7] = datetime.day % 10;
        return;
    }
    
    if (keystate[BUTTON_2].flag || keystate[BUTTON_3].flag) {
        uint32_t time = keystate[BUTTON_2].flag ? datetime.time : alarm_time;
        uint8_t hour = time / 3600;
        uint8_t min = time / 60 % 60;
        uint8_t sec = time % 60;
        
        mode = keystate[BUTTON_2].flag ? MODE_SETTIME : MODE_SETALARM;
        focus_digit = 0;
        ClearKeyFlags();
        
        // Save date to display_digit for modification
        setting_digit[0] = hour / 10 % 10;
        setting_digit[1] = hour % 10;
        setting_digit[2] = min / 10 % 10;
        setting_digit[3] = min % 10;
        setting_digit[4] = sec / 10 % 10;
        setting_digit[5] = sec % 10;
        return;
    }
    
    if (keystate[BUTTON_DOWN].flag) {
        keystate[BUTTON_DOWN].flag = 0;
        //HibernateWakeSet(HIBERNATE_WAKE_PIN);
        //HibernateRequest();
        SysCtlReset();
        return;
    }
    
    DisplayDatetime(flow_offset);
}

void ProcSetDate(void) {
    uint8_t i = 0;
    uint16_t year;
    uint8_t day, month, day_of_month;
    
    if (keystate[BUTTON_LEFT].flag) {
        keystate[BUTTON_LEFT].flag = 0;
        focus_digit = (focus_digit - 1 + 8) % 8; // focus left digit
    }
    
    if (keystate[BUTTON_RIGHT].flag) {
        keystate[BUTTON_RIGHT].flag = 0;
        focus_digit = (focus_digit + 1) % 8; // focus right digit
    }
    
    if (keystate[BUTTON_UP].flag) {
        keystate[BUTTON_UP].flag = 0;
        ++setting_digit[focus_digit];
    }
    
    if (keystate[BUTTON_DOWN].flag) {
        keystate[BUTTON_DOWN].flag = 0;
        --setting_digit[focus_digit];
    }
    
    // validate
    if (focus_digit == 4) { // first digit of month
        setting_digit[focus_digit] = (setting_digit[focus_digit] + 2) % 2; // only 0 or 1
        if (setting_digit[focus_digit] == 1) { // second digit should be 0, 1 or 2
            if (setting_digit[focus_digit + 1] > 2) {
                setting_digit[focus_digit + 1] = 2;
            }
        } else { // second digit should be 1-9
            if (setting_digit[focus_digit + 1] == 0) {
                setting_digit[focus_digit + 1] = 1;
            }
        }
    } else if (focus_digit == 5) {
        if (setting_digit[focus_digit - 1] == 0) {
            setting_digit[focus_digit] = (setting_digit[focus_digit] - 1 + 9) % 9 + 1; // should be 1-9
        } else {
            setting_digit[focus_digit] += 3; // prevent minus value
            setting_digit[focus_digit] %= 3; // only 0, 1 or 2
        }
    } else if (focus_digit == 6) { // first digit of day
        year = setting_digit[0] * 1000 + setting_digit[1] * 100 + setting_digit[2] * 10 + setting_digit[3];
        month = setting_digit[4] * 10 + setting_digit[5];
        day_of_month = GetDayOfMonth(year, month);
    
        setting_digit[focus_digit] += (day_of_month / 10 + 1); // prevent minus value
        setting_digit[focus_digit] %= (day_of_month / 10 + 1);
        if (setting_digit[focus_digit] == day_of_month / 10) {
            if (setting_digit[focus_digit + 1] > day_of_month % 10) {
                setting_digit[focus_digit + 1] = day_of_month % 10;
            }
        } else if (setting_digit[focus_digit] == 0) {
            if (setting_digit[focus_digit + 1] == 0) {
                setting_digit[focus_digit + 1] = 1;
            }
        }
    } else if (focus_digit == 7) {
        year = setting_digit[0] * 1000 + setting_digit[1] * 100 + setting_digit[2] * 10 + setting_digit[3];
        month = setting_digit[4] * 10 + setting_digit[5];
        day_of_month = GetDayOfMonth(year, month);
        
        if (setting_digit[focus_digit - 1] == 0) {
            setting_digit[focus_digit] = (setting_digit[focus_digit] - 1) % 9 + 1; // should be 1-9
        } else if (setting_digit[focus_digit - 1] == day_of_month / 10) {
            setting_digit[focus_digit] += day_of_month % 10 + 1; // prevent minus value
            setting_digit[focus_digit] %= day_of_month % 10 + 1;
        }
    } else { // digit of year
        setting_digit[focus_digit] = (setting_digit[focus_digit] + 10) % 10;
    }
    
    // update day range
    year = setting_digit[0] * 1000 + setting_digit[1] * 100 + setting_digit[2] * 10 + setting_digit[3];
    month = setting_digit[4] * 10 + setting_digit[5];
    day = setting_digit[6] * 10 + setting_digit[7];
    day_of_month = GetDayOfMonth(year, month);
    if (day > day_of_month) { // validate day
        setting_digit[6] = day_of_month / 10;
        setting_digit[7] = day_of_month % 10;
    }
    
    if (keystate[BUTTON_BACK].flag) {
        // save date
        datetime.year = setting_digit[0] * 1000 + setting_digit[1] * 100 + setting_digit[2] * 10 + setting_digit[3];
        datetime.month = setting_digit[4] * 10 + setting_digit[5];
        datetime.day = setting_digit[6] * 10 + setting_digit[7];
        mode = MODE_DISPLAY;
        ClearKeyFlags();
    }
    
    if (keystate[BUTTON_DISCARD].flag) {
        mode = MODE_DISPLAY;
        ClearKeyFlags();
    }
    
    for (i = 0; i < 8; ++i) {
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
        if (i == focus_digit && focus_flash) {
            // hide focus digit if focus_flash is set, controlled by timer per 250ms
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
        } else if (i == 3 || i == 5) { // show dot
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[setting_digit[i]] | 0x80);
        } else {
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[setting_digit[i]]);
        }
        Delay(TCA6424_DELAY);
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
    }
}

void ProcSetTime(void) {
    uint8_t i = 0;
    
    if (keystate[BUTTON_LEFT].flag) {
        keystate[BUTTON_LEFT].flag = 0;
        focus_digit = (focus_digit - 1 + 6) % 6; // focus left digit
    }
    
    if (keystate[BUTTON_RIGHT].flag) {
        keystate[BUTTON_RIGHT].flag = 0;
        focus_digit = (focus_digit + 1) % 6; // focus right digit
    }
    
    if (keystate[BUTTON_UP].flag) {
        keystate[BUTTON_UP].flag = 0;
        ++setting_digit[focus_digit];
    }
    
    if (keystate[BUTTON_DOWN].flag) {
        keystate[BUTTON_DOWN].flag = 0;
        --setting_digit[focus_digit];
    }
    
    // validate
    if (focus_digit == 0) { // first digit of hour
        setting_digit[focus_digit] = (setting_digit[focus_digit] + 3) % 3; // only 0, 1 or 2
        if (setting_digit[focus_digit] == 2) { // second digit should be 0, 1 or 2
            if (setting_digit[focus_digit + 1] > 3) {
                setting_digit[focus_digit + 1] = 3;
            }
        }
    } else if (focus_digit == 1) {
        if (setting_digit[focus_digit - 1] == 2) { // should be 0-3
            setting_digit[focus_digit] = (setting_digit[focus_digit] + 4) % 4;
        } else {
            setting_digit[focus_digit] = (setting_digit[focus_digit] + 10) % 10;
        }
    } else if (focus_digit == 2 || focus_digit == 4) { // first digit of min or sec
        setting_digit[focus_digit] = (setting_digit[focus_digit] + 6) % 6; // only 0-5
    } else {
        setting_digit[focus_digit] = (setting_digit[focus_digit] + 10) % 10;
    }
    
    if (keystate[BUTTON_BACK].flag) {
        // save time
        uint8_t hour = setting_digit[0] * 10 + setting_digit[1];
        uint8_t min = setting_digit[2] * 10 + setting_digit[3];
        uint8_t sec = setting_digit[4] * 10 + setting_digit[5];
        if (mode == MODE_SETTIME) {
            datetime.time = hour * 3600 + min * 60 + sec;
        } else { // MODE_ALARM
            alarm_time = hour * 3600 + min * 60 + sec;
        }
        mode = MODE_DISPLAY;
        ClearKeyFlags();
    }
    
    if (keystate[BUTTON_DISCARD].flag) {
        mode = MODE_DISPLAY;
        ClearKeyFlags();
    }
    
    for (i = 0; i < 8; ++i) {
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
        if ((i - 1 == focus_digit && focus_flash) || i == 0 || i == 7) {
            // hide focus digit if focus_flash is set, controlled by timer per 250ms
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
        } else if (i == 2 || i == 4) { // show dot
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[setting_digit[i - 1]] | 0x80);
        } else {
            I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[setting_digit[i - 1]]);
        }
        Delay(TCA6424_DELAY);
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
    }
}

void DisplayDatetime(uint8_t offset) {
    uint8_t data[16];
    uint8_t i;
    
    uint8_t hour = datetime.time / 3600;
    uint8_t min = datetime.time / 60 % 60;
    uint8_t sec = datetime.time % 60;
    
    data[0] = seg7[datetime.year / 1000 % 10];
    data[1] = seg7[datetime.year / 100 % 10];
    data[2] = seg7[datetime.year / 10 % 10];
    data[3] = seg7[datetime.year % 10] | 0x80; // show dot
    data[4] = seg7[datetime.month / 10 % 10];
    data[5] = seg7[datetime.month % 10] | 0x80;
    data[6] = seg7[datetime.day / 10];
    data[7] = seg7[datetime.day % 10];
    data[8] = 0x00;
    data[9] = seg7[hour / 10 % 10];
    data[10] = seg7[hour % 10] | 0x80;
    data[11] = seg7[min / 10 % 10];
    data[12] = seg7[min % 10] | 0x80;
    data[13] = seg7[sec / 10 % 10];
    data[14] = seg7[sec % 10];
    data[15] = 0x00;
    
    for (i = 0; i < 8; ++i) {
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01 << i);
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, data[(i + offset) % 16]);
        Delay(TCA6424_DELAY);
        I2C0WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // prevent ghost digit
    }
}

void DetectKey(void) {
    uint8_t i = 0;
    static uint8_t key_press = 0; // bitmask for key press state
    
    key_press = ~I2C0ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
    // I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~key_press); // show key state
    for (i = 0; i < 8; ++i) {
        uint8_t is_press = key_press & (0x01 << i);
        keystate[i].state = (keystate[i].state << 1) | (is_press ? 0x01 : 0x00);
        
        if (is_press) {
            if (keystate[i].config & KEY_CONFIG_PRESS) {
                // long press key
                    if ((keystate[i].state & 0x02) == 0) { // check previous state
                        keystate[i].flag = 1;
                        keystate[i].timer = KEY_DELAY * 2; // 2 * interval at first time
                    } else {
                        if (--keystate[i].timer == 0) {
                            keystate[i].flag = 1;
                            keystate[i].timer = KEY_DELAY;
                        }
                    }
            } else {
                // oneshot key
                if ((keystate[i].state & 0x02) == 0) { // check previous state
                    keystate[i].flag = 1;
                }
            }
        }
    }
}

void ClearKeyFlags(void) {
    uint8_t i = 0;
    
    for (i = 0; i < 8; ++i) {
        keystate[i].flag = 0;
    }
}

void ProcessCommand(void) {
    datetime_t args[1];
    char buffer[128];
    error_t error;
    uint8_t partical_error = 0;
    
    // HELP
    error = ParseCommand("?", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        UART0StringPutNonBlocking(help_message);
        return;
    }
    
    // MUTE
    error = ParseCommand("MUTE", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        alarming = 0;
        return;
    }
    
    // INIT
    error = ParseCommand("CLOCK INIT", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        datetime.year = 2000;
        datetime.month = 1;
        datetime.day = 1;
        datetime.time = 0;
        alarm_time = 999;
        RTCStoreData(); // store default data
        SysCtlReset(); // restart
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    error = ParseCommand("CLOCK RESTART", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        ROMStoreData(); // store data before restart
        SysCtlReset();
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    error = ParseCommand("CLOCK HIB", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        HibernateWakeSet(HIBERNATE_WAKE_PIN);
        HibernateRequest();
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    if (partical_error) {
        uint8_t i, j, last_space = 18 + 3;
        
        strcpy(buffer, "Invalid Argument: ");
        strcat(buffer, (const char *)command);
        strcat(buffer, "\r\n");
        for (j = strlen(buffer), i = 0; i < partical_error + 18; ++i) {
            if (i > 17 && command[i - 18] == ' ') {
                last_space = i + 1; // record last space
            }
            buffer[i + j] = ' ';
        }
        buffer[i + j] = '^';
        for (i = last_space; i < j - 2 || i <= last_space; ++i) {
            if (buffer[i + j] != '^') {
                buffer[i + j] = '~';
            }
        }
        buffer[i + j] = '\0';
        strcat(buffer, "\r\nUsage: CLOCK INIT|RESTART|HIB\r\n");
        UART0StringPutNonBlocking(buffer);
        return;
    }
    
    // GET
    error = ParseCommand("GET DATE", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        StringifyDate(datetime.year, datetime.month, datetime.day, buffer);
        UART0StringPutNonBlocking(buffer);
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    error = ParseCommand("GET TIME", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        StringifyTime(datetime.time, buffer);
        UART0StringPutNonBlocking(buffer);
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    error = ParseCommand("GET ALARM", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        StringifyTime(alarm_time, buffer);
        UART0StringPutNonBlocking(buffer);
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    }
    
    if (partical_error) {
        uint8_t i, j, last_space = 18 + 3;
        
        strcpy(buffer, "Invalid Argument: ");
        strcat(buffer, (const char *)command);
        strcat(buffer, "\r\n");
        for (j = strlen(buffer), i = 0; i < partical_error + 18; ++i) {
            if (i > 17 && command[i - 18] == ' ') {
                last_space = i + 1; // record last space
            }
            buffer[i + j] = ' ';
        }
        buffer[i + j] = '^';
        for (i = last_space; i < j - 2 || i <= last_space; ++i) {
            if (buffer[i + j] != '^') {
                buffer[i + j] = '~';
            }
        }
        buffer[i + j] = '\0';
        strcat(buffer, "\r\nUsage: GET DATE|TIME|ALARM\r\n");
        UART0StringPutNonBlocking(buffer);
        return;
    }
    
    // SET
    error = ParseCommand("SET DATE $D", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        datetime.year = args[0].year;
        datetime.month = args[0].month;
        datetime.day = args[0].day;
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    } else if (error & ERROR_FORMAT) {
        return; // This is solved in ParseCommand
    }
    
    error = ParseCommand("SET TIME $T", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        datetime.time = args[0].time;
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    } else if (error & ERROR_FORMAT) {
        return; // This is solved in ParseCommand
    }
    
    error = ParseCommand("SET ALARM $T", (char *)command, args);
    if (error == ERROR_SUCCESS) {
        alarm_time = args[0].time;
        return;
    } else if (error & ERROR_PARTIAL) {
        partical_error = MAX(partical_error, error & 0x00ff);
    } else if (error & ERROR_FORMAT) {
        return; // This is solved in ParseCommand
    }
    
    if (partical_error) {
        uint8_t i, j, last_space = 18 + 3;
        
        strcpy(buffer, "Invalid Argument: ");
        strcat(buffer, (const char *)command);
        strcat(buffer, "\r\n");
        for (j = strlen(buffer), i = 0; i < partical_error + 18; ++i) {
            if (i > 17 && command[i - 18] == ' ') {
                last_space = i + 1; // record last space
            }
            buffer[i + j] = ' ';
        }
        buffer[i + j] = '^';
        for (i = last_space; i < j - 2 || i <= last_space; ++i) {
            if (buffer[i + j] != '^') {
                buffer[i + j] = '~';
            }
        }
        buffer[i + j] = '\0';
        strcat(buffer, "\r\nUsage: SET DATE <YYYY/MM/DD> Or SET ALARM|TIME <HH:MM:SS>\r\n");
        UART0StringPutNonBlocking(buffer);
        return;
    }
    
    // no match
    UART0StringPutNonBlocking("Invalid Command: ");
    UART0StringPutNonBlocking((const char *)command);
    UART0StringPutNonBlocking("\r\n");
    UART0StringPutNonBlocking(help_message);
}

error_t ParseCommand(const char *pattern, const char *command, datetime_t *args) {
    uint8_t i = 0, j = 0;
    uint8_t skip_space = 0, has_space = 0;
    uint8_t current_arg = 0;
    
    for (i = 0, j = 0; command[i] && pattern[j]; ++i) {
        char comp_1 = ToUpperCase(pattern[j]);
        char comp_2 = ToUpperCase(command[i]);
        
        if (comp_2 == ' ') { // skip multiple spaces as one
            if (skip_space) continue;
            else skip_space = has_space = 1;
        } else {
            skip_space = 0;
        }
        
        if (comp_1 == '$') { // pattern
            char pattern_type = ToUpperCase(pattern[j + 1]);
            ++j;
            
            args[current_arg].year = args[current_arg].month = args[current_arg].day = 0;
            args[current_arg].time = 0;
            
            if (pattern_type == 'T') {
                int temp;
                error_t error;
                
                // Parse a time value like HH:MM:SS
                error = ParseIntegerUntil(command, ':', &i, &temp);
                if (error == ERROR_SUCCESS) {
                    if (temp >= 0 && temp <= 23) {
                        args[current_arg].time += temp * 60 * 60;
                    } else {
                        UART0StringPutNonBlocking("Invalid Hour: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 00 and 23\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nTime should be HH:MM:SS\r\n");
                    return ERROR_FORMAT;
                }
                
                error = ParseIntegerUntil(command, ':', &i, &temp);
                if (error == ERROR_SUCCESS) {
                    if (temp >= 0 && temp <= 59) {
                        args[current_arg].time += temp * 60;
                    } else {
                        UART0StringPutNonBlocking("Invalid Minute: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 00 and 59\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nTime should be HH:MM:SS\r\n");
                    return ERROR_FORMAT;
                }
                
                error = ParseIntegerUntil(command, pattern[j + 1], &i, &temp);
                if (error == ERROR_SUCCESS) {
                    if (temp >= 0 && temp <= 59) {
                        args[current_arg].time += temp;
                    } else {
                        UART0StringPutNonBlocking("Invalid Second: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 00 and 59\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nTime should be HH:MM:SS\r\n");
                    return ERROR_FORMAT;
                }
            } else if (pattern_type == 'D') {
                // Parse a date value like YYYY/MM/DD
                int temp;
                error_t error;
                
                error = ParseIntegerUntil(command, '/', &i, &temp);
                if (error == ERROR_SUCCESS) {
                    if (temp >= 0 && temp <= 9999) {
                        args[current_arg].year = temp;
                    } else {
                        UART0StringPutNonBlocking("Invalid Year: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 0000 and 9999\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nDate should be YYYY/MM/DD\r\n");
                    return ERROR_FORMAT;
                }
                
                error = ParseIntegerUntil(command, '/', &i, &temp);
                if (error == ERROR_SUCCESS) {
                    if (temp >= 0 && temp <= 12) {
                        args[current_arg].month = temp;
                    } else {
                        UART0StringPutNonBlocking("Invalid Month: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 01 and 12\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nDate should be YYYY/MM/DD\r\n");
                    return ERROR_FORMAT;
                }
                
                error = ParseIntegerUntil(command, pattern[j + 1], &i, &temp);
                if (error == ERROR_SUCCESS) {
                    uint8_t day_of_month = GetDayOfMonth(args[current_arg].year, args[current_arg].month);
                    if (temp >= 0 && temp <= day_of_month) {
                        args[current_arg].day = temp;
                    } else {
                        UART0StringPutNonBlocking("Invalid Day: ");
                        UART0NumberPutNonBlocking(temp);
                        UART0StringPutNonBlocking("\r\nShould between 00 and ");
                        UART0NumberPutNonBlocking(day_of_month);
                        UART0StringPutNonBlocking("\r\n");
                        return ERROR_FORMAT;
                    }
                } else {
                    UART0StringPutNonBlocking("Invalid Format: ");
                    UART0StringPutNonBlocking(command);
                    UART0StringPutNonBlocking("\r\nDate should be YYYY/MM/DD\r\n");
                    return ERROR_FORMAT;
                }
            }
            
            ++current_arg; // move to next argument
        } else { // directly compare
            if (comp_1 != comp_2) {
                return (has_space ? ERROR_PARTIAL : ERROR_NOT_MATCH) | (i & 0x00ff); // save index to error
            }
        }
        
        ++j;
    }
    
    if (command[i] || pattern[j]) { // pattern or command doesn't end
        return ((has_space || pattern[j] == ' ') ? ERROR_PARTIAL : ERROR_NOT_MATCH) | (i & 0x00ff);
    }
    
    return ERROR_SUCCESS;
}

error_t ParseIntegerUntil(const char *str, char delim, uint8_t *index, int *result) {
    *result = 0;
    
    while (str[*index] && str[*index] != delim) {
        if (str[*index] >= '0' && str[*index] <= '9') {
            *result = *result * 10 + str[*index] - '0';
            ++*index;
        } else {
            return ERROR_NOT_DIGIT;
        }
    }
    
    // string ends with delim
    if (str[*index] == delim) {
        ++*index; // skip delim
        return ERROR_SUCCESS;
    }
    
    return ERROR_NO_DELIM;
}

void StringifyDate(uint16_t year, uint8_t month, uint8_t day, char *buffer) {
    buffer[0] = year / 1000 % 10 + '0';
    buffer[1] = year / 100 % 10 + '0';
    buffer[2] = year / 10 % 10 + '0';
    buffer[3] = year % 10 + '0';
    buffer[4] = '/';
    buffer[5] = month / 10 % 10 + '0';
    buffer[6] = month % 10 + '0';
    buffer[7] = '/';
    buffer[8] = month / 10 % 10 + '0';
    buffer[9] = month % 10 + '0';
    buffer[10] = '\r';
    buffer[11] = '\n';
    buffer[12] = '\0';
}

void StringifyTime(uint32_t time, char *buffer) {
    uint8_t hour = time / 3600, min = time / 60 % 60, sec = time % 60;
    
    buffer[0] = hour / 10 % 10 + '0';
    buffer[1] = hour % 10 + '0';
    buffer[2] = ':';
    buffer[3] = min / 10 % 10 + '0';
    buffer[4] = min % 10 + '0';
    buffer[5] = ':';
    buffer[6] = sec / 10 % 10 + '0';
    buffer[7] = sec % 10 + '0';
    buffer[8] = '\r';
    buffer[9] = '\n';
    buffer[10] = '\0';
}

char ToUpperCase(char x) {
    if (x >= 'a' && x <= 'z') {
        return x - 'a' + 'A';
    }
    return x;
}


uint8_t GetDayOfMonth(uint16_t year, uint8_t month) {
    static const uint8_t month_day[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t is_leap;
    
    if (year % 100 == 0) {
        is_leap = (year % 400 == 0);
    } else {
        is_leap = (year % 4 == 0);
    }
    
    if (month == 2 && is_leap) {
        return 29; // day in February, leap year
    } else {
        return month_day[month - 1]; // month is 1-base
    }
}

void Delay(uint32_t loop) {
	uint32_t i;
	for (i = 0; i < loop; i++);
}

void ClearSystickCounter(void) {
    systick_20ms_counter = systick_20ms_flag = 0;
    systick_250ms_counter = systick_250ms_flag = 0;
    systick_500ms_counter = systick_500ms_flag = 0;
    
    // do not clear 1s counter, treat it as ms counter
    // systick_1s_counter = systick_1s_flag = 0;
}

void GPIOInit(void) {
    // Input: PJ0, PJ1
    // Output: PF0, PN0, PN1
    
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

void UART0Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    
    // PA0 -> UART0_RX, PA1 -> UART0_TX
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // 115200 baud, 8-N-1 format
    UARTConfigSetExpClk(UART0_BASE, sys_clock_freq, 115200,
        UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);
    
    // Enable UART0 interrupter
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    
    DEBUG("UART0 Setup\r\n");
}

void UART0StringPutNonBlocking(const char *message) {
	uint8_t fifo_free = 0;
    while(*message != '\0') {
        fifo_free = UARTCharPutNonBlocking(UART0_BASE, *(message));
        if(fifo_free) {  
            message++;
        }
        fifo_free = 0;
    }
}

void UART0NumberPutNonBlocking(int64_t data) {
    static uint8_t buffer[20];
    uint8_t flag = 0;
    uint8_t cur = 19;
    
    if (data == 0) {
        UART0StringPutNonBlocking("0");
        return;
    }
    
    if (data < 0) {
        flag = 1;
        data = -data;
    }
    
    buffer[19] = '\0';
    while (cur > 0 && data) {
        --cur;
        buffer[cur] = data % 10 + '0';
        data /= 10;
    }
    
    if (flag) {
        buffer[--cur] = '-';
    }
    
    UART0StringPutNonBlocking((const char *)buffer + cur);
}

void I2C0Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    
    // PB2 -> I2C0_SCL, PB3 -> I2C0_SDA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    
    I2CMasterInitExpClk(I2C0_BASE, sys_clock_freq, true);
	I2CMasterEnable(I2C0_BASE);
    
    // TCA6424 config
    I2C0WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0xff); // port0: input
    I2C0WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x00); // port1: output
    I2C0WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x00); // port2: output
    
    // PCA9557 config
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00); // port: output
    I2C0WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff); // turn off led1-8
}

uint8_t I2C0WriteByte(uint8_t device, uint8_t reg, uint8_t data) {
	uint8_t error;
    
    while (I2CMasterBusy(I2C0_BASE));
	I2CMasterSlaveAddrSet(I2C0_BASE, device, false);
	I2CMasterDataPut(I2C0_BASE, reg);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
	while (I2CMasterBusy(I2C0_BASE));
	error = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, data);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE));
	error = (uint8_t)I2CMasterErr(I2C0_BASE);
    
	return error;
}

uint8_t I2C0ReadByte(uint8_t device, uint8_t reg) {
	uint8_t data, error;
    
    while (I2CMasterBusy(I2C0_BASE));
	I2CMasterSlaveAddrSet(I2C0_BASE, device, false);
	I2CMasterDataPut(I2C0_BASE, reg);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    
	while (I2CMasterBusBusy(I2C0_BASE));
	error = (uint8_t)I2CMasterErr(I2C0_BASE);
    Delay(10);

	// Receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, device, true);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    
	while (I2CMasterBusBusy(I2C0_BASE));
	data = I2CMasterDataGet(I2C0_BASE);
    Delay(10);
	
    return data;
}

void BuzzerInit(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
}

void BuzzerStart(uint32_t freq) {
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, sys_clock_freq / freq);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 2); // 0.5
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void BuzzerStop(void) {
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
}

void RTCInit(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_HIBERNATE));
    
    HibernateEnableExpClk(sys_clock_freq);
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
    HibernateCounterMode(HIBERNATE_COUNTER_24HR);
}

void RTCStoreData(void) {
    struct tm ps_time;
    
    ps_time.tm_year = datetime.year - 1900;
    ps_time.tm_mon = datetime.month - 1;
    ps_time.tm_mday = datetime.day;
    ps_time.tm_hour = datetime.time / 3600;
    ps_time.tm_min = datetime.time / 60 % 60;
    ps_time.tm_sec = datetime.time % 60;
    
    HibernateCalendarSet(&ps_time);
    // RTC has a 1/32768s counter
    HibernateRTCSSMatchSet(0, (uint32_t)systick_1s_counter * 32768 / 1000);
    
    ROMStoreData();
}

void RTCLoadData(void) {
    struct tm ps_time;
    
    if (load_rom) {
        ROMLoadData();
        load_rom = 0;
        return;
    }
    
    HibernateCalendarGet(&ps_time);
    
    systick_1s_counter = HibernateRTCSSGet() * 1000 / 32768;
    
    datetime.year = ps_time.tm_year + 1900;
    datetime.month = ps_time.tm_mon + 1;
    datetime.day = ps_time.tm_mday;
    datetime.time = ps_time.tm_hour * 3600 + ps_time.tm_min * 60 + ps_time.tm_sec;
}

void ROMInit(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));
    
    EEPROMInit();
}

void ROMStoreData(void) {
    uint32_t data[4];
    
    data[0] = ROM_MAGIC;
    data[1] = ((uint32_t)(datetime.year) << 16) | ((uint32_t)(datetime.month) << 8) | datetime.day;
    data[2] = datetime.time;
    data[3] = alarm_time;
}

void ROMLoadData(void) {
    uint32_t data[4];
    EEPROMRead(data, ROM_ADDRESS, sizeof(data));
    
    if (data[0] != ROM_MAGIC) {
        return; // no data stored
    }
    
    datetime.year = data[1] >> 16;
    datetime.month = (data[1] >> 8) & 0xff;
    datetime.day = data[1] & 0xff;
    datetime.time = data[2];
    alarm_time = data[3];
}

void SysTick_Handler(void) {
    if (++systick_20ms_counter >= SYSTICK_FREQUENCY / 50) {
        systick_20ms_counter = 0;
        systick_20ms_flag = 1;
    }
    
    if (++systick_250ms_counter >= SYSTICK_FREQUENCY / 4) {
        systick_250ms_counter = 0;
        systick_250ms_flag = 1;
    }
    
    if (++systick_500ms_counter >= SYSTICK_FREQUENCY / 2) {
        systick_500ms_counter = 0;
        systick_500ms_flag = 1;
    }
    
    if (++systick_1s_counter >= SYSTICK_FREQUENCY) {
        systick_1s_counter = 0;
        systick_1s_flag = 1;
    }
}

void UART0_Handler(void) {
    int32_t uart0_int_status;
    static uint8_t uart_receive_cmd_cur = 0; // pointer to index of next char
    
    // Get and clear the interrrupt status.
    uart0_int_status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, uart0_int_status);

    // Loop while there are characters in the receive FIFO.
    while (UARTCharsAvail(UART0_BASE)) {
        command[uart_receive_cmd_cur] = UARTCharGetNonBlocking(UART0_BASE);
        if (command[uart_receive_cmd_cur] == '\n') {
            // A command should end with \r\n
            if (uart_receive_cmd_cur > 0 && command[uart_receive_cmd_cur - 1] == '\r') {
                command[uart_receive_cmd_cur - 1] = '\0'; // directly replace \r with \0
                command[uart_receive_cmd_cur] = '\0';
                uart_receive_cmd_cur = 0;
                command_ready = 1;
            }
        } else {
            uart_receive_cmd_cur++;
            if (uart_receive_cmd_cur > 128) {
                // TODO: command to long
            }
        }
    }
}
