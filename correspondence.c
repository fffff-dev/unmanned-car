#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"

// ================= 宏定义与全局变量 =================
#define EDGE_LEFT_PIN   WIFI_IOT_IO_NAME_GPIO_13  
#define EDGE_RIGHT_PIN  WIFI_IOT_IO_NAME_GPIO_14  

// 定义电平逻辑：根据你的传感器调整，通常悬空或检测到边缘是高电平(1)
#define LINE_LOST 1   
#define LINE_DETECTED 0   
#define SAFE_DISTANCE_CM 15.0f
#define DIST_DROP_THRESHOLD 5.0f
uint8_t uart_sendbuf[20];

#define GPIO_8 8
#define GPIO_7 7
#define GPIO_FUNC 0
#define GPIO_SERVO 2

// ================= 电机控制函数 =================
void stm32motor_control(int motorA, int motorB) {
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    if(motorA < 0) { A_dir = 1; motorA = -motorA; }
    if(motorB < 0) { B_dir = 1; motorB = -motorB; }

    if(motorA > 150) motorA = 150;
    if(motorB > 150) motorB = 150;

    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = motorB;
    uart_sendbuf[5] = 0xFD;
    
    // 发送指令
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

void car_forward(void) { 
    stm32motor_control(100, 100); 
}

static int fork_detect_count = 0;
static int stop = 0;

void line_follow_control(int left_line, int right_line) { 
    if(left_line == LINE_LOST && right_line == LINE_LOST) {
         // 两侧同时检测到黑线 → 到达分叉点
        fork_detect_count++;
        printf("Fork Detected! Count = %d\n", fork_detect_count);

        if(stop > 0) {
            stm32motor_control(0, 0);
            return;
        }

        if (fork_detect_count > 0) {
            // 第一次同时检测到：强制左转
            printf(">>> First fork: Force LEFT\n");
            stm32motor_control(-30, 60);
            usleep(100000);
        } 
        else if (fork_detect_count < 0) {
            // 第二次同时检测到：强制右转
            stm32motor_control(60, -30);
            usleep(50000);
        } 
    }
    else if (left_line == LINE_DETECTED && right_line == LINE_LOST) {
        // 左侧检测到黑线 → 小车偏左了，需要右转修正
        // 左轮快，右轮慢
        if(fork_detect_count > 0)fork_detect_count = -9999999;
        if(fork_detect_count > -9999999 && fork_detect_count < 0) {
            stop++;
        }
        stm32motor_control(60, -30);
    } 
    else if (left_line == LINE_LOST && right_line == LINE_DETECTED) {
        // 右侧检测到黑线 → 小车偏右了，需要左转修正
        // 左轮慢，右轮快
        if(fork_detect_count > 0)fork_detect_count = -9999999;
        if(fork_detect_count > -9999999 && fork_detect_count < 0) {
            stop++;
        }
        stm32motor_control(-30, 60);
    } 
    else {
        if(fork_detect_count > -9999999 && fork_detect_count < 0) {
            stop++;
        }
        car_forward();
    }
}

// ================= 主任务线程 =================
static void car_task(void) {
    printf("Car Task Running...\n");
    
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);

    while (1) {
        // 读取左右巡线传感器
        WifiIotGpioValue val_left = WIFI_IOT_GPIO_VALUE0;
        WifiIotGpioValue val_right = WIFI_IOT_GPIO_VALUE0;
        
        GpioGetInputVal(EDGE_LEFT_PIN, &val_left);
        GpioGetInputVal(EDGE_RIGHT_PIN, &val_right);

        // 巡线逻辑
        printf("Line: L=%d R=%d\n", val_left, val_right);

        line_follow_control(val_left, val_right);

        usleep(30000);
    }
}

// ================= 系统入口 (关键修改区) =================
static void correspondence(void) {
    // GPIO 初始化 (传感器)
    GpioInit();
    IoSetFunc(EDGE_LEFT_PIN, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    GpioSetDir(EDGE_LEFT_PIN, WIFI_IOT_GPIO_DIR_IN);
    IoSetFunc(EDGE_RIGHT_PIN, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(EDGE_RIGHT_PIN, WIFI_IOT_GPIO_DIR_IN);

    // 将 GPIO11/12 切换为 UART2 功能
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);

    // UART2 初始化
    WifiIotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr, NULL);

    // 创建线程
    osThreadAttr_t attr;
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4; 
    attr.priority = 25;
    attr.name = "CarControlThread";

    if ((osThreadNew((osThreadFunc_t)car_task, NULL, &attr)) == NULL) {
        printf("Failed to create Car Task!\n");
    }
}

APP_FEATURE_INIT(correspondence);