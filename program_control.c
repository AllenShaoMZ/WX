#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "define.h"
#include "flyprog.h"
#include "program_control.h"
#include "FPGAUart.h"

// 定义全局标志变量
bool PTmode_Finished;   // 子星模式完成标志
bool PTmode_Status;      // 子星模式状态标志
bool GNCmode_Finished;  // GNC模式完成标志
bool GNCmode_Status;     // GNC模式状态标志

// 声明 setSelfTestStatus 函数
bool setSelfTestStatus(bool status);
bool modeFinished = false;  // 初始化为 false  后面全部去掉---


//中间制导开始时间
static int g_middleEstablishStartTime = 0;

// 全局标记
bool initExecuted = false;
bool selfTestCompleted = false;
bool preheatCompleted = false;
unsigned int g_commTimer = 0;
unsigned int g_naviDataTimer = 0;

// 声明全局变量，使用 0 和 1 表示状态
int selfTestCommandReceived = 0;  // 0 表示未接收到自检指令
int preheatCommandReceived = 0;   // 0 表示未接收到预热指令
int faultDetected = 0;            // 0 表示没有故障



// 声明全局变量
Mode mode = STORAGE_MODE;

// 初始化标志函数
void initializeFlags() {
    PTmode_Finished = false;
    PTmode_Status = true;
    GNCmode_Finished = false;
    GNCmode_Status = true;
    printf("标志已初始化：PTmode_Finished = %d, PTmode_Status = %d, GNCmode_Finished = %d, GNCmode_Status = %d\n",
           PTmode_Finished, PTmode_Status, GNCmode_Finished, GNCmode_Status);
}

unsigned int getCurrentTime(void) {
    // 这里的实现可以是获取系统时间、计时器值等，具体实现取决于硬件和应用场景
    return 0;  // 假设当前时间返回 0，仅作为示例
}

// 硬件控制函数
void powerOnCentralControl() {
	  //中控板供电和蓄电池开好像不一样？
	  FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
    printf("中控组件开机\n");
}

void powerOffStarSensor() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x04AA;
    printf("星敏单机关机\n");
}

void powerOffCommunication() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x03AA;
    printf("通信机关机\n");
}

void powerOffInertialNavigation() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x05AA;
    printf("惯导关机\n");
}

void sleepGNC() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x07AA;
    printf("GNC休眠\n");
}


bool checkForFault() {
    // 故障检测
    return faultDetected; // 返回全局故障标志
}

// bit0，分离后装订数据异常
bool checkBindingDataError() {
    // TODO: 分离后装订数据异常
    
    return false; 
}
// bit1，定轨失败
bool checkOrbitDeterminationFailure() {
    // TODO: 定轨失败
    return false; 
}

// bit2，姿控异常
bool checkAttitudeControlAnomaly() {
    // TODO: 姿控异常
    return false; 
}

// bit3，通信断链
// TODO: g_commTimer更新
bool checkCommunicationLinkLoss() {
    // 中制导模式开始，连续10S没有收到遥控帧（包含联合导航数据）
    // 计时器
    // 全局计时器 TODO: 收到通信包时，更新g_commTimer

    // 检查当前模式，如果是中制导模式，开始计时
    if (mode == MIDDLE_GUIDANCE_MODE) {
        if (g_commTimer != 0) {
            unsigned int currentTime = getCurrentTime();
            unsigned int deltaTime = currentTime - g_commTimer;
            if (deltaTime >= 10) {
                return false; // 通信断链
            }
            else
            {
                return true; // 通信正常
            }

        }
    }
    
    return true; 
}

// bit4, 多次遥控帧校验失败
bool checkRemoteControlFrameCheckFail() {
    // TODO: 多次遥控帧校验失败
    return false; 
}

// bit5, 不满足GNC运算最小系统
bool checkGNCMinSystemNotMet() {
    // TODO: GNC运算最小系统
    return false; 
}
// bit6, GNC模式异常
bool checkGNCModeAnomaly() {
    // TODO: GNC模式异常
    return false; 
}
// bit7, GNC相对位置控制超差
bool checkGNCRelativePositionControl() {
    // TODO: GNC相对位置控制超差
    return false; 
}

// bit8, 子星任务模式超时未完成
bool checkSubStarTaskTimeout() {
    unsigned int maxDeltaTime = 600;
    if(mode == MIDDLE_GUIDANCE_ESTABLISHMENT_MODE){
        maxDeltaTime = 600; // 10分钟
    }
    else if(mode == MIDDLE_FINAL_HANDOVER_MODE){
        maxDeltaTime = 300; // 50分钟
    }
    else{
        return true;
    }

    unsigned int currentTime = getCurrentTime();
    if (difftime(currentTime, g_middleEstablishStartTime) > maxDeltaTime) {
        // 保护操作
        return false;
    }

    return true; 
}

// bit9, GNC规定时间内未完成相对导航滤波
bool checkGNCRelativeNavTimeout() {
    // TODO: 未完成相对导航滤波
    return false; 
}

// bit10, 相对导航滤波发散
bool checkRelativeNavDivergence() {
    // TODO: 相对导航滤波发散
    return false; 
}

// bit11, 联合导航数据不可用，连续30S导航数据不可用
bool checkCombinedNavDataUnavailable() {
    // TODO: 收到通信包时，更新g_commTimer
    // 检查当前模式，如果是中制导模式、中末制导模式，开始计时
    if (mode == MIDDLE_GUIDANCE_MODE || mode == MIDDLE_FINAL_HANDOVER_MODE) {
        if (g_naviDataTimer != 0) {
            unsigned int currentTime = getCurrentTime();
            unsigned int deltaTime = currentTime - g_naviDataTimer;
            if (deltaTime >= 30) {
                return false; // 导航数据不可用
            }
            else
            {
                return true; // 导航数据正常
            }

        }
        else{
            return true;
        }
    }
    
    return true; 
}

// bit12, 星敏通信故障
// 连续3S无遥测，且重启一次3S无遥测
// 遥测更新时间，重启计数
unsigned int telemetryUpdateTime = 0;
unsigned int telemetryRestartCount = 0;
// TODO: 星敏通信更新时间
// 检查星敏通信故障
bool checkStarSensorCommunication() {
    // 遥测更新时间，重启计数
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - telemetryUpdateTime;
    if (deltaTime >= 3) {
        telemetryRestartCount++;
        // TODO: 重启星敏通信
    }
    else {
        return true; // 星敏通信正常
    }

    if (telemetryRestartCount > 1) {
        return false; // 星敏通信故障
    }
    else {
        return true; // 星敏通信正常
    }
}

// bit13, 惯组通信故障
unsigned int imuUpdateTime = 0;
unsigned int imuRestartCount = 0;
// TODO: 惯组通信更新时间

bool checkIMUCommunication() {
    // 遥测更新时间，重启计数
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - imuUpdateTime;
    if (deltaTime >= 3) {
        imuRestartCount++;
        // TODO: 重启惯组通信
    }
    else {
        return true; // 惯组通信正常
    }

    if (imuRestartCount > 1) {
        return false; // 惯组通信故障
    }
    else {
        return true; // 惯组通信正常
    }
}

// bit14, GNSS通信故障
unsigned int gnssUpdateTime = 0;
unsigned int gnssRestartCount = 0;
// TODO: GNSS通信更新时间

bool checkGNSSCommunication() {
    // 遥测更新时间，重启计数
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - gnssUpdateTime;
    if (deltaTime >= 3) {
        gnssRestartCount++;
        // TODO: 重启GNSS通信
    } else {
        return true; // GNSS通信正常
    }

    if (gnssRestartCount > 1) {
        return false; // GNSS通信故障
    } else {
        return true; // GNSS通信正常
    }
}

// bit15, 通信机通信故障
unsigned int communicationDeviceUpdateTime = 0;
unsigned int communicationDeviceRestartCount = 0;
// TODO: 通信机通信更新时间
bool checkCommunicationDevice() {
    // 遥测更新时间，重启计数
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - communicationDeviceUpdateTime;
    if (deltaTime >= 3) {
        communicationDeviceRestartCount++;
        // TODO: 重启通信机通信
    } else {
        return true; // 通信机通信正常
    }

    if (communicationDeviceRestartCount > 1) {
        return false; // 通信机通信故障
    } else {
        return true; // 通信机通信正常
    }
}

// bit16, 观瞄处理通信故障
unsigned int observationDeviceUpdateTime = 0;
unsigned int observationDeviceRestartCount = 0;
// TODO: 观瞄处理通信更新时间
bool checkObservationDevice() {
    // 遥测更新时间，重启计数
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - observationDeviceUpdateTime;
    if (deltaTime >= 3) {
        observationDeviceRestartCount++;
        // TODO: 重启观瞄处理通信
    } else {
        return true; // 观瞄处理通信正常
    }

    if (observationDeviceRestartCount > 1) {
        return false; // 观瞄处理通信故障
    } else {
        return true; // 观瞄处理通信正常
    }
}
// bit17, 相机丢失目标
bool checkCameraTargetLoss() {
    // TODO：相机丢失目标
    return false; 
}
// bit18, 激光测距机丢失目标
bool checkLaserRangefinderTargetLoss() {
    // TODO：激光测距机丢失目标
    return false; 
}

// bit19, 载荷开盖失败
unsigned int payloadCoverCount = 0;
// TODO：开盖指令计数++
bool checkPayloadCoverFailure() {
    // 全局载荷开盖计数，判断是否大于3
    if (payloadCoverCount >= 3) {
        return false; // 载荷开盖失败
    } else {
        return true; // 载荷开盖正常
    }

}

// bit20, 作战超时
unsigned int fightCommandCount = 0;
// TODO：fight指令计数++
bool checkCombatTimeout() {

    // fight指令计数，判断是否大于3
    if (fightCommandCount >= 3) {
        return false; // 作战超时
    } else {
        return true; // 作战正常
    } 
}

// 检查故障
void checkFaults(unsigned int *faultFlags, unsigned int faultsMask) {
    if (faultsMask & FAULT_BINDING_DATA_ERROR) {
        if (checkBindingDataError()) {
            *faultFlags |= FAULT_BINDING_DATA_ERROR;
            printf("分离后装订数据异常\n");
        }
    }
    if (faultsMask & FAULT_ORBIT_DETERMINATION_FAILURE) {
        if (checkOrbitDeterminationFailure()) {
            *faultFlags |= FAULT_ORBIT_DETERMINATION_FAILURE;
            printf("定轨失败\n");
        }
    }
    if (faultsMask & FAULT_ATTITUDE_CONTROL_ANOMALY) {
        if (checkAttitudeControlAnomaly()) {
            *faultFlags |= FAULT_ATTITUDE_CONTROL_ANOMALY;
            printf("姿控异常\n");
        }
    }
    if (faultsMask & FAULT_COMMUNICATION_LINK_LOSS) {
        if (checkCommunicationLinkLoss()) {
            *faultFlags |= FAULT_COMMUNICATION_LINK_LOSS;
            printf("通信断链\n");
        }
    }
    if (faultsMask & FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL) {
        if (checkRemoteControlFrameCheckFail()) {
            *faultFlags |= FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL;
            printf("多次遥控帧校验失败\n");
        }
    }
    if (faultsMask & FAULT_GNC_MIN_SYSTEM_NOT_MET) {
        if (checkGNCMinSystemNotMet()) {
            *faultFlags |= FAULT_GNC_MIN_SYSTEM_NOT_MET;
            printf("不满足GNC运算最小系统\n");
        }
    }
    if (faultsMask & FAULT_GNC_MODE_ANOMALY) {
        if (checkGNCModeAnomaly()) {
            *faultFlags |= FAULT_GNC_MODE_ANOMALY;
            printf("GNC模式异常\n");
        }
    }
    if (faultsMask & FAULT_GNC_RELATIVE_POSITION_CONTROL) {
        if (checkGNCRelativePositionControl()) {
            *faultFlags |= FAULT_GNC_RELATIVE_POSITION_CONTROL;
            printf("GNC相对位置控制超差\n");
        }
    }
    if (faultsMask & FAULT_SUB_STAR_TASK_TIMEOUT) {
        if (checkSubStarTaskTimeout()) {
            *faultFlags |= FAULT_SUB_STAR_TASK_TIMEOUT;
            printf("子星任务模式超时未完成\n");
        }
    }
    if (faultsMask & FAULT_GNC_RELATIVE_NAV_TIMEOUT) {
        if (checkGNCRelativeNavTimeout()) {
            *faultFlags |= FAULT_GNC_RELATIVE_NAV_TIMEOUT;
            printf("GNC规定时间内未完成相对导航滤波\n");
        }
    }
    if (faultsMask & FAULT_RELATIVE_NAV_DIVERGENCE) {
        if (checkRelativeNavDivergence()) {
            *faultFlags |= FAULT_RELATIVE_NAV_DIVERGENCE;
            printf("相对导航滤波发散\n");
        }
    }
    if (faultsMask & FAULT_COMBINED_NAV_DATA_UNAVAILABLE) {
        if (checkCombinedNavDataUnavailable()) {
            *faultFlags |= FAULT_COMBINED_NAV_DATA_UNAVAILABLE;
            printf("联合导航数据不可用\n");
        }
    }
    if (faultsMask & FAULT_STAR_SENSOR_COMMUNICATION) {
        if (checkStarSensorCommunication()) {
            *faultFlags |= FAULT_STAR_SENSOR_COMMUNICATION;
            printf("星敏通信故障\n");
        }
    }
    if (faultsMask & FAULT_IMU_COMMUNICATION) {
        if (checkIMUCommunication()) {
            *faultFlags |= FAULT_IMU_COMMUNICATION;
            printf("惯组通信故障\n");
        }
    }
    if (faultsMask & FAULT_GNSS_COMMUNICATION) {
        if (checkGNSSCommunication()) {
            *faultFlags |= FAULT_GNSS_COMMUNICATION;
            printf("GNSS 通信故障\n");
        }
    }
    if (faultsMask & FAULT_COMMUNICATION_DEVICE) {
        if (checkCommunicationDevice()) {
            *faultFlags |= FAULT_COMMUNICATION_DEVICE;
            printf("通信机通信故障\n");
        }
    }
    if (faultsMask & FAULT_OBSERVATION_PROCESSING) {
        if (checkObservationDevice()) {
            *faultFlags |= FAULT_OBSERVATION_PROCESSING;
            printf("观瞄处理通信故障\n");
        }
    }
    if (faultsMask & FAULT_CAMERA_TARGET_LOSS) {
        if (checkCameraTargetLoss()) {
            *faultFlags |= FAULT_CAMERA_TARGET_LOSS;
            printf("相机丢失目标\n");
        }
    }
    if (faultsMask & FAULT_LASER_RANGEFINDER_TARGET_LOSS) {
        if (checkLaserRangefinderTargetLoss()) {
            *faultFlags |= FAULT_LASER_RANGEFINDER_TARGET_LOSS;
            printf("激光测距机丢失目标\n");
        }
    }
    if (faultsMask & FAULT_PAYLOAD_COVER_FAILURE) {
        if (checkPayloadCoverFailure()) {
            *faultFlags |= FAULT_PAYLOAD_COVER_FAILURE;
            printf("载荷开盖失败\n");
        }
    }
    if (faultsMask & FAULT_COMBAT_TIMEOUT) {
        if (checkCombatTimeout()) {
            *faultFlags |= FAULT_COMBAT_TIMEOUT;
            printf("作战超时\n");
        }
    }
}



// 设置自检指令接收状态 --在CX发送自检指令处调用
void setSelfTestCommandReceived(void) {
	
		// 判断是否收到cx发来的自检指令
	
    selfTestCommandReceived = 1;
}

// 设置预热指令接收状态 --在CX发送预热指令处调用
void setPreheatCommandReceived(void) {
    
		// 判断是否收到cx发来的自检指令
		preheatCommandReceived = 1;
}



//存储模式
void storage_mode(){
		
		// 初始化标志
    initializeFlags();
     
		// 只执行一次的功能
		if (!initExecuted) {
				powerOnCentralControl();
				powerOffStarSensor();
				powerOffCommunication();
				powerOffInertialNavigation();
				sleepGNC();
				initExecuted = true;
		}
		
		//GNC 处于休眠模式（不调用） -- 完成标志默认完成，状态标志默认正常
		GNCmode_Finished = true;
		
		// 存储热控 
		temperature_control();
		
		// 故障监控--不需要故障检查，默认子星状态正常
		PTmode_Status = true;      // 子星模式状态标志
		// 自检指令监控
		if (selfTestCommandReceived == 1) {
				printf("自检指令完成\n");
				mode = SELF_TEST_MODE; // 切换模式为自检类型
				PTmode_Finished = true;   // 子星模式完成标志
				return;
		}

		// 预热指令监控  --
		if (preheatCommandReceived == 1) {
				printf("预热指令完成\n");
				// 设置完成标志
				preheatCompleted = true;
				mode = PREHEAT_MODE; // 切换模式为预热类型
				PTmode_Finished = true;   // 子星模式完成标志
				return;
		}                   
}

bool setSelfTestStatus(bool status) {
    // 设置自检状态的逻辑
    return status;
}


// 惯组通信检查
bool checkInertialNavigationCommunication(uint32_t tx_addr, uint32_t rx_addr) {
    uint8_t receive_buffer[50]; // 缓冲区大小设置为44字节，以容纳所有返回的数据

    // 发送自检命令 "55H 68H"（假设该命令为字节数组）
    uint8_t selfTestCommand[] = {0x55, 0x68}; 
    FPGAUart_SendArray(tx_addr, selfTestCommand, sizeof(selfTestCommand));

    // 延迟1ms以确保模块有时间响应
    delay_1ms(1);

    // 接收反馈数据，期望接收50字节
    FPGAUart_ReceiveArray(rx_addr, receive_buffer, 50);

    // 检查反馈数据是否以0x97开头，表示通信正常
    if (receive_buffer[0] == 0x97) {
        // 解析陀螺和加速度数据等
        int32_t gyroX_angleIncrement = (receive_buffer[1] << 16) | (receive_buffer[2] << 8) | receive_buffer[3];
        uint16_t gyroX_integralTime = (receive_buffer[4] << 8) | receive_buffer[5];

        int32_t gyroY_angleIncrement = (receive_buffer[6] << 16) | (receive_buffer[7] << 8) | receive_buffer[8];
        uint16_t gyroY_integralTime = (receive_buffer[9] << 8) | receive_buffer[10];

        int32_t gyroZ_angleIncrement = (receive_buffer[11] << 16) | (receive_buffer[12] << 8) | receive_buffer[13];
        uint16_t gyroZ_integralTime = (receive_buffer[14] << 8) | receive_buffer[15];

        int32_t accelX_velocityIncrement = (receive_buffer[16] << 16) | (receive_buffer[17] << 8) | receive_buffer[18];
        uint16_t accelX_integralTime = (receive_buffer[19] << 8) | receive_buffer[20];

        int32_t accelY_velocityIncrement = (receive_buffer[21] << 16) | (receive_buffer[22] << 8) | receive_buffer[23];
        uint16_t accelY_integralTime = (receive_buffer[24] << 8) | receive_buffer[25];

        int32_t accelZ_velocityIncrement = (receive_buffer[26] << 16) | (receive_buffer[27] << 8) | receive_buffer[28];
        uint16_t accelZ_integralTime = (receive_buffer[29] << 8) | receive_buffer[30];

        int16_t gyroX_temperature = (receive_buffer[31] << 8) | receive_buffer[32];
        int16_t gyroY_temperature = (receive_buffer[33] << 8) | receive_buffer[34];
        int16_t gyroZ_temperature = (receive_buffer[35] << 8) | receive_buffer[36];

        int16_t accelX_temperature = (receive_buffer[37] << 8) | receive_buffer[38];
        int16_t accelY_temperature = (receive_buffer[39] << 8) | receive_buffer[40];
        int16_t accelZ_temperature = (receive_buffer[41] << 8) | receive_buffer[42];

        // 输出解析结果（可以根据需要输出其他数据）
        printf("惯组通信正常\n");
        printf("陀螺X角度增量: %d, 积分时间: %u\n", gyroX_angleIncrement, gyroX_integralTime);
        printf("陀螺Y角度增量: %d, 积分时间: %u\n", gyroY_angleIncrement, gyroY_integralTime);
        printf("陀螺Z角度增量: %d, 积分时间: %u\n", gyroZ_angleIncrement, gyroZ_integralTime);
        printf("加速度表X速度增量: %d, 积分时间: %u\n", accelX_velocityIncrement, accelX_integralTime);
        printf("加速度表Y速度增量: %d, 积分时间: %u\n", accelY_velocityIncrement, accelY_integralTime);
        printf("加速度表Z速度增量: %d, 积分时间: %u\n", accelZ_velocityIncrement, accelZ_integralTime);

        printf("陀螺X温度: %d\n", gyroX_temperature);
        printf("陀螺Y温度: %d\n", gyroY_temperature);
        printf("陀螺Z温度: %d\n", gyroZ_temperature);
        printf("加速度表X温度: %d\n", accelX_temperature);
        printf("加速度表Y温度: %d\n", accelY_temperature);
        printf("加速度表Z温度: %d\n", accelZ_temperature);

        return true;
    } else {
        printf("惯组通信失败，返回数据异常\n");
        return false;
    }
}

// 通信机通信检查
bool checkCommunicationCommunication() {
    // 模拟通信机通信检查
    return true; // 假设通信正常
}

// 观察瞄准组件通信检查
bool checkObservationAimingComponentCommunication() {
    // 模拟观察瞄准组件通信检查
    return true; // 假设通信正常
}

// 热控温度检查
bool checkTemperatureControlTemperature(int temperature) {
    // 检查温度是否在正常范围内
    if (temperature < 20 || temperature > 25) {
        return false; // 温度异常
    }
    return true; // 温度正常
}

// 中控组件遥测功能检查
bool checkCentralControlTelemetry() {
    // 模拟中控组件遥测功能检查
    return true; // 假设功能正常
}

// 冷气推进压力检查
bool checkColdGasThrustPressure() {
    // 模拟冷气推进压力检查
    return true; // 假设压力正常
}

// 单元推进压力检查
bool checkUnitThrustPressure() {
    // 模拟单元推进压力检查
    return true; // 假设压力正常
}

// 放电开关检查
bool checkDischargeSwitch() {
    // 模拟放电开关检查
    return true; // 假设开关正常
}

// 处理自检模式的代码
bool selfTestMode() {
    // 执行自检操作
    printf("执行自检操作\n");
		// 初始化标志函数
		initializeFlags(); 
		// 在自检模式中进行各项检查
	
		// 有线遥测包中设置自检状态 --- 待完成。。
//    bool setSelfTestStatus(bool status){
//        // 设置自检状态
//        return status;
//    }
	
    // 星敏通信检查
    bool starSensorCommunication = checkStarSensorCommunication();
    printf("星敏通信检查结果: %d\n", starSensorCommunication);

    // GNSS通信检查
    bool gnssCommunication = checkGNSSCommunication();
    printf("GNSS通信检查结果: %d\n", gnssCommunication);

    // 惯组通信检查
    bool inertialNavigationCommunication = checkInertialNavigationCommunication(UART_GZ_TXDATA_ADDR_DEBUG, UART_GZ_RXDATA_ADDR_DEBUG);
    printf("惯组通信检查结果: %d\n", inertialNavigationCommunication);

    // 通信机通信检查
    bool communicationCommunication = checkCommunicationCommunication();
    printf("通信机通信检查结果: %d\n", communicationCommunication);

    // 观察瞄准组件通信检查
    bool observationAimingComponentCommunication = checkObservationAimingComponentCommunication();
    printf("观察瞄准组件通信检查结果: %d\n", observationAimingComponentCommunication);

    // 热控温度检查
    bool temperatureControlTemperature = checkTemperatureControlTemperature(22); // 设置
    printf("热控温度检查结果: %d\n", temperatureControlTemperature);

    // 中控组件遥测功能检查
    bool centralControlTelemetry = checkCentralControlTelemetry();
    printf("中控组件遥测功能检查结果: %d\n", centralControlTelemetry);

    // 冷气推进压力检查
    bool coldGasThrustPressure = checkColdGasThrustPressure();
    printf("冷气推进压力检查结果: %d\n", coldGasThrustPressure);

    // 单元推进压力检查
    bool unitThrustPressure = checkUnitThrustPressure();
    printf("单元推进压力检查结果: %d\n", unitThrustPressure);

    // 放电开关检查
    bool dischargeSwitch = checkDischargeSwitch();
    printf("放电开关检查结果: %d\n", dischargeSwitch);


    // 检查是否有任何一项检查失败
    if (!starSensorCommunication || !gnssCommunication || !inertialNavigationCommunication || !communicationCommunication ||
        !observationAimingComponentCommunication || !temperatureControlTemperature || !centralControlTelemetry ||
        !coldGasThrustPressure || !unitThrustPressure || !dischargeSwitch) {
        printf("检查失败，等待地面介入处理\n");
					
				PTmode_Status = false;
				
        return false;  // 返回 false 表示自检失败
    }
    else{
			  GNCmode_Finished = true;
			  GNCmode_Status = true;
			  PTmode_Status = true;
				PTmode_Finished = true; // 后面应该需要判断一下，子星模式状态标志、GNC模式状态标志、GNC模式完成标志，是否都完成和正常

        mode = STORAGE_MODE; // 切换模式为存储类型
        printf("自检通过\n");
			return true;
    }
    // 返回自检状态 ?
}

// 装订信息存储
bool storeBindingInfo() {
    // 存储装订信息的逻辑
    printf("装订信息存储\n");
    return true; // 假设存储成功
}

bool checkBindingInfo() {
    // 实现逻辑
    return true;
}

bool validateBaselineInfo() {
    // 实现逻辑
    return true;
}

bool storeBaselineInfo() {
    // 实现逻辑
    return true;
}

bool checkInternalPowerCommand() {
    // 实现逻辑
    return true;
}

bool openDischargeSwitch() {
    // 实现逻辑
		FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
    return true;
}

void startTimer() {
    // 实现逻辑
}

bool waitForInternalPowerCommand() {
    // 实现逻辑
    return true;
}

void sendCommand(unsigned int command) {
    // 实现逻辑
}

bool checkFaultCode() {
    // 实现逻辑
    return false;
}

void setFaultStatus(bool status) {
    // 实现逻辑
}

unsigned int checkTimer() {
    // 实现逻辑
    return 0;
}

bool checkSeparationSwitch() {
    // 实现逻辑
    return true;
}

bool validateBindingInfo() {
    // 添加实际的校验逻辑
    return true; // 假设校验通过
}



// 基准信息接收，并校验，成功后存储
bool receiveBaselineInfo() {
    // 模拟接收基准信息
    bool receivedBaselineInfo = true; // 假设接收到基准信息
    if (receivedBaselineInfo) {
        printf("接收到基准信息\n");
        // 进行校验
        bool baselineInfoValid = validateBaselineInfo();
        if (baselineInfoValid) {
            printf("基准信息校验通过\n");
            // 存储基准信息
            bool baselineInfoStored = storeBaselineInfo();
            if (baselineInfoStored) {
                printf("基准信息存储成功\n");
                mode = STATE_ESTABLISHMENT_MODE; // 切换模式为状态建立类型
            } else {
                printf("基准信息存储失败\n");
                // 处理存储失败情况
            }
        } else {
            printf("基准信息校验失败\n");
            // 处理校验失败情况
        }
    } else {
        printf("等待基准信息\n");
    }
    return receivedBaselineInfo;
}

//预热模式
void preheatMode() {
    // 执行预热操作
    printf("执行预热操作\n");
			// 初始化标志函数
		initializeFlags();
		//自主预热
		temperature_control();
	
	// 设置预热状态标志
    bool bindingInfoValid = false;
    bool preheatCompleted = false;
		
		GNCmode_Finished = true;  // GNC模式完成标志

    // 等待装订信息
    // 如果收到装订信息，进行信息校验
    bool receivedBindingInfo = checkBindingInfo();
    if (receivedBindingInfo) {
        printf("收到装订信息\n");
        // 进行信息校验
        bindingInfoValid = validateBindingInfo();
        if (bindingInfoValid) {
            printf("装订信息校验通过\n");

            // 在预热模式中进行装订信息存储
                if (storeBindingInfo()) {
                    printf("装订信息存储成功\n");
                } else {
                    printf("装订信息存储失败\n");
                    // 处理存储失败情况
                }

        } else {
            printf("装订信息校验失败\n");
            // 处理校验失败情况
        }
    } 
    else
    {
        printf("等待装订信息\n");
    }

    // 基准信息接收，并校验，成功后存储
    if (receiveBaselineInfo()) {
        printf("基准信息接收成功\n");
    } else {
        printf("基准信息接收失败\n");
    }

    // 接收“转内电”指令
    if (checkInternalPowerCommand()) {
        printf("转内电指令完成\n");
        // 检查装订信息和预热状态
        if (receivedBindingInfo && preheatCompleted) {
            // 打开蓄电池放电开关
            if (openDischargeSwitch()) {
                printf("蓄电池放电开关已打开\n");
                // 开始计时
                startTimer();
            } else {
                printf("蓄电池放电开关打开失败\n");
                // 处理开关打开失败情况
            }
        } else {
            printf("装订信息或预热未完成，无法打开蓄电池放电开关\n");
            // 处理装订信息或预热未完成情况
        }
    }

    // 等待“转内电成功”指令
		bool internalPowerCommandSuccess = waitForInternalPowerCommand();
    if (internalPowerCommandSuccess) {
        printf("转内电成功\n");
        sendCommand(0xC9C9); // 发送指令，内容为0xC9C9

    } else {
        printf("转内电失败\n");
        sendCommand(0X9C9C); // 发送指令，内容为0xC9C9
    }

    // 根据故障码有无，设置状态标志
    if (checkFaultCode()) {
        printf("检测到故障码\n");
        // 设置故障标志
        setFaultStatus(true);
			
				PTmode_Status = false;      // 子星模式状态标志
    } else {
        printf("未检测到故障码\n");
        // 清除故障标志
        setFaultStatus(false);
				
    }

    // 检查装订信息、预热完成和转内电完成状态
    if (receivedBindingInfo && preheatCompleted && internalPowerCommandSuccess) {
        // 设置完成标志
        modeFinished = true; 
			
				PTmode_Finished = true;   // 子星模式完成标志
    }

    // 检查计时是否达到70秒
    if (checkTimer() >= 70) {
        // 检查分离开关状态
        bool separationSwitchStatus = checkSeparationSwitch();
        if (separationSwitchStatus) {
            printf("分离开关未断开，继续等待\n");
        } else {
            printf("分离开关已断开，进入状态建立模式\n");
            mode = STATE_ESTABLISHMENT_MODE; // 切换模式为状态建立类型
        }
    }
}

void GNSSCapture() {
    // 暂时的占位符实现
}

bool accelerometerCalibration() {
    // 暂时的占位符实现
    return true;
}

bool establishGroundOrientation() {
    // 暂时的占位符实现
    return true;
}

void establishInterCommunication() {
    // 暂时的占位符实现
}

void exhaustUnitThrustPipeline() {
    // 暂时的占位符实现
}

void powerOnCommunication() {
    // 暂时的占位符实现
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0355;
}

void powerOnControl() {
    // 暂时的占位符实现
		FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
}

void powerOnGNSS() {
    // 暂时的占位符实现
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0755;
}

void powerOnInertialNavigation() {
    // 暂时的占位符实现
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0555;	
}

void powerOnStarSensor() {
    // 暂时的占位符实现
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0455;
}

void prePulse() {
    // 暂时的占位符实现
}

void sendImportantTelemetry() {
    // 暂时的占位符实现
}

void spinStabilizationControl() {
    // 暂时的占位符实现
}

void starSensorCapture() {
    // 暂时的占位符实现
}

// 连续判断-- 故障码，完成标志


// 状态建立模式
void stateEstablishmentMode(){
	
		int i = 0; // 初始化i
		OS_ERR err;
		// 初始化标志函数
    initializeFlags();
	
    // 上电启动处理 存疑？
    powerOnControl();//一直有电
    powerOnStarSensor();
    powerOnCommunication();
    powerOnInertialNavigation();
    powerOnGNSS();
    
    // 10S禁止姿控 --控制姿控的标志
    printf("禁止姿控: %d\n", i+1);
//    sleep(10);
	  OSTimeDly(10000, OS_OPT_TIME_DLY, &err);

    // 20S禁止开启推力器、热控单元自主热控
    printf("禁止开启推力器、热控单元自主热控: %d\n", i+1);
//    sleep(20);
		OSTimeDly(20000, OS_OPT_TIME_DLY, &err);


    exhaustUnitThrustPipeline();
    prePulse();
    spinStabilizationControl();
    starSensorCapture();
    GNSSCapture();
    accelerometerCalibration();
    establishGroundOrientation();

    //建立星间通信状态
    establishInterCommunication();
    //发送重要遥测
    sendImportantTelemetry();//需要发几包遥测包
		
		

   // 在状态建立模式中处理故障码
    
    // 故障码处理 -- 连续判断几拍 计数
    unsigned int faultFlags = 0;
    unsigned int faultsMask = (FAULT_ORBIT_DETERMINATION_FAILURE |
                               FAULT_ATTITUDE_CONTROL_ANOMALY |
                               FAULT_GNC_MIN_SYSTEM_NOT_MET |
                               FAULT_GNC_MODE_ANOMALY |
                               FAULT_STAR_SENSOR_COMMUNICATION |
                               FAULT_IMU_COMMUNICATION |
                               FAULT_GNSS_COMMUNICATION |
                               FAULT_COMMUNICATION_DEVICE); 
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
		
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        PTmode_Status = false;
        mode = STANDBY_MODE;
        return;
    }
		else{
        PTmode_Status = true;
    }


    //模式完成标志更新
    //如果没有故障码、加速度计完成标定、GNC对地定向完成，模式完成设置为1
    if(PTmode_Status && accelerometerCalibration() && establishGroundOrientation()){
        PTmode_Finished = true; 
    }

    //模式完成标志为1，进入初始制导模式
    if(PTmode_Finished){
        mode = INITIAL_GUIDANCE_MODE;
    }
    return;
}

bool GNCCompleted() {
    // 模拟GNC完成状态
	  GNCmode_Finished = true;  // GNC模式完成标志
    return GNCmode_Finished;  // 假设GNC完成
}

bool trackDataEstablished(void) { return true; }
void powerOnObservationAimingCamera(void) {}
void powerOnRangeFinder(void) {}
bool checkObservationAimingCamera(void) { return true; }
bool checkRangeFinder(void) { return true; }
void outputRelativeInfo(void) {}
bool relativeNavigationConverged(void) { return true; }
void CWEquation(void) {}
void updateRelativeTrack(void) {}
bool checkFightFinished(void) { return true; }
void telemetryDataTransmission(void) {}
void powerOffObservationCamera(void) {}
void powerOffRangeFinder(void) {}
void powerOffObservationComponent(void) {}

// 处理初始制导模式的代码
void initialGuidanceMode(){
	
		// 初始化标志函数
    initializeFlags();
    // 保持星间通信（TODO）

    // 可见-》不可见-》可见，通信可能断开 =》正常
    // 基于装订的目标预报轨道参数，拟合目标轨道，用于制导规划解算
    // 基于自身当前轨道和拟合的目标轨道，开展制导规划解算
    // 开展轨道机动，每次轨道机动前，需姿态机动至变轨所需的速度增量方向，轨道机动后需姿态机动至对地定向方向，最长时间45MIN
    // 到达目标后方5KM，对地定向保持

    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = (FAULT_ORBIT_DETERMINATION_FAILURE |
                               FAULT_ATTITUDE_CONTROL_ANOMALY |
                               FAULT_COMMUNICATION_LINK_LOSS |
                               FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL |
                               FAULT_GNC_MIN_SYSTEM_NOT_MET |
                               FAULT_GNC_MODE_ANOMALY |
                               FAULT_GNC_RELATIVE_POSITION_CONTROL |
                               FAULT_STAR_SENSOR_COMMUNICATION |
                               FAULT_IMU_COMMUNICATION |
                               FAULT_GNSS_COMMUNICATION |
                               FAULT_COMMUNICATION_DEVICE |
                               FAULT_OBSERVATION_PROCESSING |
                               FAULT_CAMERA_TARGET_LOSS |
                               FAULT_LASER_RANGEFINDER_TARGET_LOSS |
                               FAULT_PAYLOAD_COVER_FAILURE |
                               FAULT_COMBAT_TIMEOUT); 
    
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        PTmode_Status = false;      // 子星模式状态标志
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // 子星模式状态标志
    }

    // 状态标志处理：状态完成
    // 判断状态标志=》正常，GNC完成=》完成
    if(PTmode_Status && GNCCompleted()){
        PTmode_Finished = true;   // 子星模式完成标志
    }

    // 模式完成
    if(PTmode_Finished){
        mode = MIDDLE_GUIDANCE_ESTABLISHMENT_MODE;
    } 

}

/**
 * @brief 中间制导建立模式
 * 
 * 在中间制导建立模式下，进行目标定位轨道数据的制导解算和故障检查。
 * 如果检测到故障，将进入待机模式。
 * 如果制导解算和故障检查都通过，将进入中间制导模式。
 */

void middleGuidanceEstablishmentMode(){
    
	// 初始化标志函数
    initializeFlags();
	
    // TODO: 600S保护
    if (g_middleEstablishStartTime == 0) {
        g_middleEstablishStartTime = time(NULL);
    }


    // 每周期1S，基于目标定位轨道数据开展制导解算

    // 判断目标定位轨道数据的目标轨道可用标志，如果可用，将接收的目标轨道外推至当前时刻，更新至目标参考轨道
    // 如果目标轨道不可用，取上一时刻更新的目标参考轨道，外推至当前时刻，更新至目标参考轨道

    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = (FAULT_ORBIT_DETERMINATION_FAILURE |
                               FAULT_ATTITUDE_CONTROL_ANOMALY |
                               FAULT_COMMUNICATION_LINK_LOSS |
                               FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL |
                               FAULT_GNC_MIN_SYSTEM_NOT_MET |
                               FAULT_GNC_MODE_ANOMALY |
                               FAULT_GNC_RELATIVE_POSITION_CONTROL |
                               FAULT_STAR_SENSOR_COMMUNICATION |
                               FAULT_IMU_COMMUNICATION |
                               FAULT_GNSS_COMMUNICATION |
                               FAULT_COMMUNICATION_DEVICE); 

    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        PTmode_Status = false;
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // 子星模式状态标志
    }

    // 状态标志处理
    // 判断状态标志=》正常，GNC完成=》完成，轨道数据中中制导状态建立成功
    if(PTmode_Status && GNCCompleted() && trackDataEstablished()){
        PTmode_Finished = false;   // 子星模式完成标志
    }

    // 模式完成
    if(PTmode_Finished){
        mode = MIDDLE_GUIDANCE_MODE;
    }
    return;
}

void middleGuidanceMode(){
    // 初始化标志函数
    initializeFlags();
    
    // 保持星间通信

    // 基于目标定位轨道数据，自主完成递进计划和控制

    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = (FAULT_ORBIT_DETERMINATION_FAILURE |
                               FAULT_ATTITUDE_CONTROL_ANOMALY |
                               FAULT_COMMUNICATION_LINK_LOSS |
                               FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL |
                               FAULT_GNC_MIN_SYSTEM_NOT_MET |
                               FAULT_GNC_MODE_ANOMALY |
                               FAULT_GNC_RELATIVE_POSITION_CONTROL |
                               FAULT_STAR_SENSOR_COMMUNICATION |
                               FAULT_IMU_COMMUNICATION |
                               FAULT_GNSS_COMMUNICATION |
                               FAULT_COMMUNICATION_DEVICE |
                               FAULT_OBSERVATION_PROCESSING |
                               FAULT_CAMERA_TARGET_LOSS |
                               FAULT_LASER_RANGEFINDER_TARGET_LOSS |
                               FAULT_PAYLOAD_COVER_FAILURE |
                               FAULT_COMBAT_TIMEOUT);
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        PTmode_Status = false;      // 子星模式状态标志
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // 子星模式状态标志
    }

    // 状态标志处理
    // 判断状态标志=》正常，GNC完成=》完成，轨道数据中中制导状态建立成功
    if(PTmode_Status && GNCCompleted()){
        PTmode_Finished = true;   // 子星模式完成标志
    }

    // 模式完成
    if(PTmode_Finished){
        mode = MIDDLE_FINAL_HANDOVER_MODE;
    }
    return;
}


void middleFinalHandoverMode(){
    // 保持星间通信
    // 开启观瞄相机和测距机
    powerOnObservationAimingCamera();
    powerOnRangeFinder();

    // 姿态转为对目标定向

    // 如果观瞄相机和测距机数据无异常，启动“测角+测距”算法，输出目标相对信息
    if(!checkObservationAimingCamera() && !checkRangeFinder()){
        // 输出目标相对信息
        outputRelativeInfo();
    }

    // 故障码处理
    
    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // 假设我们要检查所有故障
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // 状态标志处理
    // 判断状态标志=》正常，GNC完成=》完成，相对导航=》收敛
    if(!checkFaultCode() && GNCCompleted() && relativeNavigationConverged()){
        modeFinished = 1;
    }

    // 模式完成
    if(modeFinished){
        mode = FINAL_GUIDANCE_MODE;
    }
    return;
}

void finalGuidanceMode(){
    // 保持星间通信

    // 如果通信中断，任务不中断

    // 每个任务周期，判断“子星测角数据有效标识”和“子星测距数据有效标识”，如果都有效，C-W方程自主目标定位
    if(checkObservationAimingCamera() && checkRangeFinder()){
        // C-W方程自主目标定位
        CWEquation();
        // 更新至目标相对轨道
        updateRelativeTrack();

    }
    else {
        // 取上一时刻更新的目标相对轨道
        // 利用C-W方程外推至当前时刻
        // 更新至目标相对轨道

    }

    // 基于目标相对轨道，自主开展末制导规划解算
    // 基于观瞄相机数据，自主完成目标识别


    // 故障码处理
    
    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // 假设我们要检查所有故障
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // 状态标志处理
    // 判断状态标志=》正常，GNC完成=》完成，轨道数据中中制导状态建立成功
    if(!checkFaultCode() && GNCCompleted()){
        modeFinished = 1;
    }

    // 模式完成
    if(modeFinished){
        mode = HOVER_MODE;
    }
    return;
}

bool fireControlOnline() {
    // 模拟火控在线状态
    return true; // 假设火控在线
}

bool isTaskEnabled() {
    // 模拟任务是否使能
    return true; // 假设任务已使能
}

bool minsys() {
    // 模拟最小系统条件
    return true; // 假设最小系统条件已满足
}


void hoverMode(){
    // 保持星间通信
    // 通信中断，悬停任务不中断

    // 保持目标视场中心

    // 保持相对速度和位置

    // 判断使能标志为“允许”，进入“载荷工作”子模式
    if(isTaskEnabled())
    {
        // 发送打击指令（1S3次）

        // 判断火工品在线标志：起爆
        if(fireControlOnline())
        {
            // 等待12S

            // 生成任务完成标志

            // 如果超出30S，且火工在线标志 != 起爆，生成任务标志为“完成”

        }
    }
    else{
        modeFinished = 1;
    }


    // 故障码处理
    
    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // 假设我们要检查所有故障
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        
        mode = STANDBY_MODE;
        return;
    }


    // 状态标志处理
    // 判断状态标志=》正常，GNC完成=》完成，战斗完成
    if(!checkFaultCode() && GNCCompleted() &&checkFightFinished() ){
        modeFinished = 1;
    }

    // 悬停模式下切换方向（TODO）

    // 

    // 模式完成
    if(modeFinished){
        mode = IMAGE_TRANSMISSION_MODE;
    }
    return;
}

void imageTransmissionMode(){
    // 姿态转为对地定向，降低轨道高度远离目标

    // 自动执行“遥测数据回传” 发送图像，执行1次
    telemetryDataTransmission();

    // 观瞄相机、测距机、观瞄处理组件关机
    powerOffObservationCamera();
    powerOffRangeFinder();
    powerOffObservationComponent();

    // 故障码处理
    
    // 故障码处理
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // 假设我们要检查所有故障
    // 在主程序中调用故障检查函数
    checkFaults(&faultFlags, faultsMask);
    // 检查故障码
    if (faultFlags != 0) {
        printf("检测到故障，进入待机模式\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // 状态标志处理
    if(!checkFaultCode() && GNCCompleted() ){
        modeFinished = 1;
    }

    // 模式停留回传

}

// 各种故障情况，进入该模式
// 母星指令进入该模式
void standbyMode(){
    //保持单机原来状态

    // 保持对地定向姿态

    // 保持通信，执行“遥测数据回传”指令

    // 等待指令

    // 是否满足最小系统条件，不满足停止姿态控制
    if(minsys()){

    }
    else{

    }

    // 状态标志处理
    if(!checkFaultCode() && GNCCompleted() ){
        modeFinished = 1;
    }

}

//接收姿控的完成信号

void processMode(Mode mode){
    switch (mode) {
        case STORAGE_MODE:
            storage_mode();
            break;
				case SELF_TEST_MODE:
            selfTestMode();
            break;
				case PREHEAT_MODE:
            // 处理预热模式的代码
            preheatMode();
            break;
        case STATE_ESTABLISHMENT_MODE:
            // 处理状态建立模式的代码
            stateEstablishmentMode();
            break;
        case INITIAL_GUIDANCE_MODE:
            // 处理初始制导模式的代码
            initialGuidanceMode();
            break;
        case MIDDLE_GUIDANCE_ESTABLISHMENT_MODE:
            // 处理中间制导建立模式的代码
            middleGuidanceEstablishmentMode();
            break;
        case MIDDLE_GUIDANCE_MODE:
            // 处理中间制导模式的代码
            middleGuidanceMode();
            break;
        case MIDDLE_FINAL_HANDOVER_MODE:
            // 处理中末交接模式的代码
            middleFinalHandoverMode();
            break;
        case FINAL_GUIDANCE_MODE:
            // 处理最终制导模式的代码
            finalGuidanceMode();
            break;
        case HOVER_MODE:
            // 处理悬停模式的代码
            hoverMode();
            break;
        case IMAGE_TRANSMISSION_MODE:
            // 处理图像传输模式的代码
            imageTransmissionMode();
            break;
        case STANDBY_MODE:
            // 处理待机模式的代码
            standbyMode();
            break;
        default:
            break;
    }
}



