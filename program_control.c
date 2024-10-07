#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "define.h"
#include "flyprog.h"
#include "program_control.h"
#include "FPGAUart.h"

// ����ȫ�ֱ�־����
bool PTmode_Finished;   // ����ģʽ��ɱ�־
bool PTmode_Status;      // ����ģʽ״̬��־
bool GNCmode_Finished;  // GNCģʽ��ɱ�־
bool GNCmode_Status;     // GNCģʽ״̬��־

// ���� setSelfTestStatus ����
bool setSelfTestStatus(bool status);
bool modeFinished = false;  // ��ʼ��Ϊ false  ����ȫ��ȥ��---


//�м��Ƶ���ʼʱ��
static int g_middleEstablishStartTime = 0;

// ȫ�ֱ��
bool initExecuted = false;
bool selfTestCompleted = false;
bool preheatCompleted = false;
unsigned int g_commTimer = 0;
unsigned int g_naviDataTimer = 0;

// ����ȫ�ֱ�����ʹ�� 0 �� 1 ��ʾ״̬
int selfTestCommandReceived = 0;  // 0 ��ʾδ���յ��Լ�ָ��
int preheatCommandReceived = 0;   // 0 ��ʾδ���յ�Ԥ��ָ��
int faultDetected = 0;            // 0 ��ʾû�й���



// ����ȫ�ֱ���
Mode mode = STORAGE_MODE;

// ��ʼ����־����
void initializeFlags() {
    PTmode_Finished = false;
    PTmode_Status = true;
    GNCmode_Finished = false;
    GNCmode_Status = true;
    printf("��־�ѳ�ʼ����PTmode_Finished = %d, PTmode_Status = %d, GNCmode_Finished = %d, GNCmode_Status = %d\n",
           PTmode_Finished, PTmode_Status, GNCmode_Finished, GNCmode_Status);
}

unsigned int getCurrentTime(void) {
    // �����ʵ�ֿ����ǻ�ȡϵͳʱ�䡢��ʱ��ֵ�ȣ�����ʵ��ȡ����Ӳ����Ӧ�ó���
    return 0;  // ���赱ǰʱ�䷵�� 0������Ϊʾ��
}

// Ӳ�����ƺ���
void powerOnCentralControl() {
	  //�пذ幩������ؿ�����һ����
	  FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
    printf("�п��������\n");
}

void powerOffStarSensor() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x04AA;
    printf("���������ػ�\n");
}

void powerOffCommunication() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x03AA;
    printf("ͨ�Ż��ػ�\n");
}

void powerOffInertialNavigation() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x05AA;
    printf("�ߵ��ػ�\n");
}

void sleepGNC() {
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x07AA;
    printf("GNC����\n");
}


bool checkForFault() {
    // ���ϼ��
    return faultDetected; // ����ȫ�ֹ��ϱ�־
}

// bit0�������װ�������쳣
bool checkBindingDataError() {
    // TODO: �����װ�������쳣
    
    return false; 
}
// bit1������ʧ��
bool checkOrbitDeterminationFailure() {
    // TODO: ����ʧ��
    return false; 
}

// bit2���˿��쳣
bool checkAttitudeControlAnomaly() {
    // TODO: �˿��쳣
    return false; 
}

// bit3��ͨ�Ŷ���
// TODO: g_commTimer����
bool checkCommunicationLinkLoss() {
    // ���Ƶ�ģʽ��ʼ������10Sû���յ�ң��֡���������ϵ������ݣ�
    // ��ʱ��
    // ȫ�ּ�ʱ�� TODO: �յ�ͨ�Ű�ʱ������g_commTimer

    // ��鵱ǰģʽ����������Ƶ�ģʽ����ʼ��ʱ
    if (mode == MIDDLE_GUIDANCE_MODE) {
        if (g_commTimer != 0) {
            unsigned int currentTime = getCurrentTime();
            unsigned int deltaTime = currentTime - g_commTimer;
            if (deltaTime >= 10) {
                return false; // ͨ�Ŷ���
            }
            else
            {
                return true; // ͨ������
            }

        }
    }
    
    return true; 
}

// bit4, ���ң��֡У��ʧ��
bool checkRemoteControlFrameCheckFail() {
    // TODO: ���ң��֡У��ʧ��
    return false; 
}

// bit5, ������GNC������Сϵͳ
bool checkGNCMinSystemNotMet() {
    // TODO: GNC������Сϵͳ
    return false; 
}
// bit6, GNCģʽ�쳣
bool checkGNCModeAnomaly() {
    // TODO: GNCģʽ�쳣
    return false; 
}
// bit7, GNC���λ�ÿ��Ƴ���
bool checkGNCRelativePositionControl() {
    // TODO: GNC���λ�ÿ��Ƴ���
    return false; 
}

// bit8, ��������ģʽ��ʱδ���
bool checkSubStarTaskTimeout() {
    unsigned int maxDeltaTime = 600;
    if(mode == MIDDLE_GUIDANCE_ESTABLISHMENT_MODE){
        maxDeltaTime = 600; // 10����
    }
    else if(mode == MIDDLE_FINAL_HANDOVER_MODE){
        maxDeltaTime = 300; // 50����
    }
    else{
        return true;
    }

    unsigned int currentTime = getCurrentTime();
    if (difftime(currentTime, g_middleEstablishStartTime) > maxDeltaTime) {
        // ��������
        return false;
    }

    return true; 
}

// bit9, GNC�涨ʱ����δ�����Ե����˲�
bool checkGNCRelativeNavTimeout() {
    // TODO: δ�����Ե����˲�
    return false; 
}

// bit10, ��Ե����˲���ɢ
bool checkRelativeNavDivergence() {
    // TODO: ��Ե����˲���ɢ
    return false; 
}

// bit11, ���ϵ������ݲ����ã�����30S�������ݲ�����
bool checkCombinedNavDataUnavailable() {
    // TODO: �յ�ͨ�Ű�ʱ������g_commTimer
    // ��鵱ǰģʽ����������Ƶ�ģʽ����ĩ�Ƶ�ģʽ����ʼ��ʱ
    if (mode == MIDDLE_GUIDANCE_MODE || mode == MIDDLE_FINAL_HANDOVER_MODE) {
        if (g_naviDataTimer != 0) {
            unsigned int currentTime = getCurrentTime();
            unsigned int deltaTime = currentTime - g_naviDataTimer;
            if (deltaTime >= 30) {
                return false; // �������ݲ�����
            }
            else
            {
                return true; // ������������
            }

        }
        else{
            return true;
        }
    }
    
    return true; 
}

// bit12, ����ͨ�Ź���
// ����3S��ң�⣬������һ��3S��ң��
// ң�����ʱ�䣬��������
unsigned int telemetryUpdateTime = 0;
unsigned int telemetryRestartCount = 0;
// TODO: ����ͨ�Ÿ���ʱ��
// �������ͨ�Ź���
bool checkStarSensorCommunication() {
    // ң�����ʱ�䣬��������
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - telemetryUpdateTime;
    if (deltaTime >= 3) {
        telemetryRestartCount++;
        // TODO: ��������ͨ��
    }
    else {
        return true; // ����ͨ������
    }

    if (telemetryRestartCount > 1) {
        return false; // ����ͨ�Ź���
    }
    else {
        return true; // ����ͨ������
    }
}

// bit13, ����ͨ�Ź���
unsigned int imuUpdateTime = 0;
unsigned int imuRestartCount = 0;
// TODO: ����ͨ�Ÿ���ʱ��

bool checkIMUCommunication() {
    // ң�����ʱ�䣬��������
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - imuUpdateTime;
    if (deltaTime >= 3) {
        imuRestartCount++;
        // TODO: ��������ͨ��
    }
    else {
        return true; // ����ͨ������
    }

    if (imuRestartCount > 1) {
        return false; // ����ͨ�Ź���
    }
    else {
        return true; // ����ͨ������
    }
}

// bit14, GNSSͨ�Ź���
unsigned int gnssUpdateTime = 0;
unsigned int gnssRestartCount = 0;
// TODO: GNSSͨ�Ÿ���ʱ��

bool checkGNSSCommunication() {
    // ң�����ʱ�䣬��������
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - gnssUpdateTime;
    if (deltaTime >= 3) {
        gnssRestartCount++;
        // TODO: ����GNSSͨ��
    } else {
        return true; // GNSSͨ������
    }

    if (gnssRestartCount > 1) {
        return false; // GNSSͨ�Ź���
    } else {
        return true; // GNSSͨ������
    }
}

// bit15, ͨ�Ż�ͨ�Ź���
unsigned int communicationDeviceUpdateTime = 0;
unsigned int communicationDeviceRestartCount = 0;
// TODO: ͨ�Ż�ͨ�Ÿ���ʱ��
bool checkCommunicationDevice() {
    // ң�����ʱ�䣬��������
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - communicationDeviceUpdateTime;
    if (deltaTime >= 3) {
        communicationDeviceRestartCount++;
        // TODO: ����ͨ�Ż�ͨ��
    } else {
        return true; // ͨ�Ż�ͨ������
    }

    if (communicationDeviceRestartCount > 1) {
        return false; // ͨ�Ż�ͨ�Ź���
    } else {
        return true; // ͨ�Ż�ͨ������
    }
}

// bit16, ���鴦��ͨ�Ź���
unsigned int observationDeviceUpdateTime = 0;
unsigned int observationDeviceRestartCount = 0;
// TODO: ���鴦��ͨ�Ÿ���ʱ��
bool checkObservationDevice() {
    // ң�����ʱ�䣬��������
    unsigned int currentTime = getCurrentTime();
    unsigned int deltaTime = currentTime - observationDeviceUpdateTime;
    if (deltaTime >= 3) {
        observationDeviceRestartCount++;
        // TODO: �������鴦��ͨ��
    } else {
        return true; // ���鴦��ͨ������
    }

    if (observationDeviceRestartCount > 1) {
        return false; // ���鴦��ͨ�Ź���
    } else {
        return true; // ���鴦��ͨ������
    }
}
// bit17, �����ʧĿ��
bool checkCameraTargetLoss() {
    // TODO�������ʧĿ��
    return false; 
}
// bit18, ���������ʧĿ��
bool checkLaserRangefinderTargetLoss() {
    // TODO�����������ʧĿ��
    return false; 
}

// bit19, �غɿ���ʧ��
unsigned int payloadCoverCount = 0;
// TODO������ָ�����++
bool checkPayloadCoverFailure() {
    // ȫ���غɿ��Ǽ������ж��Ƿ����3
    if (payloadCoverCount >= 3) {
        return false; // �غɿ���ʧ��
    } else {
        return true; // �غɿ�������
    }

}

// bit20, ��ս��ʱ
unsigned int fightCommandCount = 0;
// TODO��fightָ�����++
bool checkCombatTimeout() {

    // fightָ��������ж��Ƿ����3
    if (fightCommandCount >= 3) {
        return false; // ��ս��ʱ
    } else {
        return true; // ��ս����
    } 
}

// ������
void checkFaults(unsigned int *faultFlags, unsigned int faultsMask) {
    if (faultsMask & FAULT_BINDING_DATA_ERROR) {
        if (checkBindingDataError()) {
            *faultFlags |= FAULT_BINDING_DATA_ERROR;
            printf("�����װ�������쳣\n");
        }
    }
    if (faultsMask & FAULT_ORBIT_DETERMINATION_FAILURE) {
        if (checkOrbitDeterminationFailure()) {
            *faultFlags |= FAULT_ORBIT_DETERMINATION_FAILURE;
            printf("����ʧ��\n");
        }
    }
    if (faultsMask & FAULT_ATTITUDE_CONTROL_ANOMALY) {
        if (checkAttitudeControlAnomaly()) {
            *faultFlags |= FAULT_ATTITUDE_CONTROL_ANOMALY;
            printf("�˿��쳣\n");
        }
    }
    if (faultsMask & FAULT_COMMUNICATION_LINK_LOSS) {
        if (checkCommunicationLinkLoss()) {
            *faultFlags |= FAULT_COMMUNICATION_LINK_LOSS;
            printf("ͨ�Ŷ���\n");
        }
    }
    if (faultsMask & FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL) {
        if (checkRemoteControlFrameCheckFail()) {
            *faultFlags |= FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL;
            printf("���ң��֡У��ʧ��\n");
        }
    }
    if (faultsMask & FAULT_GNC_MIN_SYSTEM_NOT_MET) {
        if (checkGNCMinSystemNotMet()) {
            *faultFlags |= FAULT_GNC_MIN_SYSTEM_NOT_MET;
            printf("������GNC������Сϵͳ\n");
        }
    }
    if (faultsMask & FAULT_GNC_MODE_ANOMALY) {
        if (checkGNCModeAnomaly()) {
            *faultFlags |= FAULT_GNC_MODE_ANOMALY;
            printf("GNCģʽ�쳣\n");
        }
    }
    if (faultsMask & FAULT_GNC_RELATIVE_POSITION_CONTROL) {
        if (checkGNCRelativePositionControl()) {
            *faultFlags |= FAULT_GNC_RELATIVE_POSITION_CONTROL;
            printf("GNC���λ�ÿ��Ƴ���\n");
        }
    }
    if (faultsMask & FAULT_SUB_STAR_TASK_TIMEOUT) {
        if (checkSubStarTaskTimeout()) {
            *faultFlags |= FAULT_SUB_STAR_TASK_TIMEOUT;
            printf("��������ģʽ��ʱδ���\n");
        }
    }
    if (faultsMask & FAULT_GNC_RELATIVE_NAV_TIMEOUT) {
        if (checkGNCRelativeNavTimeout()) {
            *faultFlags |= FAULT_GNC_RELATIVE_NAV_TIMEOUT;
            printf("GNC�涨ʱ����δ�����Ե����˲�\n");
        }
    }
    if (faultsMask & FAULT_RELATIVE_NAV_DIVERGENCE) {
        if (checkRelativeNavDivergence()) {
            *faultFlags |= FAULT_RELATIVE_NAV_DIVERGENCE;
            printf("��Ե����˲���ɢ\n");
        }
    }
    if (faultsMask & FAULT_COMBINED_NAV_DATA_UNAVAILABLE) {
        if (checkCombinedNavDataUnavailable()) {
            *faultFlags |= FAULT_COMBINED_NAV_DATA_UNAVAILABLE;
            printf("���ϵ������ݲ�����\n");
        }
    }
    if (faultsMask & FAULT_STAR_SENSOR_COMMUNICATION) {
        if (checkStarSensorCommunication()) {
            *faultFlags |= FAULT_STAR_SENSOR_COMMUNICATION;
            printf("����ͨ�Ź���\n");
        }
    }
    if (faultsMask & FAULT_IMU_COMMUNICATION) {
        if (checkIMUCommunication()) {
            *faultFlags |= FAULT_IMU_COMMUNICATION;
            printf("����ͨ�Ź���\n");
        }
    }
    if (faultsMask & FAULT_GNSS_COMMUNICATION) {
        if (checkGNSSCommunication()) {
            *faultFlags |= FAULT_GNSS_COMMUNICATION;
            printf("GNSS ͨ�Ź���\n");
        }
    }
    if (faultsMask & FAULT_COMMUNICATION_DEVICE) {
        if (checkCommunicationDevice()) {
            *faultFlags |= FAULT_COMMUNICATION_DEVICE;
            printf("ͨ�Ż�ͨ�Ź���\n");
        }
    }
    if (faultsMask & FAULT_OBSERVATION_PROCESSING) {
        if (checkObservationDevice()) {
            *faultFlags |= FAULT_OBSERVATION_PROCESSING;
            printf("���鴦��ͨ�Ź���\n");
        }
    }
    if (faultsMask & FAULT_CAMERA_TARGET_LOSS) {
        if (checkCameraTargetLoss()) {
            *faultFlags |= FAULT_CAMERA_TARGET_LOSS;
            printf("�����ʧĿ��\n");
        }
    }
    if (faultsMask & FAULT_LASER_RANGEFINDER_TARGET_LOSS) {
        if (checkLaserRangefinderTargetLoss()) {
            *faultFlags |= FAULT_LASER_RANGEFINDER_TARGET_LOSS;
            printf("���������ʧĿ��\n");
        }
    }
    if (faultsMask & FAULT_PAYLOAD_COVER_FAILURE) {
        if (checkPayloadCoverFailure()) {
            *faultFlags |= FAULT_PAYLOAD_COVER_FAILURE;
            printf("�غɿ���ʧ��\n");
        }
    }
    if (faultsMask & FAULT_COMBAT_TIMEOUT) {
        if (checkCombatTimeout()) {
            *faultFlags |= FAULT_COMBAT_TIMEOUT;
            printf("��ս��ʱ\n");
        }
    }
}



// �����Լ�ָ�����״̬ --��CX�����Լ�ָ�����
void setSelfTestCommandReceived(void) {
	
		// �ж��Ƿ��յ�cx�������Լ�ָ��
	
    selfTestCommandReceived = 1;
}

// ����Ԥ��ָ�����״̬ --��CX����Ԥ��ָ�����
void setPreheatCommandReceived(void) {
    
		// �ж��Ƿ��յ�cx�������Լ�ָ��
		preheatCommandReceived = 1;
}



//�洢ģʽ
void storage_mode(){
		
		// ��ʼ����־
    initializeFlags();
     
		// ִֻ��һ�εĹ���
		if (!initExecuted) {
				powerOnCentralControl();
				powerOffStarSensor();
				powerOffCommunication();
				powerOffInertialNavigation();
				sleepGNC();
				initExecuted = true;
		}
		
		//GNC ��������ģʽ�������ã� -- ��ɱ�־Ĭ����ɣ�״̬��־Ĭ������
		GNCmode_Finished = true;
		
		// �洢�ȿ� 
		temperature_control();
		
		// ���ϼ��--����Ҫ���ϼ�飬Ĭ������״̬����
		PTmode_Status = true;      // ����ģʽ״̬��־
		// �Լ�ָ����
		if (selfTestCommandReceived == 1) {
				printf("�Լ�ָ�����\n");
				mode = SELF_TEST_MODE; // �л�ģʽΪ�Լ�����
				PTmode_Finished = true;   // ����ģʽ��ɱ�־
				return;
		}

		// Ԥ��ָ����  --
		if (preheatCommandReceived == 1) {
				printf("Ԥ��ָ�����\n");
				// ������ɱ�־
				preheatCompleted = true;
				mode = PREHEAT_MODE; // �л�ģʽΪԤ������
				PTmode_Finished = true;   // ����ģʽ��ɱ�־
				return;
		}                   
}

bool setSelfTestStatus(bool status) {
    // �����Լ�״̬���߼�
    return status;
}


// ����ͨ�ż��
bool checkInertialNavigationCommunication(uint32_t tx_addr, uint32_t rx_addr) {
    uint8_t receive_buffer[50]; // ��������С����Ϊ44�ֽڣ����������з��ص�����

    // �����Լ����� "55H 68H"�����������Ϊ�ֽ����飩
    uint8_t selfTestCommand[] = {0x55, 0x68}; 
    FPGAUart_SendArray(tx_addr, selfTestCommand, sizeof(selfTestCommand));

    // �ӳ�1ms��ȷ��ģ����ʱ����Ӧ
    delay_1ms(1);

    // ���շ������ݣ���������50�ֽ�
    FPGAUart_ReceiveArray(rx_addr, receive_buffer, 50);

    // ��鷴�������Ƿ���0x97��ͷ����ʾͨ������
    if (receive_buffer[0] == 0x97) {
        // �������ݺͼ��ٶ����ݵ�
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

        // ���������������Ը�����Ҫ����������ݣ�
        printf("����ͨ������\n");
        printf("����X�Ƕ�����: %d, ����ʱ��: %u\n", gyroX_angleIncrement, gyroX_integralTime);
        printf("����Y�Ƕ�����: %d, ����ʱ��: %u\n", gyroY_angleIncrement, gyroY_integralTime);
        printf("����Z�Ƕ�����: %d, ����ʱ��: %u\n", gyroZ_angleIncrement, gyroZ_integralTime);
        printf("���ٶȱ�X�ٶ�����: %d, ����ʱ��: %u\n", accelX_velocityIncrement, accelX_integralTime);
        printf("���ٶȱ�Y�ٶ�����: %d, ����ʱ��: %u\n", accelY_velocityIncrement, accelY_integralTime);
        printf("���ٶȱ�Z�ٶ�����: %d, ����ʱ��: %u\n", accelZ_velocityIncrement, accelZ_integralTime);

        printf("����X�¶�: %d\n", gyroX_temperature);
        printf("����Y�¶�: %d\n", gyroY_temperature);
        printf("����Z�¶�: %d\n", gyroZ_temperature);
        printf("���ٶȱ�X�¶�: %d\n", accelX_temperature);
        printf("���ٶȱ�Y�¶�: %d\n", accelY_temperature);
        printf("���ٶȱ�Z�¶�: %d\n", accelZ_temperature);

        return true;
    } else {
        printf("����ͨ��ʧ�ܣ����������쳣\n");
        return false;
    }
}

// ͨ�Ż�ͨ�ż��
bool checkCommunicationCommunication() {
    // ģ��ͨ�Ż�ͨ�ż��
    return true; // ����ͨ������
}

// �۲���׼���ͨ�ż��
bool checkObservationAimingComponentCommunication() {
    // ģ��۲���׼���ͨ�ż��
    return true; // ����ͨ������
}

// �ȿ��¶ȼ��
bool checkTemperatureControlTemperature(int temperature) {
    // ����¶��Ƿ���������Χ��
    if (temperature < 20 || temperature > 25) {
        return false; // �¶��쳣
    }
    return true; // �¶�����
}

// �п����ң�⹦�ܼ��
bool checkCentralControlTelemetry() {
    // ģ���п����ң�⹦�ܼ��
    return true; // ���蹦������
}

// �����ƽ�ѹ�����
bool checkColdGasThrustPressure() {
    // ģ�������ƽ�ѹ�����
    return true; // ����ѹ������
}

// ��Ԫ�ƽ�ѹ�����
bool checkUnitThrustPressure() {
    // ģ�ⵥԪ�ƽ�ѹ�����
    return true; // ����ѹ������
}

// �ŵ翪�ؼ��
bool checkDischargeSwitch() {
    // ģ��ŵ翪�ؼ��
    return true; // ���迪������
}

// �����Լ�ģʽ�Ĵ���
bool selfTestMode() {
    // ִ���Լ����
    printf("ִ���Լ����\n");
		// ��ʼ����־����
		initializeFlags(); 
		// ���Լ�ģʽ�н��и�����
	
		// ����ң����������Լ�״̬ --- ����ɡ���
//    bool setSelfTestStatus(bool status){
//        // �����Լ�״̬
//        return status;
//    }
	
    // ����ͨ�ż��
    bool starSensorCommunication = checkStarSensorCommunication();
    printf("����ͨ�ż����: %d\n", starSensorCommunication);

    // GNSSͨ�ż��
    bool gnssCommunication = checkGNSSCommunication();
    printf("GNSSͨ�ż����: %d\n", gnssCommunication);

    // ����ͨ�ż��
    bool inertialNavigationCommunication = checkInertialNavigationCommunication(UART_GZ_TXDATA_ADDR_DEBUG, UART_GZ_RXDATA_ADDR_DEBUG);
    printf("����ͨ�ż����: %d\n", inertialNavigationCommunication);

    // ͨ�Ż�ͨ�ż��
    bool communicationCommunication = checkCommunicationCommunication();
    printf("ͨ�Ż�ͨ�ż����: %d\n", communicationCommunication);

    // �۲���׼���ͨ�ż��
    bool observationAimingComponentCommunication = checkObservationAimingComponentCommunication();
    printf("�۲���׼���ͨ�ż����: %d\n", observationAimingComponentCommunication);

    // �ȿ��¶ȼ��
    bool temperatureControlTemperature = checkTemperatureControlTemperature(22); // ����
    printf("�ȿ��¶ȼ����: %d\n", temperatureControlTemperature);

    // �п����ң�⹦�ܼ��
    bool centralControlTelemetry = checkCentralControlTelemetry();
    printf("�п����ң�⹦�ܼ����: %d\n", centralControlTelemetry);

    // �����ƽ�ѹ�����
    bool coldGasThrustPressure = checkColdGasThrustPressure();
    printf("�����ƽ�ѹ�������: %d\n", coldGasThrustPressure);

    // ��Ԫ�ƽ�ѹ�����
    bool unitThrustPressure = checkUnitThrustPressure();
    printf("��Ԫ�ƽ�ѹ�������: %d\n", unitThrustPressure);

    // �ŵ翪�ؼ��
    bool dischargeSwitch = checkDischargeSwitch();
    printf("�ŵ翪�ؼ����: %d\n", dischargeSwitch);


    // ����Ƿ����κ�һ����ʧ��
    if (!starSensorCommunication || !gnssCommunication || !inertialNavigationCommunication || !communicationCommunication ||
        !observationAimingComponentCommunication || !temperatureControlTemperature || !centralControlTelemetry ||
        !coldGasThrustPressure || !unitThrustPressure || !dischargeSwitch) {
        printf("���ʧ�ܣ��ȴ�������봦��\n");
					
				PTmode_Status = false;
				
        return false;  // ���� false ��ʾ�Լ�ʧ��
    }
    else{
			  GNCmode_Finished = true;
			  GNCmode_Status = true;
			  PTmode_Status = true;
				PTmode_Finished = true; // ����Ӧ����Ҫ�ж�һ�£�����ģʽ״̬��־��GNCģʽ״̬��־��GNCģʽ��ɱ�־���Ƿ���ɺ�����

        mode = STORAGE_MODE; // �л�ģʽΪ�洢����
        printf("�Լ�ͨ��\n");
			return true;
    }
    // �����Լ�״̬ ?
}

// װ����Ϣ�洢
bool storeBindingInfo() {
    // �洢װ����Ϣ���߼�
    printf("װ����Ϣ�洢\n");
    return true; // ����洢�ɹ�
}

bool checkBindingInfo() {
    // ʵ���߼�
    return true;
}

bool validateBaselineInfo() {
    // ʵ���߼�
    return true;
}

bool storeBaselineInfo() {
    // ʵ���߼�
    return true;
}

bool checkInternalPowerCommand() {
    // ʵ���߼�
    return true;
}

bool openDischargeSwitch() {
    // ʵ���߼�
		FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
    return true;
}

void startTimer() {
    // ʵ���߼�
}

bool waitForInternalPowerCommand() {
    // ʵ���߼�
    return true;
}

void sendCommand(unsigned int command) {
    // ʵ���߼�
}

bool checkFaultCode() {
    // ʵ���߼�
    return false;
}

void setFaultStatus(bool status) {
    // ʵ���߼�
}

unsigned int checkTimer() {
    // ʵ���߼�
    return 0;
}

bool checkSeparationSwitch() {
    // ʵ���߼�
    return true;
}

bool validateBindingInfo() {
    // ���ʵ�ʵ�У���߼�
    return true; // ����У��ͨ��
}



// ��׼��Ϣ���գ���У�飬�ɹ���洢
bool receiveBaselineInfo() {
    // ģ����ջ�׼��Ϣ
    bool receivedBaselineInfo = true; // ������յ���׼��Ϣ
    if (receivedBaselineInfo) {
        printf("���յ���׼��Ϣ\n");
        // ����У��
        bool baselineInfoValid = validateBaselineInfo();
        if (baselineInfoValid) {
            printf("��׼��ϢУ��ͨ��\n");
            // �洢��׼��Ϣ
            bool baselineInfoStored = storeBaselineInfo();
            if (baselineInfoStored) {
                printf("��׼��Ϣ�洢�ɹ�\n");
                mode = STATE_ESTABLISHMENT_MODE; // �л�ģʽΪ״̬��������
            } else {
                printf("��׼��Ϣ�洢ʧ��\n");
                // ����洢ʧ�����
            }
        } else {
            printf("��׼��ϢУ��ʧ��\n");
            // ����У��ʧ�����
        }
    } else {
        printf("�ȴ���׼��Ϣ\n");
    }
    return receivedBaselineInfo;
}

//Ԥ��ģʽ
void preheatMode() {
    // ִ��Ԥ�Ȳ���
    printf("ִ��Ԥ�Ȳ���\n");
			// ��ʼ����־����
		initializeFlags();
		//����Ԥ��
		temperature_control();
	
	// ����Ԥ��״̬��־
    bool bindingInfoValid = false;
    bool preheatCompleted = false;
		
		GNCmode_Finished = true;  // GNCģʽ��ɱ�־

    // �ȴ�װ����Ϣ
    // ����յ�װ����Ϣ��������ϢУ��
    bool receivedBindingInfo = checkBindingInfo();
    if (receivedBindingInfo) {
        printf("�յ�װ����Ϣ\n");
        // ������ϢУ��
        bindingInfoValid = validateBindingInfo();
        if (bindingInfoValid) {
            printf("װ����ϢУ��ͨ��\n");

            // ��Ԥ��ģʽ�н���װ����Ϣ�洢
                if (storeBindingInfo()) {
                    printf("װ����Ϣ�洢�ɹ�\n");
                } else {
                    printf("װ����Ϣ�洢ʧ��\n");
                    // ����洢ʧ�����
                }

        } else {
            printf("װ����ϢУ��ʧ��\n");
            // ����У��ʧ�����
        }
    } 
    else
    {
        printf("�ȴ�װ����Ϣ\n");
    }

    // ��׼��Ϣ���գ���У�飬�ɹ���洢
    if (receiveBaselineInfo()) {
        printf("��׼��Ϣ���ճɹ�\n");
    } else {
        printf("��׼��Ϣ����ʧ��\n");
    }

    // ���ա�ת�ڵ硱ָ��
    if (checkInternalPowerCommand()) {
        printf("ת�ڵ�ָ�����\n");
        // ���װ����Ϣ��Ԥ��״̬
        if (receivedBindingInfo && preheatCompleted) {
            // �����طŵ翪��
            if (openDischargeSwitch()) {
                printf("���طŵ翪���Ѵ�\n");
                // ��ʼ��ʱ
                startTimer();
            } else {
                printf("���طŵ翪�ش�ʧ��\n");
                // �����ش�ʧ�����
            }
        } else {
            printf("װ����Ϣ��Ԥ��δ��ɣ��޷������طŵ翪��\n");
            // ����װ����Ϣ��Ԥ��δ������
        }
    }

    // �ȴ���ת�ڵ�ɹ���ָ��
		bool internalPowerCommandSuccess = waitForInternalPowerCommand();
    if (internalPowerCommandSuccess) {
        printf("ת�ڵ�ɹ�\n");
        sendCommand(0xC9C9); // ����ָ�����Ϊ0xC9C9

    } else {
        printf("ת�ڵ�ʧ��\n");
        sendCommand(0X9C9C); // ����ָ�����Ϊ0xC9C9
    }

    // ���ݹ��������ޣ�����״̬��־
    if (checkFaultCode()) {
        printf("��⵽������\n");
        // ���ù��ϱ�־
        setFaultStatus(true);
			
				PTmode_Status = false;      // ����ģʽ״̬��־
    } else {
        printf("δ��⵽������\n");
        // ������ϱ�־
        setFaultStatus(false);
				
    }

    // ���װ����Ϣ��Ԥ����ɺ�ת�ڵ����״̬
    if (receivedBindingInfo && preheatCompleted && internalPowerCommandSuccess) {
        // ������ɱ�־
        modeFinished = true; 
			
				PTmode_Finished = true;   // ����ģʽ��ɱ�־
    }

    // ����ʱ�Ƿ�ﵽ70��
    if (checkTimer() >= 70) {
        // �����뿪��״̬
        bool separationSwitchStatus = checkSeparationSwitch();
        if (separationSwitchStatus) {
            printf("���뿪��δ�Ͽ��������ȴ�\n");
        } else {
            printf("���뿪���ѶϿ�������״̬����ģʽ\n");
            mode = STATE_ESTABLISHMENT_MODE; // �л�ģʽΪ״̬��������
        }
    }
}

void GNSSCapture() {
    // ��ʱ��ռλ��ʵ��
}

bool accelerometerCalibration() {
    // ��ʱ��ռλ��ʵ��
    return true;
}

bool establishGroundOrientation() {
    // ��ʱ��ռλ��ʵ��
    return true;
}

void establishInterCommunication() {
    // ��ʱ��ռλ��ʵ��
}

void exhaustUnitThrustPipeline() {
    // ��ʱ��ռλ��ʵ��
}

void powerOnCommunication() {
    // ��ʱ��ռλ��ʵ��
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0355;
}

void powerOnControl() {
    // ��ʱ��ռλ��ʵ��
		FPGA_REG_RW_2BYTE(XDC_OPEN_ADDR) = 0x3333;
}

void powerOnGNSS() {
    // ��ʱ��ռλ��ʵ��
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0755;
}

void powerOnInertialNavigation() {
    // ��ʱ��ռλ��ʵ��
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0555;	
}

void powerOnStarSensor() {
    // ��ʱ��ռλ��ʵ��
		FPGA_REG_RW_2BYTE(LEVEL_CMD_ADDR) = 0x0455;
}

void prePulse() {
    // ��ʱ��ռλ��ʵ��
}

void sendImportantTelemetry() {
    // ��ʱ��ռλ��ʵ��
}

void spinStabilizationControl() {
    // ��ʱ��ռλ��ʵ��
}

void starSensorCapture() {
    // ��ʱ��ռλ��ʵ��
}

// �����ж�-- �����룬��ɱ�־


// ״̬����ģʽ
void stateEstablishmentMode(){
	
		int i = 0; // ��ʼ��i
		OS_ERR err;
		// ��ʼ����־����
    initializeFlags();
	
    // �ϵ��������� ���ɣ�
    powerOnControl();//һֱ�е�
    powerOnStarSensor();
    powerOnCommunication();
    powerOnInertialNavigation();
    powerOnGNSS();
    
    // 10S��ֹ�˿� --�����˿صı�־
    printf("��ֹ�˿�: %d\n", i+1);
//    sleep(10);
	  OSTimeDly(10000, OS_OPT_TIME_DLY, &err);

    // 20S��ֹ�������������ȿص�Ԫ�����ȿ�
    printf("��ֹ�������������ȿص�Ԫ�����ȿ�: %d\n", i+1);
//    sleep(20);
		OSTimeDly(20000, OS_OPT_TIME_DLY, &err);


    exhaustUnitThrustPipeline();
    prePulse();
    spinStabilizationControl();
    starSensorCapture();
    GNSSCapture();
    accelerometerCalibration();
    establishGroundOrientation();

    //�����Ǽ�ͨ��״̬
    establishInterCommunication();
    //������Ҫң��
    sendImportantTelemetry();//��Ҫ������ң���
		
		

   // ��״̬����ģʽ�д��������
    
    // �����봦�� -- �����жϼ��� ����
    unsigned int faultFlags = 0;
    unsigned int faultsMask = (FAULT_ORBIT_DETERMINATION_FAILURE |
                               FAULT_ATTITUDE_CONTROL_ANOMALY |
                               FAULT_GNC_MIN_SYSTEM_NOT_MET |
                               FAULT_GNC_MODE_ANOMALY |
                               FAULT_STAR_SENSOR_COMMUNICATION |
                               FAULT_IMU_COMMUNICATION |
                               FAULT_GNSS_COMMUNICATION |
                               FAULT_COMMUNICATION_DEVICE); 
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
		
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        PTmode_Status = false;
        mode = STANDBY_MODE;
        return;
    }
		else{
        PTmode_Status = true;
    }


    //ģʽ��ɱ�־����
    //���û�й����롢���ٶȼ���ɱ궨��GNC�Եض�����ɣ�ģʽ�������Ϊ1
    if(PTmode_Status && accelerometerCalibration() && establishGroundOrientation()){
        PTmode_Finished = true; 
    }

    //ģʽ��ɱ�־Ϊ1�������ʼ�Ƶ�ģʽ
    if(PTmode_Finished){
        mode = INITIAL_GUIDANCE_MODE;
    }
    return;
}

bool GNCCompleted() {
    // ģ��GNC���״̬
	  GNCmode_Finished = true;  // GNCģʽ��ɱ�־
    return GNCmode_Finished;  // ����GNC���
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

// �����ʼ�Ƶ�ģʽ�Ĵ���
void initialGuidanceMode(){
	
		// ��ʼ����־����
    initializeFlags();
    // �����Ǽ�ͨ�ţ�TODO��

    // �ɼ�-�����ɼ�-���ɼ���ͨ�ſ��ܶϿ� =������
    // ����װ����Ŀ��Ԥ��������������Ŀ�����������Ƶ��滮����
    // ��������ǰ�������ϵ�Ŀ��������չ�Ƶ��滮����
    // ��չ���������ÿ�ι������ǰ������̬���������������ٶ��������򣬹������������̬�������Եض������ʱ��45MIN
    // ����Ŀ���5KM���Եض��򱣳�

    // �����봦��
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
    
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        PTmode_Status = false;      // ����ģʽ״̬��־
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // ����ģʽ״̬��־
    }

    // ״̬��־����״̬���
    // �ж�״̬��־=��������GNC���=�����
    if(PTmode_Status && GNCCompleted()){
        PTmode_Finished = true;   // ����ģʽ��ɱ�־
    }

    // ģʽ���
    if(PTmode_Finished){
        mode = MIDDLE_GUIDANCE_ESTABLISHMENT_MODE;
    } 

}

/**
 * @brief �м��Ƶ�����ģʽ
 * 
 * ���м��Ƶ�����ģʽ�£�����Ŀ�궨λ������ݵ��Ƶ�����͹��ϼ�顣
 * �����⵽���ϣ����������ģʽ��
 * ����Ƶ�����͹��ϼ�鶼ͨ�����������м��Ƶ�ģʽ��
 */

void middleGuidanceEstablishmentMode(){
    
	// ��ʼ����־����
    initializeFlags();
	
    // TODO: 600S����
    if (g_middleEstablishStartTime == 0) {
        g_middleEstablishStartTime = time(NULL);
    }


    // ÿ����1S������Ŀ�궨λ������ݿ�չ�Ƶ�����

    // �ж�Ŀ�궨λ������ݵ�Ŀ�������ñ�־��������ã������յ�Ŀ������������ǰʱ�̣�������Ŀ��ο����
    // ���Ŀ���������ã�ȡ��һʱ�̸��µ�Ŀ��ο��������������ǰʱ�̣�������Ŀ��ο����

    // �����봦��
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

    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        PTmode_Status = false;
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // ����ģʽ״̬��־
    }

    // ״̬��־����
    // �ж�״̬��־=��������GNC���=����ɣ�������������Ƶ�״̬�����ɹ�
    if(PTmode_Status && GNCCompleted() && trackDataEstablished()){
        PTmode_Finished = false;   // ����ģʽ��ɱ�־
    }

    // ģʽ���
    if(PTmode_Finished){
        mode = MIDDLE_GUIDANCE_MODE;
    }
    return;
}

void middleGuidanceMode(){
    // ��ʼ����־����
    initializeFlags();
    
    // �����Ǽ�ͨ��

    // ����Ŀ�궨λ������ݣ�������ɵݽ��ƻ��Ϳ���

    // �����봦��
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
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        PTmode_Status = false;      // ����ģʽ״̬��־
        mode = STANDBY_MODE;
        return;
    }
    else{
        PTmode_Status = true;      // ����ģʽ״̬��־
    }

    // ״̬��־����
    // �ж�״̬��־=��������GNC���=����ɣ�������������Ƶ�״̬�����ɹ�
    if(PTmode_Status && GNCCompleted()){
        PTmode_Finished = true;   // ����ģʽ��ɱ�־
    }

    // ģʽ���
    if(PTmode_Finished){
        mode = MIDDLE_FINAL_HANDOVER_MODE;
    }
    return;
}


void middleFinalHandoverMode(){
    // �����Ǽ�ͨ��
    // ������������Ͳ���
    powerOnObservationAimingCamera();
    powerOnRangeFinder();

    // ��̬תΪ��Ŀ�궨��

    // �����������Ͳ����������쳣�����������+��ࡱ�㷨�����Ŀ�������Ϣ
    if(!checkObservationAimingCamera() && !checkRangeFinder()){
        // ���Ŀ�������Ϣ
        outputRelativeInfo();
    }

    // �����봦��
    
    // �����봦��
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // ��������Ҫ������й���
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // ״̬��־����
    // �ж�״̬��־=��������GNC���=����ɣ���Ե���=������
    if(!checkFaultCode() && GNCCompleted() && relativeNavigationConverged()){
        modeFinished = 1;
    }

    // ģʽ���
    if(modeFinished){
        mode = FINAL_GUIDANCE_MODE;
    }
    return;
}

void finalGuidanceMode(){
    // �����Ǽ�ͨ��

    // ���ͨ���жϣ������ж�

    // ÿ���������ڣ��жϡ����ǲ��������Ч��ʶ���͡����ǲ��������Ч��ʶ�����������Ч��C-W��������Ŀ�궨λ
    if(checkObservationAimingCamera() && checkRangeFinder()){
        // C-W��������Ŀ�궨λ
        CWEquation();
        // ������Ŀ����Թ��
        updateRelativeTrack();

    }
    else {
        // ȡ��һʱ�̸��µ�Ŀ����Թ��
        // ����C-W������������ǰʱ��
        // ������Ŀ����Թ��

    }

    // ����Ŀ����Թ����������չĩ�Ƶ��滮����
    // ���ڹ���������ݣ��������Ŀ��ʶ��


    // �����봦��
    
    // �����봦��
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // ��������Ҫ������й���
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // ״̬��־����
    // �ж�״̬��־=��������GNC���=����ɣ�������������Ƶ�״̬�����ɹ�
    if(!checkFaultCode() && GNCCompleted()){
        modeFinished = 1;
    }

    // ģʽ���
    if(modeFinished){
        mode = HOVER_MODE;
    }
    return;
}

bool fireControlOnline() {
    // ģ��������״̬
    return true; // ����������
}

bool isTaskEnabled() {
    // ģ�������Ƿ�ʹ��
    return true; // ����������ʹ��
}

bool minsys() {
    // ģ����Сϵͳ����
    return true; // ������Сϵͳ����������
}


void hoverMode(){
    // �����Ǽ�ͨ��
    // ͨ���жϣ���ͣ�����ж�

    // ����Ŀ���ӳ�����

    // ��������ٶȺ�λ��

    // �ж�ʹ�ܱ�־Ϊ�����������롰�غɹ�������ģʽ
    if(isTaskEnabled())
    {
        // ���ʹ��ָ�1S3�Σ�

        // �жϻ�Ʒ���߱�־����
        if(fireControlOnline())
        {
            // �ȴ�12S

            // ����������ɱ�־

            // �������30S���һ����߱�־ != �𱬣����������־Ϊ����ɡ�

        }
    }
    else{
        modeFinished = 1;
    }


    // �����봦��
    
    // �����봦��
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // ��������Ҫ������й���
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        
        mode = STANDBY_MODE;
        return;
    }


    // ״̬��־����
    // �ж�״̬��־=��������GNC���=����ɣ�ս�����
    if(!checkFaultCode() && GNCCompleted() &&checkFightFinished() ){
        modeFinished = 1;
    }

    // ��ͣģʽ���л�����TODO��

    // 

    // ģʽ���
    if(modeFinished){
        mode = IMAGE_TRANSMISSION_MODE;
    }
    return;
}

void imageTransmissionMode(){
    // ��̬תΪ�Եض��򣬽��͹���߶�Զ��Ŀ��

    // �Զ�ִ�С�ң�����ݻش��� ����ͼ��ִ��1��
    telemetryDataTransmission();

    // ������������������鴦������ػ�
    powerOffObservationCamera();
    powerOffRangeFinder();
    powerOffObservationComponent();

    // �����봦��
    
    // �����봦��
    unsigned int faultFlags = 0;
    unsigned int faultsMask = 0xFFFFFFFF; // ��������Ҫ������й���
    // ���������е��ù��ϼ�麯��
    checkFaults(&faultFlags, faultsMask);
    // ��������
    if (faultFlags != 0) {
        printf("��⵽���ϣ��������ģʽ\n");
        
        mode = STANDBY_MODE;
        return;
    }

    // ״̬��־����
    if(!checkFaultCode() && GNCCompleted() ){
        modeFinished = 1;
    }

    // ģʽͣ���ش�

}

// ���ֹ�������������ģʽ
// ĸ��ָ������ģʽ
void standbyMode(){
    //���ֵ���ԭ��״̬

    // ���ֶԵض�����̬

    // ����ͨ�ţ�ִ�С�ң�����ݻش���ָ��

    // �ȴ�ָ��

    // �Ƿ�������Сϵͳ������������ֹͣ��̬����
    if(minsys()){

    }
    else{

    }

    // ״̬��־����
    if(!checkFaultCode() && GNCCompleted() ){
        modeFinished = 1;
    }

}

//�����˿ص�����ź�

void processMode(Mode mode){
    switch (mode) {
        case STORAGE_MODE:
            storage_mode();
            break;
				case SELF_TEST_MODE:
            selfTestMode();
            break;
				case PREHEAT_MODE:
            // ����Ԥ��ģʽ�Ĵ���
            preheatMode();
            break;
        case STATE_ESTABLISHMENT_MODE:
            // ����״̬����ģʽ�Ĵ���
            stateEstablishmentMode();
            break;
        case INITIAL_GUIDANCE_MODE:
            // �����ʼ�Ƶ�ģʽ�Ĵ���
            initialGuidanceMode();
            break;
        case MIDDLE_GUIDANCE_ESTABLISHMENT_MODE:
            // �����м��Ƶ�����ģʽ�Ĵ���
            middleGuidanceEstablishmentMode();
            break;
        case MIDDLE_GUIDANCE_MODE:
            // �����м��Ƶ�ģʽ�Ĵ���
            middleGuidanceMode();
            break;
        case MIDDLE_FINAL_HANDOVER_MODE:
            // ������ĩ����ģʽ�Ĵ���
            middleFinalHandoverMode();
            break;
        case FINAL_GUIDANCE_MODE:
            // ���������Ƶ�ģʽ�Ĵ���
            finalGuidanceMode();
            break;
        case HOVER_MODE:
            // ������ͣģʽ�Ĵ���
            hoverMode();
            break;
        case IMAGE_TRANSMISSION_MODE:
            // ����ͼ����ģʽ�Ĵ���
            imageTransmissionMode();
            break;
        case STANDBY_MODE:
            // �������ģʽ�Ĵ���
            standbyMode();
            break;
        default:
            break;
    }
}



