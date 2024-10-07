#ifndef PROGRAM_CONTROL_H
#define PROGRAM_CONTROL_H

#include <stdbool.h>

/*
�����װ�������쳣
����ʧ��
�˿��쳣
ͨ�Ŷ���
���ң��֡У��ʧ��
������GNC������Сϵͳ
GNCģʽ�쳣
GNC���λ�ÿ��Ƴ���
��������ģʽ��ʱδ���
GNC�涨ʱ����δ�����Ե����˲�
��Ե����˲���ɢ
���ϵ������ݲ�����
����ͨ�Ź���
����ͨ�Ź���
GNSS ͨ�Ź���
ͨ�Ż�ͨ�Ź���
���鴦��ͨ�Ź���
�����ʧĿ��
���������ʧĿ��
�غɿ���ʧ��
��ս��ʱ
    */
// ���ϱ�־λ
#define FAULT_BINDING_DATA_ERROR               (1 << 0)
#define FAULT_ORBIT_DETERMINATION_FAILURE      (1 << 1)
#define FAULT_ATTITUDE_CONTROL_ANOMALY         (1 << 2)
#define FAULT_COMMUNICATION_LINK_LOSS          (1 << 3)
#define FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL  (1 << 4)
#define FAULT_GNC_MIN_SYSTEM_NOT_MET           (1 << 5)
#define FAULT_GNC_MODE_ANOMALY                 (1 << 6)
#define FAULT_GNC_RELATIVE_POSITION_CONTROL    (1 << 7)
#define FAULT_SUB_STAR_TASK_TIMEOUT            (1 << 8)
#define FAULT_GNC_RELATIVE_NAV_TIMEOUT         (1 << 9)
#define FAULT_RELATIVE_NAV_DIVERGENCE          (1 << 10)
#define FAULT_COMBINED_NAV_DATA_UNAVAILABLE    (1 << 11)
#define FAULT_STAR_SENSOR_COMMUNICATION        (1 << 12)
#define FAULT_IMU_COMMUNICATION                (1 << 13)
#define FAULT_GNSS_COMMUNICATION               (1 << 14)
#define FAULT_COMMUNICATION_DEVICE             (1 << 15)
#define FAULT_OBSERVATION_PROCESSING           (1 << 16)
#define FAULT_CAMERA_TARGET_LOSS               (1 << 17)
#define FAULT_LASER_RANGEFINDER_TARGET_LOSS    (1 << 18)
#define FAULT_PAYLOAD_COVER_FAILURE            (1 << 19)
#define FAULT_COMBAT_TIMEOUT                   (1 << 20)

/*
�����װ�������쳣
����ʧ��
�˿��쳣
ͨ�Ŷ���
���ң��֡У��ʧ��
������GNC������Сϵͳ
GNCģʽ�쳣
GNC���λ�ÿ��Ƴ���
��������ģʽ��ʱδ���
GNC�涨ʱ����δ�����Ե����˲�
��Ե����˲���ɢ
���ϵ������ݲ�����
����ͨ�Ź���
����ͨ�Ź���
GNSS ͨ�Ź���
ͨ�Ż�ͨ�Ź���
���鴦��ͨ�Ź���
�����ʧĿ��
���������ʧĿ��
�غɿ���ʧ��
��ս��ʱ
    */
// ���ϱ�־λ
#define FAULT_BINDING_DATA_ERROR               (1 << 0)
#define FAULT_ORBIT_DETERMINATION_FAILURE      (1 << 1)
#define FAULT_ATTITUDE_CONTROL_ANOMALY         (1 << 2)
#define FAULT_COMMUNICATION_LINK_LOSS          (1 << 3)
#define FAULT_REMOTE_CONTROL_FRAME_CHECK_FAIL  (1 << 4)
#define FAULT_GNC_MIN_SYSTEM_NOT_MET           (1 << 5)
#define FAULT_GNC_MODE_ANOMALY                 (1 << 6)
#define FAULT_GNC_RELATIVE_POSITION_CONTROL    (1 << 7)
#define FAULT_SUB_STAR_TASK_TIMEOUT            (1 << 8)
#define FAULT_GNC_RELATIVE_NAV_TIMEOUT         (1 << 9)
#define FAULT_RELATIVE_NAV_DIVERGENCE          (1 << 10)
#define FAULT_COMBINED_NAV_DATA_UNAVAILABLE    (1 << 11)
#define FAULT_STAR_SENSOR_COMMUNICATION        (1 << 12)
#define FAULT_IMU_COMMUNICATION                (1 << 13)
#define FAULT_GNSS_COMMUNICATION               (1 << 14)
#define FAULT_COMMUNICATION_DEVICE             (1 << 15)
#define FAULT_OBSERVATION_PROCESSING           (1 << 16)
#define FAULT_CAMERA_TARGET_LOSS               (1 << 17)
#define FAULT_LASER_RANGEFINDER_TARGET_LOSS    (1 << 18)
#define FAULT_PAYLOAD_COVER_FAILURE            (1 << 19)
#define FAULT_COMBAT_TIMEOUT                   (1 << 20)


// ģʽ����
typedef enum {
    STORAGE_MODE,
    SELF_TEST_MODE,
    PREHEAT_MODE,
    STATE_ESTABLISHMENT_MODE,
    INITIAL_GUIDANCE_MODE,
    MIDDLE_GUIDANCE_ESTABLISHMENT_MODE,
    MIDDLE_GUIDANCE_MODE,
    MIDDLE_FINAL_HANDOVER_MODE,
    FINAL_GUIDANCE_MODE,
    HOVER_MODE,
    IMAGE_TRANSMISSION_MODE,
    STANDBY_MODE
} Mode;

unsigned int getCurrentTime(void);  // ��ȷ��������
double difftime(unsigned int time1, unsigned int time2);  // difftime ��������
void GNSSCapture(void);
bool accelerometerCalibration(void);
void checkFaults(unsigned int *faultFlags, unsigned int faultsMask);
bool establishGroundOrientation(void);
void establishInterCommunication(void);
void exhaustUnitThrustPipeline(void);
void powerOnCommunication(void);
void powerOnControl(void);
void powerOnGNSS(void);
void powerOnInertialNavigation(void);
void powerOnStarSensor(void);
void prePulse(void);
void sendImportantTelemetry(void);
void spinStabilizationControl(void);
void starSensorCapture(void);







#endif
