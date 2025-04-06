/********************************** (C) COPYRIGHT *******************************
 * File Name          : broadcaster.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : �㲥Ӧ�ó��򣬳�ʼ���㲥���Ӳ�����Ȼ���ڹ㲥̬һֱ�㲥

 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CONFIG.h"
#include "devinfoservice.h"
#include "broadcaster.h"
#include "app_i2c.h"

// �㲥��� (units of 625us, min is 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL 1600 * 2
// �ɼ����
#define SBP_PERIODIC_EVT_PERIOD 1600 * 20

// ADC �����ֵ�ƫ��ֵ
static signed short RoughCalib_Value = 0;
// ��ص�ѹ
static uint16_t bat = 0;

// Task ID for internal task/event processing
static uint8_t Broadcaster_TaskID;

// �㲥���� (����UUID�Ͳ�������)
static uint8_t advertData[] = {
    0x02, GAP_ADTYPE_FLAGS, GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,//0-2
    0x07, 0x16, 0xD2, 0xFC, // ���ȡ�AD���͡�UUID (BTHome UUID FCD2) 3-6
    0x40,                   // BTHome v2 �޼��ܣ����ڹ㲥 7
    0x01, 0x00,             // ���� (ռλ��) 8-9
    0x04, 0x00              // B14��״̬ (0x04���Զ�������) 10-11
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Broadcaster_ProcessTMOSMsg(tmos_event_hdr_t* pMsg);
static void Broadcaster_StateNotificationCB(gapRole_States_t newState);
extern bStatus_t GAP_UpdateAdvertisingData(uint8_t taskID, uint8_t adType, uint16_t dataLen, uint8_t* pAdvertData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    Broadcaster_StateNotificationCB, // Profile State Change Callbacks
    NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

// ��ص�ѹ����
__HIGH_CODE
uint16_t sample_battery_voltage()
{
    // VINA ʵ�ʵ�ѹֵ 1050��15mV
    const int vref = 1050;

    ADC_InterBATSampInit();

    // ÿ200�ν���һ�δֵ�У׼
    static uint8_t calib_count = 0;
    calib_count++;
    if (calib_count == 1) {
        RoughCalib_Value = ADC_DataCalib_Rough();
    }
    calib_count %= 200;

    ADC_ChannelCfg(CH_INTE_VBAT);
    return (ADC_ExcutSingleConver() + RoughCalib_Value) * vref / 512 - 3 * vref;
}

// SHT4x ��ȡ��ʪ��
// ģʽ: low    med    high
// ʪ��: 0.25%  0.15%  0.08%
// �¶�: 0.1C   0.07C  0.04C
// ��ʱ: 1.6ms  4.5ms  8.3ms
// CMD: 0xE0   0xF6   0xFD
__HIGH_CODE
int read_sht4x_data(float* temperature, float* humidity)
{
    i2c_app_init(0x01);
    const uint8_t cmd = 0xE0;
    int ret = i2c_write_to(0x44, &cmd, sizeof(cmd), true, true);
    DelayMs(2);
    uint8_t rx_bytes[6];
    ret = i2c_read_from(0x44, rx_bytes, 6, true, 100);
    if (ret != 6) return 0;

    uint32_t temp_raw = ((uint32_t)rx_bytes[0] << 8) | rx_bytes[1];
    uint32_t humid_raw = ((uint32_t)rx_bytes[3] << 8) | rx_bytes[4];

    float temp_f = -45.0 + 175.0 * temp_raw / 65535.0;
    float humid_f = -6.0 + 125.0 * humid_raw / 65535.0;
    humid_f = (humid_f > 100.0f) ? 100.0f : humid_f;
    humid_f = (humid_f < 0.0f) ? 0.0f : humid_f;

    *temperature = temp_f;
    *humidity = humid_f;

    return 1;
}


//void update_advert_data()
//{
//    // �㲥�豸������ advertData ��λ��
//    const uint8_t name_index = 5;
//    // �㲥�豸���Ƴ���
//    const uint8_t name_len = sizeof(advertData) - name_index;
//
//    // ��չ㲥�豸����
//    memset(advertData + name_index, 0x00, name_len);
//
//    // ��ȡ��ʪ��
//    float temp, humid;
//    int sht4x_ret = read_sht4x_data(&temp, &humid);
//
//    // ��ȡ��ص�ѹ
//    bat = sample_battery_voltage();
//
//    // ���¹㲥�豸����
//    if (sht4x_ret) {
//        snprintf(advertData + name_index, name_len, "%.2fV/%.2fC/%.2f%%", bat / 1000.0f, temp, humid);
//    } else {
//        snprintf(advertData + name_index, name_len, "%.2fV/ERR/ERR", bat / 1000.0f);
//    }
//}


__HIGH_CODE
void update_advert_data() {
    uint8_t BL_bat;
    
    // ����B14Ϊ����ģʽ
    GPIOB_ModeCfg(GPIO_Pin_14, GPIO_ModeIN_PU);
    
    // ��ȡB14״̬
    uint8_t b14_state = GPIOB_ReadPortPin(GPIO_Pin_14);

    // ��ȡ��ص�ѹ
    bat = sample_battery_voltage();
    BL_bat = (uint8_t)((bat-2500)/8);
    if (BL_bat > 100)
        BL_bat = 100;
        
    // ���¹㲥���еĵ�������
    advertData[9] = BL_bat; // �����ٷֱ�

    // ����B14״̬
    advertData[11] = b14_state ? 1 : 0;
}



/*********************************************************************
 * @fn      Broadcaster_Init
 *
 * @brief   Initialization function for the Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Broadcaster_Init()
{
    Broadcaster_TaskID = TMOS_ProcessEventRegister(Broadcaster_ProcessEvent);

    // Setup the GAP Broadcaster Role Profile
    {
        // Device starts advertising upon initialization
        uint8_t initial_advertising_enable = TRUE;
        uint8_t initial_adv_event_type = GAP_ADTYPE_ADV_NONCONN_IND;
        // uint8_t initial_adv_event_type = GAP_ADTYPE_ADV_IND;
        // uint8_t initial_adv_event_type = GAP_ADTYPE_EXT_NONCONN_NONSCAN_UNDIRECT;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &initial_adv_event_type);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    }

    // Set advertising interval
    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);

        // GAP_SetParamValue(TGAP_ADV_SECONDARY_PHY, GAP_PHY_VAL_LE_CODED); // 125K
        // GAP_SetParamValue(TGAP_ADV_PRIMARY_PHY, GAP_PHY_VAL_LE_CODED); // 125K
    }

    // Setup a delayed profile startup
    // tmos_set_event(Broadcaster_TaskID, SBP_START_DEVICE_EVT);
    tmos_start_task(Broadcaster_TaskID, SBP_START_DEVICE_EVT, DEFAULT_ADVERTISING_INTERVAL);

    // ���ö�ʱ��ȡ�����������¹㲥
    tmos_start_task(Broadcaster_TaskID, SBP_PERIODIC_EVT, 2 * DEFAULT_ADVERTISING_INTERVAL - 320);
}

/*********************************************************************
 * @fn      Broadcaster_ProcessEvent
 *
 * @brief   Broadcaster Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Broadcaster_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if (events & SYS_EVENT_MSG) {
        uint8_t* pMsg;

        if ((pMsg = tmos_msg_receive(Broadcaster_TaskID)) != NULL) {
            Broadcaster_ProcessTMOSMsg((tmos_event_hdr_t*)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & SBP_START_DEVICE_EVT) {
        // Start the Device
        GAPRole_BroadcasterStartDevice(&Broadcaster_BroadcasterCBs);

        return (events ^ SBP_START_DEVICE_EVT);
    }

    if (events & SBP_PERIODIC_EVT) {
        tmos_start_task(Broadcaster_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);

        // ���ݲɼ������¹㲥
        update_advert_data();
        GAP_UpdateAdvertisingData(0, TRUE, sizeof(advertData), advertData);

        return (events ^ SBP_PERIODIC_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Broadcaster_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Broadcaster_ProcessTMOSMsg(tmos_event_hdr_t* pMsg)
{
    switch (pMsg->event) {
    default:
        break;
    }
}

/*********************************************************************
 * @fn      Broadcaster_StateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void Broadcaster_StateNotificationCB(gapRole_States_t newState)
{
    switch (newState) {
    case GAPROLE_STARTED:
        PRINT("Initialized..\n");
        break;

    case GAPROLE_ADVERTISING:
        PRINT("Advertising..\n");
        break;

    case GAPROLE_WAITING:
        PRINT("Waiting for advertising..\n");
        break;

    case GAPROLE_ERROR:
        PRINT("Error..\n");
        break;

    default:
        break;
    }
}
