#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/UART.h>
#include "board.h"
#include "delay.h"
#include "blink.h"
#include "LSM6DS3.h"
#include "Battery.h"
#include "smart_shirt.h"
#include "app2bleBridge.h"
#include "MadgwickAHRS.h"

#define BLINK_FAST_ON_INTERVAL_MS       (5)
#define BLINK_FAST_OFF_INTERVAL_MS      (295)

#define BLINK_SLOW_ON_INTERVAL_MS       (5)
#define BLINK_SLOW_OFF_INTERVAL_MS      (2995)

#define BUTTON_HOLD_POWER_OFF_TIME_MS   (3000)

#define PERIODIC_DATA_UPDATE_INTERVAL_MS    (5000)
#define BLE_CONNECTION_TIMEOUT_MS           (30 * 1000)

// Clocks
static Clock_Handle user_connTimeoutClock;
static Clock_Handle user_dataUpdateClock;
static Clock_Handle user_lsmDataUpdateClock;
static Clock_Handle user_powerOffClock;


static LSM6DS3 lsm;
static Battery  batt;
static Blink blink(user_Led);
static bool bleConnected = false;

#if (USE_MADGWICK_AHRS != 0)
static Madgwick filter;
#endif

static uint32_t odrMs = 0;
static uint32_t lsmTimeout = 0;

// Task configuration
#define USER_TASK_STACK_SIZE    (1024)
#define USER_TASK_PRIORITY      (2)
Task_Struct user_task;
Char user_taskStack[USER_TASK_STACK_SIZE];


Event_Handle userEvent;

static uint32_t timestamp = 0;

static bool lsmStarted = false;
static bool lsmDisableInterrupt = false;

static UART_Handle uart = NULL;

static uint32_t imuSkipValues = 0;

#define USER_EVENT_LSM_INTERRUPT        (Event_Id_00)
#define USER_EVENT_LSM_DATA_UPDATE      (Event_Id_01)
#define USER_EVENT_BUTTON_INTERRUPT     (Event_Id_02)
#define USER_EVENT_BLE_ADVERTISING      (Event_Id_03)
#define USER_EVENT_BLE_CONNECTED        (Event_Id_04)
#define USER_EVENT_BLE_DISCONNECTED     (Event_Id_05)
#define USER_EVENT_POWER_OFF            (Event_Id_06)
#define USER_EVENT_CHARGE_ON            (Event_Id_07)
#define USER_EVENT_CHARGE_OFF           (Event_Id_08)


#define USER_EVENTS  (  USER_EVENT_LSM_INTERRUPT | \
                        USER_EVENT_LSM_DATA_UPDATE | \
                        USER_EVENT_BUTTON_INTERRUPT | \
                        USER_EVENT_BLE_ADVERTISING | \
                        USER_EVENT_BLE_CONNECTED | \
                        USER_EVENT_BLE_DISCONNECTED | \
                        USER_EVENT_POWER_OFF | \
                        USER_EVENT_CHARGE_ON | \
                        USER_EVENT_CHARGE_OFF \
                     )


void user_exportLsmDataToUart(uint8_t *data);


void user_Init()
{
    GPIO_init();

    GPIO_write(BOARD_GPIO_LED, BOARD_GPIO_LED_OFF);
    GPIO_write(BOARD_GPIO_PS_HOLD, BOARD_GPIO_HIGH);
    GPIO_write(BOARD_GPIO_VIBRATOR, BOARD_GPIO_LOW);

    delay(1000); // needed for power on button debounce time (power management)

    // Vibrate on power on
    user_Vibration(true);
    delay(100);
    user_Vibration(false);

    GPIO_setCallback(BOARD_GPIO_BUTTON, user_ButtonCallback);
    GPIO_enableInt(BOARD_GPIO_BUTTON);

    GPIO_setCallback(BOARD_GPIO_IMU_INT1, user_LsmCallback);
    GPIO_enableInt(BOARD_GPIO_IMU_INT1);

    // TODO
    //GPIO_setCallback(BOARD_GPIO_IMU_INT2, user_LsmCallback);
    //GPIO_enableInt(BOARD_GPIO_IMU_INT2);

    GPIO_setCallback(BOARD_GPIO_CHARGE_ON, user_ChargeCallback);
    GPIO_enableInt(BOARD_GPIO_CHARGE_ON);

    bool charge = GPIO_read(BOARD_GPIO_CHARGE_ON);

    bleConnected = false;
    lsmStarted = false;

    batt.begin();

    if (!charge)
    {
        blink.begin();
    }
    else
    {
        user_Led(true);
        Event_post(userEvent, USER_EVENT_CHARGE_ON);
    }

#if (USE_MADGWICK_AHRS != 0)
    filter.begin(LSM6DS3_DEFAULT_ACCEL_SAMPLE_RATE);
#endif

    UART_init();
    UART_Params params;

    UART_Params_init(&params);

    params.baudRate = 115200;

    uart = UART_open(CC2640R2_LAUNCHXL_UART0, &params);

    const char * h = "hello\n";
    UART_write(uart, h, strlen(h));

    // ******************************************************************
    // Initialization of clock objects used for notifiable characterisics
    // ******************************************************************
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);

    clockParams.period = BLE_CONNECTION_TIMEOUT_MS * (1000 / Clock_tickPeriod);
    user_connTimeoutClock = Clock_create(user_connTimeoutClockSwiHandler,
                                         clockParams.period,
                                         &clockParams,
                                         NULL);

    clockParams.period = PERIODIC_DATA_UPDATE_INTERVAL_MS * (1000 / Clock_tickPeriod);
    user_dataUpdateClock = Clock_create(user_dataUpdateClockSwiHandler,
                                        clockParams.period,
                                        &clockParams,
                                        NULL);

    clockParams.period = 5 * (1000 / Clock_tickPeriod);
    user_lsmDataUpdateClock = Clock_create(user_lsmDataUpdateClockSwiHandler,
                                         clockParams.period,
                                         &clockParams,
                                         NULL);


    clockParams.period = BUTTON_HOLD_POWER_OFF_TIME_MS * (1000 / Clock_tickPeriod);
    user_powerOffClock = Clock_create(user_powerOffClockSwiHandler,
                                      clockParams.period,
                                      &clockParams,
                                      NULL);

    userEvent = Event_create(NULL, NULL);

    user_createTask();
}



void user_createTask()
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = user_taskStack;
    taskParams.stackSize = USER_TASK_STACK_SIZE;
    taskParams.priority = USER_TASK_PRIORITY;

    Task_construct(&user_task, user_taskFxn, &taskParams, NULL);
}



void user_taskFxn(UArg a0, UArg a1)
{
    while (1)
    {
        uint32_t events = Event_pend(userEvent, Event_Id_NONE, USER_EVENTS, BIOS_WAIT_FOREVER);

        if (events & USER_EVENT_BLE_ADVERTISING)
        {
            bleConnected = false;

            blink.start(BLINK_INFINITELY,
                        BLINK_FAST_ON_INTERVAL_MS,
                        BLINK_FAST_OFF_INTERVAL_MS);

            Clock_start(user_connTimeoutClock);
        }

        if (events & USER_EVENT_BLE_CONNECTED)
        {
            Clock_stop(user_connTimeoutClock);

            bleConnected = true;

            blink.start(BLINK_INFINITELY,
                        BLINK_SLOW_ON_INTERVAL_MS,
                        BLINK_SLOW_OFF_INTERVAL_MS);

            Clock_start(user_dataUpdateClock);

            bleSetBatteryVoltage(batt.getVbatt());
            bleSetBatteryMaxVoltage(batt.getVbattMax());
            bleSetBatteryCapacity(batt.getCapacity());
            bleSetBatteryTemperature(batt.getTemperature());

            user_LsmStart();
        }

        if (events & USER_EVENT_BLE_DISCONNECTED)
        {
            bleConnected = false;

            user_LsmStop();

            Clock_stop(user_dataUpdateClock);

            blink.start(BLINK_INFINITELY,
                        BLINK_FAST_ON_INTERVAL_MS,
                        BLINK_FAST_OFF_INTERVAL_MS);

            Clock_start(user_connTimeoutClock);
        }

        if (events & USER_EVENT_LSM_INTERRUPT)
        {
            if (lsmStarted)
            {
                if (lsmTimeout <= 1)
                {
                    // if the TO is less than 1 ms don't use timer and send immediately
                    user_readAndSendLsmData();
                    //GPIO_write(BOARD_GPIO_LED_GREEN, BOARD_GPIO_LED_OFF);
                }
                else
                {
                    lsmDisableInterrupt = true;
                    Clock_stop(user_lsmDataUpdateClock);
                    Clock_setTimeout(user_lsmDataUpdateClock, (lsmTimeout * (1000 / Clock_tickPeriod)));
                    Clock_start(user_lsmDataUpdateClock);
                }
            }
        }

        if (events & USER_EVENT_LSM_DATA_UPDATE)
        {
            Clock_stop(user_lsmDataUpdateClock);
            timestamp = millis();
            user_readAndSendLsmData();
            lsmDisableInterrupt = false;
        }

        if (events & USER_EVENT_BUTTON_INTERRUPT)
        {
            uint8_t state = (GPIO_read(BOARD_GPIO_BUTTON) == BOARD_GPIO_BUTTON_PUSHED) ? 1 : 0;

            if (state)
            {
                //blink.stop(true);
                //user_Led(true);

                user_Vibration(true);
                delay(35);
                user_Vibration(false);

                Clock_start(user_powerOffClock);
            }
            else
            {
                if (bleConnected)
                {
                    blink.start(BLINK_INFINITELY,
                                BLINK_SLOW_ON_INTERVAL_MS,
                                BLINK_SLOW_OFF_INTERVAL_MS);
                }
                else
                {
                    blink.start(BLINK_INFINITELY,
                                BLINK_FAST_ON_INTERVAL_MS,
                                BLINK_FAST_OFF_INTERVAL_MS);
                }

                Clock_stop(user_powerOffClock);
            }

            if (bleConnected)
            {
                bleSetButtonState(state);
            }
        }

        if (events & USER_EVENT_POWER_OFF)
        {
            uint8_t state = (GPIO_read(BOARD_GPIO_BUTTON) == BOARD_GPIO_BUTTON_PUSHED) ? 1 : 0;

            user_Vibration(true);
            delay(200);
            user_Vibration(false);

            user_Led(false);

            while (state)
            {
                state = (GPIO_read(BOARD_GPIO_BUTTON) == BOARD_GPIO_BUTTON_PUSHED) ? 1 : 0;
            }

            user_ShutDown();
        }

        if (events & USER_EVENT_CHARGE_ON)
        {
            Clock_stop(user_powerOffClock);
            Clock_stop(user_connTimeoutClock);

            user_LsmStop();

            // TODO disconnect the BLE

            user_Vibration(true);
            delay(100);
            user_Vibration(false);

            blink.stop(true);
            user_Led(true);
        }

        if (events & USER_EVENT_CHARGE_OFF)
        {
            user_Led(false);
            user_ShutDown();
        }
    }
}



void user_readAndSendLsmData()
{
    static uint8_t data[20];

    BleAccGyroData_t agd;


    if (lsm.getAccEnabled())
    {
        agd.ax = lsm.readRawAccelX();
        agd.ay = lsm.readRawAccelY();
        agd.az = lsm.readRawAccelZ();

        agd.am = lsm.getAccelRangeMultiplier();
    }

    if (lsm.getGyroEnabled())
    {
        agd.gx = lsm.readRawGyroX();
        agd.gy = lsm.readRawGyroY();
        agd.gz = lsm.readRawGyroZ();

        agd.gm = lsm.getGyroRangeMultiplier();
    }

    if (imuSkipValues > 0)
    {
        imuSkipValues--;
        return;
    }

    if (bleConnected && imuSkipValues == 0)
    {
        memset(data, 0x00, sizeof(data));
        bleSetAccGyroData(timestamp, &agd, data);
        user_exportLsmDataToUart(data);
    }
}



void user_ButtonCallback(uint_least8_t index)
{
    Event_post(userEvent, USER_EVENT_BUTTON_INTERRUPT);
}



void user_LsmCallback(uint_least8_t index)
{
    if (!lsmDisableInterrupt)
    {
        timestamp = millis();
        Event_post(userEvent, USER_EVENT_LSM_INTERRUPT);
    }
}




void user_ChargeCallback(uint_least8_t index)
{
    if (GPIO_read(BOARD_GPIO_CHARGE_ON))
    {
        Event_post(userEvent, USER_EVENT_CHARGE_ON);
    }
    else
    {
        Event_post(userEvent, USER_EVENT_CHARGE_OFF);
    }
}



void user_Vibration(bool on)
{
    GPIO_write(BOARD_GPIO_VIBRATOR, on ? BOARD_GPIO_HIGH : BOARD_GPIO_LOW);

    if (bleConnected)
    {
        bleSetVibrationState(on);
    }
}



void user_Led(bool on)
{
    GPIO_write(BOARD_GPIO_LED, on ? BOARD_GPIO_LED_ON : BOARD_GPIO_LED_OFF);

    if (bleConnected)
    {
        bleSetLedState(on);
    }
}



void user_LsmStart()
{
    if (lsmStarted)
    {
        return;
    }

    lsm.begin();
    lsmStarted = true;
}



void user_LsmStop()
{
    if (!lsmStarted)
    {
        return;
    }

    lsmStarted = false;
    lsm.end();
}



void user_AdvertisingEvent()
{
    Event_post(userEvent, USER_EVENT_BLE_ADVERTISING);
}



void user_ConnectedEvent()
{
    uint32_t connInterval = 0;
    uint32_t connLatency = 0;

    GAPRole_GetParameter(GAPROLE_CONN_INTERVAL, &connInterval);
    GAPRole_GetParameter(GAPROLE_CONN_LATENCY, &connLatency);

    Event_post(userEvent, USER_EVENT_BLE_CONNECTED);
}



void user_DisconnectedEvent()
{
    Event_post(userEvent, USER_EVENT_BLE_DISCONNECTED);
}



void user_ConnectionTimeoutEvent()
{
    Event_post(userEvent, USER_EVENT_POWER_OFF);
}



void user_ShutDown()
{
    Clock_stop(user_connTimeoutClock);
    Clock_stop(user_dataUpdateClock);
    Clock_stop(user_lsmDataUpdateClock);
    Clock_stop(user_powerOffClock);

    Clock_delete(&user_connTimeoutClock);
    Clock_delete(&user_dataUpdateClock);
    Clock_delete(&user_lsmDataUpdateClock);
    Clock_delete(&user_powerOffClock);

    user_LsmStop();
    batt.end();
    blink.end();

    //CC2640R2_LAUNCHXL_shutDown();

    GPIO_write(BOARD_GPIO_PS_HOLD, BOARD_GPIO_LOW);

    while(1);
}



void user_connTimeoutClockSwiHandler(UArg paramID)
{
    user_ConnectionTimeoutEvent();
}



void user_dataUpdateClockSwiHandler(UArg paramID)
{
    bleSetBatteryVoltage(batt.getVbatt());
    bleSetBatteryMaxVoltage(batt.getVbattMax());
    bleSetBatteryCapacity(batt.getCapacity());
    bleSetBatteryTemperature(batt.getTemperature());
}



void user_setBatteryMaxVoltage(uint16_t vbattMax)
{
    batt.setVbattMax(vbattMax);
}



void user_setAccEnable(bool enable)
{
    lsm.enableAcc(enable);

    if (bleConnected)
    {
        bleSetAccEnable(enable);
    }
}



void user_setAccODR(uint32_t odr)
{
    user_setAccGyroODR(odr);
}



void user_setAccFS(uint16_t fs)
{
    lsm.setAccRange(fs);

    if (bleConnected)
    {
        bleSetAccFS(fs);
    }

    imuSkipValues = SMSH_SKIP_VALUES_ON_FS_CHANGE;
}



void user_setGyroEnable(bool enable)
{
    lsm.enableGyro(enable);

    if (bleConnected)
    {
        bleSetGyroEnable(enable);
    }
}

// ODR is in milliseconds
void user_setAccGyroODR(uint32_t odr)
{
    LSM6DS3_ODR_t lsmODR;

    odrMs = odr;

    if (odrMs < 10)
    {
        lsmODR = LSM6DS3_ODR_208HZ; //  4.8 ms

        if (odrMs < 5)
        {
            lsmTimeout = 0;
        }
        else
        {
            lsmTimeout = odrMs - 5;
        }
    }
    else if (odrMs >= 10 && odrMs < 19)
    {
        lsmODR = LSM6DS3_ODR_104HZ; //  9.6 ms
        lsmTimeout = odrMs - 10;
    }
    else if (odrMs >= 19 && odrMs < 38)
    {
        lsmODR = LSM6DS3_ODR_52HZ; // 19.2 ms
        lsmTimeout = odrMs - 19;
    }
    else if (odrMs >= 38 && odrMs < 76)
    {
        lsmODR = LSM6DS3_ODR_26HZ; // 38.4 ms
        lsmTimeout = odrMs - 38;
    }
    else if (odrMs >= 76)
    {
        lsmODR = LSM6DS3_ODR_13HZ; // 76.9 ms
        lsmTimeout = odrMs - 76;
    }

    lsm.setAccGyroODR(lsmODR); // keep both ODRs the same

    if (bleConnected)
    {
        bleSetAccODR(odrMs);
        bleSetGyroODR(odrMs);
    }
}


void user_setGyroODR(uint32_t odr)
{
    user_setAccGyroODR(odr);
}



void user_setGyroFS(uint16_t fs)
{
    lsm.setGyroRange(fs);

    if (bleConnected)
    {
        bleSetGyroFS(fs);
    }

    imuSkipValues = SMSH_SKIP_VALUES_ON_FS_CHANGE;
}



bool user_getAccEnable()
{
    return lsm.getAccEnabled();
}



uint32_t user_getAccODR()
{
    return 1000 / lsm.getAccODR();
}



uint16_t user_getAccFS()
{
    return lsm.getAccRange();
}



bool user_getGyroEnable()
{
    return lsm.getGyroEnabled();
}



uint32_t user_getGyroODR()
{
    return 1000 / lsm.getGyroODR();
}



uint16_t user_getGyroFS()
{
    return lsm.getGyroRange();
}



uint16_t user_getBatteryVoltage()
{
    return batt.getVbatt();
}



uint16_t user_getBatteryMaxVoltage()
{
    return batt.getVbattMax();
}



uint16_t user_getBatteryCapacity()
{
    return batt.getCapacity();
}



int16_t user_getBatteryTemperature()
{
    return batt.getTemperature();
}



bool user_getButtonState()
{
    return (GPIO_read(BOARD_GPIO_BUTTON) == BOARD_GPIO_BUTTON_PUSHED);
}



bool user_getVibrationState()
{
    return (GPIO_read(BOARD_GPIO_VIBRATOR) == BOARD_GPIO_HIGH);
}



bool user_getLedState()
{
    return (GPIO_read(BOARD_GPIO_LED) == BOARD_GPIO_LED_ON);
}


void user_lsmDataUpdateClockSwiHandler(UArg paramID)
{
    Event_post(userEvent, USER_EVENT_LSM_DATA_UPDATE);
}


void user_powerOffClockSwiHandler(UArg paramID)
{
    Event_post(userEvent, USER_EVENT_POWER_OFF);
}





void user_exportLsmDataToUart(uint8_t *data)
{
#if (1)
    uint32_t timestamp;

    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    uint32_t multipliers;
    uint32_t am;
    uint32_t gm;

    size_t offset = 0;

    memcpy(&timestamp, &data[offset], sizeof(timestamp));
    offset += sizeof(timestamp);

    memcpy(&ax, &data[offset], sizeof(ax));
    offset += sizeof(ax);

    memcpy(&ay, &data[offset], sizeof(ay));
    offset += sizeof(ay);

    memcpy(&az, &data[offset], sizeof(az));
    offset += sizeof(az);


    memcpy(&gx, &data[offset], sizeof(gx));
    offset += sizeof(gx);

    memcpy(&gy, &data[offset], sizeof(gy));
    offset += sizeof(gy);

    memcpy(&gz, &data[offset], sizeof(gz));
    offset += sizeof(gz);

    memcpy(&multipliers, &data[offset], sizeof(multipliers));
    offset += sizeof(multipliers);

    am = (multipliers >>  0) & 0x00000FFF;
    gm = (multipliers >> 12) & 0x000FFFFF;

    int32_t i32ax = (int32_t) ax;
    int32_t i32ay = (int32_t) ay;
    int32_t i32az = (int32_t) az;

    int32_t i32gx = (int32_t) gx;
    int32_t i32gy = (int32_t) gy;
    int32_t i32gz = (int32_t) gz;

    i32ax = (int32_t) (( (int64_t)i32ax * (int64_t)am) / 1000);
    i32ay = (int32_t) (( (int64_t)i32ay * (int64_t)am) / 1000);
    i32az = (int32_t) (( (int64_t)i32az * (int64_t)am) / 1000);

    i32gx = (int32_t) (( (int64_t)i32gx * (int64_t)gm) / 1000);
    i32gy = (int32_t) (( (int64_t)i32gy * (int64_t)gm) / 1000);
    i32gz = (int32_t) (( (int64_t)i32gz * (int64_t)gm) / 1000);



    float fax = ((float) i32ax) / 1000.0;
    float fay = ((float) i32ay) / 1000.0;
    float faz = ((float) i32az) / 1000.0;

    float fgx = ((float) i32gx) / 1000.0;
    float fgy = ((float) i32gy) / 1000.0;
    float fgz = ((float) i32gz) / 1000.0;
#else
    float fax = lsm.readFloatAccelX();
    float fay = lsm.readFloatAccelY();
    float faz = lsm.readFloatAccelZ();

    float fgx = lsm.readFloatGyroX();
    float fgy = lsm.readFloatGyroY();
    float fgz = lsm.readFloatGyroZ();
#endif

    fgx -= IMU_GYRO_CALLIBRATION_X;
    fgy -= IMU_GYRO_CALLIBRATION_Y;
    fgz -= IMU_GYRO_CALLIBRATION_Z;

#if (USE_MADGWICK_AHRS != 0)

    // update the filter, which computes orientation
    filter.updateIMU(fgx, fgy, fgz, fax, fay, faz);

    // print the heading, pitch and roll
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    const char * start = "!ANG:";

    char sroll[10];
    char spitch[10];
    char syaw[10];

    System_sprintf(sroll, "%.2f", roll);
    System_sprintf(spitch, "%.2f", pitch);
    System_sprintf(syaw, "%.2f", yaw);

    UART_write(uart, start, strlen(start));
    UART_write(uart, sroll, strlen(sroll));
    UART_write(uart, ",", 1);
    UART_write(uart, spitch, strlen(spitch));
    UART_write(uart, ",", 1);
    UART_write(uart, syaw, strlen(syaw));
    UART_write(uart, "\r\n", 2);

#else
    // export RAW data
    static char sax[15] = "";
    static char say[15] = "";
    static char saz[15] = "";

    static char sgx[15] = "";
    static char sgy[15] = "";
    static char sgz[15] = "";


    System_sprintf(sax, "%.2f", fax);
    System_sprintf(say, "%.2f", fay);
    System_sprintf(saz, "%.2f", faz);

    System_sprintf(sgx, "%.2f", fgx);
    System_sprintf(sgy, "%.2f", fgy);
    System_sprintf(sgz, "%.2f", fgz);


    UART_write(uart, sax, strlen(sax)); UART_write(uart, ",", 1);
    UART_write(uart, say, strlen(say)); UART_write(uart, ",", 1);
    UART_write(uart, saz, strlen(saz)); UART_write(uart, ",", 1);

    UART_write(uart, sgx, strlen(sgx)); UART_write(uart, ",", 1);
    UART_write(uart, sgy, strlen(sgy)); UART_write(uart, ",", 1);
    UART_write(uart, sgz, strlen(sgz)); //UART_write(uart, ",", 1);

    UART_write(uart, "\n", 2);


        if (faz > 0.6 || faz < -0.6)
        {
            user_Vibration(true);
        }
        else if (faz < 0.3 && faz > -0.3)
        {
            user_Vibration(false);
        }

#endif
}
