#include "l206.h"
#include "flash_table.h"

// connect sequence:
//      ATE0
//      AT+CPIN?
//      AT+CREG?
//      AT+CGATT?
//      AT+ZIPCFG=CMNET
//      AT+ZIPCALL=1
//      AT+GTPOS=1
// connect cmd
//      AT+ZIPOPEN=2,0,192.168.1.1,8880
// connect location server
//      AT+GTPOS=1
// get location info
//      AT+GTPOS=2

#define AT_CR                   '\r'

#define L206_RETRY_TIMES        8

#define KEYPOWER_PORT           GPIOB
#define KEYPOWER_PIN            GPIO_Pin_0
#define KEYPOWER_RCC_APB        RCC_APB2Periph_GPIOB

#define RESET_PORT              GPIOC
#define RESET_PIN               GPIO_Pin_15
#define RESET_RCC_APB           RCC_APB2Periph_GPIOC

#define KEYPOWER_1()            GPIO_ResetBits(KEYPOWER_PORT, KEYPOWER_PIN)
#define KEYPOWER_0()            GPIO_SetBits(KEYPOWER_PORT, KEYPOWER_PIN)

#define RESET_0()               GPIO_ResetBits(RESET_PORT, RESET_PIN)
#define RESET_1()               GPIO_SetBits(RESET_PORT, RESET_PIN)

static void vTaskL206Process(void *pvParameters);

static SemaphoreHandle_t xMutex = NULL;
static char l206_imei[15];
static char iccid[16];
static TaskHandle_t xHandleTaskL206Process = NULL;
static uint8_t mConnected = FALSE;

static void vTaskL206Process(void *pvParameters)
{
    uint8_t u_data;
    char buffer[100];
    int size;

    logi("%s: E", __func__);
    if(l206_poweron() == TRUE) {
        logi("%s: poweron successfully", __func__);
    } else {
        loge("%s: poweron failed", __func__);
        return;
    }

    l206_set_baudrate();
    l206_cgmr();
    l206_cgsn();
    l206_cpin();
    l206_iccid();
    l206_creg();
    l206_cgatt();

    flash_table_init();

#if 0
    l206_ftp_config("test.txt");
    l206_ftp_get_file(NULL);
    if(l206_connect("116.62.186.169", 8901)) {
        logi("connect socket server successfully!!");
        mConnected = TRUE;
    } else {
        loge("connect socket server failed!!");
        mConnected = FALSE;
        return;
    }
#endif

    if(l206_connect("139.224.227.174", 1884)) {
        logi("connect socket server successfully!!");
        mConnected = TRUE;
    } else {
        loge("connect socket server failed!!");
        mConnected = FALSE;
        return;
    }

    while(1) {
#if 0
        while(comGetChar(COM_L206, &u_data)) {
             printf("%c", u_data);
        }
#endif
        os_delay_ms(10);
        //l206_send("hello\n", 6, 5000);
        // l206_send("hello\n", 6);
        // memset(buffer, 0x00, 100);
        // logi("start t o read");
        // size = FreeRTOS_read(NULL, buffer, 100, 1000);
        // logi("read %d bytes: %s", size, buffer);
    }
}

void l206_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(KEYPOWER_RCC_APB, ENABLE);
    gpio.GPIO_Pin = KEYPOWER_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEYPOWER_PORT, &gpio);

    RCC_APB2PeriphClockCmd(RESET_RCC_APB, ENABLE);
    gpio.GPIO_Pin = RESET_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RESET_PORT, &gpio);

    //create mutex
    xMutex = xSemaphoreCreateMutex();
    if(xMutex == NULL) {
        loge("%s: failed to create mutex for l206", __func__);
    }

    //create l206 process task
    logi("create l206 process task");
    xTaskCreate(vTaskL206Process,
            "L206Process",
            512,
            NULL,
            4,
            &xHandleTaskL206Process);

    logi("%s done", __func__);
}

void l206_reset(void)
{
    logi("%s: E", __func__);
    RESET_1();
    os_delay(2);
    RESET_0();
    os_delay(1);
    RESET_1();
    os_delay(2);
    logi("%s: X", __func__);
}

uint8_t l206_poweron(void)
{
    uint8_t i;

    for(i = 0; i < 1; i++) {
        l206_write("ATE0\r\n");
        if(l206_wait_rsp("OK", 1000)) {
            logi("Already poweron!");
            return TRUE;
        } else {
            loge("Power on failed");
        }
    }

    l206_reset();
    KEYPOWER_0();
    os_delay(2);
    KEYPOWER_1();
    os_delay(10);
    comClearRxFifo(COM_L206);
    for(i = 0; i < 5; i++) {
        l206_write("ATE0\r\n");
        if(l206_wait_rsp("OK", 1000)) {
            logi("poweron success");
            return TRUE;
        }
    }
    loge("poweron failed");

    return FALSE;
}

void l206_poweroff(void)
{
    logi("%s", __func__);
    KEYPOWER_0();
    os_delay(2);
    KEYPOWER_1();
}


void l206_csq(void)
{
    comClearRxFifo(COM_L206);
    l206_write("AT+CSQ\r\n");
    if(l206_wait_rsp("OK", 200)) {
        logi("%s: check csq success", __func__);
    } else {
        loge("%s: check csq failed", __func__);
    }
}

void l206_cgmr(void)
{
    comClearRxFifo(COM_L206);
    l206_write("AT+CGMR\r\n");
    if(l206_wait_rsp("OK", 2000)) {
        logi("%s: check version success", __func__);
    } else {
        loge("%s: check version failed", __func__);
    }
}

uint8_t is_digital(char c)
{
    if(c >= '0' && c <= '9')
        return TRUE;
    else
        return FALSE;
}

void l206_set_baudrate(void)
{
    comClearRxFifo(COM_L206);
    l206_write("AT+IPR=115200\r\n");
    if(l206_wait_rsp("OK", 200)) {
        logi("%s: set baudrate success", __func__);
    } else {
        loge("%s: set baudrate failed", __func__);
    }
}

void l206_iccid(void)
{
    char buf[20];
    uint8_t i;
    uint8_t offset;

    memset(buf, 0x00, 20);
    memset(iccid, 0x00, 15);
    comClearRxFifo(COM_L206);
    l206_write("AT+ICCID\r\n");
    if(l206_read_rsp("OK", buf, 200)) {
        logi("%s: check device iccid success", __func__);
    } else {
        loge("%s: check device iccid failed", __func__);
    }

    for(i = 0; i < 20; i++) {
        if(is_digital(buf[i])) {
            offset = i;
            break;
        }
    }
    if(i < 5) {
        memcpy(iccid, buf + offset, 15);
        iccid[15] = '\0';
    }
    logi("iccid: %s", iccid);
}

void l206_cgsn(void)
{
    char buf[20];
    uint8_t i;
    uint8_t offset;

    memset(buf, 0x00, 20);
    memset(l206_imei, 0x00, 15);
    comClearRxFifo(COM_L206);
    l206_write("AT+CGSN\r\n");
    if(l206_read_rsp("OK", buf, 200)) {
        logi("%s: check device imei success", __func__);
    } else {
        loge("%s: check device imei failed", __func__);
        return;
    }

    for(i = 0; i < 20; i++) {
        if(is_digital(buf[i])) {
            offset = i;
            break;
        }
    }
    if(i < 5) {
        memcpy(l206_imei, buf + offset, 15);
    }
    logi("imei: %s", l206_imei);
}

uint8_t l206_cpin(void)
{
    uint8_t ret;

    comClearRxFifo(COM_L206);
    l206_write("AT+CPIN?\r\n");
    if(l206_wait_rsp("OK", 200)) {
        logi("%s: check sim success", __func__);
        ret = TRUE;
    } else {
        loge("%s: check sim failed", __func__);
        ret = FALSE;
    }
    return ret;
}

uint8_t l206_creg(void)
{
    uint8_t ret;
    uint8_t i;
    uint8_t status;
    char buf[32];
    char *sub_str;

    memset(buf, 0x00, 32);

    for(i = 0; i < 10; i++) {
        l206_csq();
        os_delay(1);
        comClearRxFifo(COM_L206);
        l206_write("AT+CREG?\r\n");
        if(l206_read_rsp("OK", buf, 1000)) {
            logi("%s: register network success", __func__);
            ret = TRUE;
        } else {
            loge("%s: register network failed", __func__);
            ret = FALSE;
        }
        sub_str = strstr(buf, ",");
        status = sub_str[1] - '0';
        logi("status = %d", status);
        if(status == 1 || status == 5) {
            ret = TRUE;
            break;
        }
    }
    return ret;
}

uint8_t l206_cgatt(void)
{
    uint8_t ret;

    comClearRxFifo(COM_L206);
    l206_write("AT+CGATT?\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: cgatt success", __func__);
        ret = TRUE;
    } else {
        loge("%s: cgatt failed", __func__);
        ret = FALSE;
    }
    return ret;
}

uint8_t l206_ftp_get_file(char *buf)
{
    const int BLOCK_SIZE = 512;
    const int WAIT_TIME = 5;
    uint8_t cmd[64];
    uint8_t u_data;
    uint8_t buffer[600];
    uint8_t *tmp, *tmp2;
    uint32_t index = 0;
    uint8_t done = FALSE;
    int ftpget2_start;
    char buf_size[4];
    int size = 0;
    int i = 0;
    uint8_t is_last = FALSE;
    int total = 0;
    uint8_t cs = 0;

    int FLASH_START = 0x100;
    uint8_t buf2[4];

    while(done == FALSE) {
        tmp = NULL;
        tmp2 = NULL;
        comClearRxFifo(COM_L206);
        memset(cmd, 0x00, 64);
        memset(buffer, 0x00, 600);
        memset(buf_size, 0x00, 4);
        index = 0;
        snprintf(cmd, 64, "AT+FTPGET=2,%d\r\n", BLOCK_SIZE);
        l206_write(cmd);
        os_delay(WAIT_TIME);
        while(comGetChar(COM_L206, &u_data)) {
            printf("%c", u_data);
            buffer[index ++] = u_data;
        }
        //check buffer
        tmp = strstr(buffer, "ERROR\r\n");
        if(tmp != NULL) {
            loge("%s: find ERROR", __func__);
            done = TRUE;
            continue;
        }

        tmp = strstr(buffer, "+FTPGET: 1,0\r\n");
        if(tmp != NULL) {
            is_last = TRUE;
        }
        
        //1. get buffer size: +FTPGET: 2,512
        tmp = strstr(buffer, "+FTPGET: 2");
        if(tmp == NULL) {
            loge("%s: cannot find +FTPGET: 2", __func__);
            done = TRUE;
            continue;
        }
        ftpget2_start = buffer - tmp;
        tmp2 = strstr(tmp, "\r\n");
        if(tmp == NULL) {
            loge("%s: cannot find LRCR", __func__);
            return FALSE;
        }
        memcpy(buf_size, tmp + 11, tmp2 - tmp - 11);
        logi("%s: buf_size: %s", __func__, buf_size);
        size = atoi(buf_size);
        memcpy(buffer, tmp2 + 2, size);
        //for(i = 0; i < size; i++) {
        //    logi("[%d] = %02x", i, buffer[i]);
        //}
        if(is_last == TRUE) {
            logi("%s: read ftp file end!!", __func__);
            done = TRUE;
        }
        //write buffer to flash
        M25PXX_WriteBuffer(buffer, FLASH_START + total, size);
        total += size;
    }
    logi("%s: ftp file total size = %d", __func__, total);
    memcpy(buf2, &total, 4);
    logi("write: %02x, %02x, %02x, %02x", buf2[0], buf2[1], buf2[2], buf2[3]);
    M25PXX_WriteBuffer(buf2, 0x00, 4);
    memcpy(buf2, 0x00, 4);
    M25PXX_ReadBuffer(buf2, 0x00, 4);
    logi("write: %02x, %02x, %02x, %02x", buf2[0], buf2[1], buf2[2], buf2[3]);
    //checksum all the buffer
    total = 0;
    M25PXX_ReadBuffer(&total, 0x00, 4);
    logi("flash read total: %d", total);
    for(i = 0; i < total; i++) {
        M25PXX_ReadBuffer(&u_data, FLASH_START + i, 1);
        printf("%c", u_data);
        cs ^= u_data;
    }

    logi("checksum: %02x", cs);

    return TRUE;
}

uint8_t l206_ftp_config(const char *filename)
{
    uint8_t ret;
    char buf[64];

    comClearRxFifo(COM_L206);
    l206_write("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set type of connect type for ftp",
                __func__);
    } else {
        loge("%s: set type of connect type for ftp failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set apn for ftp",
                __func__);
    } else {
        loge("%s: set apn for ftp failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+SAPBR=1,1\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: open bearer",
                __func__);
    } else {
        loge("%s: open bearer failed",
                __func__);
        return FALSE;
    }

    comClearRxFifo(COM_L206);
    l206_write("AT+FTPCID=1\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set binary session",
                __func__);
    } else {
        loge("%s: set binary failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+FTPSERV=\"139.224.227.174\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set ftp server address",
                __func__);
    } else {
        loge("%s: set ftp server address failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+FTPPORT=801\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set ftp port",
                __func__);
    } else {
        loge("%s: set ftp port failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+FTPUN=\"anonymous\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set user name",
                __func__);
    } else {
        loge("%s: set user name failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    l206_write("AT+FTPPW=\"\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set user password",
                __func__);
    } else {
        loge("%s: set user password failed",
                __func__);
        return FALSE;
    }

    memset(buf, 0x00, 64);
    snprintf(buf, 64, "AT+FTPGETNAME=\"%s\"\r\n", filename);
    comClearRxFifo(COM_L206);
    l206_write(buf);
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set ftp file name",
                __func__);
    } else {
        loge("%s: set ftp file name failed",
                __func__);
        return FALSE;
    }

    comClearRxFifo(COM_L206);
    l206_write("AT+FTPGETPATH=\"\/pub\/\"\r\n");
    if(l206_wait_rsp("OK", 3000)) {
        logi("%s: set ftp file path",
                __func__);
    } else {
        loge("%s: set ftp file path failed",
                __func__);
        return FALSE;
    }
    comClearRxFifo(COM_L206);
    logi("%s: open ftp session",
            __func__);
    l206_write("AT+FTPGET=1\r\n");
    if(l206_wait_rsp("+FTPGET:1,1", 3000)) {
        logi("%s: open ftp session",
                __func__);
    } else {
        loge("%s: open ftp session failed",
                __func__);
        return FALSE;
    }

    return TRUE;
}


uint8_t l206_send(uint8_t *buf, uint16_t len, int timeout)
{
    uint8_t ret = FALSE;
    uint8_t *tx_buf;
    uint16_t index = 0, i;

    logi("%s: %s", __func__, buf);
    l206_lock();
    tx_buf = (uint8_t *)malloc(13 + 2 * len);
    memset(tx_buf, 0x00, 13 + 2 * len);
    index = snprintf((char *)tx_buf, 1024, "AT+ZIPSEND=2,");
    for(i = 0; i < len; i++) {
        index += snprintf((char *)tx_buf + index, 1024, "%02x", buf[i]);
    }
    l206_write((char *)tx_buf);
    l206_write("\r\n");
    if(l206_wait_rsp("SEND OK", timeout)) {
        ret = TRUE;
    } else {
        loge("%s: send failed", __func__);
        ret = FALSE;
    }
    free(tx_buf);
    tx_buf = NULL;
    l206_unlock();

    return ret;
}

uint8_t l206_recv(uint8_t *buf, uint16_t *len, int timeout)
{
    uint8_t ret = FALSE;

    l206_lock();

    l206_unlock();

    return ret;
}

uint8_t l206_connect(const char *host, uint32_t port)
{
    uint8_t ret;
    uint8_t tx_buf[128];

    comClearRxFifo(COM_L206);
    l206_write("AT+ZIPCFG=unim2m.njm2mapn\r\n");
    if(l206_wait_rsp("OK", 8000)) {
        logi("%s: register gprs success", __func__);
        ret = TRUE;
    } else {
        loge("%s: register gprs failed", __func__);
        ret = FALSE;
        goto ERR_CONNECT;
    }

    l206_write("AT+ZIPCALL=1\r\n");
    if(l206_wait_rsp("OK", 5000)) {
        logi("%s: zipcall success", __func__);
        ret = TRUE;
    } else {
        loge("%s: zipcall gprs failed", __func__);
        ret = FALSE;
        goto ERR_CONNECT;
    }

    memset(tx_buf, 0x00, 128);
    snprintf((char *)tx_buf, 1024, "AT+ZIPOPEN=2,0,%s,%d\r\n",
            host, port);
    l206_write((char *)tx_buf);
    if(l206_wait_rsp("CONNECT OK", 10000)) {
        logi("%s: connect %s:%d success", __func__,
                host, port);
        ret = TRUE;
    } else {
        loge("%s: connect %s:%d failed", __func__,
                host, port);
        ret = FALSE;
    }

ERR_CONNECT:
    return ret;
}

uint8_t l206_is_connected(void)
{
    return mConnected;
}

void l206_write(char *buf)
{
    comClearRxFifo(COM_L206);
    comSendBuf(COM_L206, (uint8_t *)buf, strlen(buf));
    logi("-> %s", buf);
}

uint8_t l206_wait_rsp(char *ack, uint16_t timeout_ms)
{
    uint8_t u_data;
    uint16_t pos = 0;
    uint32_t len = 0;
    uint8_t ret;
    uint8_t u_rx_buf[100];
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS;
    TimeOut_t xTimeOut;

    len = strlen(ack);
    //suspend receiver task
    // if(mConnected) {
    //     OSTaskSuspend(&L206TaskTCB, &err);
    // }
    vTaskSetTimeOutState(&xTimeOut);
    do {
        if(comGetChar(COM_L206, &u_data)) {
            printf("%c", u_data);
            if(u_data == '\n') {
                //logi("rx_buf = %s, len = %d", u_rx_buf, len);
                if(pos > 0) {
                    if(memcmp(u_rx_buf, ack, len) == 0) {
                        ret = 1;
                        break;
                    } else {
                        pos = 0;
                    }
                } else {
                    pos = 0;
                }
            } else {
                if(pos < sizeof(u_rx_buf)) {
                    if(u_data >= ' ') {
                        u_rx_buf[pos ++] = u_data;
                    }
                }
            }
        }
    } while(xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);
    //resume task
    // if(mConnected) {
    //     OSTaskResume(&L206TaskTCB, &err);
    // }
    return ret;
}

uint16_t l206_read_rsp(char *ack, char *data, uint16_t timeout_ms)
{
    uint8_t u_data;
    uint16_t pos = 0;
    uint32_t len = 0;
    uint16_t ret;
    uint8_t u_rx_buf[100];
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS;
    TimeOut_t xTimeOut;

    len = strlen(ack);
    //suspend receiver task
    // if(mConnected) {
    //     OSTaskSuspend(&L206TaskTCB, &err);
    // }
    vTaskSetTimeOutState(&xTimeOut);
    do {
        if(comGetChar(COM_L206, &u_data)) {
            printf("%c", u_data);
            u_rx_buf[pos ++] = u_data;
            if(pos >= len) {
                if(memcmp(ack, u_rx_buf + (pos - len), len) == 0) {
                    ret = pos;
                    memcpy(data, u_rx_buf, pos);
                    break;
                }
            }
        }
    } while(xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);
    //resume task
    // if(mConnected) {
    //     OSTaskResume(&L206TaskTCB, &err);
    // }
    return ret;
}

void l206_lock(void)
{
    xSemaphoreTake(xMutex, portMAX_DELAY);
}

void l206_unlock(void)
{
    xSemaphoreGive(xMutex);
}

void get_deviceid(char *id)
{
    memcpy(id, iccid, 16);
}

