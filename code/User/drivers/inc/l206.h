#ifndef __L206_H__
#define __L206_H__
#include "stm32f10x.h"
#include "drivers.h"

#define COM_L206        COM3

void vTaskL206Process(void *pvParameters);

void l206_init(void);
uint8_t l206_poweron(void);
void l206_poweroff(void);
void l206_write(char *buf);
void l206_csq(void);
void l206_set_baudrate(void);
void l206_cgmr(void);
void l206_cgsn(void);
uint8_t l206_cpin(void);
void l206_iccid(void);
uint8_t l206_creg(void);
uint8_t l206_cgatt(void);
uint8_t l206_ftp_config(const char *filename);
uint8_t l206_ftp_get_file(char *buf);
//send buf to server by tcp socket
uint8_t l206_send(uint8_t *buf, uint16_t len, int timeout);
uint16_t l206_recv(uint8_t *buf, uint16_t len, int timeout);
//connect to remote server
uint8_t l206_connect(const char *host, uint32_t port);
uint8_t l206_wait_rsp(char *ack, uint16_t timeout);
uint16_t l206_read_rsp(char *ack, char *data, uint16_t timeout);
void l206_lock(void);
void l206_unlock(void);
void get_deviceid(char *id);
uint8_t l206_is_connected(void);
#endif
