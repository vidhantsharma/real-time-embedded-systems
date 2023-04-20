#ifndef BLE_UART_H
#define BLE_UART_H

#include <stdint.h>

typedef void (*ble_recv_handler_t)(const uint8_t data_buf[], uint32_t len);
extern void ble_init(ble_recv_handler_t handler);
extern void ble_send(uint8_t data_array[], uint16_t len);

#endif /* BLE_UART_H */