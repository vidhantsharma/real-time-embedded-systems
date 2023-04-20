#include <string.h>
#include <stdio.h>
#include "ble_uart.h"

static void ble_recv_handler(const uint8_t data[], uint32_t len)
{
    static uint8_t data_array[256];

    /* echo */
    memcpy(data_array, data, len);

    ble_send(data_array, len);
}

int main(void)
{
    ble_init(ble_recv_handler);

    while(1)
        ;

    return 0;
}
