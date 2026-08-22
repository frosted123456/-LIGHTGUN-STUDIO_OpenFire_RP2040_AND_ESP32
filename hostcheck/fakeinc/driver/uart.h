#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int uart_port_t;
#define UART_NUM_0 0
extern "C" { int uart_read_bytes(uart_port_t, void*, uint32_t, uint32_t); int uart_is_driver_installed(uart_port_t); int uart_driver_install(uart_port_t,int,int,int,void*,int); }
