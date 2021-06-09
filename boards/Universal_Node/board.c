/*
 * Copyright (C) 2017 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_inga_common
 * @{
 *
 * @file
 * @brief       Common implementations for the INGA boards
 *
 * @author      Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * @}
 */

#include "cpu.h"
#include "board.h"
#include "stdio_uart.h"
#include "avr/io.h"

static int uart_putchar(char c, FILE *stream);
static int uart_getchar(FILE *stream);

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_stdin = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

/**
 * @brief Initialize the system, initialize the button
 */
void board_init(void)
{
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0x00; 
    LED1_OFF;
    LED2_OFF;

    /* initialize the CPU */
    cpu_init();

    /* initialize STDIO over UART */
    stdio_init();
    stdout = &uart_stdout;
    stdin = &uart_stdin;
    puts("\f");


    char a = 'a';
    char b = '\n';
    char c = '\f';

    stdio_write(&a, 1);
    stdio_write(&b, 1);
    stdio_write(&c, 1);



    /* Disable JTAG to be able to use PCINTs on PD5, PD6, PD7 */
    //MCUCR=(1<<JTD);

    /* Button */
    #ifdef BTN0_PIN
    gpio_init(BTN0_PIN, GPIO_IN);
    #endif

    #ifdef LED1_PIN
    gpio_init(LED1_PIN, GPIO_OUT);
    #endif
    #ifdef LED2_PIN
    gpio_init(LED2_PIN, GPIO_OUT);
    #endif
    LED1_ON;
    LED2_ON;

    

    irq_enable();
}

static int uart_putchar(char c, FILE *stream)
{
    (void) stream;
    stdio_write(&c, 1);
    return 0;
}

static int uart_getchar(FILE *stream)
{
    (void) stream;
    char c;
    stdio_read(&c, 1);
    return (int)c;
}

