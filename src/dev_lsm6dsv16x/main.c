/**
 * @file main.c
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief メイン
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025 Chimipupu All Rights Reserved.
 * 
 */
#include "common.h"
#include "app_imu_lsm6dsv16x.h"

// ---------------------------------------------------
// [Define]

// ---------------------------------------------------
// [Global]

// ---------------------------------------------------
// [Static]

// ---------------------------------------------------
// [Prototype]
static void drv_pio_init(void);
static void drv_dma_init(void);
static void drv_spi_init(void);
static void drv_uart_init(void);
static void drv_wdt_init(void);
static void drv_tim_init(void);

// ---------------------------------------------------
// [Func]

#ifdef HW_PIO_USE
#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq)
{
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
#endif

static void drv_pio_init(void)
{
#ifdef HW_PIO_USE
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
#endif
}

int64_t alarm_callback(alarm_id_t id, void *user_data)
{
    // TODO:タイマー割り込みのコールバック関数実装
    return 0;
}

static void drv_dma_init(void)
{
    const char src[] = "Hello, world! (from DMA)";
    char dst[count_of(src)];

    int chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);

    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        dst,           // The initial write address
        src,           // The initial read address
        count_of(src), // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );

    dma_channel_wait_for_finish_blocking(chan);

#if 1
    printf("%s\r\n", dst);
#else
    puts(dst);
#endif
}

static void drv_spi_init(void)
{
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

static void drv_uart_init(void)
{
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_puts(UART_PORT, " Hello, UART!\n");
}

static void drv_wdt_init(void)
{
#ifdef WDT_USE
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    }
    watchdog_enable(100, 1);
    watchdog_update();
#endif
}

static void drv_tim_init(void)
{
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
}

int main()
{
    // [Pico SDK Init]
    stdio_init_all();

    // [UART Init]
    // drv_uart_init();

    // [I2C Init]
    app_lsm6dsv16x_init();

#if 0
    // [DMA Init]
    drv_dma_init();

    // [PIO Init]
    drv_pio_init();

    // [SPI Init]
    drv_spi_init();

    // [TIM Init]
    drv_tim_init();

    // [WDT Init]
    drv_wdt_init();

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
#endif

    // [App Init]

    while (true)
    {
        app_lsm6dsv16x_main();
        sleep_ms(1000);
    }
}