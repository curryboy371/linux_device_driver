#define pr_fmt(fmt) "[ledbar] " fmt

#include "../rotary/rotary_module.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>

#define BCM2711_PERI_BASE   0xFE000000
#define GPIO_BASE           (BCM2711_PERI_BASE + 0x200000)

#define GPIO_OUT(g)    (*(gpio + ((g)/10)) |= (1 << (((g)%10)*3)))
#define GPIO_SET(g)    (*(gpio + 7) = (1 << (g)))
#define GPIO_CLR(g)    (*(gpio + 10) = (1 << (g)))

#define LED_COUNT 8 
static const int gpio_led[LED_COUNT] = {
    19, 26, 16, 20, // led 0,1,2,3 
    8,          // led4: unused or reserved
    9,          // led5: toggle 표시
    10,         // led6: CCW
    11          // led7: CW
};

static volatile unsigned int *gpio;
static struct task_struct *led_thread;

static void led_write(uint8_t data) {
    int i;
    for (i = 0; i < LED_COUNT; i++) {
        if (data & (1 << i))
            GPIO_SET(gpio_led[i]);
        else
            GPIO_CLR(gpio_led[i]);
    }
}

static int led_fn(void *data) {

    /* rotary 모듈이 준비될 때까지 대기 */
    while (!rotary_is_ready() && !kthread_should_stop()) {
        pr_info("Waiting for rotary module to be ready...\n");
        msleep(1000);
    }

    if (kthread_should_stop()) {
        pr_info("LED thread stopping before rotary ready\n");
        return 0;
    }

    pr_info("Rotary module ready, starting LED updates\n");

    while (!kthread_should_stop()) {
        /* rotary 모듈이 언로드되면 중단 */
        if (!rotary_is_ready()) {
            pr_info("Rotary module not ready, stopping LED updates\n");
            break;
        }

        int count = rotary_get_count();
        int toggle = rotary_get_toggle();
        int direction = rotary_get_direction();

        uint8_t led = 0;

        // led3~led0 : count 값 (0~15)
        led |= (count & 0x0F);

        // led5 toggle
        if (toggle) {
            led |= (1 << 5);
        }

        // led6/7: cw ccw
        if (direction == 1) {
            led |= (1 << 7); // CW
        } else if (direction == -1) {
            led |= (1 << 6); // CCW
        }

        led_write(led);
        msleep(50);
    }

    return 0;
}

static int __init ledbar_init(void) {
    int i;

    pr_info("init\n");

    /* GPIO 메모리 매핑 */
    gpio = ioremap(GPIO_BASE, PAGE_SIZE);
    if (!gpio) {
        pr_err("failed to ioremap GPIO\n");
        return -ENOMEM;
    }

    /* GPIO 배열 출력으로 설정 */
    for (i = 0; i < LED_COUNT; i++) {
        GPIO_OUT(gpio_led[i]);
    }

    /* LED 스레드 시작 */
    led_thread = kthread_run(led_fn, NULL, "ledbar_thread");
    if (IS_ERR(led_thread)) {
        pr_err("failed to create LED thread\n");
        iounmap(gpio);
        return PTR_ERR(led_thread);
    }

    pr_info("ledbar module initialized (waiting for rotary module)\n");
    return 0;
}

static void __exit ledbar_exit(void) {
    pr_info("exit\n");

    if (led_thread) {
        kthread_stop(led_thread);
        led_thread = NULL;
    }

    if (gpio) {
        iounmap(gpio);
        gpio = NULL;
    }
}

module_init(ledbar_init);
module_exit(ledbar_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("ledbar module with rotary_module");