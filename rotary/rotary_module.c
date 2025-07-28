#define pr_fmt(fmt) "[rotary] " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h> // input 사용

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "rotary_module.h"


#define GPIO_S1 17
#define GPIO_S2 27
#define GPIO_KEY 22

// BCM2711 기준 주소
#define BCM2711_PERI_BASE   0xFE000000
#define GPIO_BASE           (BCM2711_PERI_BASE + 0x200000)

// GPIO 관련 매크로
#define GPIO_IN(g)   (*(gpio + ((g)/10)) &= ~(7 << (((g)%10)*3)))
#define GPIO_READ(g) ((*(gpio + 13)) & (1 << (g)))


#define COUNT_MIN 0
#define COUNT_MAX 15 

static struct task_struct *rotary_thread;
static volatile unsigned int *gpio;
static int last_state = 0;

static int count = 0;
static int key_prev = 0;
static int toggle_state = 0;


int rotary_get_count(void) {
    return count;
}
EXPORT_SYMBOL(rotary_get_count);


int rotary_get_toggle(void) {
    return toggle_state;
}
EXPORT_SYMBOL(rotary_get_toggle);


static void update_key_toggle(void) {
    int now = (GPIO_READ(GPIO_KEY)) ? 1 : 0;

    if (key_prev == 0 && now == 1) {
        toggle_state ^= 1;

		if(toggle_state) {
			pr_info("toggle state 1\n");

		}
		else {
			pr_info("toggle state 0\n");
		}
    }

    key_prev = now;
}

// polling thread
static int rotary_fn(void *data) {
    int s1, s2, state;

    while (!kthread_should_stop()) {
        s1 = (GPIO_READ(GPIO_S1)) ? 1 : 0;
        s2 = (GPIO_READ(GPIO_S2)) ? 1 : 0;
        state = (s1 << 1) | s2;

        if ((last_state == 0b00 && state == 0b01) ||
            (last_state == 0b01 && state == 0b11) ||
            (last_state == 0b11 && state == 0b10) ||
            (last_state == 0b10 && state == 0b00)) {
			if (count < COUNT_MAX) {
				count++;
				pr_info("CW → count: %d\n", count);
			}

        } else if ((last_state == 0b00 && state == 0b10) ||
                   (last_state == 0b10 && state == 0b11) ||
                   (last_state == 0b11 && state == 0b01) ||
                   (last_state == 0b01 && state == 0b00)) {
			if (count > COUNT_MIN) {
				count--;
				pr_info("CCW → count: %d\n", count);
			}
        }

        last_state = state;

		// 키 버튼 상태 업데이트
		update_key_toggle();

		// debounce 10ms
        msleep(10);
    }

    return 0;
}


// static struct input_dev* rotary_input_dev;
// static int irq_s1, irq_s2;
// static int last_state = 0;


// static irqreturn_t rotary_isr(int irq, void* dev_id) {

// 	// pin 값 얻기
// 	int s1 = gpio_get_value(GPIO_S1);
// 	int s2 = gpio_get_value(GPIO_S2);



// 	// debounce 제거 

//     int state = (s1 << 1) | s2;

//     if ((last_state == 0b00 && state == 0b01) ||
//         (last_state == 0b01 && state == 0b11) ||
//         (last_state == 0b11 && state == 0b10) ||
//         (last_state == 0b10 && state == 0b00)) {
//         input_report_rel(rotary_input_dev, REL_DIAL, 1);  // CW
//     } else if (
//         (last_state == 0b00 && state == 0b10) ||
//         (last_state == 0b10 && state == 0b11) ||
//         (last_state == 0b11 && state == 0b01) ||
//         (last_state == 0b01 && state == 0b00)) {
//         input_report_rel(rotary_input_dev, REL_DIAL, -1); // CCW
//     }

//     last_state = state;
//     input_sync(rotary_input_dev);
//     return IRQ_HANDLED;

// }

int __init rotary_init(void) {

	//int err = 0;

	pr_info("rotary init \n");

	gpio = ioremap(GPIO_BASE, PAGE_SIZE);
    if (!gpio) {
        pr_err("ioremap failed\n");
        return -ENOMEM;
    }

	// GPIO input 설정 (S1, S2)
    GPIO_IN(GPIO_S1);
    GPIO_IN(GPIO_S2);

	// 초기 상태 저장
    last_state = ((GPIO_READ(GPIO_S1) ? 1 : 0) << 1) | (GPIO_READ(GPIO_S2) ? 1 : 0);


	// polling thread 시작
    rotary_thread = kthread_run(rotary_fn, NULL, "rotary_thread");
    if (IS_ERR(rotary_thread)) {
        pr_err("Failed to create thread\n");
        iounmap(gpio);
        return PTR_ERR(rotary_thread);
    }


	// // GPIO 핀 유효성 검사
    // if (!gpio_is_valid(GPIO_S1) || !gpio_is_valid(GPIO_S2)) {
    //     pr_err("Invalid GPIO pins\n");
    //     return -EINVAL;
    // }
	
	// // gpio 등록, input mode 설정
	// err = gpio_request(GPIO_S1, "s1");
	// if (err) {
	// 	pr_err("Failed to request GPIO_S1: %d\n", err);
	// 	return err;
	// }

	// err = gpio_direction_input(GPIO_S1);
	// if (err) {
	// 	pr_err("Failed to set GPIO_S1 as input: %d\n", err);
	// 	gpio_free(GPIO_S1);
	// 	return err;
	// }

	// // err = gpio_request_one(GPIO_S1, GPIOF_IN, "s1");
	// // if (err) {
	// //     pr_err("Failed to request GPIO_S1: %d\n", err);
	// // 	return err;
	// // }

	// irq_s1 = gpio_to_irq(GPIO_S1);
	// if (irq_s1 < 0) {
    // 	pr_err("gpio_to_irq s1 failed: %d\n", irq_s1);
	//     gpio_free(GPIO_S1);
    // 	return irq_s1;
	// }

	// err = gpio_request_one(GPIO_S2, GPIOF_IN, "s2");
	// if (err) {
	//     pr_err("Failed to request GPIO_S2: %d\n", err);
    // 	gpio_free(GPIO_S1);
	// 	return err;
	// }

	// irq_s2 = gpio_to_irq(GPIO_S2);
	// if (irq_s2 < 0) {
    // 	pr_err("gpio_to_irq s2 failed: %d\n", irq_s2);
	//     gpio_free(GPIO_S1);
	//     gpio_free(GPIO_S2);
    // 	return irq_s2;
	// }

	// // input dev 구조체를 위한 메모리 할당
	// rotary_input_dev = input_allocate_device();
	// if(!rotary_input_dev) {
	// 	pr_err("failed to allocate device\n");
	//     gpio_free(GPIO_S1);
	//     gpio_free(GPIO_S2);

	// 	return -ENOMEM;
	// }

	// rotary_input_dev->name = "rotary-encoder";
	// rotary_input_dev->phys = "rotary/input0";
	// rotary_input_dev->id.bustype = BUS_HOST;

	// // register device 보다 먼저 호출
	// // REL_DIAL 이벤트 생성
	// input_set_capability(rotary_input_dev, EV_REL, REL_DIAL);


	// err = input_register_device(rotary_input_dev);
	// if(err) {
	// 	pr_err("failed to register device\n");
	//     gpio_free(GPIO_S1);
	//     gpio_free(GPIO_S2);
	// 	input_free_device(rotary_input_dev);
	// 	return err;
	// }

	// // riging, fall

	// err = request_irq(irq_s1, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s1", NULL);
	// if (err) {
	//     pr_err("Failed to request IRQ for S1: %d\n", err);
	//     gpio_free(GPIO_S1);
	//     gpio_free(GPIO_S2);
		
    // 	input_unregister_device(rotary_input_dev);
    // 	return err;
	// }

	// err = request_irq(irq_s2, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s2", NULL);
	// if (err) {
    // 	pr_err("Failed to request IRQ for S2: %d\n", err);
	//     free_irq(irq_s1, NULL);

	//     gpio_free(GPIO_S1);
	//     gpio_free(GPIO_S2);

    // 	input_unregister_device(rotary_input_dev);
    // 	return err;
	// }

    return 0;
}

void __exit rotary_exit(void) {
	pr_info("rotary exit \n");


	if (rotary_thread) {
        kthread_stop(rotary_thread);
	}

    if (gpio) {
        iounmap(gpio);
	}

    // free_irq(irq_s1, NULL);
    // free_irq(irq_s2, NULL);
    // gpio_free(GPIO_S1);
    // gpio_free(GPIO_S2);
    // input_unregister_device(rotary_input_dev);
}

// 서브 모듈로 사용되므로 주석
// module_init(rotary_init);
// module_exit(rotary_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("Rotary sw submodule");
