#define pr_fmt(fmt) "[rotary] " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h> // input 사용

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>


#define GPIO_S1 17
#define GPIO_S2 27
#define GPIO_KEY 22


static struct input_dev* rotary_input_dev;
static int irq_s1, irq_s2;
static int last_state = 0;


static irqreturn_t rotary_isr(int isq, void* dev_id) {

	// pin 값 얻기
	int s1 = gpio_get_value(GPIO_S1);
	int s2 = gpio_get_value(GPIO_S2);



	// debounce 제거 

    int state = (s1 << 1) | s2;

    if ((last_state == 0b00 && state == 0b01) ||
        (last_state == 0b01 && state == 0b11) ||
        (last_state == 0b11 && state == 0b10) ||
        (last_state == 0b10 && state == 0b00)) {
        input_report_rel(rotary_input_dev, REL_DIAL, 1);  // CW
    } else if (
        (last_state == 0b00 && state == 0b10) ||
        (last_state == 0b10 && state == 0b11) ||
        (last_state == 0b11 && state == 0b01) ||
        (last_state == 0b01 && state == 0b00)) {
        input_report_rel(rotary_input_dev, REL_DIAL, -1); // CCW
    }

    last_state = state;
    input_sync(rotary_input_dev);
    return IRQ_HANDLED;

}

static int __init rotary_init(void) {

	int err = 0;

	pr_info("rotary init \n");
	
	// gpio 등록, input mode 설정
	err = gpio_request_one(GPIO_S1, GPIOF_IN, "s1");
	if (err) {
	    pr_err("Failed to request GPIO_S1: %d\n", err);
		return err;
	}

	irq_s1 = gpio_to_irq(GPIO_S1);
	if (irq_s1 < 0) {
    	pr_err("gpio_to_irq s1 failed: %d\n", irq_s1);
	    gpio_free(GPIO_S1);
    	return irq_s1;
	}

	err = gpio_request_one(GPIO_S2, GPIOF_IN, "s2");
	if (err) {
	    pr_err("Failed to request GPIO_S2: %d\n", err);
    	gpio_free(GPIO_S1);
		return err;
	}

	irq_s2 = gpio_to_irq(GPIO_S2);
	if (irq_s2 < 0) {
    	pr_err("gpio_to_irq s2 failed: %d\n", irq_s2);
	    gpio_free(GPIO_S1);
	    gpio_free(GPIO_S2);
    	return irq_s2;
	}

	// input dev 구조체를 위한 메모리 할당
	rotary_input_dev = input_allocate_device();
	if(!rotary_input_dev) {
		pr_err("failed to allocate device\n");
	    gpio_free(GPIO_S1);
	    gpio_free(GPIO_S2);

		return -ENOMEM;
	}

	rotary_input_dev->name = "rotary-encoder";
	rotary_input_dev->phys = "rotary/input0";
	rotary_input_dev->id.bustype = BUS_HOST;

	// register device 보다 먼저 호출
	// REL_DIAL 이벤트 생성
	input_set_capability(rotary_input_dev, EV_REL, REL_DIAL);


	err = input_register_device(rotary_input_dev);
	if(err) {
		pr_err("failed to register device\n");
	    gpio_free(GPIO_S1);
	    gpio_free(GPIO_S2);

		return err;
	}

	// riging, fall

	err = request_irq(irq_s1, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s1", NULL);
	if (err) {
	    pr_err("Failed to request IRQ for S1: %d\n", err);
	    gpio_free(GPIO_S1);
	    gpio_free(GPIO_S2);
		
    	input_unregister_device(rotary_input_dev);
    	return err;
	}

	err = request_irq(irq_s2, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s2", NULL);
	if (err) {
    	pr_err("Failed to request IRQ for S2: %d\n", err);
	    free_irq(irq_s1, NULL);

	    gpio_free(GPIO_S1);
	    gpio_free(GPIO_S2);

    	input_unregister_device(rotary_input_dev);
    	return err;
	}

    return 0;
}

static void __exit rotary_exit(void) {
	pr_info("rotary exit \n");
    free_irq(irq_s1, NULL);
    free_irq(irq_s2, NULL);
    gpio_free(GPIO_S1);
    gpio_free(GPIO_S2);
    input_unregister_device(rotary_input_dev);
}

module_init(rotary_init);
module_exit(rotary_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("Simple Hello World module");
