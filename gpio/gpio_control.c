
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/cdev.h> // cdev 사용
#include <linux/io.h> // iomap 사용


/*
 * gpio_control : GPIO 제어 캐릭터 디바이스 드라이버
 *
 * - BCM2711 (라즈베리파이4 SoC) GPIO MMIO를 직접 접근하여 GPIO를 제어하는 커널 모듈
 * - /dev/gpio_control 캐릭터 디바이스 파일 생성 (udev가 자동으로 생성)
 *
 * - write()를 통해 "pin value" 포맷으로 GPIO 출력 제어:
 *       echo "13 1" > /dev/gpio_control  # GPIO 13 출력 High
 *       echo "13 0" > /dev/gpio_control  # GPIO 13 출력 Low
 *
 * - device_open() 시 enable_gpio_pins 배열에 정의된 핀을 OUTPUT 으로 설정
 * - cdev, class, device_create를 이용해 udev에 의해 자동으로 /dev/gpio_control 생성
 */


// 물리적 메모리 주소 def
#define BCM2711_PERI_BASE        0xFE000000
#define GPIO_BASE                (BCM2711_PERI_BASE + 0x200000) /* GPIO controller */

// 커널이 접근할 가상주소
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET(g) (*(gpio+7) = (1 << g))  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR(g) (*(gpio+10) = (1 << g)) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock



#define DEV_NAME "gpio_control"

#define BUF_LEN 8


// 이 파일 내에서만 사용하도록 static화 (name 충돌 방지)
//static int major_num;

static dev_t dev_num;                           // cdev에서 관리하는 num
static struct cdev gpio_control_cdev;           // cdev 구조체
static struct class *gpio_control_class;        // device_create용 class


static char msg[BUF_LEN];
static int msg_len = 0;


// pin info...
static int enable_gpio_pins[] = {13, 19, 26};    // 컨트롤 가능 gpio pins
static int num_pins = sizeof(enable_gpio_pins) / sizeof(enable_gpio_pins[0]);

// open 함수
static int device_open(struct inode *inode, struct file *file) {

    // 사용 가능한 gpio pin out 설정
    for (int i = 0; i < num_pins; i++) {
        int pin = enable_gpio_pins[i];
        INP_GPIO(pin);
        OUT_GPIO(pin);
        pr_info("[gpio_control] Set OUTPUT GPIO pin %d\n", pin);

    }
        
    pr_info("[gpio_control] Device opened\n");
    return 0;
}

// read 함수
static ssize_t device_read(struct file *file, char __user *buffer, size_t length, loff_t *offset) {
    pr_info("[gpio_control] Device read\n");
    return 0;
}

// write 함수
static ssize_t device_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset) {
    if (length > BUF_LEN) {
        length = BUF_LEN;
    }

    ssize_t ret = 0;
    ret = copy_from_user(msg, buffer, length);
    if (ret != 0) {
        pr_warn("[gpio_control] Failed to copy\n");
        return -1;
    }

    // "pin value"
    int pin, value;

    // pin value 파싱
    if (sscanf(msg, "%d %d", &pin, &value) != 2) {
        pr_err("[gpio_control] Invalid input format input {pin value}\n");
        return -1;
    }

    // value check
    if(value < 0) {
        pr_err("[gpio_control] Invalid value %d\n", value);
        return -1;
    }

    // pin check
    int find = 0;
    for (int i = 0; i < num_pins; i++) {
        if(pin == enable_gpio_pins[i]) {
            find = 1;
        }
    }

    if (!find) {
        pr_err("[gpio_control] Invalid pin %d\n", pin);
        return -1;
    }

    value ? GPIO_SET(pin) : GPIO_CLR(pin);

    msg_len = length;

    pr_info("[gpio_control] Written: %c, %d\n", *msg, msg_len);

    return length;
}

// release 함수
static int device_release(struct inode *inode, struct file *file) {
    pr_info("[gpio_control] Device closed\n");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = device_open,
    .read = device_read,
    .write = device_write,
    .release = device_release,
};


// device init func
static int __init gpio_control_init(void)
{
    // 캐릭터 디바이스 등록
    pr_info("[gpio_control] Start Module init\n");

    // register major num

    // register_chrdev 방식
    // major num에 0을 넘기면 커널이 알아서 빈 major number을 할당함 ( 동적 할당 )
    // 유저가 사용하는 dev/ 장치 파일을 major num으로 관리하기 때문에 이 번호를 동적할당하면 장치 파일 등록시 확인해줘야하는 번거로움 발생
    // major_num = register_chrdev(0, DEV_NAME, &fops);
    // if (major_num < 0) {
    //     pr_err("[gpio_control] Error Device registration failed\n");
    //     return major_num;
    // }
    // pr_info("[gpio_control] Major number: %d\n", major_num);


    // cdev 방식
    int ret = 0;

    // dev_t 번호 할당
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) {
        pr_err("[gpio_control] Error alloc_chrdev_region failed\n");
        return ret;
    }

    pr_info("[gpio_control] Major: %d, Minor: %d\n", MAJOR(dev_num), MINOR(dev_num));


    // cdev 구조체 초기화, 등록
    cdev_init(&gpio_control_cdev, &fops);
    gpio_control_cdev.owner = THIS_MODULE;
    ret = cdev_add(&gpio_control_cdev, dev_num, 1);
    if( ret < 0) {
        pr_err("[gpio_control] Error cdev_add failed\n");
    }

    // class 생성 ( udev가 모듈을 감지해서 장치파일 등록하도록)
    gpio_control_class = class_create("gpio_control_class");
    if (IS_ERR(gpio_control_class)) {
        pr_err("[gpio_control] Error class_create failed\n");
        cdev_del(&gpio_control_cdev);
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(gpio_control_class);
    }

    // device 파일 자동 생성 ( udev가 장치파일 생성)
    device_create(gpio_control_class, NULL, dev_num, NULL, DEV_NAME);


    // mmio 장치 접근
    // 커널이 물리주소에 접근할 수 있도록 가상주소와 물리적 주소를 맵핑
    gpio = (volatile unsigned *)ioremap(GPIO_BASE, PAGE_SIZE);
    if (!gpio) {
        pr_err("[gpio_control] Error ioremap GPIO_BASE\n");
        return -1;
    }

    pr_info("[gpio_control] Finished Module init\n");
    return 0;
}

// device exit func
static void __exit gpio_control_exit(void)
{
    // register_chrdev 방식 remove
    //unregister_chrdev(major_num, DEV_NAME);

    // cdev 방식 remove
    device_destroy(gpio_control_class, dev_num);
    class_destroy(gpio_control_class);
    cdev_del(&gpio_control_cdev);
    unregister_chrdev_region(dev_num, 1);


    // gpio unmap
    if (gpio) {
        iounmap(gpio);
    }

    pr_info("[gpio_control] Module removed\n");
}

/////////////////////////////////

module_init(gpio_control_init);
module_exit(gpio_control_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("GPIO Control Character Device Driver");