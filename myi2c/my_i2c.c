// pr_debug, pr_info, pr_err 메시지 접두사 macro
#define pr_fmt(fmt) "[my_i2c] " fmt

#include <linux/module.h>        // module_init, module_exit, MODULE_LICENSE 등
#include <linux/fs.h>            // character device
#include <linux/cdev.h>          // cdev
#include <linux/gpio.h>          // gpio 사용
#include <linux/device.h>        // device 등록

#include <linux/uaccess.h>   // copy_from_user, copy_to_user
#include <linux/delay.h>         // udelay

#include <linux/kernel.h> // pr log

#include <linux/mutex.h>

#include "my_i2c.h"



/*


*/


// mutex 로 동시 접근 처리
// mutex 처리는 추후
static DEFINE_MUTEX(my_i2c_lock);

static i2c_state_t my_i2c_state = I2C_STATE_IDLE;


static dev_t my_i2c_dev_num;
static struct cdev my_i2c_cdev;
static struct class *my_i2c_class;

// file private data
typedef struct my_i2c_session {
    uint8_t slave_addr;

} my_i2c_session_t;


// i2c start condition
i2c_error_t my_i2c_start(void) {

    int ret = 0;
	// scl이 high 상태일 때 sda 라인이 high to low로 전환
	// 초기상태 scl high
    

	// SDA OUTPUT + LOW
	ret = gpio_direction_output(I2C_SDA_GPIO, LOW);
    if(ret < 0) {
        pr_err("Failed to set SDA GPIO as output\n");
        return I2C_ERR_UNKNOWN;
    }

	// start hold time : 4.0us 
	udelay(4);
	
	// clock
	// scl low hold time ( 4.7us)
	gpio_set_value(I2C_SCL_GPIO, LOW);
	udelay(4.7);

	// scl high hold time ( 4.0us)
	gpio_set_value(I2C_SCL_GPIO, HIGH);
	udelay(4.0);

    return I2C_ERR_NONE;
}
	
i2c_error_t my_i2c_stop(void) {

	// scl이 high 일 때 sda 라인이 low에서 high로 전환

    // SDA stop condition output + LOW (SDA HIGH 보장을 위함)
    if (gpio_direction_output(I2C_SDA_GPIO, LOW) < 0) {
        pr_err("Failed to set SDA LOW for stop condition\n");
        return I2C_ERR_UNKNOWN;
    }


	// stop setup time 4.0 us
    // scl high hold time 4.0 us
	gpio_set_value(I2C_SCL_GPIO, HIGH);
	udelay(4.0);

    // SDA HIGH stop condition
    gpio_set_value(I2C_SDA_GPIO, HIGH);


	// bus free time( sda high hold time 대체)
	// restart condition 4.7us
	udelay(4.7);
	

    return I2C_ERR_NONE;

}

i2c_error_t my_i2c_write_byte(uint8_t data) {

	// write byte

    // 여기 들어올 때 scl은 HIGH 상태여야 함
	
	// msb to lsb 
	for(int i = 7; i >= 0; --i) {

        // scl low
        // scl low hold time 4.7us
        gpio_set_value(I2C_SCL_GPIO, LOW);
	    udelay(4.7);

        // sda 라인에 data bit write
		if(gpio_direction_output(I2C_SDA_GPIO, (data >> i) & 1) < 0) {
            pr_err("Failed to set SDA GPIO as output\n");
            return I2C_ERR_UNKNOWN;
        }

        // sda hold time 5.0us
        udelay(5);

        // scl high setup time 250ns
        // 이 time은 sda hold time에 종속됨
        // udelay(0.25); // 250ns
        
        // scl low to high
        gpio_set_value(I2C_SCL_GPIO, HIGH);

        // scl high hold time 4.0us
        udelay(4.0);
	}

    return I2C_ERR_NONE;

}

i2c_error_t my_i2c_read_byte(uint8_t *byte, int send_ack) {


    i2c_error_t ret = I2C_ERR_NONE;
    int data = 0;


    // SDA를 입력 모드로 설정
    if(gpio_direction_input(I2C_SDA_GPIO) < 0) {
        pr_err("Failed to set SDA GPIO as input\n");
        return I2C_ERR_UNKNOWN;
    } 

    for(int i =0; i < 8; ++i) {

        // SCL low hold time 4.7us
        gpio_set_value(I2C_SCL_GPIO, LOW);
        udelay(4.7);

        // SCL high hold time 4.0us
        gpio_set_value(I2C_SCL_GPIO, HIGH);
        udelay(4.0);

        data <<= 1;
        // SDA 확인 후 data에 입력
        if(gpio_get_value(I2C_SDA_GPIO)) {
            data |= 1;  // 1 입력
        } else {
            data &= ~1; // 0 입력
        }
    }

    // Master ACK/NACK
    ret = my_i2c_master_ack(send_ack);
    if(ret != I2C_ERR_NONE) {
        return ret;
    }

    *byte = data;

    return ret;
}


// slave의 receiver acknowledge
i2c_error_t my_i2c_ack(void) {

    int ack_received = 0;

    // slave to master ack

    // scl은 HIGH 상태여야 함

    // sda를 input mode
    gpio_direction_input(I2C_SDA_GPIO); 

    // scl low hold time 4.7us
    gpio_set_value(I2C_SCL_GPIO, LOW);
    udelay(4.7);

    // scl low to high
    gpio_set_value(I2C_SCL_GPIO, HIGH);

    // ACK 수신 대기
    for(int i = 0; i < I2C_ACK_TIMEOUT_US; ++i) {
        if(gpio_get_value(I2C_SDA_GPIO) == I2C_ACK) {
            ack_received = 1; // ACK 수신
            break;
        }
        udelay(1);
    }

    // scl high hold time 4.0us
    udelay(4.0);

    // ACK 수신 실패
    if(!ack_received) {
        pr_err("ACK not received\n");
        return I2C_ERR_NO_ACK; 
    }

    return I2C_ERR_NONE;

}

// master의 receiver acknowledge
i2c_error_t my_i2c_master_ack(int ack) {

    // scl은 HIGH 상태여야 함

    // scl low hold time 4.7us
    gpio_set_value(I2C_SCL_GPIO, LOW);
    udelay(4.7);

    // sda를 input mode
    if(gpio_direction_output(I2C_SDA_GPIO, ack) < 0) {
        pr_err("Failed to set SDA GPIO as output\n");
        return I2C_ERR_UNKNOWN;
    }

    // scl low to high
    gpio_set_value(I2C_SCL_GPIO, HIGH);

    // scl high hold time 4.0us
    udelay(4.0);

    return I2C_ERR_NONE;
}

static int my_i2c_open(struct inode *inode, struct file *file) {
    pr_info("device opened\n");

    sess = kmalloc(sizeof(*sess), GFP_KERNEL);
    if (!sess) {
        return -ENOMEM;
    }

    sess->slave_addr = 0x00;     // 기본값
    file->private_data = sess;

    return 0;
}

static int my_i2c_release(struct inode *inode, struct file *file) {
    pr_info("device closed\n");

    struct my_i2c_session *sess = file->private_data;
    
    if (sess) {
        kfree(sess);
    }


    return 0;
}

static ssize_t my_i2c_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    pr_info("read called\n");

    struct my_i2c_session *sess = file->private_data;
    if(sess == NULL) {
        pr_err("Session not initialized\n");
        return -EINVAL;
    }


    if(sess->slave_addr == 0x00) {
        pr_err("Slave address not set\n");
        return -EINVAL;
    }

    uint8_t kbuf[MAX_BUF_SIZE]; // stack mem

    if( count > MAX_BUF_SIZE) {
        pr_err("Failed to copy data size\n");
        return -EINVAL;
    }


    int cur_byte = 0;

    if(my_i2c_start() != I2C_ERR_NONE) {
        return -EIO; // I/O error
    }

    // write address + read bit
    if(my_i2c_write_byte((sess->slave_addr << 1) | 1) != I2C_ERR_NONE) {
        my_i2c_stop();
        return -EIO;
    }

    // ACK check
    if(my_i2c_ack() != I2C_ERR_NONE) {
        my_i2c_stop();
        return -EIO; // I/O error
    }

    for (int i = 0; i < count; ++i) {
        int is_not_last = (i < count - 1);
        if (my_i2c_read_byte(&kbuf[i], is_not_last) != I2C_ERR_NONE) {
            my_i2c_stop();
            return -EIO;
        }
    }

    if(my_i2c_stop() != I2C_ERR_NONE) {
        return -EIO;
    }

    if (copy_to_user(buf, kbuf, count)) {
        return -EFAULT;
    }

    return count;
}

static ssize_t my_i2c_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    pr_info("write called\n");


    uint8_t kbuf[MAX_BUF_SIZE]; // stack mem

    // max 길이 초과
    if (count > MAX_BUF_SIZE) {
        pr_err("Failed to copy data size\n");
        return -EINVAL;
    }
    
    // user space data(buf)를 kernel spcae data(kbuf)로 복사
    if (copy_from_user(kbuf, buf, count)) {
        pr_err("Failed to copy data from user\n");
        return -EFAULT;
    }

    if(my_i2c_start() != I2C_ERR_NONE) {
        return -EIO;
    }

    pr_info("I2C data: ");
    for (int i = 0; i < count; ++i) {
        pr_cont("%02X ", kbuf[i]);          // 16진수(2) 한 줄로 이어서 출력

        if(my_i2c_write_byte(kbuf[i]) != I2C_ERR_NONE) {
            my_i2c_stop();
            return -EIO;
        }

        if(my_i2c_ack() != I2C_ERR_NONE) {
            my_i2c_stop();
            return -EIO;
        }

    }
    pr_cont("\n");
    
    if(my_i2c_stop() != I2C_ERR_NONE) {
        return -EIO;
    }

    return count;
}

// file_operations 구조체
static const struct file_operations my_i2c_fops = {
    .owner = THIS_MODULE,
    .open = my_i2c_open,
    .release = my_i2c_release,
    .read = my_i2c_read,
    .write = my_i2c_write,
};

// myi2c module init
static int __init my_i2c_init(void) {
    int ret = 0;

    pr_info("driver init...\n");

    // GPIO 핀 유효성 검사
    if (!gpio_is_valid(I2C_SDA_GPIO) || !gpio_is_valid(I2C_SCL_GPIO)) {
        pr_err("Invalid GPIO pins\n");
        return -EINVAL;
    }

    // GPIO 핀 Request ( Kernel에 pin 사용 요청)
    // 출력모드, 초기값 high ( pull up )
    ret = gpio_request_one(I2C_SDA_GPIO, GPIOF_OUT_INIT_HIGH, "i2c_sda");
    if (ret < 0) {
        pr_err("Failed to request SDA GPIO %d\n", I2C_SDA_GPIO);
        return ret;
    }

    ret = gpio_request_one(I2C_SCL_GPIO, GPIOF_OUT_INIT_HIGH, "i2c_scl");
    if (ret < 0) {
        pr_err("Failed to request SCL GPIO %d\n", I2C_SCL_GPIO);

        gpio_free(I2C_SDA_GPIO); // request된 sda는 해제
        return ret;
    }

    // Character Device 등록 - device 번호 할당
    ret = alloc_chrdev_region(&my_i2c_dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate char device\n");
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // 디바이스 커널에 등록
    cdev_init(&my_i2c_cdev, &my_i2c_fops);
    my_i2c_cdev.owner = THIS_MODULE;

    ret = cdev_add(&my_i2c_cdev, my_i2c_dev_num, 1);
    if (ret < 0) {
        pr_err("Failed to add char device\n");
        unregister_chrdev_region(my_i2c_dev_num, 1); // 장치 번호 해제
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // 디바이스의 클래스를 sysfs에 등록 sys/class... (관리 목적)
    // 디바이스가 어떤 그룹에 속하는가
    my_i2c_class = class_create("my_i2c_class");
    if (IS_ERR(my_i2c_class)) {
        pr_err("Failed to create device class\n");
        ret = PTR_ERR(my_i2c_class);
        cdev_del(&my_i2c_cdev); // cdev 삭제
        unregister_chrdev_region(my_i2c_dev_num, 1);
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }

    // device file 생성 ( /dev...)
    if (device_create(my_i2c_class, NULL, my_i2c_dev_num, NULL, "my_i2c_dev") == NULL) {
        pr_err("Failed to create device file\n");
        ret = -ENOMEM;
        class_destroy(my_i2c_class); // 클래스 삭제
        cdev_del(&my_i2c_cdev);
        unregister_chrdev_region(my_i2c_dev_num, 1);
        gpio_free(I2C_SCL_GPIO);
        gpio_free(I2C_SDA_GPIO);
        return ret;
    }


    g_slave_addr = 0x00;
    my_i2c_state = I2C_STATE_IDLE;

    pr_info("driver loaded successfully (Major: %d, Minor: %d)\n", MAJOR(my_i2c_dev_num), MINOR(my_i2c_dev_num));
    return 0;
}

// myi2c module exit
static void __exit my_i2c_exit(void) {
    pr_info("driver exit...\n");

    device_destroy(my_i2c_class, my_i2c_dev_num);
    class_destroy(my_i2c_class);
    cdev_del(&my_i2c_cdev);
    unregister_chrdev_region(my_i2c_dev_num, 1);
    gpio_free(I2C_SCL_GPIO);
    gpio_free(I2C_SDA_GPIO);

    pr_info("driver exit successfully\n");

}

module_init(my_i2c_init);
module_exit(my_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("MY I2C Character Device Driver");
MODULE_VERSION("0.1");
