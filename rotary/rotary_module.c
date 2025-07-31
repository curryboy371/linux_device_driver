#define pr_fmt(fmt) "[rotary] " fmt
#include "rotary_module.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/kthread.h>

#define DEVICE_NAME "rotary"


#define COUNT_MIN 0
#define COUNT_MAX 15
#define DEBOUNCE_DELAY_MS 1 // rotary 디바운스 지연 시간 (밀리초)
#define KEY_DEBOUNCE_DELAY_MS 20 // key 디바운스 지연 시간 (밀리초)



static unsigned long DEBOUNCE_JIFFIES;
static unsigned long KEY_DEBOUNCE_JIFFIES;


#define HISTORY_SIZE 4
#define HISTORY_MASK 0xFF

// 정석대로 하면 인식이 잘 안됨
// 회전 로그를 찍어보고, 인식되는 패턴을 찾아 배열에 넣고 매칭시키는 방식을 사용
static const uint8_t cw_patterns[] = {
    0x1E, 0x78, 0xE1, 0x87
};

static const uint8_t ccw_patterns[] = {
    0x4D, 0x34, 0xD3, 0x8D
};

// rotary driver 데이터 구조체
struct rotary_data {
    struct input_dev* input_dev;
    struct gpio_desc *s1_gpio_desc;
    struct gpio_desc *s2_gpio_desc;
    struct gpio_desc *key_gpio_desc;
    int irq_s1, irq_s2, irq_key;
    struct delayed_work rotary_dw; // 로터리 엔코더 디바운싱용 delayed work
    struct delayed_work key_dw;    // 버튼 디바운싱용 delayed work
    struct task_struct *polling_thread; // 로터리 thread ( polling 방식으로 체크하기 위함)
    uint8_t shift_reg;
    //int last_rotary_state; // 이전 유효한 로터리 상태 (s1 << 1) | s2
    int count;             // 현재 로터리 카운트
    int toggle_state;      // 버튼 토글 상태
    int direction;         // 마지막 회전 방향 (1: CW, -1: CCW, 0: 없음)
};

static struct rotary_data *g_rotary_data = NULL; // 전역 데이터 포인터
static int rotary_ready = 0; // 드라이버 ready

////// Export 함수
int rotary_get_count(void) {
    return (g_rotary_data && rotary_ready) ? g_rotary_data->count : 0;
}
EXPORT_SYMBOL(rotary_get_count);

int rotary_get_toggle(void) {
    return (g_rotary_data && rotary_ready) ? g_rotary_data->toggle_state : 0;
}
EXPORT_SYMBOL(rotary_get_toggle);

int rotary_get_direction(void) {
    return (g_rotary_data && rotary_ready) ? g_rotary_data->direction : 0;
}
EXPORT_SYMBOL(rotary_get_direction);

int rotary_is_ready(void) {
    return rotary_ready;
}
EXPORT_SYMBOL(rotary_is_ready);


static bool match_pattern(uint8_t value, const uint8_t *patterns, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (value == patterns[i])
            return true;
    }
    return false;
}

static int rotary_polling_fn(void *arg)
{
    struct rotary_data *data = arg;

    int prev_s1 = -1;
    int prev_s2 = -1;

    while (!kthread_should_stop()) {
        int s1 = gpiod_get_value(data->s1_gpio_desc);
        int s2 = gpiod_get_value(data->s2_gpio_desc);

        // 이전 상태와 같으면 처리 생략
        if (s1 == prev_s1 && s2 == prev_s2) {
            msleep(DEBOUNCE_DELAY_MS);
            continue;
        }

        prev_s1 = s1;
        prev_s2 = s2;

        int cur_state = (s2 << 1) | s1;

        // shift_reg 갱신 (2bit씩 shift, 최신 샘플이 LSB)
        data->shift_reg = ((data->shift_reg << 2) | cur_state);

        uint8_t recent = data->shift_reg;

        //pr_info("[rotary] s1=%d, s2=%d, shift_reg=0x%02X\n", s1, s2, data->shift_reg);

        // 패턴 매칭
        if (match_pattern(recent, cw_patterns, ARRAY_SIZE(cw_patterns))) {
            if(data->count < COUNT_MAX) {
                data->count++;
                data->direction = 1;
                pr_info("[rotary] CW detected, count = %d\n", data->count);
                input_report_rel(data->input_dev, REL_DIAL, 1);
                input_sync(data->input_dev);
            }

        } else if (match_pattern(recent, ccw_patterns, ARRAY_SIZE(ccw_patterns))) {
            if(data->count > COUNT_MIN) {
                data->count--;
                data->direction = -1;
                pr_info("[rotary] CCW detected, count = %d\n", data->count);
                input_report_rel(data->input_dev, REL_DIAL, -1);
                input_sync(data->input_dev);
            }
        }

        msleep(DEBOUNCE_DELAY_MS);
    }

    return 0;
}

// //// work function
// 인터럽트 방식으로 dw 회전 로그 보니 중간단계가 생략되서 인식이 잘 안됨
// static void rotary_work_func(struct work_struct *work) {
//     struct rotary_data *data = container_of(work, struct rotary_data, rotary_dw.work);

//     static uint8_t state_buf[STEP_BUF_LEN] = {0xFF, 0xFF, 0xFF, 0xFF};
//     static int buf_index = 0;

//     int s1 = gpiod_get_value(data->s1_gpio_desc);
//     int s2 = gpiod_get_value(data->s2_gpio_desc);
//     uint8_t current_state = (s1 << 1) | s2;

//     pr_info("rotary irq: s1=%d, s2=%d, current_state=%d\n", s1, s2, current_state);

//     // 상태 버퍼에 현재 상태 저장
//     state_buf[buf_index] = current_state;
//     buf_index = (buf_index + 1) % STEP_BUF_LEN;

//     pr_info("state_buf: [%d %d %d %d] (buf_index=%d)\n",
//         state_buf[0], state_buf[1], state_buf[2], state_buf[3], buf_index);

//     // 비교할 회전 시퀀스 정의
//     const uint8_t cw_seq[STEP_BUF_LEN]  = {0b00, 0b01, 0b11, 0b10};
//     const uint8_t ccw_seq[STEP_BUF_LEN] = {0b00, 0b10, 0b11, 0b01};

//     // 순환 비교
//     bool match_cw = true, match_ccw = true;
//     for (int i = 0; i < STEP_BUF_LEN; i++) {
//         int idx = (buf_index + i) % STEP_BUF_LEN;
//         if (state_buf[idx] != cw_seq[i]) match_cw = false;
//         if (state_buf[idx] != ccw_seq[i]) match_ccw = false;
//     }

//     pr_info("match_cw=%d, match_ccw=%d\n", match_cw, match_ccw);

//     if (match_cw && data->count < COUNT_MAX) {
//         data->count++;
//         data->direction = 1;
//         pr_info("CW detected: count=%d\n", data->count);
//         input_report_rel(data->input_dev, REL_DIAL, 1);
//         input_sync(data->input_dev);
//         memset(state_buf, 0xFF, STEP_BUF_LEN);
//     }
//     else if (match_ccw && data->count > COUNT_MIN) {
//         data->count--;
//         data->direction = -1;
//         pr_info("CCW detected: count=%d\n", data->count);
//         input_report_rel(data->input_dev, REL_DIAL, -1);
//         input_sync(data->input_dev);
//         memset(state_buf, 0xFF, STEP_BUF_LEN);
//     } else {
//         data->direction = 0;
//     }
// }


// 이 방식은 디바운스 처리가 제대로 안돼
// 로터리 엔코더 워크 함수: 디바운싱 후 실제 값 처리
// static void rotary_work_func(struct work_struct *work) {
//     struct rotary_data *data = container_of(work, struct rotary_data, rotary_dw.work);
    
//     int s1_1 = gpiod_get_value(data->s1_gpio_desc);
//     int s2_1 = gpiod_get_value(data->s2_gpio_desc);
//     // 짧게 한번 더 delay
//      msleep(2);  
//     int s1_2 = gpiod_get_value(data->s1_gpio_desc);
//     int s2_2 = gpiod_get_value(data->s2_gpio_desc);

//     if (s1_1 != s1_2 || s2_1 != s2_2) {
//         return;
//     }

//     // {s1, s2}
//     int current_state = (s1_2 << 1) | s2_2;
//     int dir = rotary_table[(data->last_rotary_state << 2) | current_state];


//     if (dir == 1 && data->count < COUNT_MAX) { // // 시계 방향 (CW)
//         data->count++;
//         data->direction = 1;
//         pr_info("CW : count %d\n", data->count);
//         input_report_rel(data->input_dev, REL_DIAL, 1);
//     } else if (dir == -1 && data->count > COUNT_MIN) { // // 반시계 방향 (CCW)
//         data->count--;
//         data->direction = -1;
//         pr_info("CCW : count %d\n", data->count);
//         input_report_rel(data->input_dev, REL_DIAL, -1);
//     } else {
//         data->direction = 0;
//     }

//     // 유효한 방향 변화가 있을때만 last state 업데이트
//     if (dir != 0) {
//         data->last_rotary_state = current_state;
//     }

//     // input event 동기화
//     input_sync(data->input_dev);
// }

// 버튼 워크 함수: 디바운싱 후 실제 값 처리
static void key_work_func(struct work_struct *work) {
    struct rotary_data *data = container_of(work, struct rotary_data, key_dw.work);
    int key_value = gpiod_get_value(data->key_gpio_desc); 
    
    if (key_value == 1) {  // 버튼이 눌림
        data->toggle_state ^= 1; // 토글 상태 반전
        pr_info("toggle state: %d\n", data->toggle_state);

    } else { // 버튼이 떼어졌을 때

    }

    // 키 눌림 이벤트 보고
    input_report_key(data->input_dev, KEY_ENTER, key_value); 

    // 입력 이벤트 동기화
    input_sync(data->input_dev);

}

//// ISR 함수

// 로터리 엔코더 IRQ 핸들러
// static irqreturn_t rotary_isr(int irq, void* dev_id) {
//     struct rotary_data *data = (struct rotary_data *)dev_id;

//     // 이미 예약된 인터럽트가 있다면 취소 ( 중복 실행 방지 )
//     cancel_delayed_work(&data->rotary_dw);
    
//     // DEBOUNCE_JIFFIES 후에 rotary_work_func enqueue
//     schedule_delayed_work(&data->rotary_dw, DEBOUNCE_JIFFIES);
    
//     return IRQ_HANDLED;
// }

// 버튼 IRQ 핸들러
static irqreturn_t key_isr(int irq, void* dev_id) {
    struct rotary_data *data = (struct rotary_data *)dev_id;
    
    // 이미 예약된 인터럽트가 있다면 취소 ( 중복 실행 방지 )
    cancel_delayed_work(&data->key_dw);

    // KEY_DEBOUNCE_JIFFIES 후에 rotary_work_func enqueue
    schedule_delayed_work(&data->key_dw, KEY_DEBOUNCE_JIFFIES);
    
    return IRQ_HANDLED;
}

/* --- 플랫폼 드라이버 등록 --- */

static int rotary_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct rotary_data *data;
    int err;
    
    pr_info("rotary_probe started\n");
    
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;
        
    // DEBOUNCE를 jiffies 단위로
    // 리눅스 커널 내 타이머, 워크큐는 jiffies 단위로 동작함
    DEBOUNCE_JIFFIES = msecs_to_jiffies(DEBOUNCE_DELAY_MS);
    KEY_DEBOUNCE_JIFFIES = msecs_to_jiffies(KEY_DEBOUNCE_DELAY_MS);

    // device tree gpio와 매칭
    // devm은 리소스 자동 관리
    data->s1_gpio_desc = devm_gpiod_get(dev, "rotary-s1", GPIOD_IN);
    if (IS_ERR(data->s1_gpio_desc)) {
        pr_err("Failed to get rotary-s1 GPIO: %ld\n", PTR_ERR(data->s1_gpio_desc));
        return PTR_ERR(data->s1_gpio_desc);
    }

    data->s2_gpio_desc = devm_gpiod_get(dev, "rotary-s2", GPIOD_IN);
    if (IS_ERR(data->s2_gpio_desc)) {
        pr_err("Failed to get rotary-s2 GPIO: %ld\n", PTR_ERR(data->s2_gpio_desc));
        return PTR_ERR(data->s2_gpio_desc);
    }

    data->key_gpio_desc = devm_gpiod_get(dev, "rotary-key", GPIOD_IN);
    if (IS_ERR(data->key_gpio_desc)) {
        pr_err("Failed to get rotary-key GPIO: %ld\n", PTR_ERR(data->key_gpio_desc));
        return PTR_ERR(data->key_gpio_desc);
    }

    // 초기 상태 설정
    data->count = 0;
    data->toggle_state = 0;
    data->direction = 0;

    // input device 생성
    data->input_dev = devm_input_allocate_device(dev);
    if (!data->input_dev) {
        pr_err("Failed to allocate input device\n");
        return -ENOMEM;
    }

    data->input_dev->name = "Rotary Encoder with Button";
    data->input_dev->phys = "rotary/input0";
    data->input_dev->id.bustype = BUS_VIRTUAL;
    data->input_dev->dev.parent = dev;
    
    input_set_capability(data->input_dev, EV_REL, REL_DIAL); // 로터리 엔코더 회전 (상대적 변화)
    input_set_capability(data->input_dev, EV_KEY, KEY_ENTER); // 버튼 (Enter 키)

    err = input_register_device(data->input_dev);
    if (err) {
        pr_err("Failed to register input device: %d\n", err);
        return err;
    }

    // Interrupt routine Queue
    data->irq_s1 = gpiod_to_irq(data->s1_gpio_desc);
    data->irq_s2 = gpiod_to_irq(data->s2_gpio_desc);
    data->irq_key = gpiod_to_irq(data->key_gpio_desc);

    pr_info("IRQ numbers: s1=%d, s2=%d, key=%d\n", data->irq_s1, data->irq_s2, data->irq_key);

    if ((data->irq_s1 < 0) || (data->irq_s2 < 0) || (data->irq_key < 0)) {
        pr_err("Invalid IRQ numbers\n");
        return -EINVAL;
    }

    // S1 rising, falling 인터럽트 발생
    // err = devm_request_irq(dev, data->irq_s1, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s1_isr", data);
    // if (err) {
    //     pr_err("Failed to request IRQ for s1: %d\n", err);
    //     return err;
    // }
    
    // s2 rising, falling 인터럽트 발생
    // err = devm_request_irq(dev, data->irq_s2, rotary_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_s2_isr", data);
    // if (err) {
    //     pr_err("Failed to request IRQ for s2: %d\n", err);
    //     return err;
    // }
    
    // key rising, falling 인터럽트 발생
    err = devm_request_irq(dev, data->irq_key, key_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_key_isr", data);
    if (err) {
        pr_err("Failed to request IRQ for key: %d\n", err);
        return err;
    }

    // work queue 초기화
    // 커널에서 시간이 오래 걸리는 작업을 ISR에서 직접 수행하지 않고
    // 미뤄두고 프로세스 컨텍스트에서 안전하게 수행하도록 함
    //INIT_DELAYED_WORK(&data->rotary_dw, rotary_work_func);
    INIT_DELAYED_WORK(&data->key_dw, key_work_func);

    // dw는 polling 방식으로 처리하기 위해 thread
    // 모듈 초기화 시
    data->polling_thread = kthread_run(rotary_polling_fn, data, "rotary_polling_thread");
    if (IS_ERR(data->polling_thread)) {
        pr_err("Failed to create polling thread\n");
        return PTR_ERR(data->polling_thread);
    }
    
    // 플랫폼 디바이스와 this driver간 data를 공유하도록 set
    platform_set_drvdata(pdev, data);
    g_rotary_data = data;

    // rotary ready ok
    rotary_ready = 1;
    pr_info("Rotary platform driver probed successfully\n");
    
    return 0;
}

static void rotary_remove(struct platform_device *pdev) {
    struct rotary_data *data = platform_get_drvdata(pdev);
    
    pr_info("rotary_remove\n");
    
    rotary_ready = 0;
    
    if (data) {
        // 모듈 제거 시 진행 중인 모든 delayed work를 취소
        //cancel_delayed_work_sync(&data->rotary_dw);
        cancel_delayed_work_sync(&data->key_dw);
        g_rotary_data = NULL;

        if (data->polling_thread) {
            kthread_stop(data->polling_thread);
        }
    }
}

/* Device Tree 매칭 테이블 */
static const struct of_device_id rotary_of_match[] = {
    { .compatible = "chan,rotary-overlay", }, // DTS의 compatible 문자열과 일치해야함
    { }
};
MODULE_DEVICE_TABLE(of, rotary_of_match);

// platfomr driver 구조체
// 하드웨어가 버스(PCI, USB 등)에 연결되어 있지 않을 때 사용함

static struct platform_driver rotary_driver = {
    .probe = rotary_probe,
    .remove = rotary_remove,
    .driver = {
        .name = DEVICE_NAME, // dmesg에서 이 이름으로 드라이버를 식별할 수 있습니다.
        .of_match_table = rotary_of_match,
    },
};

static int __init rotary_init(void) {
    pr_info("rotary driver init\n");
    return platform_driver_register(&rotary_driver);
}

static void __exit rotary_exit(void) {
    pr_info("rotary driver exit\n");
    platform_driver_unregister(&rotary_driver);
}

module_init(rotary_init);
module_exit(rotary_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chan");
MODULE_DESCRIPTION("Rotary switch platform driver with improved debouncing");