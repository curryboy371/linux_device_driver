#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#define VS1003_DEVICE "/dev/vs1003"
#define BUFFER_SIZE 256

int main(int argc, char *argv[]) {
    int mp3_fd, vs1003_fd;
    ssize_t read_bytes, write_bytes;
    ssize_t read_total = 0, write_total = 0;
    char buffer[BUFFER_SIZE];

    if (argc != 2) {
        fprintf(stderr, "사용법: %s <MP3 파일 경로>\n", argv[0]);
        return 1;
    }

    // 1. MP3 파일 열기
    mp3_fd = open(argv[1], O_RDONLY);
    if (mp3_fd < 0) {
        perror("MP3 파일 열기 실패");
        return 1;
    }

    // 2. VS1003 디바이스 파일 열기
    vs1003_fd = open(VS1003_DEVICE, O_WRONLY);
    if (vs1003_fd < 0) {
        perror("VS1003 디바이스 열기 실패");
        close(mp3_fd);
        return 1;
    }

    printf("MP3 파일 '%s'을(를) VS1003 디바이스로 스트리밍 시작\n", argv[1]);

    // 3. MP3 파일 내용을 읽어 VS1003 디바이스로 쓰기
    while ((read_bytes = read(mp3_fd, buffer, sizeof(buffer))) > 0) {
        read_total += read_bytes;

        write_bytes = write(vs1003_fd, buffer, read_bytes);
        if (write_bytes < 0) {
            perror("VS1003 디바이스에 쓰기 실패");
            break;
        }

        write_total += write_bytes;

        if (write_bytes != read_bytes) {
            fprintf(stderr, "경고: 일부 데이터만 쓰여졌습니다. (%zd / %zd)\n", write_bytes, read_bytes);
        }

        if (write_bytes == BUFFER_SIZE)
            usleep(2000);
        else
            usleep(5000);

    }

    if (read_bytes < 0) {
        perror("MP3 파일 읽기 실패");
    }

    printf("스트리밍 완료\n");
    printf("총 읽은 바이트: %zd bytes\n", read_total);
    printf("총 쓴 바이트:   %zd bytes\n", write_total);

    // 4. 파일 닫기
    close(mp3_fd);
    close(vs1003_fd);

    return 0;
}
