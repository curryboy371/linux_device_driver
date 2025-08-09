#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#define VS1003_DEVICE "/dev/vs1003"
#define BUFFER_SIZE 8192

int main(int argc, char *argv[]) {
    int mp3_fd, vs1003_fd;
    ssize_t read_bytes, write_bytes;
    ssize_t read_total = 0, write_total = 0;
    off_t file_size = 0;
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

    // 2. MP3 파일 크기 구하기
    struct stat st;
    if (fstat(mp3_fd, &st) == -1) {
        perror("파일 크기 확인 실패");
        close(mp3_fd);
        return 1;
    }
    file_size = st.st_size;

    // 3. VS1003 디바이스 열기
    vs1003_fd = open(VS1003_DEVICE, O_WRONLY);
    if (vs1003_fd < 0) {
        perror("VS1003 디바이스 열기 실패");
        close(mp3_fd);
        return 1;
    }

    printf("🎧 MP3 파일 '%s'을(를) VS1003으로 스트리밍 시작합니다...\n", argv[1]);

    // 4. MP3 데이터를 읽고 스트리밍
    while ((read_bytes = read(mp3_fd, buffer, sizeof(buffer))) > 0) {
        read_total += read_bytes;

        ssize_t offset = 0;
        while (offset < read_bytes) {
            write_bytes = write(vs1003_fd, buffer + offset, read_bytes - offset);
            if (write_bytes < 0) {
                perror("VS1003 디바이스에 쓰기 실패");
                goto cleanup;
            }
            offset += write_bytes;
            write_total += write_bytes;
        }

        // 진행률 출력
        float percent = (read_total / (float)file_size) * 100.0f;
        printf("\r진행률: %.2f%%", percent);
        fflush(stdout);

        // DREQ는 커널에서 처리하므로 간단한 sleep
        //usleep(1000);  // 1ms
    }

    if (read_bytes < 0) {
        perror("MP3 파일 읽기 실패");
    }

    printf("\n✅ 스트리밍 완료!\n");
    printf("총 읽은 바이트: %zd bytes\n", read_total);
    printf("총 쓴 바이트:   %zd bytes\n", write_total);

cleanup:
    close(mp3_fd);
    close(vs1003_fd);
    return 0;
}
