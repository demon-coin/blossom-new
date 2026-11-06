#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

/*
 * `/dev/kmsg` 内核日志非阻塞导出工具（安装为 `/bin/klog-read`）。
 *
 * 工作原理：
 * 1. 以 O_RDONLY | O_NONBLOCK 模式打开 `/dev/kmsg`，每次调用 read() 从内核日志环形缓冲区读取一条格式化好的记录。
 * 2. 若内核环形缓冲区在读取期间发生回绕覆盖，驱动会返回 EPIPE 并将文件游标自动重置到最早的未覆盖记录，本工具捕获 EPIPE 后直接重试即可继续读取。
 * 3. 当所有现存日志读取完毕时，非阻塞 read() 返回 EAGAIN，工具正常退出（返回 0）。
 */
int main(void)
{
    /* 单条日志正文最长 512 字节，特殊字符转义最坏膨胀 4 倍（2048 字节），加上 96 字节固定头部前缀。 */
    char record[512 * 4 + 96];
    int result = 0;
    int fd = open("/dev/kmsg", O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        perror("klog-read: open /dev/kmsg");
        return 1;
    }

    for (;;) {
        ssize_t count = read(fd, record, sizeof(record));
        if (count == -1) {
            if (errno == EPIPE)
                continue;
            if (errno == EAGAIN)
                break;
            perror("klog-read: read /dev/kmsg");
            result = 1;
            break;
        }
        if (count == 0) {
            fprintf(stderr, "klog-read: unexpected end of /dev/kmsg\n");
            result = 1;
            break;
        }

        size_t done = 0;
        while (done < (size_t)count) {
            ssize_t written = write(STDOUT_FILENO, record + done, (size_t)count - done);
            if (written == -1) {
                perror("klog-read: write stdout");
                result = 1;
                goto out;
            }
            if (written == 0) {
                fprintf(stderr, "klog-read: stdout write made no progress\n");
                result = 1;
                goto out;
            }
            done += (size_t)written;
        }
    }

out:
    if (close(fd) == -1) {
        perror("klog-read: close /dev/kmsg");
        result = 1;
    }
    return result;
}
