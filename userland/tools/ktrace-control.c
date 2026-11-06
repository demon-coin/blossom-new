#include <blossom-trace.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/*
 * `/dev/ktrace` 内核事件追踪启停控制工具（安装为 `/bin/ktrace-control`）。
 *
 * 用法：
 * - `ktrace-control start`：向 `/dev/ktrace` 发送 BLOSSOM_TRACE_START，清空并开启追踪。
 * - `ktrace-control stop` ：向 `/dev/ktrace` 发送 BLOSSOM_TRACE_STOP，冻结当前追踪记录供读取。
 */
int main(int argc, char **argv)
{
    int request;
    if (argc == 2 && strcmp(argv[1], "start") == 0)
        request = BLOSSOM_TRACE_START;
    else if (argc == 2 && strcmp(argv[1], "stop") == 0)
        request = BLOSSOM_TRACE_STOP;
    else {
        fprintf(stderr, "usage: ktrace-control start|stop\n");
        return 1;
    }

    int fd = open("/dev/ktrace", O_RDONLY);
    if (fd == -1) {
        perror("ktrace-control: open /dev/ktrace");
        return 1;
    }
    int result = 0;
    if (ioctl(fd, request, 0) == -1) {
        perror("ktrace-control: ioctl /dev/ktrace");
        result = 1;
    }
    if (close(fd) == -1) {
        perror("ktrace-control: close /dev/ktrace");
        result = 1;
    }
    return result;
}
