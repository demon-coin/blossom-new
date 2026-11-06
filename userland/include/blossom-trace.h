#ifndef BLOSSOM_TRACE_H
#define BLOSSOM_TRACE_H

/*
 * `/dev/ktrace` 内核轻量级事件追踪设备的 ioctl 命令号定义。
 *
 * 这两个命令均不传递用户态指针参数（第三个参数被内核忽略）：
 * - BLOSSOM_TRACE_START：清空历史记录并启动新一轮内核事件采样。
 * - BLOSSOM_TRACE_STOP ：停止事件采样并冻结当前环形缓冲区，以便用户态通过 read() 读取完整快照。
 */
#define BLOSSOM_TRACE_START 0x4201
#define BLOSSOM_TRACE_STOP 0x4202

#endif
