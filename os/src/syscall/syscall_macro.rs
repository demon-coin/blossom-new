//! 早期系统调用套接字查表与用户地址切片转换辅助宏（历史参考模块）。
//!
//! # 模块职责与架构定位
//! 本文件提供早期原型阶段编写网络与进程系统调用时的语法糖宏：
//! - **模块状态**：当前活跃内核已统一改用 `FileTable`（套接字作为 `FileLike` 对象管理）以及 `mm::space::Mm` 提供的按页安全拷贝接口（`copyin`/`copyout`/`ReadBuffer`），本文件未挂载到当前编译模块树，仅作为早期设计参考保留。
//!
//! # 核心设计与早期地址转换模型
//! ```text
//!   get_socket!(sockfd)
//!      └──► 锁定 current_task().socket_table ──► 命中则 clone()，未命中直接 return ENOTSOCK
//!
//!   trans_ref!(addr, addrlen)
//!      └──► translated_ref(token, addr) 查页表得首字节内核指针
//!              ├──► Err(errno) ──► 直接从外层系统调用函数 return errno
//!              └──► Ok(ptr)    ──► from_raw_parts(ptr, addrlen) 构造连续只读字节切片
//! ```
//! **边界限制**：`trans_ref!` 假设 `[addr, addr + addrlen)` 在物理内存中连续且在切片存活期间页表映射保持不变，跨页非连续的用户缓冲区不能使用该宏（现代实现已改由逐页 `copyin` 处理）。
//!
//! # 锁获取顺序与并发契约
//! - `get_socket!` 内部短暂获取当前任务的 `socket_table` 互斥锁，克隆句柄后立即释放锁，不跨后续 I/O 持锁。
//! - `trans_ref!` 在未持有页表/VMA 锁的情况下返回裸切片引用，调用方必须确保在使用该切片期间不会并发修改或卸载对应用户地址区间。

/// 在当前任务的 `socket_table` 中按描述符下标 `$sockfd` 查找并克隆套接字句柄；若槽位为空则直接从外层系统调用返回 `ENOTSOCK`。
macro_rules! get_socket {
    ($sockfd:expr) => {{
        match current_task().unwrap().socket_table.lock().get_ref($sockfd as usize) {
            Some(socket) => socket.clone(),
            None => return ENOTSOCK,
        }
    }}
}

/// 将用户空间虚拟地址 `$addr`（长度 `$addrlen` 字节）通过页表翻译转换为内核只读切片 `&[u8]`；若首地址翻译失败则直接从外层函数返回对应负 `errno`。
macro_rules! trans_ref {
    ($addr:expr, $addrlen:expr) => {{
        let token = current_task().unwrap().get_user_token();
        match translated_ref(token, $addr as *const u8) {
            Ok(addr) => {
                // SAFETY:
                // 1. `translated_ref` 已校验用户页表项有效可读，并返回映射对应的内核虚拟首地址 `addr`（`u8` 对齐恒为 1）。
                // 2. 早期单线程/非抢占调用契约要求 `[addr, addr + $addrlen)` 落在同一连续物理映射内，且在当前系统调用帧结束前不会被解除映射或并发写入。
                unsafe { core::slice::from_raw_parts(addr as *const u8, $addrlen as usize) }
            }
            Err(errno) => {
                return errno;
            }
        }
    }}
}