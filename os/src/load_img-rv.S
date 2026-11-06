    # 历史内嵌根文件系统镜像数据段（RISC-V 64 遗留构建路径，当前生产启动改用 VirtIO/SD/SATA/RAM 块设备）。
    # 放入 .data 段并导出起止符号 simg / eimg，按 4KiB（2^12 字节）页边界对齐以便直接按页映射。
    .section .data
    .global simg
    .global eimg
    # 将数据对齐到2^12次方也就是4096B
    .align 12
# 文件系统镜像起始地址
simg:
    # 将二进制文件包含到当前文件中
    .incbin "../fs-img-dir/rootfs-rv.img"
# 文件系统镜像结束地址
eimg:
    # 再次将数据对齐到4KB边界
    .align 12
