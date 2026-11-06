//! 基于硬件周期计数器与 SplitMix64 算法的非密码学伪随机字节生成器（Non-Cryptographic PRNG）。
//!
//! # 模块职责与架构定位
//! 本模块为内核提供轻量、无锁、零阻塞的伪随机字节填充服务：
//! - **上游调用者**：ELF 栈辅助向量（`AT_RANDOM` 16 字节种子）、系统调用 `getrandom`（在教学环境中提供非阻塞随机字节）等。
//! - **下游依赖**：调用 [`crate::arch::read_counter`] 读取当前 CPU 硬件周期/时间计数器，结合全局原子递增序号生成种子。
//!
//! # 核心设计与生成流水线
//! ```text
//! arch::read_counter()        NONCE.fetch_add(1, Relaxed)
//!          │                              │
//!          └──────────────┬───────────────┘
//!                         ▼ (XOR 异或合成初始 seed)
//!                   seed_0 = counter ^ nonce
//!                         │
//!                         ├──► mix_random_word(seed_0) ──► 小端字节写入 chunk[0]
//!                         │
//!                   seed_1 = seed_0.wrapping_add(0x9e3779b97f4a7c15)
//!                         │
//!                         └──► mix_random_word(seed_1) ──► 小端字节写入 chunk[1] ...
//! ```
//!
//! # 同步与安全契约
//! - **无锁并发**：全局序号 [`NONCE`] 使用 `AtomicU64::fetch_add(1, Ordering::Relaxed)` 更新，
//!   保证多核或同核连续调用不会产生完全相同的初始种子，可在任意上下文（含关中断上下文）安全调用；
//! - **安全边界**：本模块仅对计数器与单调序号做确定性位混淆（SplitMix64），**不具备密码学安全性（CSPRNG）**，
//!   不能用于生产级密钥生成。

use core::sync::atomic::{AtomicU64, Ordering};

use crate::arch;

/// SplitMix64 状态步进增量常量（源自黄金分割比 $\lfloor 2^{64} / \phi \rfloor$）。
const WORD_INCREMENT: u64 = 0x9e37_79b9_7f4a_7c15;
/// 全局单调递增调用序号，确保短时间内连续调用或跨核同时调用 [`fill_insecure`] 时获得不同的初始种子。
static NONCE: AtomicU64 = AtomicU64::new(0);

/// 使用硬件计时器计数与全局单调序号生成的伪随机序列填满目标切片 `bytes`。
///
/// 可在任意任务或关中断上下文中无锁调用；若 `bytes` 为空则直接返回且不消耗 [`NONCE`]。
pub(crate) fn fill_insecure(bytes: &mut [u8]) {
    if bytes.is_empty() {
        return;
    }

    // ponytail: 此来源没有安全熵承诺；密钥和安全随机必须使用真实熵源。
    let mut seed = arch::read_counter() ^ NONCE.fetch_add(1, Ordering::Relaxed);
    for chunk in bytes.chunks_mut(core::mem::size_of::<u64>()) {
        let word = mix_random_word(seed).to_le_bytes();
        chunk.copy_from_slice(&word[..chunk.len()]);
        seed = seed.wrapping_add(WORD_INCREMENT);
    }
}

/// SplitMix64 64 位整数混淆函数（Stafford Variant 13）：通过三次异或移位与奇常数乘法打散输入种子的比特分布。
fn mix_random_word(mut word: u64) -> u64 {
    word = (word ^ (word >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    word = (word ^ (word >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    word ^ (word >> 31)
}
