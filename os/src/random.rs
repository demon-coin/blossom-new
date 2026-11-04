//! 显式非安全随机请求与用户初始化共用的教学字节源，不提供密码学熵保证。

use core::sync::atomic::{AtomicU64, Ordering};

use crate::arch;

const WORD_INCREMENT: u64 = 0x9e37_79b9_7f4a_7c15;
static NONCE: AtomicU64 = AtomicU64::new(0);

/// 用硬件计数和全局单调序号填充初始化字节；只能用于允许非安全随机的调用者。
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

/// 固定整数混合只扩散初始化种子的位，不增加熵或密码学保证。
fn mix_random_word(mut word: u64) -> u64 {
    word = (word ^ (word >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    word = (word ^ (word >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    word ^ (word >> 31)
}
