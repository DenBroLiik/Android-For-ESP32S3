use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryError {
    GpuOutOfMemory,
    GpuWriteOutOfBounds,
}

#[allow(clippy::upper_case_acronyms)]
pub struct RAM {
    bootloader_reserved_bytes: usize,
    gpu_reserved_bytes: usize,
    gpu: Vec<u8>,
    sram_total_bytes: usize,
    psram_total_bytes: usize,
}

impl RAM {
    pub const BYTES_PER_KIB: usize = 1024;

    pub fn new(
        bootloader_kib: u32,
        gpu_capacity_bytes: usize,
        sram_kib: u32,
        psram_kib: u32,
    ) -> Self {
        let gpu = Vec::with_capacity(gpu_capacity_bytes);

        Self {
            bootloader_reserved_bytes: bootloader_kib as usize * Self::BYTES_PER_KIB,
            gpu_reserved_bytes: gpu_capacity_bytes,
            gpu,
            sram_total_bytes: sram_kib as usize * Self::BYTES_PER_KIB,
            psram_total_bytes: psram_kib as usize * Self::BYTES_PER_KIB,
        }
    }

    pub fn sram_bytes(&self) -> usize {
        self.sram_total_bytes
    }

    pub fn sram(&self) -> u32 {
        self.to_kib(self.sram_bytes())
    }

    pub fn psram_bytes(&self) -> usize {
        self.psram_total_bytes
    }

    pub fn psram(&self) -> u32 {
        self.to_kib(self.psram_bytes())
    }

    pub fn ram_bytes(&self) -> usize {
        self.sram_bytes() + self.psram_bytes()
    }

    pub fn ram(&self) -> u32 {
        self.to_kib(self.ram_bytes())
    }

    pub fn bootloader_memory_bytes(&self) -> usize {
        self.bootloader_reserved_bytes
    }

    pub fn bootloader_memory(&self) -> u32 {
        self.to_kib(self.bootloader_memory_bytes())
    }

    pub fn gpu_memory_bytes(&self) -> usize {
        self.gpu.len()
    }

    pub fn gpu_memory(&self) -> u32 {
        self.to_kib(self.gpu_memory_bytes())
    }

    pub fn gpu_capacity_bytes(&self) -> usize {
        self.gpu_reserved_bytes
    }

    pub fn gpu_capacity(&self) -> u32 {
        self.to_kib(self.gpu_capacity_bytes())
    }

    pub fn gpu_free_bytes(&self) -> usize {
        self.gpu_capacity_bytes().saturating_sub(self.gpu_memory_bytes())
    }

    pub fn gpu_free(&self) -> u32 {
        self.to_kib(self.gpu_free_bytes())
    }

    pub fn total_reserved_bytes(&self) -> usize {
        self.bootloader_memory_bytes() + self.gpu_capacity_bytes()
    }

    pub fn total_used_bytes(&self) -> usize {
        self.bootloader_memory_bytes() + self.gpu_memory_bytes()
    }

    pub fn total_memory_bytes(&self) -> usize {
        self.ram_bytes().saturating_sub(self.total_reserved_bytes())
    }

    pub fn total_memory(&self) -> u32 {
        self.to_kib(self.total_memory_bytes())
    }

    pub fn available_memory_bytes(&self) -> usize {
        self.ram_bytes().saturating_sub(self.total_used_bytes())
    }

    pub fn available_memory(&self) -> u32 {
        self.to_kib(self.available_memory_bytes())
    }

    pub fn reserve_gpu_buffer(&mut self, bytes: usize) -> Result<usize, MemoryError> {
        let next_len = self
            .gpu
            .len()
            .checked_add(bytes)
            .ok_or(MemoryError::GpuOutOfMemory)?;

        if next_len > self.gpu.capacity() {
            return Err(MemoryError::GpuOutOfMemory);
        }

        let offset = self.gpu.len();
        self.gpu.resize(next_len, 0);
        Ok(offset)
    }

    pub fn write_gpu_buffer(&mut self, offset: usize, data: &[u8]) -> Result<(), MemoryError> {
        let end = offset
            .checked_add(data.len())
            .ok_or(MemoryError::GpuWriteOutOfBounds)?;

        if end > self.gpu.len() {
            return Err(MemoryError::GpuWriteOutOfBounds);
        }

        self.gpu[offset..end].copy_from_slice(data);
        Ok(())
    }

    pub fn gpu_slice(&self, offset: usize, len: usize) -> Result<&[u8], MemoryError> {
        let end = offset
            .checked_add(len)
            .ok_or(MemoryError::GpuWriteOutOfBounds)?;

        self.gpu
            .get(offset..end)
            .ok_or(MemoryError::GpuWriteOutOfBounds)
    }

    #[allow(dead_code)]
    pub fn reset_gpu(&mut self) {
        self.gpu.clear();
    }

    pub fn gpu_is_empty(&self) -> bool {
        self.gpu.is_empty()
    }

    fn to_kib(&self, bytes: usize) -> u32 {
        (bytes / Self::BYTES_PER_KIB) as u32
    }
}
