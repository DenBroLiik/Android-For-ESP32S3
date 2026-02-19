use alloc::vec::Vec;

pub struct RAM {
    bootloader: u32,  // Размер памяти для загрузчика (4KB) в PSRAM
    pub gpu: Vec<u8>, // Динамическая память для GPU (64KB по умолчанию)
    sram_size: u32,   // Размер SRAM
    psram_size: u32,  // Размер PSRAM
}

// Распределение данных между SRAM и RAM
// Разметка памяти
impl RAM {
    pub fn new(bootloader: u32, gpu_capacity: usize, sram_size: u32, psram_size: u32) -> Self {
        let mut gpu = Vec::with_capacity(gpu_capacity); // Резервируем память (64KB), но не выделяем
        gpu.resize(0, 0); // Начинаем с нулевого размера
        RAM {
            bootloader,
            gpu,
            sram_size,
            psram_size,
        }
    }

    /// Инициализация SRAM
    pub fn sram(&self) -> u32 {
        self.sram_size
    }

    /// Инициализация PSRAM
    pub fn psram(&self) -> u32 {
        self.psram_size
    }

    /// Объединение SRAM и PSRAM в единое адресное пространство
    pub fn ram(&self) -> u32 {
        self.sram_size + self.psram_size
    }

    /// Выделение памяти для загрузчика (в PSRAM)
    pub fn bootloader_memory(&self) -> u32 {
        self.bootloader
    }

    /// Динамическая память для GPU (в KB)
    pub fn gpu_memory(&self) -> u32 {
        self.gpu.len() as u32
    }

    /// Зарезервированная память для GPU (в KB)
    pub fn gpu_capacity(&self) -> u32 {
        self.gpu.capacity() as u32
    }

    /// Общий размер доступной памяти
    pub fn total_memory(&self) -> u32 {
        self.ram() - self.bootloader_memory() - self.gpu_memory()
    }
}
