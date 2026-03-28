use alloc::vec::Vec;

use crate::vendor::ram::{MemoryError, RAM};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PixelFormat {
    Rgb565,
    Rgb888,
    Grayscale8,
}

impl PixelFormat {
    pub const fn supported() -> [Self; 3] {
        [Self::Rgb565, Self::Rgb888, Self::Grayscale8]
    }

    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Rgb565 => 2,
            Self::Rgb888 => 3,
            Self::Grayscale8 => 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameBufferHandle {
    pub offset: usize,
    pub len: usize,
    pub width: u16,
    pub height: u16,
    pub stride: usize,
    pub pixel_format: PixelFormat,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GpuCommand {
    Upload(FrameBufferHandle),
    Present(FrameBufferHandle),
    Clear(u16),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CpuToGpuError {
    InvalidFrameGeometry,
    Memory(MemoryError),
}

impl From<MemoryError> for CpuToGpuError {
    fn from(value: MemoryError) -> Self {
        Self::Memory(value)
    }
}

pub struct XtensaLx7CpuToGpu {
    command_queue: Vec<GpuCommand>,
    submitted_frames: u32,
}

impl XtensaLx7CpuToGpu {
    pub fn new() -> Self {
        Self {
            command_queue: Vec::new(),
            submitted_frames: 0,
        }
    }

    pub fn create_framebuffer(
        &mut self,
        ram: &mut RAM,
        width: u16,
        height: u16,
        pixel_format: PixelFormat,
    ) -> Result<FrameBufferHandle, CpuToGpuError> {
        if width == 0 || height == 0 {
            return Err(CpuToGpuError::InvalidFrameGeometry);
        }

        let stride = width as usize * pixel_format.bytes_per_pixel();
        let len = stride
            .checked_mul(height as usize)
            .ok_or(CpuToGpuError::InvalidFrameGeometry)?;
        let offset = ram.reserve_gpu_buffer(len)?;

        Ok(FrameBufferHandle {
            offset,
            len,
            width,
            height,
            stride,
            pixel_format,
        })
    }

    pub fn upload_frame(
        &mut self,
        ram: &mut RAM,
        framebuffer: FrameBufferHandle,
        data: &[u8],
    ) -> Result<(), CpuToGpuError> {
        if data.len() != framebuffer.len {
            return Err(CpuToGpuError::InvalidFrameGeometry);
        }

        ram.write_gpu_buffer(framebuffer.offset, data)?;
        self.command_queue.push(GpuCommand::Upload(framebuffer));
        Ok(())
    }

    pub fn clear(&mut self, color: u16) {
        self.command_queue.push(GpuCommand::Clear(color));
    }

    pub fn present(&mut self, framebuffer: FrameBufferHandle) {
        self.submitted_frames = self.submitted_frames.saturating_add(1);
        self.command_queue.push(GpuCommand::Present(framebuffer));
    }

    pub fn submitted_frames(&self) -> u32 {
        self.submitted_frames
    }

    pub fn pending_commands(&self) -> usize {
        self.command_queue.len()
    }

    pub fn drain_commands(&mut self) -> Vec<GpuCommand> {
        core::mem::take(&mut self.command_queue)
    }
}
