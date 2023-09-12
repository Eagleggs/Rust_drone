use log::disk::DiskError;
use log::Disk;
use tudelft_quadrupel::flash::FlashError;
// use log::
pub struct FuncDisk {
    write: fn(u32, &[u8]) -> Result<(), FlashError>,
    read: fn(u32, &mut [u8]) -> Result<(), FlashError>,
    erase: fn() -> Result<(), FlashError>,
    write_one_byte: fn(u32, u8) -> Result<(), FlashError>,
    read_one_byte: fn(u32) -> Result<u8, FlashError>,
}

impl FuncDisk {
    pub fn func(
        write: fn(u32, &[u8]) -> Result<(), FlashError>,
        read: fn(u32, &mut [u8]) -> Result<(), FlashError>,
        erase: fn() -> Result<(), FlashError>,
        write_one_byte: fn(u32, u8) -> Result<(), FlashError>,
        read_one_byte: fn(u32) -> Result<u8, FlashError>,
    ) -> FuncDisk {
        FuncDisk {
            write: write,
            read: read,
            erase: erase,
            write_one_byte: write_one_byte,
            read_one_byte: read_one_byte,
        }
    }
}

impl Disk for FuncDisk {
    fn write_bytes(&mut self, address: u32, bytes: &[u8]) -> Result<(), DiskError> {
        match (self.write)(address, bytes) {
            Ok(()) => Ok(()),
            Err(FlashError::OutOfSpace) => Err(DiskError::OutOfSpace),
            Err(FlashError::SpiError(_)) => Err(DiskError::SpiError),
        }
    }

    fn read_bytes(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), DiskError> {
        match (self.read)(address, buffer) {
            Ok(()) => Ok(()),
            Err(FlashError::OutOfSpace) => Err(DiskError::OutOfSpace),
            Err(FlashError::SpiError(_)) => Err(DiskError::SpiError),
        }
    }

    fn erase_data(&mut self) -> Result<(), DiskError> {
        match (self.erase)() {
            Ok(()) => Ok(()),
            Err(_) => Err(DiskError::SpiError),
        }
    }

    fn write_byte(&mut self, address: u32, byte: u8) -> Result<(), DiskError> {
        match (self.write_one_byte)(address, byte) {
            Ok(()) => Ok(()),
            Err(FlashError::OutOfSpace) => Err(DiskError::OutOfSpace),
            Err(FlashError::SpiError(_)) => Err(DiskError::SpiError),
        }
    }

    fn read_byte(&mut self, address: u32) -> Result<u8, DiskError> {
        match (self.read_one_byte)(address) {
            Ok(read_byte) => Ok(read_byte),
            Err(FlashError::OutOfSpace) => Err(DiskError::OutOfSpace),
            Err(FlashError::SpiError(_)) => Err(DiskError::SpiError),
        }
    }
}
