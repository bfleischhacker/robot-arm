#![feature(generic_const_exprs)]

use std::collections::HashMap;
use serialport::*;
use std::fmt::{Display, Formatter};
use std::io::{Read, Write};
use std::iter::Map;
use std::marker::PhantomData;
use std::time::Duration;
use log::error;

#[derive(Debug)]
enum Error {
    Unknown,
    ServoError(CommStatus),
    IoError(std::io::Error),
    SerialError(serialport::Error),
    ChecksumError,
    IdError(u8, u8),
    BroadcastUnsupported,
    ServoIdOutOfRange,
    Other(String),
}

type Result<T> = std::result::Result<T, Error>;

impl From<std::io::Error> for Error {
    fn from(value: std::io::Error) -> Self {
        Error::IoError(value)
    }
}

impl From<serialport::Error> for Error {
    fn from(value: serialport::Error) -> Self {
        Error::SerialError(value)
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Unknown => write!(f, "Unknown error"),
            Error::ServoError(status) => write!(f, "Comm error: {:?}", status),
            Error::IoError(error) => write!(f, "IO error: {:?}", error),
            Error::SerialError(e) => {
                write!(f, "Serial error: {:?}", e)
            }
            Error::ChecksumError => {
                write!(f, "Checksum error")
            }
            Error::IdError(expected, actual) => {
                write!(f, "Invalid ID expected: {}, actual: {}", expected, actual)
            }
            Error::Other(e) => {
               write!(f, "{}", e)
            }
            Error::BroadcastUnsupported => {
                write!(f, "Broadcast ID not supported")
            }
            Error::ServoIdOutOfRange => {
                write!(f, "Servo Id out of range")
            }
        }
    }
}

impl std::error::Error for Error {
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Address(u8);

trait Addressable {
    fn address(&self) -> Address;
}




enum Instruction {
    PING,
    SYNCREAD,
    SYNCWRITE,
    RESET,
}

impl Instruction {
    fn id(&self) -> u8 {
        match self {
            Instruction::PING => 1,
            Instruction::SYNCREAD => 0x82,
            Instruction::SYNCWRITE => 0x83,
            Instruction::RESET => 0x06,
        }
    }

    fn header(&self) -> [u8; 2] {
        [0xFF, 0xFF]
    }
}
//
// struct Address<const S: usize, const R: bool, const W: bool> {
//     id: u8,
// }
//
// trait Address<const N: usize> {
//
//
// }

// enum ServoParameterSize {
//     OneByte,
//     TwoByte,
//     ThreeByte,
//     FourByte,
// }
//
// impl ServoParameterSize {
//     fn size(&self) -> usize {
//         match self {
//             ServoParameterSize::OneByte => 1,
//             ServoParameterSize::TwoByte => 2,
//             ServoParameterSize::ThreeByte => 3,
//             ServoParameterSize::FourByte => 4,
//         }
//     }
//
//     fn parse(&self, data: &[u8]) -> Result<u32> {
//         match self {
//             ServoParameterSize::OneByte => Ok(data[0] as u32),
//             ServoParameterSize::TwoByte => Ok(u16::from_le_bytes([data[0], data[1]]) as u32),
//             ServoParameterSize::ThreeByte => Ok(u32::from_le_bytes([data[0], data[1], data[2], 0])),
//             ServoParameterSize::FourByte => Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]])),
//         }
//     }
// }
//
// trait IsServoParameterSize {
//
// }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommStatus {
    Success,
    PortBusy,
    TxFail,
    RxFail,
    TxError,
    RxWaiting,
    RxTimeout,
    RxCorrupt,
    NotAvailable,
}

impl CommStatus {
    fn from_code(code: i8) -> CommStatus {
        match code {
            0 => CommStatus::Success,
            -1 => CommStatus::PortBusy,
            -2 => CommStatus::TxFail,
            -3 => CommStatus::RxFail,
            -4 => CommStatus::TxError,
            -5 => CommStatus::RxWaiting,
            -6 => CommStatus::RxTimeout,
            -7 => CommStatus::RxCorrupt,
            -9 => CommStatus::NotAvailable,
            _ => panic!("Unknown error code: {}", code),
        }
    }
}

const HEADER: [u8; 2] = [0xFF, 0xFF];
const BROADCAST_ID: u8 = 0xFE;
const PING_CMD: u8 = 0x01;
const READ_CMD: u8 = 0x02;
const WRITE_CMD: u8 = 0x03;

struct ServoController {
    port: Box<dyn SerialPort>,
}

struct SyncWriteCommand {
    servo_id: u8,
    data: Vec<u8>,
}
impl ServoController {
    fn new(port_name: &str, baud_rate: u32) -> Self {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(1000))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .open()
            .expect("Failed to open port");
        Self { port }
    }

    // Checksum = ~ (ID+length+instruction+parameter 1+... parameter n)
    fn calculate_checksum(data: &[u8]) -> u8 {
        let sum: u8 = data.iter().map(|&b| b).sum();
        (!sum as u8) & 0xFF
    }

    fn send_packet(&mut self, id: u8, instruction: u8, params: &[u8]) -> Result<()> {
        let mut packet = vec![
            HEADER[0],
            HEADER[1],
            id,
            params.len() as u8 + 2,
            instruction,
        ];
        packet.extend_from_slice(params);
        if packet.len() > 250 {
            error!(
                "Packet too long. Max length is 250 bytes. Packet length: {}",
                packet.len()
            );
        }
        let checksum = Self::calculate_checksum(&packet[2..]);
        packet.push(checksum);
        self.port.clear(ClearBuffer::All)?;
        self.port.write_all(&packet)?;
        println!("tx {:?}", &packet);
        self.port.flush()?;
        Ok(())
    }

    fn read_response(&mut self, id: u8, expected_params_len: usize) -> Result<Vec<u8>> {
        // header{2}, id{1}, length{1}, error{1}, ..., checksum{1}
        let mut buffer = Vec::<u8>::with_capacity(expected_params_len + 4 + 1);
        buffer.resize(expected_params_len + 6, 0);
        self.port.read_exact(&mut buffer)?;
        println!("rx {:?}", buffer);
        if buffer[2] != id {
            return Err(Error::IdError(id, buffer[2]));
        }
        let expected_checksum = buffer[buffer.len() - 1];
        let actual_checksum = Self::calculate_checksum(&buffer[2..buffer.len() - 1]);
        if actual_checksum != expected_checksum {
            return Err(Error::ChecksumError);
        }
        let status = CommStatus::from_code(buffer[4] as i8);
        if status != CommStatus::Success {
            return Err(Error::ServoError(status));
        }
        Ok(buffer[5..buffer.len() - 1].to_vec())
    }

    fn ping(&mut self, id: u8) -> Result<u16> {
        if id == BROADCAST_ID {
            return Err(Error::Other("Broadcast ID not supported".to_string()));
        }
        if id > BROADCAST_ID {
            return Err(Error::Other("Servo Id out of range".to_string()));
        }
        self.send_packet(id, PING_CMD, &[])?;
        self.read_response(id, 0)?;
        self.read_data_2w(id, 3)
    }

    fn read_data(&mut self, id: u8, address: u8, data_byte_len: usize) -> Result<Vec<u8>> {
        // id, len, inst, checksum
        self.send_packet(id, READ_CMD, &[address, data_byte_len as u8])?;
        self.read_response(id, data_byte_len)
    }

    fn write_data(&mut self, id: u8, address: u8, data: &[u8]) -> Result<()> {
        let mut params = vec![address];
        params.extend_from_slice(data);
        self.send_packet(id, WRITE_CMD, &params)
    }

    fn read_data_1w(&mut self, id: u8, address: u8) -> Result<u8> {
        let data = self.read_data(id, address, 1)?;
        Ok(data[0])
    }

    fn read_data_2w(&mut self, id: u8, address: u8) -> Result<u16> {
        let data = self.read_data(id, address, 2)?;
        Ok(u16::from_le_bytes([data[0], data[1]]))
    }

    fn read_data_3w(&mut self, id: u8, address: u8) -> Result<u32> {
        let data = self.read_data(id, address, 3)?;
        Ok(u32::from_le_bytes([data[0], data[1], data[2], 0]))
    }

    fn read_data_4w(&mut self, id: u8, address: u8) -> Result<u32> {
        let data = self.read_data(id, address, 4)?;
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    fn sync_write<const N: usize>(&mut self, address_start: u8, data: HashMap<u8, &[u8; N]>) -> Result<()> {
        // let total_size = (data.len() + 1) * ids.len() + 4;
        let mut params = Vec::new();
        params.push(address_start);
        params.push(N as u8);
        for (id, d) in data {
            params.push(id);
            params.extend_from_slice(d);
        }
        self.send_packet(BROADCAST_ID, Instruction::SYNCWRITE.id(), &params)?;
        Ok(())
    }

    fn sync_read<const N: usize>(&mut self, servo_ids: &[u8], start_address: ) -> Result<HashMap<u8, &[u8; N]>> {
        let mut params = Vec::new();
        params.push(start_address);
        params.push(servo_ids.len() as u8);
        params.extend_from_slice(servo_ids);
        self.send_packet(BROADCAST_ID, Instruction::SYNCREAD.id(), &params)?;
        self.read
    }
}

fn main() {
    // let port_name = "/dev/tty.usbmodem58FA0952081";
    let port_name = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FA095208-if00";
    let mut servo = ServoController::new(port_name, 1_000_000);
    println!("{:?}", servo.ping(1).unwrap());

    // println!(
    //     "Connected to servo controller at {} baud",
    //     servo.port.baud_rate().unwrap()
    // )
    //
    // for i in 0..8 {
    //     if servo.ping(i) {
    //         println!("Servo {} is online.", i);
    //     } else {
    //         println!("Servo {} did not respond.", i);
    //     }
    // }
    // // Ping Servo with ID 1
    // if servo.ping(2) {
    //     println!("Servo 1 is online.");
    // } else {
    //     println!("Servo 1 did not respond.");
    // }

    // Read position from servo
    // let position = servo.read_data(1, 0x38, 2);
    // if !position.is_empty() {
    //     let pos_value = u16::from_le_bytes([position[0], position[1]]);
    //     println!("Servo 1 Position: {}", pos_value);
    // }
    //
    // // Move Servo 1 to position 2048 at speed 1000
    // let position_bytes = 2048u16.to_le_bytes();
    // let speed_bytes = 1000u16.to_le_bytes();
    // servo.write_data(1, 0x2A, &[position_bytes[0], position_bytes[1], 0x00, 0x00, speed_bytes[0], speed_bytes[1]]);
}
