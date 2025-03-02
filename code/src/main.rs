// use std::arch::x86_64::_bextr_u32;
// use serialport;
// use serialport::SerialPort;
// use std::collections::HashMap;
// use std::io::Write;

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

//
// struct Client {
//     baud: u32,
//     port_name: String,
//     port: Box<dyn SerialPort>,
// }
//
// impl Client {
//     fn connect(baud: i32, port: String) -> Client {
//         let sp = serialport::new("/dev/ttyACM0", 115200)
//             // .stop_bits(serialport::StopBits::One)
//             // .data_bits()
//             .open()
//             .unwrap();
//         Client {
//             baud: baud.into(),
//             port_name: port.to_string(),
//             port: sp,
//         }
//     }
//
//     // fn broadcast(self, instruction: Instruction, data_len: u8, data: Vec<u8>) {
//     //
//     // }
//
//     fn send(mut self, id: u8, instruction: Instruction, data_len: u8, data: &[u8]) {
//         let header  = instruction.header().clone();
//         let packet: &[u8] = [**header, id, instruction.id(), data_len, **data];
//         self.port.write(&packet).unwrap();
//     }
// }
//
// struct Manager {
//     client: Client,
//     servo_ids: HashMap<&'static str, u32>,
//     port: Option<dyn SerialPort>,
// }
//
// impl Manager {
//     fn new(client: Client, servo_ids: HashMap<&str, u32>) -> Manager {
//         Manager {
//             client,
//             servo_ids,
//             port: None,
//         }
//     }
//
//     fn write(self, id: u8, instruction: Instruction, data_len: u8) {
//         self.client.port.write()
//
//
//     }
//
//     // fn ping()
// }
//
// fn main() {
//     let client = Client::connect(115200, "/dev/ttyACM0".parse().unwrap());
//     let servo_ids = HashMap::from([
//         ("base_rot", 2),
//         ("bicep_base", 3),
//         ("forearm_base", 4),
//         ("wrist_lat", 5),
//         ("wrist_rot", 6),
//         ("grip", 7),
//     ]);
//     let manager = Manager::new(client, servo_ids);
// }


use serialport::*;
use std::io::{Read, Write};
use std::time::Duration;

const HEADER: [u8; 2] = [0xFF, 0xFF];
const BROADCAST_ID: u8 = 0xFE;
const PING_CMD: u8 = 0x01;
const READ_CMD: u8 = 0x02;
const WRITE_CMD: u8 = 0x03;

struct ServoController {
    port: Box<dyn SerialPort>,
}

struct Response {
    id: u8,
    status: CommStatus,
    data: Vec<u8>,
}

impl ServoController {
    fn new(port_name: &str, baud_rate: u32) -> Self {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
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
        let mut packet = vec!(HEADER[0], HEADER[1], id, params.len() as u8 + 2, instruction);
        println!("{}", packet.len());
        packet.extend_from_slice(params);
        if packet.len() > 250 {
            return Err(Error::new(ErrorKind::Unknown, "Packet too long"));
        }
        let checksum = Self::calculate_checksum(&packet[2..]);
        packet.push(checksum);
        self.port.clear(ClearBuffer::All)?;
        self.port.write_all(&packet)?;
        println!("tx {:?}", &packet);
        println!("tx bytes {} {}", self.port.bytes_to_read()?, self.port.bytes_to_write()?);
        self.port.flush()?;
        Ok(())
    }


    fn read_response(&mut self, id: u8, expected_params_len: usize) -> Result<Response> {
        // header{2}, id{1}, length{1}, error{1}, ..., checksum{1}
        let mut buffer = Vec::<u8>::with_capacity(expected_params_len + 4 + 1);
        // println!("tx bytes {} {}", self.port.bytes_to_read()?, self.port.bytes_to_write()?);
        self.port.read_exact(&mut buffer)?;
        if buffer[2] != id {
            return Err(Error::new(ErrorKind::Unknown, "Invalid ID"))
        }
        let expected_checksum = buffer[buffer.len() - 1];
        let actual_checksum = Self::calculate_checksum(&buffer[2.. buffer.len() - 1]);
        if actual_checksum != expected_checksum {
            return Err(Error::new(ErrorKind::Unknown, "Checksum error"));
        }
        println!("rx {:?}", buffer);
        Ok(Response {
            id: buffer[2],
            status: CommStatus::from_code(buffer[3] as i8),
            data: buffer[4..buffer.len() - 1].to_vec(),
        })
    }

    fn ping(&mut self, id: u8) -> Result<bool> {
        if id == BROADCAST_ID {
            return Err(Error::new(ErrorKind::Unknown, "Broadcast ID not supported"));
        }
        if id > BROADCAST_ID {
            return Err(Error::new(ErrorKind::Unknown, "Servo Id out of range"));
        }
        self.send_packet(id, PING_CMD, &[])?;
        let resp = self.read_response(id, 0)?;

        // self.read_data(1, 0x38, 2);

        // Ok(response.len() > 0 && response[2] == id)
        Ok(true)
    }

    // fn read_data(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>> {
    //     // id, len, inst, checksum
    //     self.send_packet(id, READ_CMD, &[address, length + 4])?;
    //     let response = self.read_response();
    //     if response.len() >= 6 {
    //         Ok(response[5..].to_vec())
    //     } else {
    //         Ok(vec![])
    //     }
    // }

    fn write_data(&mut self, id: u8, address: u8, data: &[u8]) -> Result<()> {
        let mut params = vec![address];
        params.extend_from_slice(data);
        self.send_packet(id, WRITE_CMD, &params)
    }
    //
    // fn reset(&mut self) {
    //     self.send_packet(0, 0x06, &[]);
    // }
}

fn main() {
    let port_name = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FA095208-if00";
    // let port_name = "/dev/ttyACM0";
    let mut servo = ServoController::new(port_name, 1_000_000);
    // servo.reset();
    // println!("{:?}", servo.read_response());
    servo.ping(1).unwrap();

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
