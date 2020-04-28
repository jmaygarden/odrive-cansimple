use crate::enumerations::{AxisId, Command};
use socketcan::{CANFrame, ConstructionError};

pub fn decode_id(frame: CANFrame) -> (AxisId, Command) {
    let id = frame.id();
    let node = ((id >> 5) & 0x3F) as AxisId;
    let command = num::FromPrimitive::from_u32(id & 0x1F);

    if let Some(command) = command {
        (node, command)
    } else {
        (node, Command::Undefined)
    }
}

pub struct FrameBuilder {
    id: u32,
    arg0: Option<u32>,
    arg1: Option<u32>,
    rtr: bool,
}

impl FrameBuilder {
    pub fn new(node: u32, command: Command) -> FrameBuilder {
        FrameBuilder {
            id: ((node & 0x3F) << 5) | (command as u32 & 0x1F),
            arg0: None,
            arg1: None,
            rtr: false,
        }
    }

    pub fn arg0(&mut self, value: u32) -> &mut FrameBuilder {
        self.arg0 = Some(value);
        self
    }

    pub fn arg1(&mut self, value: u32) -> &mut FrameBuilder {
        self.arg1 = Some(value);
        self
    }

    pub fn rtr(&mut self, value: bool) -> &mut FrameBuilder {
        self.rtr = value;
        self
    }

    pub fn finalize(&self) -> Result<CANFrame, ConstructionError> {
        let mut data = Vec::<u8>::with_capacity(8);

        if let Some(arg0) = self.arg0 {
            data.extend(&arg0.to_le_bytes());
        }

        if let Some(arg1) = self.arg1 {
            data.extend(&arg1.to_le_bytes());
        }

        CANFrame::new(self.id, &data, self.rtr, false)
    }
}
