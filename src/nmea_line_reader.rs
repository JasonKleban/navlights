use core::str;

const LINE_CAP: usize = 96;

pub struct NmeaLineReader {
    line: [u8; LINE_CAP],
    len: usize,
}

impl NmeaLineReader {
    pub const fn new() -> Self {
        Self {
            line: [0; LINE_CAP],
            len: 0,
        }
    }

    pub fn push_byte(&mut self, byte: u8) -> Option<&str> {
        match byte {
            b'$' => {
                self.len = 0;
                self.line[self.len] = byte;
                self.len += 1;
            }
            b'\n' => {
                if self.len > 0 {
                    self.line[self.len] = byte;
                    self.len += 1;

                    let slice = &self.line[..self.len];

                    self.len = 0;

                    return str::from_utf8(slice).ok();
                }
            }
            b'\r' => {}
            _ => {
                if self.len > 0 {
                    if self.len < LINE_CAP {
                        self.line[self.len] = byte;
                        self.len += 1;
                    } else {
                        // overflow → reset
                        self.len = 0;
                    }
                }
            }
        }

        None
    }
}