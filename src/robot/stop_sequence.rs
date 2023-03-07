use core::fmt::Debug;
use crate::math::{ FloatType as float, Vector2, Vector3 };
use super::{ StopSequenceFn };

#[derive(Debug, Clone)]
pub struct StopSequence {
    x: float,
    sequence_fns: [StopSequenceFn; 6],
    delays: [bool; 6],
    delay_complete: bool
}

impl StopSequence {

    pub fn new(positions: [Vector3; 6], step_height_weight: float, delays: [bool; 6]) -> Self {
        let seq_fns = [
            StopSequenceFn::new(&positions[0], Vector2::new(positions[0][0], positions[0][1]).len() * step_height_weight),
            StopSequenceFn::new(&positions[1], Vector2::new(positions[1][0], positions[1][1]).len() * step_height_weight),
            StopSequenceFn::new(&positions[2], Vector2::new(positions[2][0], positions[2][1]).len() * step_height_weight),
            StopSequenceFn::new(&positions[3], Vector2::new(positions[3][0], positions[3][1]).len() * step_height_weight),
            StopSequenceFn::new(&positions[4], Vector2::new(positions[4][0], positions[4][1]).len() * step_height_weight),
            StopSequenceFn::new(&positions[5], Vector2::new(positions[5][0], positions[5][1]).len() * step_height_weight)
        ];

        StopSequence {
            x: 0.0,
            sequence_fns: seq_fns,
            delays,
            delay_complete: false
        }
    }

    pub fn advance(&mut self, speed: float, time: u32) {
        self.x += speed * (time as f32) / 1000.0;

        if !self.delay_complete {
            let mut delay_complete = true;
            for i in 0..6 {
                if !self.delays[i] && !self.sequence_fns[i].has_finished() {
                    self.sequence_fns[i].advance(self.x);
                    delay_complete = false;
                }
            }
            if delay_complete {
                self.x = 0.0;
            }
            self.delay_complete = delay_complete;
        }
        else {
            for i in 0..6 {
                if self.delays[i] && !self.sequence_fns[i].has_finished() {
                    self.sequence_fns[i].advance(self.x);
                }
            }
        }
    }

    pub fn get_leg_pos(&self, leg_id: usize) -> Vector3 {
        self.sequence_fns[leg_id].get()
    }

    pub fn has_finished(&self) -> bool {
        for i in 0..6 {
            if !self.sequence_fns[i].has_finished() {
                return false
            }
        }
        return true
    }
}
