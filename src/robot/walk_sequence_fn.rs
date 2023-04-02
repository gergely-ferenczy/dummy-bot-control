use core::fmt::Debug;
use crate::{math::{ FloatType as float, FloatEq, Vector2, Vector3, transform }, float_ne, float_eq };
use super::{ functions::{ quad_step_height } };

#[derive(Debug, Clone)]
struct WalkSequenceFnConfig {
    step: Vector2,
    step_center: Vector2,
    turn_angle: float,
    turn_origin: Vector2,
    step_height_weight: float
}

#[derive(Debug, Clone)]
pub struct WalkSequenceFn {
    x: float,
    id: u32,
    config_active: WalkSequenceFnConfig,
    config_update: Option<WalkSequenceFnConfig>,
    phase: u32,
    offset: float,
    lift_ratio: float,
    step_len: float
}

impl WalkSequenceFn {
    pub fn new(id: u32, offset: float, lift_ratio: float) -> Self {
        debug_assert!(offset >= 0.0 && offset < 1.0);
        debug_assert!(lift_ratio >= 0.0 && lift_ratio < 1.0);

        let empty_config = WalkSequenceFnConfig {
            step: Vector2::zero(),
            step_center: Vector2::zero(),
            turn_angle: 0.0,
            turn_origin: Vector2::zero(),
            step_height_weight: 0.0
        };

        WalkSequenceFn {
            id, x: 0.0,
            config_active: empty_config,
            config_update: None,
            phase: 0,
            offset,
            lift_ratio,
            step_len: 0.0
        }
    }

    fn find_step_scale_factor(step_cfg: &WalkSequenceFnConfig, step_update: &WalkSequenceFnConfig, max_step_radius: float,
            max_move_radius: float, leg_static_pos: &Vector3) -> float {

        let max_step = Self::calc_max_step(step_cfg, step_update);
        let max_step_radius_calc = max_step.len();
        let max_move_radius_calc = (leg_static_pos + Vector3::from(max_step)).len();

        let mut scale = 1.0;

        if max_step_radius_calc > max_step_radius || max_move_radius_calc > max_move_radius {
            let mut scale_min = 0.0;
            let mut scale_max = 2.0;
            let mut step_update_scaled = step_update.clone();

            while float_ne!(scale_min, scale_max, 1e-6, abs) {
                scale = (scale_min + scale_max) / 2.0;
                step_update_scaled.step = &step_update.step * scale;
                step_update_scaled.turn_angle = step_update.turn_angle * scale;
                let max_step = Self::calc_max_step(step_cfg, &step_update_scaled);
                let max_step_radius_calc = max_step.len();
                let max_move_radius_calc = (leg_static_pos + Vector3::from(max_step)).len();

                if max_step_radius_calc > max_step_radius || max_move_radius_calc > max_move_radius {
                    scale_max = scale;
                }
                else {
                    scale_min = scale;
                }
            }
        }

        return scale
    }

    pub fn update(&mut self, step: &Vector2, turn_origin: &Vector2, turn_angle: float, step_height_weight: float,
            max_step_radius: float, max_move_radius: float, leg_static_pos: &Vector3) -> Result<(), float> {

        let config_update = WalkSequenceFnConfig{
            step: step.clone(),
            step_center: Vector2::zero(),
            turn_angle,
            turn_origin: turn_origin.clone(),
            step_height_weight
        };

        let scale = Self::find_step_scale_factor(&self.config_active, &config_update, max_step_radius, max_move_radius, leg_static_pos);

        if float_eq!(scale, 1.0, 1e-3, abs) {
            if self.dist() == 0.0 {
                self.config_active = config_update;
            }
            else {
                self.config_update = Some(config_update);
            }
            self.phase = 0;
            Ok(())
        }
        else {
            Err(scale)
        }
    }

    fn get_phase_shift_points(&self) -> (float, float, float) {
        let o = self.offset;
        let rl = self.lift_ratio;
        let rp = 1.0 - self.lift_ratio;

        if self.offset < 0.5 {
            let c1 = rp * o;
            let c2 = rl / 2.0 + o;
            let c3 = rl / 2.0 + rp + o;
            (c1, c2, c3)
        }
        else {
            let c1 = 0.0;
            let c2 = -rl/2.0 + rl*o;
            let c3 = -rl/2.0 + o;
            (c1, c2, c3)
        }
    }

    fn dissect_step_cfg(cfg: &WalkSequenceFnConfig) -> (&Vector2, &Vector2, &Vector2, float) {
        (&cfg.step_center, &cfg.step, &cfg.turn_origin, cfg.turn_angle)
    }

    #[allow(non_snake_case)]
    fn step_pos(S_c: &Vector2, s: &Vector2, S_to: &Vector2, t: float, x: float) -> Vector2 {
        let rm = &transform::rotate_matrix2(t * x);
        s*x + rm * (S_c - S_to) + S_to
    }

    #[allow(non_snake_case)]
    fn calc_intersecting_step_center(a_cfg: &WalkSequenceFnConfig, b_cfg: &WalkSequenceFnConfig, x: float) -> Vector2 {
        let (A_c, a, A_to, a_t) = Self::dissect_step_cfg(a_cfg);
        let A_p = Self::step_pos(A_c, a, A_to, a_t, x);

        let b = &b_cfg.step;
        let B_to = &b_cfg.turn_origin;
        let b_rm = &transform::rotate_matrix2(b_cfg.turn_angle * x);
        let b_rm_neg = &transform::rotate_matrix2(-b_cfg.turn_angle * x);

        b_rm_neg * (A_p - b*x + b_rm * B_to - B_to)
    }

    #[allow(non_snake_case)]
    fn calc_step_pos(&self, x: float) -> Vector2 {
        let (A_c, a, A_to, a_t) = Self::dissect_step_cfg(&self.config_active);
        Self::step_pos(A_c, a, A_to, a_t, x)
    }

    #[allow(non_snake_case)]
    fn calc_cross_step(a_cfg: &WalkSequenceFnConfig, b_cfg: &WalkSequenceFnConfig) -> (Vector2, Vector2) {
        let (A_c, a, A_to, a_t) = Self::dissect_step_cfg(a_cfg);
        let A_start = &Self::step_pos(A_c, a, A_to, a_t, -0.5);

        let (B_c, b, B_to, b_t) = Self::dissect_step_cfg(b_cfg);
        let B_end = &Self::step_pos(B_c, b, B_to, b_t, 0.5);
        let step = B_end - A_start;
        (A_start + &step / 2.0, step)
    }

    #[allow(non_snake_case)]
    fn calc_max_step(a_cfg: &WalkSequenceFnConfig, b_cfg: &WalkSequenceFnConfig) -> Vector2 {
        let (A_c, a, A_to, a_t) = Self::dissect_step_cfg(a_cfg);
        let A_end = Self::step_pos(A_c, a, A_to, a_t, 0.5);

        let b = &b_cfg.step;
        let B_to = &b_cfg.turn_origin;
        let b_rm = transform::rotate_matrix2(b_cfg.turn_angle);
        let b_rm_neg = transform::rotate_matrix2(-b_cfg.turn_angle);

        b_rm_neg * (A_end - b + b_rm * B_to - B_to)
    }

    fn copy_step_details(&mut self, other: &WalkSequenceFnConfig) {
        self.config_active.step = other.step.clone();
        self.config_active.turn_origin = other.turn_origin.clone();
        self.config_active.turn_angle = other.turn_angle;
    }

    pub fn advance(&mut self, x: float) {
        if let Some(b_cfg) = &self.config_update.clone() {

            let x_prev = self.x;
            let rl = self.lift_ratio;
            let rp = 1.0 - self.lift_ratio;
            let (c1, c2, c3) = self.get_phase_shift_points();
            let a_cfg = &self.config_active.clone();

            if self.phase == 0 {
                if x_prev < c1 {
                    self.config_active.step_center = Self::calc_intersecting_step_center(a_cfg, b_cfg, -x_prev/rp);
                    self.copy_step_details(b_cfg);
                    self.phase = 2;
                }
                else if x_prev >= c1 && x_prev < c2 {
                    self.phase = 1;
                }
                else if x_prev >= c2 && x_prev < c3 {
                    self.config_active.step_center = Self::calc_intersecting_step_center(a_cfg, b_cfg, -c1/rp + (c2-c1)/rl - (x_prev-c2)/rp);
                    self.copy_step_details(b_cfg);
                    self.phase = 2;
                }
                else if x_prev >= c3 {
                    let xm_prev = (x_prev - c3) % 1.0;
                    if xm_prev < rl {
                        self.phase = 1;
                    }
                    else {
                        self.config_active.step_center = Self::calc_intersecting_step_center(a_cfg, b_cfg, 0.5 - (xm_prev-rl)/rp);
                        self.copy_step_details(b_cfg);
                        self.phase = 2;
                    }
                }
            }

            if self.phase == 1 {
                if x_prev < c2 && x >= c2 {
                    self.config_active.step_center = Self::calc_intersecting_step_center(a_cfg, b_cfg, -c1/rp + (c2-c1)/rl);
                    self.copy_step_details(b_cfg);
                    self.phase = 2;
                }
                else if x >= c3 {
                    let xm = (x - c3) % 1.0;
                    let xm_prev = (x_prev - c3) % 1.0;
                    if xm_prev < rl && xm > rl {
                        self.config_active.step_center = Self::calc_intersecting_step_center(a_cfg, b_cfg, 0.5);
                        self.copy_step_details(b_cfg);
                        self.phase = 2;
                    }
                }
            }
            else if self.phase == 2 {
                if x_prev < c1 && x >= c1 {
                    (self.config_active.step_center, self.config_active.step) = Self::calc_cross_step(a_cfg, b_cfg);
                    self.config_active.turn_angle = 0.0;
                    self.config_active.turn_origin = Vector2::zero();
                    self.config_active.step_height_weight = b_cfg.step_height_weight;
                    self.phase = 3;
                }
                else if x >= c3 {
                    let xm = (x - c3) % 1.0;
                    let xm_prev = (x_prev - c3) % 1.0;
                    if xm_prev > xm {
                        (self.config_active.step_center, self.config_active.step) = Self::calc_cross_step(a_cfg, b_cfg);
                        self.config_active.turn_angle = 0.0;
                        self.config_active.turn_origin = Vector2::zero();
                        self.config_active.step_height_weight = b_cfg.step_height_weight;
                        self.phase = 3;
                    }
                }
            }
            else if self.phase == 3 {
                if x >= c3 {
                    let xm = (x - c3) % 1.0;
                    let xm_prev = (x_prev - c3) % 1.0;
                    if xm >= rl && xm_prev < rl {
                        self.config_active = self.config_update.take().unwrap();
                    }
                }
            }

            #[allow(non_snake_case)]
            let (S_c, s, S_to, s_t) = Self::dissect_step_cfg(&self.config_active);
            self.step_len = Self::step_pos(S_c, s, S_to, s_t, -0.5).dist(&Self::step_pos(S_c, s, S_to, s_t, 0.5));
        }

        self.x = x;
    }

    #[allow(non_snake_case)]
    pub fn dist(&self) -> float {
        let (A_c, a, A_to, a_t) = Self::dissect_step_cfg(&self.config_active);
        let A_start = Self::step_pos(A_c, a, A_to, a_t, -0.5);
        let A_end = Self::step_pos(A_c, a, A_to, a_t, 0.5);
        A_start.dist(&A_end)
    }

    pub fn get(&self) -> Vector3 {
        let x = self.x;
        let s_height = self.step_len * self.config_active.step_height_weight;
        let rl = self.lift_ratio;
        let rp = 1.0 - self.lift_ratio;
        let (c1, c2, c3) = self.get_phase_shift_points();

        if x < c1 {
            Vector3::from(self.calc_step_pos(-x/rp))
        }
        else if x >= c1 && x < c2 {
            let vertical_pos = quad_step_height((x-c1) / (c2-c1), 0.0, s_height);
            let horizontal_pos = self.calc_step_pos(-c1/rp + (x-c1)/rl);
            Vector3::new(horizontal_pos[0], horizontal_pos[1], vertical_pos)
        }
        else if x >= c2 && x < c3 {
            Vector3::from(self.calc_step_pos(-c1/rp + (c2-c1)/rl - (x-c2)/rp))
        }
        else {
            let xm = (x - c3) % 1.0;
            if xm < rl {
                let vertical_pos = quad_step_height(xm/rl, 0.0, s_height);
                let horizontal_pos = self.calc_step_pos(-0.5 + xm/rl);
                Vector3::new(horizontal_pos[0], horizontal_pos[1], vertical_pos)
            }
            else {
                Vector3::from(self.calc_step_pos(0.5 - (xm-rl)/rp))
            }
        }
    }
}
