use crate::mesh::Aabb;
use glam::{Mat4, Quat, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewDir {
    X,
    Y,
    Z,
    Iso,
}

const ISO_BACK: [f32; 3] = [-0.8, 0.9, -0.8];

pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    pub orient: Quat,
    pub fov_y: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Camera {
            target: Vec3::ZERO,
            distance: 200.0,
            orient: Quat::from_rotation_arc(Vec3::Z, Vec3::from(ISO_BACK).normalize()),
            fov_y: 45.0f32.to_radians(),
            aspect: 1.0,
            near: 0.01,
            far: 1.0e7,
        }
    }
}

impl Camera {
    pub fn fit(&mut self, bb: &Aabb) {
        self.target = bb.center();
        self.distance = (bb.diagonal() * 1.8).max(1e-3);
    }

    pub fn right(&self) -> Vec3 {
        self.orient * Vec3::X
    }

    pub fn up(&self) -> Vec3 {
        self.orient * Vec3::Y
    }

    pub fn back(&self) -> Vec3 {
        self.orient * Vec3::Z
    }

    pub fn eye(&self) -> Vec3 {
        self.target + self.back() * self.distance
    }

    pub fn view(&self) -> Mat4 {
        glam::camera::rh::view::look_at_mat4(self.eye(), self.target, self.up())
    }

    pub fn proj(&self) -> Mat4 {
        glam::camera::rh::proj::directx::perspective(
            self.fov_y,
            self.aspect.max(1e-3),
            self.near,
            self.far,
        )
    }

    pub fn view_proj(&self) -> Mat4 {
        self.proj() * self.view()
    }

    pub fn rotate(&mut self, dx: f32, dy: f32) {
        const K: f32 = 0.008;
        let yaw_q = Quat::from_axis_angle(Vec3::Y, -dx * K);
        let pitch_q = Quat::from_axis_angle(self.right(), dy * K);
        self.orient = (yaw_q * pitch_q * self.orient).normalize();
    }

    pub fn pan_drag(&mut self, dx: f32, dy: f32, viewport_h: f32) {
        let right = self.right();
        let up = self.up();
        let k = self.distance * 1.6 / viewport_h.max(1.0);
        self.target += (up * dy - right * dx) * k;
    }

    pub fn zoom(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(1e-4, 1.0e7);
    }

    pub fn set_view(&mut self, dir: ViewDir) {
        self.orient = match dir {
            ViewDir::X => Quat::from_rotation_arc(Vec3::Z, -Vec3::X),
            ViewDir::Y => Quat::from_rotation_arc(Vec3::Z, Vec3::Y),
            ViewDir::Z => Quat::from_axis_angle(Vec3::X, std::f32::consts::PI),
            ViewDir::Iso => Quat::from_rotation_arc(Vec3::Z, Vec3::from(ISO_BACK).normalize()),
        };
    }

    pub fn world_per_pixel_at(&self, depth: f32, viewport_h_px: f32) -> f32 {
        let h = viewport_h_px.max(1.0);
        2.0 * (self.fov_y * 0.5).tan() * depth.max(0.0) / h
    }

    pub fn screen_ray(&self, sx: f32, sy: f32, w: f32, h: f32) -> (Vec3, Vec3) {
        let ndc_x = (sx / w.max(1.0)) * 2.0 - 1.0;
        let ndc_y = 1.0 - 2.0 * (sy / h.max(1.0));
        let t = (self.fov_y * 0.5).tan();
        let dir = -self.back() + self.right() * (ndc_x * t * self.aspect) + self.up() * (ndc_y * t);
        (self.eye(), dir.normalize_or_zero())
    }
}
