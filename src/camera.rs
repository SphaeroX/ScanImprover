//! Turntable ("orbit") camera used by the 3D viewport.
//!
//! The camera looks at `target` from a `distance` along its local +Z (`back`)
//! axis. The orientation is stored as a quaternion so that callers can set it
//! directly, but all interactive rotation goes through a turntable model
//! (azimuth around the world up axis, clamped elevation) so the view never
//! rolls and never flips over the poles.

use crate::mesh::Aabb;
use glam::{Mat4, Quat, Vec3};

/// Named view presets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewDir {
    /// Look along +X (eye on the -X side).
    X,
    /// Look along -Y (eye on the +Y side).
    Y,
    /// Look along +Z (eye on the -Z side).
    Z,
    /// Default isometric-like view.
    Iso,
    /// Views named relative to the current up axis.
    Top,
    Bottom,
    Front,
    Back,
    Left,
    Right,
}

/// World up axis of the scene. Fusion 360 defaults to Y-up, most scanners
/// and CAD packages to Z-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpAxis {
    Y,
    Z,
}

impl UpAxis {
    pub fn vector(self) -> Vec3 {
        match self {
            UpAxis::Y => Vec3::Y,
            UpAxis::Z => Vec3::Z,
        }
    }

    /// Rotation that maps the canonical Y-up camera frame onto this up axis.
    fn base_rotation(self) -> Quat {
        match self {
            UpAxis::Y => Quat::IDENTITY,
            UpAxis::Z => Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            UpAxis::Y => "Y up",
            UpAxis::Z => "Z up",
        }
    }
}

/// Default iso view direction (eye position relative to the target) in the
/// canonical Y-up frame.
const ISO_BACK: [f32; 3] = [-0.8, 0.9, -0.8];

/// Radians of rotation per dragged pixel.
const ROTATE_SPEED: f32 = 0.008;
/// Elevation is clamped just short of the poles so the azimuth stays defined.
const MAX_ELEVATION: f32 = std::f32::consts::FRAC_PI_2 - 1e-4;

/// In-flight smooth transition between two camera poses.
#[derive(Clone, Copy, Debug)]
struct Transition {
    from_orient: Quat,
    to_orient: Quat,
    from_target: Vec3,
    to_target: Vec3,
    from_distance: f32,
    to_distance: f32,
    start: f64,
    duration: f64,
}

pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    pub orient: Quat,
    pub fov_y: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
    pub up_axis: UpAxis,
    transition: Option<Transition>,
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
            up_axis: UpAxis::Y,
            transition: None,
        }
    }
}

#[cfg_attr(not(test), allow(dead_code))]
impl Camera {
    // ----------------------------------------------------------------------
    // Basis vectors and matrices
    // ----------------------------------------------------------------------

    pub fn right(&self) -> Vec3 {
        self.orient * Vec3::X
    }

    pub fn up(&self) -> Vec3 {
        self.orient * Vec3::Y
    }

    pub fn back(&self) -> Vec3 {
        self.orient * Vec3::Z
    }

    pub fn forward(&self) -> Vec3 {
        -self.back()
    }

    pub fn eye(&self) -> Vec3 {
        self.target + self.back() * self.distance
    }

    pub fn view(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye(), self.target, self.up())
    }

    pub fn proj(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov_y, self.aspect.max(1e-3), self.near, self.far)
    }

    pub fn view_proj(&self) -> Mat4 {
        self.proj() * self.view()
    }

    pub fn is_animating(&self) -> bool {
        self.transition.is_some()
    }

    // ----------------------------------------------------------------------
    // Turntable parametrisation
    // ----------------------------------------------------------------------

    /// Orientation for the given azimuth / elevation in the current up frame.
    fn compose(&self, azimuth: f32, elevation: f32) -> Quat {
        (self.up_axis.base_rotation()
            * Quat::from_rotation_y(azimuth)
            * Quat::from_rotation_x(-elevation))
        .normalize()
    }

    /// Azimuth / elevation of the current orientation (roll is discarded).
    fn decompose(&self) -> (f32, f32) {
        let local = self.up_axis.base_rotation().inverse() * self.orient;
        let back = (local * Vec3::Z).normalize_or_zero();
        let elevation = back.y.clamp(-1.0, 1.0).asin();
        let azimuth = if back.x.abs() + back.z.abs() > 1e-6 {
            back.x.atan2(back.z)
        } else {
            // Exactly at a pole: derive the azimuth from the up vector instead.
            let up = local * Vec3::Y;
            (-up.x).atan2(-up.z)
        };
        (azimuth, elevation)
    }

    /// Orientation looking from direction `back` (unit vector from target to
    /// eye) with the horizon kept level in the current up frame.
    fn orient_from_back(&self, back: Vec3) -> Quat {
        let local = (self.up_axis.base_rotation().inverse() * back).normalize_or_zero();
        let elevation = local.y.clamp(-1.0, 1.0).asin();
        let azimuth = if local.x.abs() + local.z.abs() > 1e-6 {
            local.x.atan2(local.z)
        } else {
            0.0
        };
        self.compose(azimuth, elevation)
    }

    /// Re-levels the horizon after the up axis changed while keeping the view
    /// direction.
    pub fn set_up_axis(&mut self, up: UpAxis) {
        if self.up_axis == up {
            return;
        }
        let back = self.back();
        self.up_axis = up;
        self.transition = None;
        self.orient = self.orient_from_back(back);
    }

    // ----------------------------------------------------------------------
    // Interaction
    // ----------------------------------------------------------------------

    /// Turntable rotation by a mouse delta (pixels).
    pub fn rotate(&mut self, dx: f32, dy: f32) {
        self.transition = None;
        let (az, el) = self.decompose();
        let az = az - dx * ROTATE_SPEED;
        let el = (el + dy * ROTATE_SPEED).clamp(-MAX_ELEVATION, MAX_ELEVATION);
        self.orient = self.compose(az, el);
    }

    /// Turntable rotation around `pivot` (a world point, typically the point
    /// under the cursor when the drag started). The pivot stays fixed on
    /// screen while the camera swings around it.
    pub fn orbit_about(&mut self, pivot: Vec3, dx: f32, dy: f32) {
        let old = self.orient;
        self.rotate(dx, dy);
        let delta = self.orient * old.inverse();
        self.target = pivot + delta * (self.target - pivot);
    }

    /// Pans the view so that a point at the given depth (distance from the
    /// eye along the view direction) follows the cursor exactly.
    pub fn pan_pixels(&mut self, dx: f32, dy: f32, viewport_h: f32, depth: f32) {
        self.transition = None;
        let k = self.world_per_pixel_at(depth.max(1e-6), viewport_h);
        self.target += (self.up() * dy - self.right() * dx) * k;
    }

    /// Dolly by `factor` keeping `anchor` (a world point, typically the point
    /// under the cursor) fixed on screen.
    pub fn zoom_towards(&mut self, factor: f32, anchor: Vec3) {
        self.transition = None;
        let new_distance = (self.distance * factor).clamp(1e-4, 1.0e7);
        let f = new_distance / self.distance.max(1e-9);
        self.target = anchor + (self.target - anchor) * f;
        self.distance = new_distance;
    }

    /// World point under a screen position at the depth of the target plane
    /// (used as a zoom / pan anchor when the cursor is not over the mesh).
    pub fn point_on_target_plane(&self, sx: f32, sy: f32, w: f32, h: f32) -> Vec3 {
        let (ro, rd) = self.screen_ray(sx, sy, w, h);
        let fwd = self.forward();
        let denom = rd.dot(fwd);
        if denom.abs() < 1e-6 {
            return self.target;
        }
        let t = (self.target - ro).dot(fwd) / denom;
        ro + rd * t.max(0.0)
    }

    // ----------------------------------------------------------------------
    // View presets and framing
    // ----------------------------------------------------------------------

    /// Eye direction (unit, target -> eye) for a preset in the current frame.
    fn preset_back(&self, dir: ViewDir) -> Vec3 {
        let up = self.up_axis.vector();
        let base = self.up_axis.base_rotation();
        // Horizontal reference axes in the current up frame: "front" is the
        // side facing the viewer in the default frame (+Z for Y-up, -Y for
        // Z-up), "right" is +X for both.
        let front = base * Vec3::Z;
        let right = Vec3::X;
        match dir {
            ViewDir::X => -Vec3::X,
            ViewDir::Y => Vec3::Y,
            ViewDir::Z => -Vec3::Z,
            ViewDir::Iso => match self.up_axis {
                UpAxis::Y => Vec3::from(ISO_BACK).normalize(),
                UpAxis::Z => Vec3::new(0.8, -0.8, 0.9).normalize(),
            },
            ViewDir::Top => up,
            ViewDir::Bottom => -up,
            ViewDir::Front => front,
            ViewDir::Back => -front,
            ViewDir::Right => right,
            ViewDir::Left => -right,
        }
    }

    /// Snaps to a view preset immediately.
    pub fn set_view(&mut self, dir: ViewDir) {
        self.transition = None;
        let back = self.preset_back(dir);
        self.orient = self.orient_from_back(back);
    }

    /// Distance at which a sphere of `radius` fills the view with a margin.
    fn fit_distance(&self, radius: f32) -> f32 {
        let half_v = self.fov_y * 0.5;
        let half_h = (half_v.tan() * self.aspect.max(1e-3)).atan();
        let half = half_v.min(half_h).max(1e-3);
        (radius.max(1e-3) / half.sin()) * 1.15
    }

    /// Frames the bounding box immediately.
    pub fn fit(&mut self, bb: &Aabb) {
        self.transition = None;
        self.target = bb.center();
        self.distance = self.fit_distance(bb.diagonal() * 0.5).max(1e-3);
    }

    /// Smoothly frames the bounding box.
    pub fn animate_fit(&mut self, bb: &Aabb, now: f64) {
        let distance = self.fit_distance(bb.diagonal() * 0.5).max(1e-3);
        self.animate_to(self.orient, bb.center(), distance, now);
    }

    /// Smoothly rotates to a view preset (target and distance unchanged).
    pub fn animate_view(&mut self, dir: ViewDir, now: f64) {
        let back = self.preset_back(dir);
        let orient = self.orient_from_back(back);
        self.animate_to(orient, self.target, self.distance, now);
    }

    /// Smoothly rotates to look from the given direction.
    pub fn animate_look_from(&mut self, back: Vec3, now: f64) {
        if back.length_squared() < 1e-12 {
            return;
        }
        let orient = self.orient_from_back(back.normalize());
        self.animate_to(orient, self.target, self.distance, now);
    }

    fn animate_to(&mut self, orient: Quat, target: Vec3, distance: f32, now: f64) {
        self.transition = Some(Transition {
            from_orient: self.orient,
            to_orient: orient,
            from_target: self.target,
            to_target: target,
            from_distance: self.distance,
            to_distance: distance,
            start: now,
            duration: 0.45,
        });
    }

    /// Advances an in-flight transition. Returns true while still animating.
    pub fn tick(&mut self, now: f64) -> bool {
        let Some(tr) = self.transition else {
            return false;
        };
        let t = ((now - tr.start) / tr.duration).clamp(0.0, 1.0) as f32;
        // Ease-out cubic.
        let s = 1.0 - (1.0 - t).powi(3);
        self.orient = tr.from_orient.slerp(tr.to_orient, s).normalize();
        self.target = tr.from_target.lerp(tr.to_target, s);
        self.distance = (tr.from_distance.ln() * (1.0 - s) + tr.to_distance.ln() * s).exp();
        if t >= 1.0 {
            self.orient = tr.to_orient;
            self.target = tr.to_target;
            self.distance = tr.to_distance;
            self.transition = None;
            return false;
        }
        true
    }

    /// Chooses near / far planes that tightly enclose the scene bounds for
    /// the current eye position (keeps the 24-bit depth buffer precise).
    pub fn update_clip_planes(&mut self, scene: &Aabb, extra_radius: f32) {
        let eye = self.eye();
        let mut d_max = 0.0f32;
        for i in 0..8 {
            let c = Vec3::new(
                if i & 1 == 0 { scene.min.x } else { scene.max.x },
                if i & 2 == 0 { scene.min.y } else { scene.max.y },
                if i & 4 == 0 { scene.min.z } else { scene.max.z },
            );
            d_max = d_max.max((c - eye).length());
        }
        let d_min = {
            let clamped = eye.clamp(scene.min, scene.max);
            (clamped - eye).length()
        };
        let far = (d_max + extra_radius).max(self.distance * 2.0).max(1.0);
        let near = (d_min * 0.5).clamp(far * 2e-5, far * 0.25).max(1e-4);
        self.near = near;
        self.far = far;
    }

    // ----------------------------------------------------------------------
    // Projection helpers
    // ----------------------------------------------------------------------

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

    /// Projects a world point to viewport pixels. Returns `None` when the
    /// point is behind the camera.
    pub fn project(&self, p: Vec3, w: f32, h: f32) -> Option<(f32, f32)> {
        let clip = self.view_proj() * p.extend(1.0);
        if clip.w <= 1e-6 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        Some(((ndc.x + 1.0) * 0.5 * w, (1.0 - ndc.y) * 0.5 * h))
    }

    /// Key and fill light directions in world space, relative to the camera
    /// frame, so surfaces facing the camera are always well lit.
    pub fn light_directions(&self) -> (Vec3, Vec3) {
        let back = self.back();
        let right = self.right();
        let up = self.up();
        let l1 = (back * 0.75 + right * 0.35 + up * 0.55).normalize();
        let l2 = (back * 0.45 - right * 0.40 - up * 0.30).normalize();
        (l1, l2)
    }
}

#[cfg(test)]
#[allow(clippy::field_reassign_with_default)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_light_directions_follow_orientation() {
        let mut cam = Camera::default();

        let (l1, l2) = cam.light_directions();
        assert!((l1.length() - 1.0).abs() < 1e-5);
        assert!((l2.length() - 1.0).abs() < 1e-5);
        assert!(
            l1.dot(cam.back()) > 0.0,
            "Key light must shine from the camera's viewing hemisphere"
        );
        assert!(
            l2.dot(cam.back()) > 0.0,
            "Fill light must shine from the camera's viewing hemisphere"
        );

        cam.rotate(0.0, 50.0);
        let (l1_down, l2_down) = cam.light_directions();
        assert!((l1_down.length() - 1.0).abs() < 1e-5);
        assert!((l2_down.length() - 1.0).abs() < 1e-5);
        assert!(l1_down.dot(cam.back()) > 0.0);
        assert!(l2_down.dot(cam.back()) > 0.0);
        assert!((l1_down - l1).length() > 0.1);

        for dir in [ViewDir::X, ViewDir::Y, ViewDir::Z, ViewDir::Iso] {
            cam.set_view(dir);
            let (l1, l2) = cam.light_directions();
            assert!(
                l1.dot(cam.back()) > 0.5,
                "Key light stays oriented with view direction"
            );
            assert!(
                l2.dot(cam.back()) > 0.3,
                "Fill light stays oriented with view direction"
            );
        }
    }

    #[test]
    fn turntable_keeps_horizon_level_and_clamps_pitch() {
        let mut cam = Camera::default();
        // Drag far past the pole: elevation must clamp, never flip.
        cam.rotate(0.0, 10_000.0);
        assert!(cam.up().y > 0.0, "camera must not flip upside down");
        assert!(
            cam.back().y > 0.99,
            "camera should be looking straight down"
        );
        // Yaw only: the up vector keeps a positive world-Y component and the
        // right vector stays horizontal (no roll).
        cam.rotate(300.0, 0.0);
        assert!(
            cam.right().y.abs() < 1e-4,
            "right vector must stay horizontal"
        );
        for _ in 0..50 {
            cam.rotate(37.0, -13.0);
        }
        assert!(cam.right().y.abs() < 1e-3, "no roll may accumulate");
    }

    #[test]
    fn presets_have_level_horizon_and_expected_directions() {
        let mut cam = Camera::default();
        cam.set_view(ViewDir::Z);
        assert!((cam.back() - (-Vec3::Z)).length() < 1e-5);
        assert!(cam.up().y > 0.99, "Z view must be upright, not mirrored");
        cam.set_view(ViewDir::X);
        assert!((cam.back() - (-Vec3::X)).length() < 1e-5);
        assert!(cam.up().y > 0.99);
        cam.set_view(ViewDir::Y);
        assert!((cam.back() - Vec3::Y).length() < 1e-5);
        assert!(cam.right().y.abs() < 1e-5);

        cam.set_up_axis(UpAxis::Z);
        cam.set_view(ViewDir::Top);
        assert!((cam.back() - Vec3::Z).length() < 1e-5);
        cam.set_view(ViewDir::Front);
        assert!(
            cam.up().z > 0.99,
            "Z-up front view must have Z pointing up on screen"
        );
        assert!(cam.right().z.abs() < 1e-5);
    }

    #[test]
    fn orbit_about_pivot_keeps_pivot_on_screen() {
        let mut cam = Camera::default();
        cam.target = Vec3::ZERO;
        cam.distance = 50.0;
        cam.aspect = 1.0;
        cam.set_view(ViewDir::Iso);
        let pivot = Vec3::new(8.0, 3.0, -4.0);
        let before = cam.project(pivot, 800.0, 800.0).unwrap();
        cam.orbit_about(pivot, 120.0, -60.0);
        let after = cam.project(pivot, 800.0, 800.0).unwrap();
        assert!((before.0 - after.0).abs() < 0.5 && (before.1 - after.1).abs() < 0.5);
    }

    #[test]
    fn zoom_towards_keeps_anchor_on_screen() {
        let mut cam = Camera::default();
        cam.distance = 100.0;
        cam.aspect = 1.5;
        let anchor = cam.point_on_target_plane(600.0, 200.0, 900.0, 600.0);
        let before = cam.project(anchor, 900.0, 600.0).unwrap();
        cam.zoom_towards(0.6, anchor);
        let after = cam.project(anchor, 900.0, 600.0).unwrap();
        assert!((before.0 - after.0).abs() < 0.5 && (before.1 - after.1).abs() < 0.5);
        assert!((cam.distance - 60.0).abs() < 1e-3);
    }

    #[test]
    fn fit_frames_bounding_sphere_inside_view() {
        let mut cam = Camera::default();
        cam.aspect = 1.0;
        let bb = Aabb {
            min: Vec3::splat(-1.0),
            max: Vec3::splat(1.0),
        };
        cam.fit(&bb);
        // The bounding sphere must fit inside the vertical field of view.
        let half_angle = (bb.diagonal() * 0.5 / cam.distance).asin();
        assert!(half_angle < cam.fov_y * 0.5);
    }

    #[test]
    fn transition_reaches_target_pose() {
        let mut cam = Camera::default();
        cam.animate_view(ViewDir::Top, 0.0);
        assert!(cam.is_animating());
        assert!(cam.tick(0.1));
        assert!(!cam.tick(5.0));
        assert!((cam.back() - Vec3::Y).length() < 1e-5);
    }
}
