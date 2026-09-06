use glam::{Quat, Vec3};

/// Reference to a geometric feature that can be assigned to an axis or origin.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FeatureRef {
    SymmetryPlane,
    Plane(u64),
    Circle(u64),
}

/// Target coordinate axis for orientation alignment.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AxisChoice {
    X,
    Y,
    Z,
}

impl AxisChoice {
    pub fn vector(&self) -> Vec3 {
        match self {
            AxisChoice::X => Vec3::X,
            AxisChoice::Y => Vec3::Y,
            AxisChoice::Z => Vec3::Z,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            AxisChoice::X => "X",
            AxisChoice::Y => "Y",
            AxisChoice::Z => "Z",
        }
    }
}

/// Origin constraint definition.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OriginRef {
    /// Compute origin from assigned feature positions (e.g. X from slot X, Y from slot Y, Z from slot Z).
    FromAssignedFeatures,
    /// Origin at the center of the specified circle.
    CircleCenter(u64),
    /// Origin on the specified plane.
    Plane(u64),
    /// Origin on the symmetry plane.
    SymmetryPlane,
    /// Origin at the bounding box center of the mesh.
    BBoxCenter,
}

/// Definition of feature alignment slots.
#[derive(Clone, Debug)]
pub struct AlignmentSlots {
    pub x: Option<FeatureRef>,
    pub y: Option<FeatureRef>,
    pub z: Option<FeatureRef>,
    pub origin: OriginRef,
    pub primary_axis: AxisChoice,
}

impl Default for AlignmentSlots {
    fn default() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            origin: OriginRef::FromAssignedFeatures,
            primary_axis: AxisChoice::Z,
        }
    }
}

/// Result of an alignment computation.
#[derive(Clone, Debug)]
pub struct AlignmentTransform {
    pub rotation: Quat,
    pub translation: Vec3,
    #[allow(dead_code)]
    pub residual_deg: f32,
    pub summary: String,
}

/// Geometry provider interface for alignment calculation.
pub trait AlignmentGeometrySource {
    fn get_feature_direction_and_point(&self, feat: FeatureRef) -> Option<(Vec3, Vec3)>;
    fn get_feature_name(&self, feat: FeatureRef) -> String;
    fn get_bbox_center(&self) -> Vec3;
}

/// Rotates vector `from` to vector `to`.
pub fn rotation_between(from: Vec3, to: Vec3) -> Quat {
    let f = from.normalize_or_zero();
    let t = to.normalize_or_zero();
    if f.length_squared() < 1e-6 || t.length_squared() < 1e-6 {
        return Quat::IDENTITY;
    }
    let d = f.dot(t).clamp(-1.0, 1.0);
    if d > 0.999999 {
        Quat::IDENTITY
    } else if d < -0.999999 {
        let mut axis = Vec3::X.cross(f);
        if axis.length_squared() < 1e-6 {
            axis = Vec3::Y.cross(f);
        }
        Quat::from_axis_angle(axis.normalize(), std::f32::consts::PI)
    } else {
        let axis = f.cross(t);
        let s = (2.0 * (1.0 + d)).sqrt();
        let inv_s = 1.0 / s;
        Quat::from_xyzw(axis.x * inv_s, axis.y * inv_s, axis.z * inv_s, 0.5 * s).normalize()
    }
}

/// Computes the 3-2-1 datum alignment transform (rotation and translation)
/// based on assigned feature slots.
pub fn compute_alignment<S: AlignmentGeometrySource>(
    slots: &AlignmentSlots,
    source: &S,
) -> Result<AlignmentTransform, String> {
    // 1. Collect assigned axes
    let mut assigned = Vec::new();
    if let Some(fx) = slots.x {
        if let Some((dir, pt)) = source.get_feature_direction_and_point(fx) {
            assigned.push((AxisChoice::X, fx, dir.normalize_or_zero(), pt));
        }
    }
    if let Some(fy) = slots.y {
        if let Some((dir, pt)) = source.get_feature_direction_and_point(fy) {
            assigned.push((AxisChoice::Y, fy, dir.normalize_or_zero(), pt));
        }
    }
    if let Some(fz) = slots.z {
        if let Some((dir, pt)) = source.get_feature_direction_and_point(fz) {
            assigned.push((AxisChoice::Z, fz, dir.normalize_or_zero(), pt));
        }
    }

    if assigned.is_empty() {
        return Err("No features assigned to any coordinate axis.".to_string());
    }

    // 2. Select primary and secondary axis
    // Prefer user's primary_axis if assigned, else pick first assigned.
    let prim_idx = assigned
        .iter()
        .position(|(ax, _, _, _)| *ax == slots.primary_axis)
        .unwrap_or(0);

    let (prim_axis, _prim_feat, prim_dir, _prim_pt) = assigned[prim_idx];
    let prim_target = prim_axis.vector();

    // Step A: Rotate primary direction to primary target axis
    let q1 = rotation_between(prim_dir, prim_target);

    // Step B: If there is a secondary axis assigned, rotate around primary target axis
    let mut rotation = q1;
    let mut residual_deg = 0.0f32;

    if assigned.len() >= 2 {
        let sec_idx = if prim_idx == 0 { 1 } else { 0 };
        let (sec_axis, _sec_feat, sec_dir, _sec_pt) = assigned[sec_idx];
        let sec_target = sec_axis.vector();

        // Direction of secondary feature after q1 rotation
        let sec_transformed = q1 * sec_dir;

        // Project onto plane perpendicular to prim_target
        let proj = sec_transformed - prim_target * sec_transformed.dot(prim_target);
        if proj.length_squared() > 1e-6 {
            let proj_norm = proj.normalize();
            // Angle around prim_target from proj_norm to sec_target
            let cos_theta = proj_norm.dot(sec_target).clamp(-1.0, 1.0);
            let sin_theta = prim_target.dot(proj_norm.cross(sec_target));
            let theta = sin_theta.atan2(cos_theta);

            let q2 = Quat::from_axis_angle(prim_target, theta);
            rotation = (q2 * q1).normalize();

            // Calculate angular residual error for secondary feature
            let final_sec_dir = rotation * sec_dir;
            let dot = final_sec_dir.dot(sec_target).clamp(-1.0, 1.0);
            residual_deg = dot.acos().to_degrees();
        }

        // Check tertiary feature if present
        if assigned.len() == 3 {
            let tert_idx = 3 - prim_idx - sec_idx;
            let (tert_axis, _tert_feat, tert_dir, _tert_pt) = assigned[tert_idx];
            let tert_target = tert_axis.vector();
            let final_tert_dir = rotation * tert_dir;
            let tert_dot = final_tert_dir.dot(tert_target).clamp(-1.0, 1.0);
            let tert_err = tert_dot.acos().to_degrees();
            residual_deg = residual_deg.max(tert_err);
        }
    }

    // 3. Compute Translation according to origin setting
    let translation = match slots.origin {
        OriginRef::FromAssignedFeatures => {
            let mut t = Vec3::ZERO;
            let bbox_c = source.get_bbox_center();
            let bbox_c_rot = rotation * bbox_c;

            // Compute component along X
            if let Some(fx) = slots.x {
                if let Some((_, pt)) = source.get_feature_direction_and_point(fx) {
                    let pt_rot = rotation * pt;
                    t.x = -pt_rot.x;
                }
            } else {
                t.x = -bbox_c_rot.x;
            }

            // Compute component along Y
            if let Some(fy) = slots.y {
                if let Some((_, pt)) = source.get_feature_direction_and_point(fy) {
                    let pt_rot = rotation * pt;
                    t.y = -pt_rot.y;
                }
            } else {
                t.y = -bbox_c_rot.y;
            }

            // Compute component along Z
            if let Some(fz) = slots.z {
                if let Some((_, pt)) = source.get_feature_direction_and_point(fz) {
                    let pt_rot = rotation * pt;
                    t.z = -pt_rot.z;
                }
            } else {
                t.z = -bbox_c_rot.z;
            }

            t
        }
        OriginRef::CircleCenter(id) => {
            if let Some((_, pt)) = source.get_feature_direction_and_point(FeatureRef::Circle(id)) {
                -(rotation * pt)
            } else {
                Vec3::ZERO
            }
        }
        OriginRef::Plane(id) => {
            if let Some((dir, pt)) = source.get_feature_direction_and_point(FeatureRef::Plane(id)) {
                let n_rot = rotation * dir;
                let pt_rot = rotation * pt;
                let dist = pt_rot.dot(n_rot);
                -n_rot * dist
            } else {
                Vec3::ZERO
            }
        }
        OriginRef::SymmetryPlane => {
            if let Some((dir, pt)) =
                source.get_feature_direction_and_point(FeatureRef::SymmetryPlane)
            {
                let n_rot = rotation * dir;
                let pt_rot = rotation * pt;
                let dist = pt_rot.dot(n_rot);
                -n_rot * dist
            } else {
                Vec3::ZERO
            }
        }
        OriginRef::BBoxCenter => -(rotation * source.get_bbox_center()),
    };

    // 4. Build summary
    let mut parts = Vec::new();
    if let Some(fx) = slots.x {
        parts.push(format!("X = {}", source.get_feature_name(fx)));
    }
    if let Some(fy) = slots.y {
        parts.push(format!("Y = {}", source.get_feature_name(fy)));
    }
    if let Some(fz) = slots.z {
        parts.push(format!("Z = {}", source.get_feature_name(fz)));
    }

    let summary = if residual_deg > 0.001 {
        format!(
            "Aligned: {} (res. {:.2}°). Primary: {}",
            parts.join(", "),
            residual_deg,
            prim_axis.name()
        )
    } else {
        format!(
            "Aligned: {}. Primary: {}",
            parts.join(", "),
            prim_axis.name()
        )
    };

    Ok(AlignmentTransform {
        rotation,
        translation,
        residual_deg,
        summary,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockSource {
        sym: Option<(Vec3, Vec3)>,
        planes: Vec<(u64, Vec3, Vec3)>,
        circles: Vec<(u64, Vec3, Vec3)>,
        bbox_center: Vec3,
    }

    impl AlignmentGeometrySource for MockSource {
        fn get_feature_direction_and_point(&self, feat: FeatureRef) -> Option<(Vec3, Vec3)> {
            match feat {
                FeatureRef::SymmetryPlane => self.sym,
                FeatureRef::Plane(id) => self
                    .planes
                    .iter()
                    .find(|(pid, _, _)| *pid == id)
                    .map(|(_, dir, pt)| (*dir, *pt)),
                FeatureRef::Circle(id) => self
                    .circles
                    .iter()
                    .find(|(cid, _, _)| *cid == id)
                    .map(|(_, dir, pt)| (*dir, *pt)),
            }
        }

        fn get_feature_name(&self, feat: FeatureRef) -> String {
            match feat {
                FeatureRef::SymmetryPlane => "Symmetry plane".to_string(),
                FeatureRef::Plane(id) => format!("Plane {id}"),
                FeatureRef::Circle(id) => format!("Circle {id}"),
            }
        }

        fn get_bbox_center(&self) -> Vec3 {
            self.bbox_center
        }
    }

    #[test]
    fn test_align_symmetry_x_circle_z_plane_y() {
        // Symmetry plane normal roughly pointing in Y, to be aligned to X:
        let sym_normal = Vec3::new(0.0, 1.0, 0.0);
        let sym_pt = Vec3::new(0.0, 5.0, 0.0);

        // Circle axis along X, to be aligned to Z:
        let circle_normal = Vec3::new(1.0, 0.0, 0.0);
        let circle_center = Vec3::new(12.0, 5.0, 20.0);

        // Plane normal along Z, to be aligned to Y:
        let plane_normal = Vec3::new(0.0, 0.0, 1.0);
        let plane_pt = Vec3::new(0.0, 0.0, -8.0);

        let source = MockSource {
            sym: Some((sym_normal, sym_pt)),
            planes: vec![(1, plane_normal, plane_pt)],
            circles: vec![(2, circle_normal, circle_center)],
            bbox_center: Vec3::ZERO,
        };

        let slots = AlignmentSlots {
            x: Some(FeatureRef::SymmetryPlane),
            y: Some(FeatureRef::Plane(1)),
            z: Some(FeatureRef::Circle(2)),
            origin: OriginRef::FromAssignedFeatures,
            primary_axis: AxisChoice::X,
        };

        let res = compute_alignment(&slots, &source).expect("alignment computation should succeed");

        // Transformed symmetry normal should be along X
        let sym_n_trans = res.rotation * sym_normal;
        assert!(
            (sym_n_trans - Vec3::X).length() < 1e-4,
            "Symmetry normal should point along X"
        );

        // Transformed circle normal should point along Z (since it was perpendicular to sym_normal)
        let circle_n_trans = res.rotation * circle_normal;
        assert!(
            (circle_n_trans - Vec3::Z).length() < 1e-4,
            "Circle axis should point along Z"
        );

        // Transformed plane normal should point along Y
        let plane_n_trans = res.rotation * plane_normal;
        assert!(
            (plane_n_trans - Vec3::Y).length() < 1e-4,
            "Plane normal should point along Y"
        );

        // Translation check:
        // Symmetry plane should pass through X = 0
        let sym_pt_final = res.rotation * sym_pt + res.translation;
        assert!(
            sym_pt_final.x.abs() < 1e-4,
            "Symmetry plane should pass through X = 0, got {}",
            sym_pt_final.x
        );

        // Plane 1 should pass through Y = 0
        let plane_pt_final = res.rotation * plane_pt + res.translation;
        assert!(
            plane_pt_final.y.abs() < 1e-4,
            "Plane 1 should pass through Y = 0, got {}",
            plane_pt_final.y
        );

        // Circle center should have Z = 0
        let circle_c_final = res.rotation * circle_center + res.translation;
        assert!(
            circle_c_final.z.abs() < 1e-4,
            "Circle center should have Z = 0, got {}",
            circle_c_final.z
        );
    }

    #[test]
    fn test_align_origin_circle_center() {
        let circle_normal = Vec3::Z;
        let circle_center = Vec3::new(10.0, 20.0, 30.0);

        let source = MockSource {
            sym: None,
            planes: vec![],
            circles: vec![(1, circle_normal, circle_center)],
            bbox_center: Vec3::ZERO,
        };

        let slots = AlignmentSlots {
            x: None,
            y: None,
            z: Some(FeatureRef::Circle(1)),
            origin: OriginRef::CircleCenter(1),
            primary_axis: AxisChoice::Z,
        };

        let res = compute_alignment(&slots, &source).expect("circle alignment should succeed");

        let circle_c_final = res.rotation * circle_center + res.translation;
        assert!(
            circle_c_final.length() < 1e-4,
            "Circle center should be exactly at (0,0,0)"
        );
    }
}
