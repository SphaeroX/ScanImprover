use crate::mesh::Mesh;

unsafe extern "C" {
    fn meshopt_simplify(
        destination: *mut u32,
        indices: *const u32,
        index_count: usize,
        vertex_positions: *const f32,
        vertex_count: usize,
        vertex_positions_stride: usize,
        target_index_count: usize,
        target_error: f32,
        options: u32,
        result_error: *mut f32,
    ) -> usize;
}

pub const LOCK_BORDER: u32 = 1 << 0;
pub const ERROR_ABSOLUTE: u32 = 1 << 2;

pub fn decimate(
    mesh: &Mesh,
    target_ratio: f32,
    max_error_abs: f32,
    lock_border: bool,
) -> Result<(Mesh, f32), String> {
    let index_count = mesh.indices.len();
    if index_count < 12 {
        return Err("Mesh is too small to simplify".to_string());
    }
    let target = ((index_count as f32) * target_ratio.clamp(0.0001, 1.0)) as usize;
    let target = target.clamp(3, index_count);
    let mut dst = vec![0u32; index_count];
    let mut result_error = 0.0f32;
    let options = ERROR_ABSOLUTE | if lock_border { LOCK_BORDER } else { 0 };
    let n = unsafe {
        meshopt_simplify(
            dst.as_mut_ptr(),
            mesh.indices.as_ptr(),
            index_count,
            mesh.positions.as_ptr() as *const f32,
            mesh.positions.len(),
            12,
            target,
            max_error_abs.max(1e-6),
            options,
            &mut result_error,
        )
    };
    if n < 3 {
        return Err(
            "Simplification produced no triangles. Increase the error tolerance.".to_string(),
        );
    }
    let old_indices = &dst[..n];
    let mut remap = vec![u32::MAX; mesh.positions.len()];
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(mesh.positions.len());
    let mut new_indices: Vec<u32> = Vec::with_capacity(n);
    for &i in old_indices {
        let old = i as usize;
        let ni = if remap[old] == u32::MAX {
            let ni = positions.len() as u32;
            remap[old] = ni;
            positions.push(mesh.positions[old]);
            ni
        } else {
            remap[old]
        };
        new_indices.push(ni);
    }
    Ok((Mesh::from_indexed(positions, new_indices), result_error))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tiny_meshes_are_rejected() {
        let m = Mesh::from_indexed(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            vec![0, 1, 2],
        );
        assert!(decimate(&m, 0.5, 0.1, false).is_err());
    }
}
