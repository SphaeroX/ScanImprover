use crate::mesh::Mesh;

pub fn load(bytes: &[u8]) -> Result<Mesh, String> {
    let text = std::str::from_utf8(bytes).map_err(|_| "OBJ file is not valid UTF-8".to_string())?;
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    // Quads are kept apart and placed first so they load as a `QuadLayout`.
    let mut quad_indices: Vec<u32> = Vec::new();
    for (li, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut toks = line.split_ascii_whitespace();
        let kw = toks.next().unwrap_or("");
        match kw {
            "v" => {
                let mut p = [0.0f32; 3];
                for c in p.iter_mut() {
                    let t = toks.next().ok_or_else(|| {
                        format!("OBJ line {}: vertex needs 3 coordinates", li + 1)
                    })?;
                    *c = t
                        .parse::<f32>()
                        .map_err(|_| format!("OBJ line {}: invalid float", li + 1))?;
                }
                positions.push(p);
            }
            "f" => {
                let refs: Vec<&str> = toks.collect();
                if refs.len() < 3 {
                    continue;
                }
                let resolve = |r: &str| -> Result<u32, String> {
                    let v = r
                        .split('/')
                        .next()
                        .unwrap_or("")
                        .parse::<i64>()
                        .map_err(|_| format!("OBJ line {}: invalid face index", li + 1))?;
                    let i = if v < 0 {
                        positions.len() as i64 + v
                    } else {
                        v - 1
                    };
                    if i < 0 || i as usize >= positions.len() {
                        return Err(format!("OBJ line {}: face index out of range", li + 1));
                    }
                    Ok(i as u32)
                };
                let out = if refs.len() == 4 {
                    &mut quad_indices
                } else {
                    &mut indices
                };
                let i0 = resolve(refs[0])?;
                for k in 1..refs.len() - 1 {
                    let ik = resolve(refs[k])?;
                    let ik1 = resolve(refs[k + 1])?;
                    out.push(i0);
                    out.push(ik);
                    out.push(ik1);
                }
            }
            _ => {}
        }
    }
    if positions.is_empty() || (indices.is_empty() && quad_indices.is_empty()) {
        return Err("OBJ contains no mesh faces".to_string());
    }
    let quads = quad_indices.len() / 6;
    quad_indices.extend_from_slice(&indices);
    let mut mesh = Mesh::from_indexed(positions, quad_indices);
    mesh.set_quad_layout(quads);
    Ok(mesh)
}

pub fn save(mesh: &Mesh) -> String {
    let mut out = String::with_capacity(mesh.positions.len() * 40 + mesh.indices.len() * 8);
    for (p, n) in mesh.positions.iter().zip(mesh.normals.iter()) {
        out.push_str(&format!("v {} {} {}\n", fmt(p[0]), fmt(p[1]), fmt(p[2])));
        let _ = n;
    }
    for (p, n) in mesh.positions.iter().zip(mesh.normals.iter()) {
        out.push_str(&format!("vn {} {} {}\n", fmt(n[0]), fmt(n[1]), fmt(n[2])));
        let _ = p;
    }
    // Quads (see `QuadLayout`) are written as real 4-sided faces.
    let quads = mesh.quad_count();
    for q in 0..quads {
        let i = &mesh.indices[6 * q..6 * q + 6];
        let (a, b, c, d) = (i[0] + 1, i[1] + 1, i[2] + 1, i[5] + 1);
        out.push_str(&format!("f {a}//{a} {b}//{b} {c}//{c} {d}//{d}\n"));
    }
    for t in 2 * quads..mesh.triangle_count() {
        let a = mesh.indices[3 * t] + 1;
        let b = mesh.indices[3 * t + 1] + 1;
        let c = mesh.indices[3 * t + 2] + 1;
        out.push_str(&format!("f {a}//{a} {b}//{b} {c}//{c}\n"));
    }
    out
}

fn fmt(v: f32) -> String {
    let s = format!("{v:.6}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygons_are_fanned_and_negative_indices_resolve() {
        let text = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n# quad with texture/normal refs\nf 1/1/1 2/2/2 3/3/3 4/4/4\nf -4 -3 -2\n";
        let m = load(text.as_bytes()).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 3);
        assert_eq!(&m.indices[6..9], &[0, 1, 2]);
    }

    #[test]
    fn rejects_out_of_range_and_empty_input() {
        assert!(load(b"v 0 0 0\nv 1 0 0\nf 1 2 3\n").is_err());
        assert!(load(b"v 0 0 0\n").is_err());
        assert!(load(b"v 0 0\n").is_err());
    }

    #[test]
    fn save_writes_normals_and_one_based_faces() {
        let m = Mesh::from_indexed(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            vec![0, 1, 2],
        );
        let s = save(&m);
        assert!(s.contains("v 0 0 0\n"));
        assert!(s.contains("vn 0 0 1\n"));
        assert!(s.contains("f 1//1 2//2 3//3\n"));
    }

    #[test]
    fn quads_are_written_as_quads_and_load_back() {
        let mut m = Mesh::from_indexed(
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [2.0, 0.0, 0.0],
            ],
            vec![0, 1, 2, 0, 2, 3, 1, 4, 2],
        );
        m.set_quad_layout(1);
        let s = save(&m);
        assert!(s.contains("f 1//1 2//2 3//3 4//4\n"), "{s}");
        assert!(s.contains("f 2//2 5//5 3//3\n"), "{s}");
        let back = load(s.as_bytes()).unwrap();
        assert_eq!(back.indices, m.indices, "fanning restores the triangles");
        assert_eq!(back.quad_count(), 1, "quads survive a round trip");
    }

    #[test]
    fn quads_load_first_as_a_quad_layout() {
        let text = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 2 0 0\nf 2 5 3\nf 1 2 3 4\n";
        let m = load(text.as_bytes()).unwrap();
        assert_eq!(m.quad_count(), 1);
        assert_eq!(m.indices, vec![0, 1, 2, 0, 2, 3, 1, 4, 2]);
        // Triangle-only files keep their order and have no quads.
        let tri = load(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
        assert_eq!(tri.quad_count(), 0);
        assert!(tri.quads.is_none());
    }
}
