use crate::mesh::Mesh;

pub fn load(bytes: &[u8]) -> Result<Mesh, String> {
    let text = std::str::from_utf8(bytes).map_err(|_| "OBJ file is not valid UTF-8".to_string())?;
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
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
                let i0 = resolve(refs[0])?;
                for k in 1..refs.len() - 1 {
                    let ik = resolve(refs[k])?;
                    let ik1 = resolve(refs[k + 1])?;
                    indices.push(i0);
                    indices.push(ik);
                    indices.push(ik1);
                }
            }
            _ => {}
        }
    }
    if positions.is_empty() || indices.is_empty() {
        return Err("OBJ contains no mesh faces".to_string());
    }
    Ok(Mesh::from_indexed(positions, indices))
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
    for t in 0..mesh.triangle_count() {
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
