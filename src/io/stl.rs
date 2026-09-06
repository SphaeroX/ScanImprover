use crate::mesh::Mesh;
use glam::Vec3;

pub fn load(bytes: &[u8]) -> Result<Mesh, String> {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes(
            bytes[80..84]
                .try_into()
                .map_err(|_| "Invalid STL header".to_string())?,
        ) as usize;
        if n.checked_mul(50).and_then(|body| body.checked_add(84)) == Some(bytes.len()) {
            return load_binary(bytes, n);
        }
    }
    load_ascii(bytes)
}

fn load_binary(bytes: &[u8], n: usize) -> Result<Mesh, String> {
    let mut raw = Vec::with_capacity(n * 3);
    let mut off = 84usize;
    for _ in 0..n {
        if off + 50 > bytes.len() {
            return Err("STL file is truncated".to_string());
        }
        let mut tri = [[0.0f32; 3]; 3];
        for (k, v) in tri.iter_mut().enumerate() {
            for c in 0..3 {
                let at = off + 12 + 12 * k + 4 * c;
                v[c] = f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
            }
        }
        for v in tri {
            raw.push(v);
        }
        off += 50;
    }
    Ok(Mesh::from_corners(&raw))
}

fn load_ascii(bytes: &[u8]) -> Result<Mesh, String> {
    let text = std::str::from_utf8(bytes)
        .map_err(|_| "STL is neither valid binary nor UTF-8 ASCII".to_string())?;
    if !text.to_ascii_lowercase().contains("facet") {
        return Err("Invalid STL file".to_string());
    }
    let mut raw: Vec<[f32; 3]> = Vec::new();
    for (li, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut toks = line.split_ascii_whitespace();
        let kw = toks.next().unwrap_or("");
        if !kw.eq_ignore_ascii_case("vertex") {
            continue;
        }
        let mut p = [0.0f32; 3];
        for c in p.iter_mut() {
            let t = toks
                .next()
                .ok_or_else(|| format!("STL line {}: vertex needs 3 coordinates", li + 1))?;
            *c = t
                .parse::<f32>()
                .map_err(|_| format!("STL line {}: invalid float '{t}'", li + 1))?;
        }
        raw.push(p);
    }
    if raw.is_empty() || raw.len() % 3 != 0 {
        return Err("STL file contains no complete triangles".to_string());
    }
    Ok(Mesh::from_corners(&raw))
}

pub fn save(mesh: &Mesh, path: &std::path::Path) -> Vec<u8> {
    let n = mesh.triangle_count();
    let mut out = Vec::with_capacity(84 + n * 50);
    let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or("mesh");
    let header = format!("ScanImprover {name}");
    let mut head = [0u8; 80];
    let hb = header.as_bytes();
    head[..hb.len().min(80)].copy_from_slice(&hb[..hb.len().min(80)]);
    out.extend_from_slice(&head);
    out.extend_from_slice(&(n as u32).to_le_bytes());
    for t in 0..n {
        let [a, b, c] = mesh.triangle(t);
        let nrm = (b - a).cross(c - a);
        let nrm = if nrm.length_squared() > 1e-20 {
            nrm.normalize()
        } else {
            Vec3::ZERO
        };
        out.extend_from_slice(&nrm.x.to_le_bytes());
        out.extend_from_slice(&nrm.y.to_le_bytes());
        out.extend_from_slice(&nrm.z.to_le_bytes());
        for v in [a, b, c] {
            out.extend_from_slice(&v.x.to_le_bytes());
            out.extend_from_slice(&v.y.to_le_bytes());
            out.extend_from_slice(&v.z.to_le_bytes());
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}
