//! STL reading (binary and ASCII) and binary writing.

use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;

const HEADER_LEN: usize = 84;
const FACET_LEN: usize = 50;

pub fn load(bytes: &[u8]) -> Result<Mesh, String> {
    if bytes.len() >= HEADER_LEN {
        let n = u32::from_le_bytes(
            bytes[80..84]
                .try_into()
                .map_err(|_| "Invalid STL header".to_string())?,
        ) as usize;
        if n.checked_mul(FACET_LEN)
            .and_then(|body| body.checked_add(HEADER_LEN))
            == Some(bytes.len())
        {
            return load_binary(bytes, n);
        }
    }
    load_ascii(bytes)
}

#[inline]
fn read_f32(bytes: &[u8], at: usize) -> f32 {
    f32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

fn load_binary(bytes: &[u8], n: usize) -> Result<Mesh, String> {
    let body = &bytes[HEADER_LEN..];
    if body.len() < n * FACET_LEN {
        return Err("STL file is truncated".to_string());
    }
    // Facets are independent: decode them in parallel (the 12-byte normal is
    // skipped, normals are recomputed from the welded geometry).
    let raw: Vec<[f32; 3]> = body
        .par_chunks_exact(FACET_LEN)
        .take(n)
        .flat_map_iter(|facet| {
            (0..3).map(move |k| {
                let at = 12 + 12 * k;
                [
                    read_f32(facet, at),
                    read_f32(facet, at + 4),
                    read_f32(facet, at + 8),
                ]
            })
        })
        .collect();
    if raw.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
        return Err("STL file contains non-finite vertex coordinates".to_string());
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
    if raw.is_empty() || !raw.len().is_multiple_of(3) {
        return Err("STL file contains no complete triangles".to_string());
    }
    Ok(Mesh::from_corners(&raw))
}

pub fn save(mesh: &Mesh, path: &std::path::Path) -> Vec<u8> {
    let n = mesh.triangle_count();
    let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or("mesh");
    let header = format!("ScanImprover {name}");
    let mut head = [0u8; 80];
    let hb = header.as_bytes();
    let hl = hb.len().min(80);
    head[..hl].copy_from_slice(&hb[..hl]);

    // Serialise facets in parallel into fixed 50-byte records.
    let facets: Vec<[u8; FACET_LEN]> = (0..n)
        .into_par_iter()
        .map(|t| {
            let [a, b, c] = mesh.triangle(t);
            let nrm = (b - a).cross(c - a);
            let nrm = if nrm.length_squared() > 1e-20 {
                nrm.normalize()
            } else {
                Vec3::ZERO
            };
            let mut rec = [0u8; FACET_LEN];
            let mut off = 0;
            for v in [nrm, a, b, c] {
                for comp in [v.x, v.y, v.z] {
                    rec[off..off + 4].copy_from_slice(&comp.to_le_bytes());
                    off += 4;
                }
            }
            rec
        })
        .collect();

    let mut out = Vec::with_capacity(HEADER_LEN + n * FACET_LEN);
    out.extend_from_slice(&head);
    out.extend_from_slice(&(n as u32).to_le_bytes());
    for rec in &facets {
        out.extend_from_slice(rec);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_roundtrip_with_parallel_parse() {
        // Enough triangles to exercise the parallel weld path.
        let n = 2000usize;
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for i in 0..n {
            let x = i as f32;
            let base = positions.len() as u32;
            positions.push([x, 0.0, 0.0]);
            positions.push([x + 1.0, 0.0, 0.0]);
            positions.push([x, 1.0, 0.0]);
            indices.extend_from_slice(&[base, base + 1, base + 2]);
        }
        let m = Mesh::from_indexed(positions, indices);
        let bytes = save(&m, std::path::Path::new("strip"));
        assert_eq!(bytes.len(), HEADER_LEN + n * FACET_LEN);
        let back = load(&bytes).unwrap();
        assert_eq!(back.triangle_count(), n);
        // Shared corners get welded: each x appears in two triangles.
        assert!(back.vertex_count() < m.vertex_count());
    }

    #[test]
    fn errors_for_garbage_and_incomplete_ascii() {
        assert!(load(b"solid x\nendsolid x\n").is_err());
        assert!(load(b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid x\n").is_err());
        assert!(load(b"solid x\nfacet\nvertex 0 0 zz\n").is_err());
        assert!(load(&[0u8; 10]).is_err());
    }
}
