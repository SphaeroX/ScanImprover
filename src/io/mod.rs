pub mod obj;
pub mod ply;
pub mod stl;

use crate::mesh::Mesh;
use std::path::Path;

pub fn load_any(path: &Path, bytes: &[u8]) -> Result<Mesh, String> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_ascii_lowercase())
        .unwrap_or_default();
    match ext.as_str() {
        "stl" => stl::load(bytes),
        "ply" => ply::load(bytes),
        "obj" => obj::load(bytes),
        other => Err(format!("Unsupported file type: .{other}")),
    }
}

pub fn save_any(path: &Path, mesh: &Mesh) -> Result<(), String> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_ascii_lowercase())
        .unwrap_or_default();
    let bytes = match ext.as_str() {
        "stl" => stl::save(mesh, path),
        "ply" => ply::save(mesh),
        "obj" => obj::save(mesh).into_bytes(),
        other => return Err(format!("Unsupported export type: .{other}")),
    };
    std::fs::write(path, bytes).map_err(|e| format!("Failed to write {}: {e}", path.display()))
}
