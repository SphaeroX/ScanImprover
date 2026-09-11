//! B-Rep topology read off a closed triangle mesh and its face groups.
//!
//! Faces are the connected triangle regions of one group, edges the chains
//! of mesh edges between two faces, vertices the mesh vertices where more
//! than two faces meet. Face loops follow the mesh orientation, so every
//! loop has its face on the left when seen from outside, and every edge is
//! used exactly twice with opposite senses.

use crate::mesh::Mesh;
use std::collections::{HashMap, HashSet};

pub(super) struct TopoFace {
    pub group: i32,
    pub tris: Vec<u32>,
    /// Loops as (edge index, same sense as the edge) uses.
    pub loops: Vec<Vec<(usize, bool)>>,
}

pub(super) struct TopoEdge {
    /// Mesh vertices from the start to the end vertex (first == last for a
    /// closed edge). `faces[0]` lies to the left of this direction.
    pub chain: Vec<u32>,
    pub faces: [usize; 2],
    pub start: usize,
    pub end: usize,
}

pub(super) struct TopoVertex {
    pub mesh_vertex: u32,
    pub faces: Vec<usize>,
}

pub(super) struct MeshTopo {
    pub faces: Vec<TopoFace>,
    pub edges: Vec<TopoEdge>,
    pub vertices: Vec<TopoVertex>,
    /// Triangles, oriented with outward normals.
    pub tris: Vec<[u32; 3]>,
    /// Euler characteristic V - E + F of the mesh.
    pub mesh_euler: i64,
    /// Ungrouped triangles given to a neighbouring group.
    pub absorbed: usize,
    /// The mesh was inside-out (negative volume) and has been flipped.
    pub flipped: bool,
}

/// Builds the topology. Refuses meshes that are not a single closed,
/// consistently oriented 2-manifold shell, and faces that are not
/// topological disks with holes.
pub(super) fn build(mesh: &Mesh, group_ids: &[i32]) -> Result<MeshTopo, String> {
    let nt = mesh.triangle_count();
    if nt == 0 {
        return Err("The mesh is empty.".to_string());
    }
    if group_ids.len() != nt {
        return Err("The face groups belong to a different mesh; re-detect them.".to_string());
    }
    let mut tris: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect();
    let degenerate = tris
        .iter()
        .filter(|t| t[0] == t[1] || t[1] == t[2] || t[0] == t[2])
        .count();
    if degenerate > 0 {
        return Err(format!(
            "{degenerate} degenerate triangles (repeated vertices). Run \"Remove degenerate faces\" \
             in Mesh repair first."
        ));
    }

    // Orientation: a closed mesh must enclose a positive volume.
    let vol: f64 = tris
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| glam::Vec3::from(mesh.positions[i as usize]).as_dvec3());
            a.dot(b.cross(c))
        })
        .sum();
    let flipped = vol < 0.0;
    if flipped {
        for t in &mut tris {
            t.swap(1, 2);
        }
    }

    // Directed half-edge -> triangle.
    let mut half: HashMap<(u32, u32), u32> = HashMap::with_capacity(nt * 3);
    let mut duplicated = 0usize;
    for (t, tri) in tris.iter().enumerate() {
        for k in 0..3 {
            if half.insert((tri[k], tri[(k + 1) % 3]), t as u32).is_some() {
                duplicated += 1;
            }
        }
    }
    let open = half
        .keys()
        .filter(|&&(a, b)| !half.contains_key(&(b, a)))
        .count();
    if open > 0 {
        return Err(format!(
            "The mesh is not closed: {open} open edges. Fill the holes (Mesh repair → Fill \
             holes), then re-detect the face groups."
        ));
    }
    if duplicated > 0 {
        return Err(format!(
            "{duplicated} edges are non-manifold or inconsistently oriented. Run \"Auto repair\" \
             or \"Unify normals\" in Mesh repair first."
        ));
    }
    let twin_tri = |a: u32, b: u32| half[&(b, a)] as usize;

    // Single connected shell.
    {
        let mut seen = vec![false; nt];
        let mut shells = 0usize;
        let mut stack = Vec::new();
        for s in 0..nt {
            if seen[s] {
                continue;
            }
            shells += 1;
            seen[s] = true;
            stack.push(s);
            while let Some(t) = stack.pop() {
                let tri = tris[t];
                for k in 0..3 {
                    let n = twin_tri(tri[k], tri[(k + 1) % 3]);
                    if !seen[n] {
                        seen[n] = true;
                        stack.push(n);
                    }
                }
            }
        }
        if shells > 1 {
            return Err(format!(
                "The mesh consists of {shells} separate shells; only a single closed shell can \
                 become a solid. Remove the debris (Mesh repair) or hide the extra parts."
            ));
        }
    }

    // Ungrouped triangles join the neighbouring group they share most edges with.
    let mut gid = group_ids.to_vec();
    if gid.iter().all(|&g| g < 0) {
        return Err("No face groups: detect the face groups first.".to_string());
    }
    let mut absorbed = 0usize;
    loop {
        let mut changes = Vec::new();
        for t in 0..nt {
            if gid[t] >= 0 {
                continue;
            }
            let tri = tris[t];
            let mut best: Option<(i32, usize)> = None;
            let nbs: Vec<i32> = (0..3)
                .map(|k| gid[twin_tri(tri[k], tri[(k + 1) % 3])])
                .collect();
            for &g in nbs.iter().filter(|&&g| g >= 0) {
                let c = nbs.iter().filter(|&&x| x == g).count();
                if best.is_none_or(|(bg, bc)| c > bc || (c == bc && g < bg)) {
                    best = Some((g, c));
                }
            }
            if let Some((g, _)) = best {
                changes.push((t, g));
            }
        }
        if changes.is_empty() {
            break;
        }
        absorbed += changes.len();
        for (t, g) in changes {
            gid[t] = g;
        }
    }

    // Faces: connected components of equal group id.
    let mut face_of = vec![usize::MAX; nt];
    let mut faces: Vec<TopoFace> = Vec::new();
    let mut order: Vec<usize> = (0..nt).collect();
    order.sort_by_key(|&t| (gid[t], t));
    for &s in &order {
        if face_of[s] != usize::MAX {
            continue;
        }
        let f = faces.len();
        let mut fl = Vec::new();
        let mut stack = vec![s];
        face_of[s] = f;
        while let Some(t) = stack.pop() {
            fl.push(t as u32);
            let tri = tris[t];
            for k in 0..3 {
                let n = twin_tri(tri[k], tri[(k + 1) % 3]);
                if face_of[n] == usize::MAX && gid[n] == gid[s] {
                    face_of[n] = f;
                    stack.push(n);
                }
            }
        }
        faces.push(TopoFace {
            group: gid[s],
            tris: fl,
            loops: Vec::new(),
        });
    }
    if faces.len() < 2 {
        return Err(
            "The whole mesh is a single face group; a solid needs at least two \
                    faces. Lower the crease angle and re-detect."
                .to_string(),
        );
    }
    let face_left = |a: u32, b: u32| face_of[half[&(a, b)] as usize];
    let is_boundary = |a: u32, b: u32| face_left(a, b) != face_left(b, a);

    // Boundary graph.
    let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in half.keys() {
        if a < b && is_boundary(a, b) {
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }
    }
    let pair = |a: u32, b: u32| {
        let (x, y) = (face_left(a, b), face_left(b, a));
        (x.min(y), x.max(y))
    };
    // Mesh vertices that become B-Rep vertices: more than two faces meet.
    let mut vkeys: Vec<u32> = adj
        .iter()
        .filter(|(v, nb)| nb.len() != 2 || pair(**v, nb[0]) != pair(**v, nb[1]))
        .map(|(v, _)| *v)
        .collect();
    vkeys.sort_unstable();
    let mut vertices: Vec<TopoVertex> = Vec::new();
    let mut vertex_of: HashMap<u32, usize> = HashMap::new();
    for &v in &vkeys {
        vertex_of.insert(v, vertices.len());
        vertices.push(TopoVertex {
            mesh_vertex: v,
            faces: Vec::new(),
        });
    }

    // Edge chains between B-Rep vertices, then closed chains.
    let ekey = |a: u32, b: u32| (a.min(b), a.max(b));
    let mut used: HashSet<(u32, u32)> = HashSet::new();
    let mut edges: Vec<TopoEdge> = Vec::new();
    let walk = |start: u32, first: u32, used: &mut HashSet<(u32, u32)>| -> Vec<u32> {
        let mut chain = vec![start, first];
        used.insert(ekey(start, first));
        loop {
            let cur = *chain.last().unwrap();
            if vertex_of.contains_key(&cur) || cur == start {
                break;
            }
            let prev = chain[chain.len() - 2];
            let Some(&next) = adj[&cur]
                .iter()
                .find(|&&n| n != prev && !used.contains(&ekey(cur, n)))
                .or_else(|| {
                    adj[&cur]
                        .iter()
                        .find(|&&n| n == start && !used.contains(&ekey(cur, n)))
                })
            else {
                break;
            };
            used.insert(ekey(cur, next));
            chain.push(next);
        }
        chain
    };
    let mut starts: Vec<u32> = vkeys.clone();
    let mut rest: Vec<u32> = adj
        .keys()
        .copied()
        .filter(|v| !vertex_of.contains_key(v))
        .collect();
    rest.sort_unstable();
    starts.extend(rest);
    for &s in &starts {
        let mut nbs = adj[&s].clone();
        nbs.sort_unstable();
        for n in nbs {
            if used.contains(&ekey(s, n)) {
                continue;
            }
            let chain = walk(s, n, &mut used);
            let (first, last) = (chain[0], *chain.last().unwrap());
            let closed = first == last;
            if !closed && (!vertex_of.contains_key(&first) || !vertex_of.contains_key(&last)) {
                return Err(
                    "Internal error: an edge chain ends at a mesh vertex that is not \
                            a corner. The mesh may have non-manifold vertices; run Auto repair."
                        .to_string(),
                );
            }
            let start = match vertex_of.get(&first) {
                Some(&v) => v,
                None => {
                    // Closed edge without corners: add a seam vertex.
                    vertices.push(TopoVertex {
                        mesh_vertex: first,
                        faces: Vec::new(),
                    });
                    vertices.len() - 1
                }
            };
            let end = if closed { start } else { vertex_of[&last] };
            edges.push(TopoEdge {
                faces: [face_left(chain[0], chain[1]), face_left(chain[1], chain[0])],
                chain,
                start,
                end,
            });
        }
    }
    // Directed mesh edge -> (edge, same sense).
    let mut edge_of: HashMap<(u32, u32), (usize, bool)> = HashMap::new();
    for (e, edge) in edges.iter().enumerate() {
        for w in edge.chain.windows(2) {
            edge_of.insert((w[0], w[1]), (e, true));
            edge_of.insert((w[1], w[0]), (e, false));
        }
    }

    // Face loops: follow the boundary half-edges with the face on the left,
    // turning around each vertex through the face's own triangles.
    let next_half = |a: u32, b: u32| -> Option<(u32, u32)> {
        let mut t = half[&(a, b)] as usize;
        for _ in 0..4096 {
            let tri = tris[t];
            let k = tri.iter().position(|&x| x == b)?;
            let c = tri[(k + 1) % 3];
            if is_boundary(b, c) {
                return Some((b, c));
            }
            t = half[&(c, b)] as usize;
        }
        None
    };
    let mut boundary_halves: Vec<(u32, u32)> = edge_of.keys().copied().collect();
    boundary_halves.sort_unstable();
    let mut visited: HashSet<(u32, u32)> = HashSet::new();
    for &h0 in &boundary_halves {
        if visited.contains(&h0) {
            continue;
        }
        let f = face_left(h0.0, h0.1);
        let mut lp = vec![h0];
        visited.insert(h0);
        let mut h = h0;
        loop {
            let Some(n) = next_half(h.0, h.1) else {
                return Err(format!(
                    "Could not trace the boundary of group {} (non-manifold vertex). Run Auto \
                     repair first.",
                    faces[f].group + 1
                ));
            };
            if n == h0 {
                break;
            }
            if !visited.insert(n) {
                return Err(format!(
                    "The boundary of group {} touches itself at a vertex; split the group or \
                     re-detect with other settings.",
                    faces[f].group + 1
                ));
            }
            lp.push(n);
            h = n;
        }
        // Start at the beginning of an edge use, then merge runs.
        let use_start = |&(a, b): &(u32, u32)| {
            let (e, fwd) = edge_of[&(a, b)];
            let ch = &edges[e].chain;
            a == if fwd { ch[0] } else { *ch.last().unwrap() }
        };
        let Some(i0) = lp.iter().position(use_start) else {
            return Err("Internal error: a face loop has no edge start.".to_string());
        };
        lp.rotate_left(i0);
        let mut uses: Vec<(usize, bool, usize)> = Vec::new();
        for hh in &lp {
            let (e, fwd) = edge_of[hh];
            match uses.last_mut() {
                Some((le, lf, n))
                    if *le == e
                        && *lf == fwd
                        && *n < edges[e].chain.len() - 1
                        && !use_start(hh) =>
                {
                    *n += 1
                }
                _ => uses.push((e, fwd, 1)),
            }
        }
        for &(e, fwd, n) in &uses {
            let expect_face = if fwd {
                edges[e].faces[0]
            } else {
                edges[e].faces[1]
            };
            if n != edges[e].chain.len() - 1 || expect_face != f {
                return Err("Internal error: inconsistent face loop.".to_string());
            }
        }
        faces[f]
            .loops
            .push(uses.into_iter().map(|(e, fwd, _)| (e, fwd)).collect());
    }

    // Each face must be a disk with holes: V - E + F = 2 - loops.
    let mut bad_faces = Vec::new();
    for face in &faces {
        let mut vs: HashSet<u32> = HashSet::new();
        let mut es: HashSet<(u32, u32)> = HashSet::new();
        for &t in &face.tris {
            let tri = tris[t as usize];
            for k in 0..3 {
                vs.insert(tri[k]);
                es.insert(ekey(tri[k], tri[(k + 1) % 3]));
            }
        }
        let chi = vs.len() as i64 - es.len() as i64 + face.tris.len() as i64;
        if chi != 2 - face.loops.len() as i64 {
            bad_faces.push(face.group + 1);
        }
    }
    if !bad_faces.is_empty() {
        bad_faces.sort_unstable();
        bad_faces.dedup();
        return Err(format!(
            "Face group(s) {} wrap around a handle of the part (not a disk with holes) and \
             cannot become a single face. Lower the crease angle or feature size and re-detect.",
            join_ids(&bad_faces)
        ));
    }

    for edge in &edges {
        for v in [edge.start, edge.end] {
            for f in edge.faces {
                if !vertices[v].faces.contains(&f) {
                    vertices[v].faces.push(f);
                }
            }
        }
    }

    let used_verts: HashSet<u32> = tris.iter().flatten().copied().collect();
    let mesh_euler = used_verts.len() as i64 - (half.len() / 2) as i64 + nt as i64;
    Ok(MeshTopo {
        faces,
        edges,
        vertices,
        tris,
        mesh_euler,
        absorbed,
        flipped,
    })
}

/// "3, 5 and 7" style list of group numbers.
pub(super) fn join_ids(ids: &[i32]) -> String {
    let s: Vec<String> = ids.iter().map(|i| i.to_string()).collect();
    match s.len() {
        0 => String::new(),
        1 => s[0].clone(),
        n => format!("{} and {}", s[..n - 1].join(", "), s[n - 1]),
    }
}
