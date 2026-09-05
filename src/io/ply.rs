use crate::mesh::Mesh;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Type {
    Char,
    UChar,
    Short,
    UShort,
    Int,
    UInt,
    Float,
    Double,
}

impl Type {
    fn parse(s: &str) -> Option<Type> {
        Some(match s {
            "char" | "int8" => Type::Char,
            "uchar" | "uint8" => Type::UChar,
            "short" | "int16" => Type::Short,
            "ushort" | "uint16" => Type::UShort,
            "int" | "int32" => Type::Int,
            "uint" | "uint32" => Type::UInt,
            "float" | "float32" => Type::Float,
            "double" | "float64" => Type::Double,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Type::Char | Type::UChar => 1,
            Type::Short | Type::UShort => 2,
            Type::Int | Type::UInt | Type::Float => 4,
            Type::Double => 8,
        }
    }
}

#[derive(Clone, Debug)]
struct Prop {
    ty: Type,
    name: String,
    list_count: Option<Type>,
}

#[derive(Clone, Debug)]
struct Elem {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

enum Format {
    Ascii,
    BinaryLe,
}

struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn read(&mut self, n: usize) -> Result<&'a [u8], String> {
        if self.pos + n > self.data.len() {
            return Err("PLY file is truncated".to_string());
        }
        let s = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }

    fn read_int(&mut self, ty: Type) -> Result<i64, String> {
        let b = self.read(ty.size())?;
        Ok(match ty {
            Type::Char => b[0] as i8 as i64,
            Type::UChar => b[0] as i64,
            Type::Short => i16::from_le_bytes(b.try_into().unwrap()) as i64,
            Type::UShort => u16::from_le_bytes(b.try_into().unwrap()) as i64,
            Type::Int => i32::from_le_bytes(b.try_into().unwrap()) as i64,
            Type::UInt => u32::from_le_bytes(b.try_into().unwrap()) as i64,
            Type::Float => f32::from_le_bytes(b.try_into().unwrap()) as i64,
            Type::Double => f64::from_le_bytes(b.try_into().unwrap()) as i64,
        })
    }

    fn read_float(&mut self, ty: Type) -> Result<f32, String> {
        let b = self.read(ty.size())?;
        Ok(match ty {
            Type::Char => b[0] as i8 as f32,
            Type::UChar => b[0] as f32,
            Type::Short => i16::from_le_bytes(b.try_into().unwrap()) as f32,
            Type::UShort => u16::from_le_bytes(b.try_into().unwrap()) as f32,
            Type::Int => i32::from_le_bytes(b.try_into().unwrap()) as f32,
            Type::UInt => u32::from_le_bytes(b.try_into().unwrap()) as f32,
            Type::Float => f32::from_le_bytes(b.try_into().unwrap()),
            Type::Double => f64::from_le_bytes(b.try_into().unwrap()) as f32,
        })
    }
}

pub fn load(bytes: &[u8]) -> Result<Mesh, String> {
    let text_end = find_header_end(bytes)?;
    let header_text = std::str::from_utf8(&bytes[..text_end])
        .map_err(|_| "PLY header is not valid UTF-8".to_string())?;
    let (format, elems) = parse_header(header_text)?;
    let body = &bytes[text_end..];
    match format {
        Format::Ascii => load_ascii(body, &elems),
        Format::BinaryLe => load_binary(body, &elems),
    }
}

fn find_header_end(bytes: &[u8]) -> Result<usize, String> {
    let key = b"end_header";
    if bytes.len() < key.len() {
        return Err("Not a PLY file".to_string());
    }
    let mut i = 0usize;
    while i + key.len() <= bytes.len() {
        if &bytes[i..i + key.len()] == key {
            let mut j = i + key.len();
            while j < bytes.len() && bytes[j] != b'\n' {
                j += 1;
            }
            if j < bytes.len() {
                j += 1;
            }
            return Ok(j);
        }
        i += 1;
    }
    Err("PLY header has no end_header".to_string())
}

fn parse_header(text: &str) -> Result<(Format, Vec<Elem>), String> {
    let mut lines = text.lines().map(str::trim);
    if lines.next() != Some("ply") {
        return Err("Not a PLY file (missing magic)".to_string());
    }
    let mut format = None;
    let mut elems: Vec<Elem> = Vec::new();
    for line in lines {
        if line.is_empty() || line.starts_with('#') || line.starts_with("comment") {
            continue;
        }
        let mut toks = line.split_ascii_whitespace();
        match toks.next() {
            Some("format") => {
                format = Some(match toks.next() {
                    Some("ascii") => Format::Ascii,
                    Some("binary_little_endian") => Format::BinaryLe,
                    Some("binary_big_endian") => {
                        return Err("Big-endian PLY is not supported".to_string())
                    }
                    _ => return Err("Invalid PLY format line".to_string()),
                });
            }
            Some("element") => {
                let name = toks
                    .next()
                    .ok_or("PLY element without name")?
                    .to_string();
                let count: usize = toks
                    .next()
                    .and_then(|c| c.parse().ok())
                    .ok_or("PLY element without count")?;
                elems.push(Elem {
                    name,
                    count,
                    props: Vec::new(),
                });
            }
            Some("property") => {
                let elem = elems
                    .last_mut()
                    .ok_or("PLY property outside element")?;
                let rest: Vec<&str> = toks.collect();
                if rest.is_empty() {
                    return Err("PLY property without type".to_string());
                }
                if rest[0] == "list" {
                    if rest.len() < 3 {
                        return Err("PLY list property is malformed".to_string());
                    }
                    let list_count = Type::parse(rest[1])
                        .ok_or_else(|| format!("Unknown PLY type '{}'", rest[1]))?;
                    let ty = Type::parse(rest[2])
                        .ok_or_else(|| format!("Unknown PLY type '{}'", rest[2]))?;
                    elem.props.push(Prop {
                        ty,
                        name: rest[3..].join("_"),
                        list_count: Some(list_count),
                    });
                } else {
                    let ty = Type::parse(rest[0])
                        .ok_or_else(|| format!("Unknown PLY type '{}'", rest[0]))?;
                    let name = rest
                        .get(1)
                        .ok_or("PLY property without name")?
                        .to_string();
                    elem.props.push(Prop {
                        ty,
                        name,
                        list_count: None,
                    });
                }
            }
            _ => {}
        }
    }
    let format = format.ok_or("PLY header has no format line")?;
    Ok((format, elems))
}

fn prop_indices(props: &[Prop], names: &[&str], elem: &str) -> Result<Vec<usize>, String> {
    names
        .iter()
        .map(|n| {
            props
                .iter()
                .position(|p| &p.name == n)
                .ok_or_else(|| format!("PLY {elem} element lacks property '{n}'"))
        })
        .collect()
}

fn push_face(face: &[i64], indices: &mut Vec<u32>, nverts: usize) -> Result<(), String> {
    if face.len() < 3 {
        return Ok(());
    }
    let mut resolved = Vec::with_capacity(face.len());
    for &i in face {
        let i = if i < 0 { nverts as i64 + i } else { i };
        if i < 0 || i as usize >= nverts {
            return Err("PLY face index out of range".to_string());
        }
        resolved.push(i as u32);
    }
    for k in 1..resolved.len() - 1 {
        indices.push(resolved[0]);
        indices.push(resolved[k]);
        indices.push(resolved[k + 1]);
    }
    Ok(())
}

fn load_binary(body: &[u8], elems: &[Elem]) -> Result<Mesh, String> {
    let mut cur = Cursor { data: body, pos: 0 };
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    for elem in elems {
        if elem.name == "vertex" {
            let idx = prop_indices(&elem.props, &["x", "y", "z"], "vertex")?;
            if elem.props.iter().any(|p| p.list_count.is_some()) {
                return Err("List properties on PLY vertices are not supported".to_string());
            }
            positions.reserve(elem.count);
            for _ in 0..elem.count {
                let mut p = [0.0f32; 3];
                for (pi, prop) in elem.props.iter().enumerate() {
                    let v = cur.read_float(prop.ty)?;
                    if let Some(slot) = idx.iter().position(|&k| k == pi) {
                        p[slot] = v;
                    }
                }
                positions.push(p);
            }
        } else if elem.name == "face" {
            let list_prop = elem
                .props
                .iter()
                .find(|p| p.list_count.is_some())
                .ok_or("PLY face element has no index list")?;
            if elem.props.iter().any(|p| p.list_count.is_none()) {
                return Err("Unsupported extra properties on PLY faces".to_string());
            }
            for _ in 0..elem.count {
                let len = cur.read_int(list_prop.list_count.unwrap())? as usize;
                if len > 64 {
                    return Err("Unreasonable PLY face length".to_string());
                }
                let mut face: Vec<i64> = Vec::with_capacity(len);
                for _ in 0..len {
                    face.push(cur.read_int(list_prop.ty)?);
                }
                push_face(&face, &mut indices, positions.len())?;
            }
        } else {
            for _ in 0..elem.count {
                for prop in &elem.props {
                    if prop.list_count.is_some() {
                        let len = cur.read_int(prop.list_count.unwrap())? as usize;
                        cur.read(len.saturating_mul(prop.ty.size()))?;
                    } else {
                        cur.read(prop.ty.size())?;
                    }
                }
            }
        }
    }
    if positions.is_empty() || indices.is_empty() {
        return Err("PLY contains no mesh data".to_string());
    }
    Ok(Mesh::from_indexed(positions, indices))
}

fn load_ascii(body: &[u8], elems: &[Elem]) -> Result<Mesh, String> {
    let text = std::str::from_utf8(body).map_err(|_| "PLY body is not UTF-8".to_string())?;
    let mut toks = text.split_ascii_whitespace();
    let mut next_f = |need: usize| -> Result<Vec<f64>, String> {
        let mut v = Vec::with_capacity(need);
        for _ in 0..need {
            match toks.next() {
                Some(t) => v.push(
                    t.parse::<f64>()
                        .map_err(|_| format!("Invalid numeric token '{t}' in PLY"))?,
                ),
                None => return Err("PLY file is truncated".to_string()),
            }
        }
        Ok(v)
    };
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    for elem in elems {
        for _ in 0..elem.count {
            if elem.name == "vertex" {
                let idx = prop_indices(&elem.props, &["x", "y", "z"], "vertex")?;
                let mut p = [0.0f32; 3];
                for (pi, _prop) in elem.props.iter().enumerate() {
                    let vals = next_f(1)?;
                    if let Some(slot) = idx.iter().position(|&k| k == pi) {
                        p[slot] = vals[0] as f32;
                    }
                }
                positions.push(p);
            } else if elem.name == "face" {
                if !elem.props.iter().any(|p| p.list_count.is_some()) {
                    return Err("PLY face element has no index list".to_string());
                }
                let mut face: Vec<i64> = Vec::new();
                for prop in &elem.props {
                    if prop.list_count.is_some() {
                        let len = next_f(1)?[0] as usize;
                        for v in next_f(len)? {
                            face.push(v as i64);
                        }
                    } else {
                        next_f(1)?;
                    }
                }
                push_face(&face, &mut indices, positions.len())?;
            } else {
                for prop in &elem.props {
                    if prop.list_count.is_some() {
                        let len = next_f(1)?[0] as usize;
                        next_f(len)?;
                    } else {
                        next_f(1)?;
                    }
                }
            }
        }
    }
    if positions.is_empty() || indices.is_empty() {
        return Err("PLY contains no mesh data".to_string());
    }
    Ok(Mesh::from_indexed(positions, indices))
}

pub fn save(mesh: &Mesh) -> Vec<u8> {
    let nv = mesh.positions.len();
    let nt = mesh.triangle_count();
    let mut out = Vec::with_capacity(1024 + nv * 24 + nt * 13);
    let header = format!(
        "ply\nformat binary_little_endian 1.0\nelement vertex {nv}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face {nt}\nproperty list uchar int vertex_indices\nend_header\n"
    );
    out.extend_from_slice(header.as_bytes());
    for (p, n) in mesh.positions.iter().zip(mesh.normals.iter()) {
        for v in p.iter().chain(n.iter()) {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    for t in 0..nt {
        out.push(3);
        for k in 0..3 {
            out.extend_from_slice(&mesh.indices[3 * t + k].to_le_bytes());
        }
    }
    out
}
