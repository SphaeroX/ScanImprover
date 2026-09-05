use glam::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct PlaneFit {
    pub point: Vec3,
    pub normal: Vec3,
    pub rms: f32,
    pub max_dev: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct CircleFit {
    pub center: Vec3,
    pub normal: Vec3,
    pub radius: f32,
    pub plane_rms: f32,
    pub radial_rms: f32,
    pub radial_max: f32,
}

pub fn eigen_3x3(m: [[f64; 3]; 3]) -> [(f64, [f64; 3]); 3] {
    let mut a = m;
    let mut v: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-15 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            let apq = a[p][q];
            if apq.abs() < 1e-18 {
                continue;
            }
            let app = a[p][p];
            let aqq = a[q][q];
            let theta = (aqq - app) / (2.0 * apq);
            let t = if theta >= 0.0 {
                1.0 / (theta + (theta * theta + 1.0).sqrt())
            } else {
                1.0 / (theta - (theta * theta + 1.0).sqrt())
            };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            a[p][p] = app - t * apq;
            a[q][q] = aqq + t * apq;
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            let other = 3 - p - q;
            let aop = a[other][p];
            let aoq = a[other][q];
            a[other][p] = c * aop - s * aoq;
            a[other][q] = s * aop + c * aoq;
            a[p][other] = a[other][p];
            a[q][other] = a[other][q];
            for k in 0..3 {
                let vkp = v[k][p];
                let vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut out = [
        (a[0][0], [1.0, 0.0, 0.0]),
        (a[1][1], [0.0, 1.0, 0.0]),
        (a[2][2], [0.0, 0.0, 1.0]),
    ];
    for i in 0..3 {
        let col = [v[0][i], v[1][i], v[2][i]];
        let len = (col[0] * col[0] + col[1] * col[1] + col[2] * col[2]).sqrt();
        let len = if len > 1e-15 { len } else { 1.0 };
        out[i].1 = [col[0] / len, col[1] / len, col[2] / len];
    }
    out.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
    out
}

fn centroid_f64(points: &[[f32; 3]]) -> [f64; 3] {
    let mut c = [0.0f64; 3];
    for p in points {
        for a in 0..3 {
            c[a] += p[a] as f64;
        }
    }
    let n = points.len().max(1) as f64;
    [c[0] / n, c[1] / n, c[2] / n]
}

pub fn fit_plane(points: &[[f32; 3]]) -> Option<PlaneFit> {
    if points.len() < 3 {
        return None;
    }
    let c = centroid_f64(points);
    let mut m = [[0.0f64; 3]; 3];
    for p in points {
        let d = [
            p[0] as f64 - c[0],
            p[1] as f64 - c[1],
            p[2] as f64 - c[2],
        ];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    let eig = eigen_3x3(m);
    let (lam, ev) = eig[0];
    if lam.is_nan() {
        return None;
    }
    let n = Vec3::new(ev[0] as f32, ev[1] as f32, ev[2] as f32);
    let len = n.length();
    if len < 1e-12 {
        return None;
    }
    let normal = n / len;
    let point = Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32);
    let mut ss = 0.0f64;
    let mut max_dev = 0.0f32;
    for p in points {
        let d = (Vec3::from(*p) - point).dot(normal);
        ss += (d * d) as f64;
        max_dev = max_dev.max(d.abs());
    }
    Some(PlaneFit {
        point,
        normal,
        rms: (ss / points.len() as f64) as f32,
        max_dev,
    })
}

pub fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let mut m = [a[0], a[1], a[2]];
    let mut r = b;
    for col in 0..3 {
        let mut piv = col;
        for row in col + 1..3 {
            if m[row][col].abs() > m[piv][col].abs() {
                piv = row;
            }
        }
        if m[piv][col].abs() < 1e-14 {
            return None;
        }
        m.swap(col, piv);
        r.swap(col, piv);
        for row in 0..3 {
            if row == col {
                continue;
            }
            let f = m[row][col] / m[col][col];
            for k in col..3 {
                m[row][k] -= f * m[col][k];
            }
            r[row] -= f * r[col];
        }
    }
    Some([
        r[0] / m[0][0],
        r[1] / m[1][1],
        r[2] / m[2][2],
    ])
}

pub fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let up = if normal.x.abs() < 0.9 {
        Vec3::X
    } else {
        Vec3::Y
    };
    let u = normal.cross(up).normalize_or_zero();
    if u.length_squared() < 0.5 {
        return (Vec3::X, Vec3::Y);
    }
    let v = normal.cross(u);
    (u.normalize(), v.normalize())
}

pub fn fit_circle(points: &[[f32; 3]]) -> Option<CircleFit> {
    let plane = fit_plane(points)?;
    let (u, v) = plane_basis(plane.normal);
    let mut su = 0.0f64;
    let mut sv = 0.0f64;
    let mut suu = 0.0f64;
    let mut svv = 0.0f64;
    let mut suv = 0.0f64;
    let mut sb = 0.0f64;
    let mut bu = 0.0f64;
    let mut bv = 0.0f64;
    for p in points {
        let d = Vec3::from(*p) - plane.point;
        let x = d.dot(u) as f64;
        let y = d.dot(v) as f64;
        let q = x * x + y * y;
        su += x;
        sv += y;
        suu += x * x;
        svv += y * y;
        suv += x * y;
        sb -= q;
        bu -= q * x;
        bv -= q * y;
    }
    let n = points.len() as f64;
    let mat = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let rhs = [bu, bv, sb];
    let sol = solve_3x3(mat, rhs)?;
    let (d_coef, e_coef, f_coef) = (sol[0], sol[1], sol[2]);
    let cx = -d_coef * 0.5;
    let cy = -e_coef * 0.5;
    let r2 = cx * cx + cy * cy - f_coef;
    if r2 <= 1e-12 {
        return None;
    }
    let radius = r2.sqrt() as f32;
    let center = plane.point + u * cx as f32 + v * cy as f32;
    let mut ss = 0.0f64;
    let mut rmax = 0.0f32;
    for p in points {
        let d = Vec3::from(*p) - center;
        let radial = (d - plane.normal * d.dot(plane.normal)).length() - radius;
        ss += (radial * radial) as f64;
        rmax = rmax.max(radial.abs());
    }
    Some(CircleFit {
        center,
        normal: plane.normal,
        radius,
        plane_rms: plane.rms,
        radial_rms: (ss / n) as f32,
        radial_max: rmax,
    })
}
