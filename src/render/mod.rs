use crate::mesh::Mesh;
use glam::{Mat4, Vec3};
use std::sync::{Arc, Mutex};
use wgpu::util::DeviceExt;

const MESH_WGSL: &str = r#"
struct Uniforms {
    viewproj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    params: vec4<f32>,
    light1: vec4<f32>,
    light2: vec4<f32>,
};
@group(0) @binding(0) var<uniform> U: Uniforms;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) aux: vec4<f32>,
};
struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) wp: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) aux: vec4<f32>,
};
@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    out.pos = U.viewproj * vec4<f32>(in.pos, 1.0);
    out.wp = in.pos;
    out.nrm = in.nrm;
    out.aux = in.aux;
    return out;
}

fn heat_color(t: f32) -> vec3<f32> {
    if (t < 0.25) {
        return mix(vec3<f32>(0.10, 0.20, 0.85), vec3<f32>(0.00, 0.85, 0.90), t / 0.25);
    }
    if (t < 0.5) {
        return mix(vec3<f32>(0.00, 0.85, 0.90), vec3<f32>(0.10, 0.90, 0.15), (t - 0.25) / 0.25);
    }
    if (t < 0.75) {
        return mix(vec3<f32>(0.10, 0.90, 0.15), vec3<f32>(1.00, 0.80, 0.00), (t - 0.50) / 0.25);
    }
    return mix(vec3<f32>(1.00, 0.80, 0.00), vec3<f32>(0.95, 0.10, 0.10), (t - 0.75) / 0.25);
}

// Distinct hue per face group id.
// Must stay in sync with `group_hue_color` in src/geom/segment.rs.
fn group_color(id: f32) -> vec3<f32> {
    let h = fract(id * 0.61803398875 + 0.04);
    let s = 0.62;
    let v = 1.0;
    let c = v * s;
    let hp = h * 6.0;
    let x = c * (1.0 - abs(fract(hp * 0.5) * 2.0 - 1.0));
    let m = v - c;
    let i = i32(hp) % 6;
    var rgb = vec3<f32>(c + m, x + m, m);
    if (i == 1) {
        rgb = vec3<f32>(x + m, c + m, m);
    } else if (i == 2) {
        rgb = vec3<f32>(m, c + m, x + m);
    } else if (i == 3) {
        rgb = vec3<f32>(m, x + m, c + m);
    } else if (i == 4) {
        rgb = vec3<f32>(x + m, m, c + m);
    } else if (i == 5) {
        rgb = vec3<f32>(c + m, m, x + m);
    }
    return rgb;
}

// Semantic type colors: code 3 = plane, 4 = cylinder, 5 = sphere, 6 = freeform.
// Must stay in sync with `KIND_COLORS` in src/geom/segment.rs.
fn type_color(code: f32) -> vec3<f32> {
    if (code < 3.5) {
        return vec3<f32>(0.30, 0.55, 0.98);
    }
    if (code < 4.5) {
        return vec3<f32>(0.30, 0.82, 0.42);
    }
    if (code < 5.5) {
        return vec3<f32>(0.98, 0.66, 0.28);
    }
    return vec3<f32>(0.78, 0.45, 0.62);
}

@fragment
fn fs_main(in: VsOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    let v = normalize(U.cam_pos.xyz - in.wp);
    var n = normalize(in.nrm);
    if (!is_front) {
        n = -n;
    }
    n = select(-n, n, dot(n, v) >= 0.0);
    let l1 = normalize(U.light1.xyz);
    let l2 = normalize(U.light2.xyz);
    let diff = 0.15 + 0.60 * max(dot(n, l1), 0.0) + 0.35 * max(dot(n, l2), 0.0);
    let h1 = normalize(l1 + v);
    var spec = pow(max(dot(n, h1), 0.0), 24.0) * 0.25;

    var col = vec3<f32>(0.72, 0.74, 0.78);
    // Backface / inside coloring: distinct warm terracotta tone with subdued specular
    if (!is_front) {
        col = vec3<f32>(0.76, 0.40, 0.28);
        spec = spec * 0.15;
    }

    // Face group coloring: aux.z >= 0 = distinct hue id,
    // <= -3.0 = type code, -2.0 = group boundary seam, -1.0 = ungrouped.
    if (U.params.z > 0.5 && in.aux.z >= 0.0) {
        let gcol = group_color(in.aux.z);
        if (is_front) {
            col = gcol;
        } else {
            col = mix(gcol * 0.65, vec3<f32>(0.76, 0.40, 0.28), 0.45);
        }
    } else if (U.params.z > 0.5 && in.aux.z <= -3.0) {
        let tcol = type_color(-in.aux.z);
        if (is_front) {
            col = tcol;
        } else {
            col = mix(tcol * 0.65, vec3<f32>(0.76, 0.40, 0.28), 0.45);
        }
    } else if (U.params.z > 0.5 && in.aux.z <= -2.5) {
        col = vec3<f32>(0.16, 0.17, 0.21);
    }
    if (U.params.x > 0.5 && in.aux.y >= 0.0) {
        let hcol = heat_color(clamp(in.aux.y * U.params.y, 0.0, 1.0));
        if (is_front) {
            col = hcol;
        } else {
            col = mix(hcol * 0.70, vec3<f32>(0.76, 0.40, 0.28), 0.35);
        }
    }
    if (U.params.z > 0.5 && U.params.w >= 0.0 && abs(in.aux.w - U.params.w) < 0.5) {
        col = mix(col, vec3<f32>(1.0, 1.0, 1.0), 0.45);
    }
    if (in.aux.x > 0.75) {
        col = mix(col, vec3<f32>(1.0, 0.45, 0.10), 0.55);
    } else if (in.aux.x > 0.25) {
        col = mix(col, vec3<f32>(1.0, 0.75, 0.15), 0.50);
    } else if (in.aux.x < -0.25) {
        col = mix(col, vec3<f32>(1.0, 0.20, 0.20), 0.55);
    }
    return vec4<f32>(col * diff + vec3<f32>(spec), 1.0);
}
"#;

const LINE_WGSL: &str = r#"
struct Uniforms {
    viewproj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    params: vec4<f32>,
    light1: vec4<f32>,
    light2: vec4<f32>,
};
@group(0) @binding(0) var<uniform> U: Uniforms;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) color: vec4<f32>,
};
struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};
@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    out.pos = U.viewproj * vec4<f32>(in.pos, 1.0);
    out.color = in.color;
    return out;
}
@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

const BLIT_WGSL: &str = r#"
@group(0) @binding(0) var samp: sampler;
@group(0) @binding(1) var tex: texture_2d<f32>;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};
@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VsOut {
    var p = array<vec2<f32>, 4>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(1.0, -1.0),
        vec2<f32>(-1.0, 1.0),
        vec2<f32>(1.0, 1.0),
    );
    let xy = p[vi];
    var out: VsOut;
    out.pos = vec4<f32>(xy, 0.0, 1.0);
    out.uv = vec2<f32>((xy.x + 1.0) * 0.5, (1.0 - xy.y) * 0.5);
    return out;
}
@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return textureSample(tex, samp, in.uv);
}
"#;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    viewproj: [[f32; 4]; 4],
    cam_pos: [f32; 4],
    params: [f32; 4],
    light1: [f32; 4],
    light2: [f32; 4],
}

struct MeshGpu {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    tri_count: u32,
    vert_count: u32,
}

struct DynBuf {
    buf: Option<wgpu::Buffer>,
    cap: u64,
}

impl DynBuf {
    fn new() -> DynBuf {
        DynBuf { buf: None, cap: 0 }
    }

    fn ensure(&mut self, device: &wgpu::Device, size: u64, label: &str) {
        if self.cap < size {
            self.cap = size.max(4096).next_power_of_two();
            self.buf = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: self.cap,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
        }
    }
}

struct Offscreen {
    view: wgpu::TextureView,
    depth: wgpu::TextureView,
    bind: wgpu::BindGroup,
    w: u32,
    h: u32,
}

pub struct GpuState {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    uniform: wgpu::Buffer,
    uniform_bind: wgpu::BindGroup,
    mesh_pipe: wgpu::RenderPipeline,
    line_pipe: wgpu::RenderPipeline,
    line_pipe_nodepth: wgpu::RenderPipeline,
    fill_pipe: wgpu::RenderPipeline,
    blit_pipe: wgpu::RenderPipeline,
    blit_layout: wgpu::BindGroupLayout,
    blit_sampler: wgpu::Sampler,
    mesh: Option<MeshGpu>,
    aux: DynBuf,
    wire: DynBuf,
    wire_verts: u32,
    lines_depth: DynBuf,
    lines_depth_verts: u32,
    lines_overlay: DynBuf,
    lines_overlay_verts: u32,
    fills: DynBuf,
    fill_verts: u32,
    off: Option<Offscreen>,
    view_px: (u32, u32),
    show_mesh: bool,
    show_wireframe: bool,
}

impl GpuState {
    pub fn new(
        device: wgpu::Device,
        queue: wgpu::Queue,
        target_format: wgpu::TextureFormat,
    ) -> GpuState {
        let bglayout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scanimprover-uniform-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scanimprover-uniform"),
            size: 192,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let uniform_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scanimprover-uniform-bind"),
            layout: &bglayout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });

        let pipe_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scanimprover-pipe-layout"),
            bind_group_layouts: &[Some(&bglayout)],
            immediate_size: 0,
        });

        let mesh_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scanimprover-mesh"),
            source: wgpu::ShaderSource::Wgsl(MESH_WGSL.into()),
        });
        let line_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scanimprover-line"),
            source: wgpu::ShaderSource::Wgsl(LINE_WGSL.into()),
        });
        let blit_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scanimprover-blit"),
            source: wgpu::ShaderSource::Wgsl(BLIT_WGSL.into()),
        });

        let opaque = wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Rgba8Unorm,
            blend: Some(wgpu::BlendState::REPLACE),
            write_mask: wgpu::ColorWrites::ALL,
        };
        let alpha = wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Rgba8Unorm,
            blend: Some(wgpu::BlendState::ALPHA_BLENDING),
            write_mask: wgpu::ColorWrites::ALL,
        };
        let depth_rw = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: Default::default(),
        };
        let depth_ro = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: Default::default(),
        };

        let geom_layout = wgpu::VertexBufferLayout {
            array_stride: 24,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        };
        let aux_layout = wgpu::VertexBufferLayout {
            array_stride: 16,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[wgpu::VertexAttribute {
                offset: 0,
                shader_location: 2,
                format: wgpu::VertexFormat::Float32x4,
            }],
        };
        let line_layout = wgpu::VertexBufferLayout {
            array_stride: 28,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        };

        let mesh_pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scanimprover-mesh-pipe"),
            layout: Some(&pipe_layout),
            vertex: wgpu::VertexState {
                module: &mesh_module,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(geom_layout.clone()), Some(aux_layout.clone())],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(depth_rw.clone()),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &mesh_module,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(opaque.clone())],
            }),
            multiview_mask: None,
            cache: None,
        });

        let make_line_pipe = |depth: Option<wgpu::DepthStencilState>,
                              target: wgpu::ColorTargetState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("scanimprover-line-pipe"),
                layout: Some(&pipe_layout),
                vertex: wgpu::VertexState {
                    module: &line_module,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[Some(line_layout.clone())],
                },
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::LineList,
                    ..Default::default()
                },
                depth_stencil: depth,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &line_module,
                    entry_point: Some("fs_main"),
                    compilation_options: Default::default(),
                    targets: &[Some(target)],
                }),
                multiview_mask: None,
                cache: None,
            })
        };

        let depth_always = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::Always),
            stencil: wgpu::StencilState::default(),
            bias: Default::default(),
        };

        let line_pipe = make_line_pipe(Some(depth_ro.clone()), opaque.clone());
        let line_pipe_nodepth = make_line_pipe(Some(depth_always), opaque.clone());

        let fill_pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scanimprover-fill-pipe"),
            layout: Some(&pipe_layout),
            vertex: wgpu::VertexState {
                module: &line_module,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(line_layout.clone())],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: Some(depth_ro),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &line_module,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(alpha)],
            }),
            multiview_mask: None,
            cache: None,
        });

        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scanimprover-blit-layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
            ],
        });
        let blit_pipe_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scanimprover-blit-pipe-layout"),
            bind_group_layouts: &[Some(&blit_layout)],
            immediate_size: 0,
        });
        let blit_pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scanimprover-blit-pipe"),
            layout: Some(&blit_pipe_layout),
            vertex: wgpu::VertexState {
                module: &blit_module,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleStrip,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &blit_module,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let blit_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("scanimprover-blit-sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::MipmapFilterMode::Nearest,
            ..Default::default()
        });

        GpuState {
            device,
            queue,
            uniform,
            uniform_bind,
            mesh_pipe,
            line_pipe,
            line_pipe_nodepth,
            fill_pipe,
            blit_pipe,
            blit_layout,
            blit_sampler,
            mesh: None,
            aux: DynBuf::new(),
            wire: DynBuf::new(),
            wire_verts: 0,
            lines_depth: DynBuf::new(),
            lines_depth_verts: 0,
            lines_overlay: DynBuf::new(),
            lines_overlay_verts: 0,
            fills: DynBuf::new(),
            fill_verts: 0,
            off: None,
            view_px: (1, 1),
            show_mesh: true,
            show_wireframe: false,
        }
    }

    pub fn upload_mesh(&mut self, mesh: &Mesh) {
        let nv = mesh.positions.len();
        let mut geom: Vec<[f32; 6]> = Vec::with_capacity(nv);
        for (p, n) in mesh.positions.iter().zip(mesh.normals.iter()) {
            geom.push([p[0], p[1], p[2], n[0], n[1], n[2]]);
        }
        let vb = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("scanimprover-mesh-vb"),
                contents: bytemuck::cast_slice(&geom),
                usage: wgpu::BufferUsages::VERTEX,
            });
        let ib = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("scanimprover-mesh-ib"),
                contents: bytemuck::cast_slice(&mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
        let aux_data = vec![[0.0f32, 0.0, -1.0, -1.0]; nv];
        self.aux
            .ensure(&self.device, (nv * 16) as u64, "scanimprover-aux");
        self.queue.write_buffer(
            self.aux.buf.as_ref().unwrap(),
            0,
            bytemuck::cast_slice(&aux_data),
        );
        self.mesh = Some(MeshGpu {
            vb,
            ib,
            tri_count: mesh.triangle_count() as u32,
            vert_count: nv as u32,
        });
        self.wire.buf = None;
        self.wire.cap = 0;
        self.wire_verts = 0;
    }

    pub fn upload_aux(&mut self, aux: &[[f32; 4]]) {
        let nv = self
            .mesh
            .as_ref()
            .map(|m| m.vert_count as usize)
            .unwrap_or(0);
        if aux.len() != nv {
            return;
        }
        if let Some(buf) = self.aux.buf.as_ref() {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(aux));
        }
    }

    pub fn upload_wireframe(&mut self, mesh: &Mesh, max_tris: usize) -> bool {
        if mesh.triangle_count() > max_tris {
            self.wire_verts = 0;
            self.wire.buf = None;
            self.wire.cap = 0;
            return false;
        }
        let mut edges = std::collections::HashSet::with_capacity(mesh.indices.len() * 2);
        let mut data: Vec<[f32; 7]> = Vec::with_capacity(mesh.indices.len() * 2);
        let push = |a: usize,
                    b: usize,
                    data: &mut Vec<[f32; 7]>,
                    mesh: &Mesh,
                    edges: &mut std::collections::HashSet<(u32, u32)>| {
            let key = if a < b {
                (a as u32, b as u32)
            } else {
                (b as u32, a as u32)
            };
            if edges.insert(key) {
                let pa = mesh.positions[a];
                let pb = mesh.positions[b];
                data.push([pa[0], pa[1], pa[2], 0.25, 0.28, 0.33, 1.0]);
                data.push([pb[0], pb[1], pb[2], 0.25, 0.28, 0.33, 1.0]);
            }
        };
        for t in 0..mesh.triangle_count() {
            let i0 = mesh.indices[3 * t] as usize;
            let i1 = mesh.indices[3 * t + 1] as usize;
            let i2 = mesh.indices[3 * t + 2] as usize;
            push(i0, i1, &mut data, mesh, &mut edges);
            push(i1, i2, &mut data, mesh, &mut edges);
            push(i2, i0, &mut data, mesh, &mut edges);
        }
        self.wire
            .ensure(&self.device, (data.len() * 28) as u64, "scanimprover-wire");
        if let Some(buf) = self.wire.buf.as_ref() {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(&data));
        }
        self.wire_verts = data.len() as u32;
        true
    }

    pub fn set_frame(
        &mut self,
        viewproj: Mat4,
        cam_pos: Vec3,
        light1: Vec3,
        light2: Vec3,
        heat_on: bool,
        heat_scale: f32,
        groups_on: bool,
        hover_group: f32,
        show_mesh: bool,
        show_wireframe: bool,
    ) {
        let u = Uniforms {
            viewproj: viewproj.to_cols_array_2d(),
            cam_pos: [cam_pos.x, cam_pos.y, cam_pos.z, 0.0],
            params: [
                if heat_on { 1.0 } else { 0.0 },
                heat_scale,
                if groups_on { 1.0 } else { 0.0 },
                hover_group,
            ],
            light1: [light1.x, light1.y, light1.z, 0.0],
            light2: [light2.x, light2.y, light2.z, 0.0],
        };
        self.queue
            .write_buffer(&self.uniform, 0, bytemuck::bytes_of(&u));
        self.show_mesh = show_mesh;
        self.show_wireframe = show_wireframe;
    }

    pub fn write_lines_depth(&mut self, lines: &[[f32; 7]]) {
        self.lines_depth.ensure(
            &self.device,
            (lines.len().max(1) * 28) as u64,
            "scanimprover-lines",
        );
        if let Some(buf) = self.lines_depth.buf.as_ref() {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(lines));
        }
        self.lines_depth_verts = lines.len() as u32;
    }

    pub fn write_lines_overlay(&mut self, lines: &[[f32; 7]]) {
        self.lines_overlay.ensure(
            &self.device,
            (lines.len().max(1) * 28) as u64,
            "scanimprover-lines-overlay",
        );
        if let Some(buf) = self.lines_overlay.buf.as_ref() {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(lines));
        }
        self.lines_overlay_verts = lines.len() as u32;
    }

    pub fn write_fills(&mut self, fills: &[[f32; 7]]) {
        self.fills.ensure(
            &self.device,
            (fills.len().max(1) * 28) as u64,
            "scanimprover-fills",
        );
        if let Some(buf) = self.fills.buf.as_ref() {
            self.queue.write_buffer(buf, 0, bytemuck::cast_slice(fills));
        }
        self.fill_verts = fills.len() as u32;
    }

    pub fn set_viewport_px(&mut self, w: u32, h: u32) {
        self.view_px = (w.max(1), h.max(1));
    }

    fn ensure_offscreen(&mut self) {
        let (w, h) = self.view_px;
        if let Some(off) = &self.off {
            if off.w == w && off.h == h {
                return;
            }
        }
        let tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("scanimprover-offscreen"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("scanimprover-depth"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth = depth_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scanimprover-blit-bind"),
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Sampler(&self.blit_sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
            ],
        });
        self.off = Some(Offscreen {
            view,
            depth,
            bind,
            w,
            h,
        });
    }

    pub fn render_offscreen(&mut self) -> Option<wgpu::CommandBuffer> {
        if self.mesh.is_none() {
            return None;
        }
        self.ensure_offscreen();
        let off = self.off.as_ref()?;
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("scanimprover-scene"),
            });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("scanimprover-scene-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &off.view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.075,
                            g: 0.078,
                            b: 0.095,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &off.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if self.show_mesh {
                if let Some(m) = &self.mesh {
                    pass.set_pipeline(&self.mesh_pipe);
                    pass.set_bind_group(0, &self.uniform_bind, &[]);
                    pass.set_vertex_buffer(0, m.vb.slice(..));
                    if let Some(aux) = self.aux.buf.as_ref() {
                        pass.set_vertex_buffer(1, aux.slice(..));
                    }
                    pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..m.tri_count * 3, 0, 0..1);
                }
            }
            if self.show_wireframe && self.wire_verts > 0 {
                pass.set_pipeline(&self.line_pipe);
                pass.set_bind_group(0, &self.uniform_bind, &[]);
                if let Some(buf) = self.wire.buf.as_ref() {
                    pass.set_vertex_buffer(0, buf.slice(..));
                }
                pass.draw(0..self.wire_verts, 0..1);
            }
            if self.fill_verts > 0 {
                pass.set_pipeline(&self.fill_pipe);
                pass.set_bind_group(0, &self.uniform_bind, &[]);
                if let Some(buf) = self.fills.buf.as_ref() {
                    pass.set_vertex_buffer(0, buf.slice(..));
                }
                pass.draw(0..self.fill_verts, 0..1);
            }
            if self.lines_depth_verts > 0 {
                pass.set_pipeline(&self.line_pipe);
                pass.set_bind_group(0, &self.uniform_bind, &[]);
                if let Some(buf) = self.lines_depth.buf.as_ref() {
                    pass.set_vertex_buffer(0, buf.slice(..));
                }
                pass.draw(0..self.lines_depth_verts, 0..1);
            }
            if self.lines_overlay_verts > 0 {
                pass.set_pipeline(&self.line_pipe_nodepth);
                pass.set_bind_group(0, &self.uniform_bind, &[]);
                if let Some(buf) = self.lines_overlay.buf.as_ref() {
                    pass.set_vertex_buffer(0, buf.slice(..));
                }
                pass.draw(0..self.lines_overlay_verts, 0..1);
            }
        }
        Some(encoder.finish())
    }

    pub fn blit(
        &self,
        info: &egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
    ) {
        let off = match &self.off {
            Some(o) => o,
            None => return,
        };
        let vp = info.viewport_in_pixels();
        render_pass.set_scissor_rect(
            vp.left_px.max(0) as u32,
            vp.top_px.max(0) as u32,
            vp.width_px.max(0) as u32,
            vp.height_px.max(0) as u32,
        );
        render_pass.set_pipeline(&self.blit_pipe);
        render_pass.set_bind_group(0, &off.bind, &[]);
        render_pass.draw(0..4, 0..1);
    }
}

pub struct ViewportCallback {
    pub gpu: Arc<Mutex<GpuState>>,
}

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        _device: &wgpu::Device,
        _queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        _callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        match self.gpu.lock() {
            Ok(mut gpu) => match gpu.render_offscreen() {
                Some(cb) => vec![cb],
                None => Vec::new(),
            },
            Err(_) => Vec::new(),
        }
    }

    fn paint(
        &self,
        info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _callback_resources: &egui_wgpu::CallbackResources,
    ) {
        if let Ok(gpu) = self.gpu.lock() {
            gpu.blit(&info, render_pass);
        }
    }
}

pub fn make_callback(gpu: Arc<Mutex<GpuState>>, rect: egui::Rect) -> egui::PaintCallback {
    egui_wgpu::Callback::new_paint_callback(rect, ViewportCallback { gpu })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_wgsl_syntax_and_front_facing() {
        assert!(MESH_WGSL.contains("@builtin(front_facing) is_front: bool"));
        assert!(MESH_WGSL.contains("if (!is_front)"));
        // Validate WGSL parsing with wgpu naga frontend
        let res = wgpu::naga::front::wgsl::parse_str(MESH_WGSL);
        assert!(res.is_ok(), "MESH_WGSL failed to parse: {:?}", res.err());
    }

    #[test]
    fn test_line_and_blit_wgsl_syntax() {
        assert!(wgpu::naga::front::wgsl::parse_str(LINE_WGSL).is_ok());
        assert!(wgpu::naga::front::wgsl::parse_str(BLIT_WGSL).is_ok());
    }
}
