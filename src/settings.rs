//! Persistent user settings (panel sizes, theme, view toggles) stored as a
//! small `key=value` text file in the user's configuration directory.

use crate::camera::UpAxis;
use crate::ui::ToolSection;
use crate::ui::theme::ThemeMode;
use std::path::PathBuf;

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub left_panel_width: f32,
    pub right_panel_width: f32,
    pub theme: ThemeMode,
    pub up_axis: UpAxis,
    pub show_grid: bool,
    pub show_bbox: bool,
    pub show_triad: bool,
    pub show_wireframe: bool,
    pub show_object_browser: bool,
    pub brush_connected: bool,
    pub brush_radius: f32,
    pub open_sections: Vec<ToolSection>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            left_panel_width: 330.0,
            right_panel_width: 300.0,
            theme: ThemeMode::Dark,
            up_axis: UpAxis::Y,
            show_grid: true,
            show_bbox: true,
            show_triad: true,
            show_wireframe: false,
            show_object_browser: true,
            brush_connected: true,
            brush_radius: 25.0,
            open_sections: Vec::new(),
        }
    }
}

impl Settings {
    /// Location of the settings file, if a configuration directory exists.
    pub fn path() -> Option<PathBuf> {
        let base = if cfg!(target_os = "windows") {
            std::env::var_os("APPDATA").map(PathBuf::from)
        } else if cfg!(target_os = "macos") {
            std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Library/Application Support"))
        } else {
            std::env::var_os("XDG_CONFIG_HOME")
                .map(PathBuf::from)
                .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        }?;
        Some(base.join("ScanImprover").join("settings.txt"))
    }

    pub fn load() -> Settings {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|text| Self::parse(&text))
            .unwrap_or_default()
    }

    pub fn save(&self) -> Result<(), String> {
        let path = Self::path().ok_or("no configuration directory")?;
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
        std::fs::write(&path, self.serialize()).map_err(|e| e.to_string())
    }

    pub fn serialize(&self) -> String {
        let sections: Vec<&str> = self.open_sections.iter().map(|s| s.key()).collect();
        format!(
            "left_panel_width={}\nright_panel_width={}\ntheme={}\nup_axis={}\nshow_grid={}\nshow_bbox={}\nshow_triad={}\nshow_wireframe={}\nshow_object_browser={}\nbrush_connected={}\nbrush_radius={}\nopen_sections={}\n",
            self.left_panel_width,
            self.right_panel_width,
            match self.theme {
                ThemeMode::Dark => "dark",
                ThemeMode::Light => "light",
            },
            match self.up_axis {
                UpAxis::Y => "y",
                UpAxis::Z => "z",
            },
            self.show_grid,
            self.show_bbox,
            self.show_triad,
            self.show_wireframe,
            self.show_object_browser,
            self.brush_connected,
            self.brush_radius,
            sections.join(",")
        )
    }

    pub fn parse(text: &str) -> Settings {
        let mut s = Settings::default();
        let flag = |v: &str| v.trim() == "true";
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else {
                continue;
            };
            let v = v.trim();
            match k.trim() {
                "left_panel_width" => {
                    if let Ok(w) = v.parse::<f32>() {
                        s.left_panel_width = w.clamp(290.0, 460.0);
                    }
                }
                "right_panel_width" => {
                    if let Ok(w) = v.parse::<f32>() {
                        s.right_panel_width = w.clamp(240.0, 460.0);
                    }
                }
                "theme" => {
                    s.theme = if v == "light" {
                        ThemeMode::Light
                    } else {
                        ThemeMode::Dark
                    }
                }
                "up_axis" => s.up_axis = if v == "z" { UpAxis::Z } else { UpAxis::Y },
                "show_grid" => s.show_grid = flag(v),
                "show_bbox" => s.show_bbox = flag(v),
                "show_triad" => s.show_triad = flag(v),
                "show_wireframe" => s.show_wireframe = flag(v),
                "show_object_browser" => s.show_object_browser = flag(v),
                "brush_connected" => s.brush_connected = flag(v),
                "brush_radius" => {
                    if let Ok(r) = v.parse::<f32>() {
                        s.brush_radius = r.clamp(2.0, 150.0);
                    }
                }
                "open_sections" => {
                    s.open_sections = v.split(',').filter_map(ToolSection::from_key).collect();
                }
                _ => {}
            }
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_roundtrip_and_clamping() {
        let s = Settings {
            left_panel_width: 400.0,
            right_panel_width: 250.0,
            theme: ThemeMode::Light,
            up_axis: UpAxis::Z,
            show_grid: false,
            show_bbox: false,
            show_triad: true,
            show_wireframe: true,
            show_object_browser: false,
            brush_connected: false,
            brush_radius: 40.0,
            open_sections: vec![ToolSection::Repair, ToolSection::Decimation],
        };
        assert_eq!(Settings::parse(&s.serialize()), s);
        let clamped =
            Settings::parse("left_panel_width=10\nbrush_radius=9999\nbogus=1\ntheme=purple\n");
        assert_eq!(clamped.left_panel_width, 290.0);
        assert_eq!(clamped.brush_radius, 150.0);
        assert_eq!(clamped.theme, ThemeMode::Dark);
        assert_eq!(Settings::parse(""), Settings::default());
    }
}
