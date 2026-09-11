//! "Experimental" accordion: work-in-progress tools kept apart from the
//! established workflow. Each tool renders its own group box below the note.

use crate::app::App;
use crate::ui::theme;

pub fn render_experimental(_app: &mut App, ui: &mut egui::Ui) {
    ui.label(
        egui::RichText::new(
            "Work in progress. Results may change between versions; \
             every operation can be undone.",
        )
        .size(11.5)
        .color(theme::pal().text_muted),
    );
}
