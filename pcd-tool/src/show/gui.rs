use kiss3d::{
    event::{Action, Key, Modifiers, WindowEvent},
    light::Light,
    window::{State, Window},
};

pub struct PointAndColor {
    pub point: [f32; 3],
    pub color: [f32; 3],
}

pub fn run_gui<I>(mut iter: I)
where
    I: Iterator<Item = Vec<PointAndColor>> + 'static,
{
    let mut window = Window::new("pcd-tool");
    window.set_light(Light::StickToCamera);

    let points = iter.next();
    let gui = Gui { iter, points };
    window.render_loop(gui);
}

struct Gui<I>
where
    I: Iterator<Item = Vec<PointAndColor>>,
{
    iter: I,
    points: Option<Vec<PointAndColor>>,
}

impl<I> State for Gui<I>
where
    I: Iterator<Item = Vec<PointAndColor>> + 'static,
{
    fn step(&mut self, window: &mut Window) {
        use WindowEvent as E;

        let mut go_next = false;
        let mut go_prev = false;

        // Process keyboard events
        for evt in window.events().iter() {
            match evt.value {
                E::Key(key, action, modifiers) => {
                    use Action as A;
                    use Key as K;

                    let has_alt = modifiers.contains(Modifiers::Alt);
                    let has_shift = modifiers.contains(Modifiers::Shift);
                    let has_ctrl = modifiers.contains(Modifiers::Control);
                    let has_super = modifiers.contains(Modifiers::Super);

                    match (key, action, has_shift, has_ctrl, has_alt, has_super) {
                        (K::N, A::Press, false, false, false, false) => {
                            go_next = true;
                        }
                        // (K::P, A::Press, false, false, false, false) => {
                        //     go_prev = true;
                        // }
                        _ => {}
                    };
                }
                _ => {}
            }
        }

        match (go_prev, go_next) {
            (true, false) => {
                // if let Some(points) = self.iter.next_back() {
                //     self.points = Some(points);
                // }
            }
            (false, true) => {
                if let Some(points) = self.iter.next() {
                    self.points = Some(points);
                }
            }
            _ => {}
        }

        // Render
        if let Some(points) = &self.points {
            for item in points {
                let PointAndColor { point, color } = *item;
                window.draw_point(&point.into(), &color.into());
            }
        }
    }
}
