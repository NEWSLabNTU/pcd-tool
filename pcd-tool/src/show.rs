use crate::{opts::Show, types::FileFormat, utils::guess_file_format};
use anyhow::{anyhow, Result};
use itertools::Itertools;
use kiss3d::{
    light::Light,
    nalgebra as na,
    window::{State, Window},
};
use std::path::Path;

struct Gui {
    points: Vec<na::Point3<f32>>,
}

impl State for Gui {
    fn step(&mut self, window: &mut Window) {
        let color = na::Point3::new(1.0, 1.0, 1.0);

        self.points
            .iter()
            .for_each(|point| window.draw_point(point, &color));
    }
}

pub fn show(args: Show) -> Result<()> {
    let Show { format, input } = args;

    let format = match format {
        Some(format) => format,
        None => guess_file_format(&input)
            .ok_or_else(|| anyhow!("unable to guess file format of '{}'", input.display()))?,
    };

    use FileFormat as F;
    match format {
        F::LibpclPcd | F::NewslabPcd => show_pcd(&input)?,
        F::VelodynePcap => show_velodyne_pcap(&input)?,
    }

    Ok(())
}

fn show_velodyne_pcap<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    todo!()
}

fn show_pcd<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let reader = pcd_rs::DynReader::open(path)?;
    let points: Vec<_> = reader
        .map(|record| -> Result<_> {
            let record = record?;
            let [x, y, z] = record
                .to_xyz()
                .ok_or_else(|| anyhow!("No x, y or z field found"))?;
            let point = na::Point3::new(x, y, z);
            Ok(point)
        })
        .try_collect()?;

    let mut window = Window::new("pcd-tool");
    window.set_light(Light::StickToCamera);

    let gui = Gui { points };
    window.render_loop(gui);

    Ok(())
}
