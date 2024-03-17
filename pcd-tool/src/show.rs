mod gui;

use self::gui::run_gui;
use crate::{
    opts::{Show, VelodyneReturnMode},
    show::gui::PointAndColor,
    types::FileFormat,
    utils::{build_velodyne_config, guess_file_format},
};
use anyhow::{anyhow, Result};
use itertools::Itertools;
use measurements::Length;
use std::path::Path;
use velodyne_lidar::ProductID;

pub fn show(args: Show) -> Result<()> {
    let Show {
        format,
        input,
        velodyne_model,
        velodyne_return_mode,
    } = args;

    let format = match format {
        Some(format) => format,
        None => guess_file_format(&input)
            .ok_or_else(|| anyhow!("unable to guess file format of '{}'", input.display()))?,
    };

    use FileFormat as F;
    match format {
        F::LibpclPcd | F::NewslabPcd => show_pcd(&input)?,
        F::VelodynePcap => {
            let velodyne_model =
                velodyne_model.ok_or_else(|| anyhow!("--velodyne-mode must be set"))?;
            let velodyne_return_mode = velodyne_return_mode
                .ok_or_else(|| anyhow!("--velodyne-return-mode must be set"))?;

            show_velodyne_pcap(&input, velodyne_model, velodyne_return_mode)?;
        }
        F::RawBin => todo!(),
    }

    Ok(())
}

fn show_velodyne_pcap<P>(path: P, model: ProductID, mode: VelodyneReturnMode) -> Result<()>
where
    P: AsRef<Path>,
{
    let config = build_velodyne_config(model, mode.0)?;
    let frames = velodyne_lidar::iter::frame_xyz_iter_from_file(config, path)?;

    let frames: Vec<_> = frames
        .map(|frame| -> Result<_> {
            let frame = frame?;

            let points: Vec<_> = frame
                .into_firing_iter()
                .flat_map(|firing| {
                    firing.into_point_iter().flat_map(|point| {
                        use velodyne_lidar::types::point::Point as P;

                        let length_to_f32 = |[x, y, z]: [Length; 3]| {
                            [
                                x.as_meters() as f32,
                                y.as_meters() as f32,
                                z.as_meters() as f32,
                            ]
                        };

                        match point {
                            P::Single(point) => {
                                vec![PointAndColor {
                                    point: length_to_f32(point.measurement.xyz),
                                    color: [1.0, 1.0, 1.0],
                                }]
                            }
                            P::Dual(point) => {
                                vec![
                                    PointAndColor {
                                        point: length_to_f32(point.measurements.strongest.xyz),
                                        color: [0.0, 1.0, 0.0],
                                    },
                                    PointAndColor {
                                        point: length_to_f32(point.measurements.last.xyz),
                                        color: [0.0, 0.0, 1.0],
                                    },
                                ]
                            }
                        }
                    })
                })
                .collect();

            Ok(points)
        })
        .try_collect()?;

    run_gui(frames.into_iter());

    Ok(())
}

fn show_pcd<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let reader = pcd_rs::DynReader::open(path)?;
    let points: Vec<_> = reader
        .map(|record| -> Result<_> {
            let record = record?;
            let point = record
                .to_xyz()
                .ok_or_else(|| anyhow!("No x, y or z field found"))?;
            let color = [1.0, 1.0, 1.0];
            Ok(gui::PointAndColor { point, color })
        })
        .try_collect()?;

    run_gui([points].into_iter());

    Ok(())
}
