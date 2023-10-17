use crate::{
    opts::{Convert, VelodyneReturnMode},
    types::FileFormat,
    utils::{build_velodyne_config, guess_file_format},
};
use anyhow::{anyhow, bail, Result};
use approx::abs_diff_eq;
use pcd_format::{LibpclPoint, NewslabV1Point};
use pcd_rs::DataKind;
use std::{
    f64::{
        self,
        consts::{FRAC_PI_2, PI},
    },
    fs,
    path::Path,
};
use velodyne_lidar::{
    iter::frame_xyz_iter_from_file,
    types::{
        format::FormatKind,
        measurements::{Measurement, MeasurementDual},
        point::{PointD, PointS},
    },
    ProductID, ReturnMode,
};

pub fn convert(opts: Convert) -> Result<()> {
    let input_path = &opts.input;
    let output_path = &opts.output;

    let input_format = match opts.from {
        Some(format) => format,
        None => guess_file_format(input_path)
            .ok_or_else(|| anyhow!("cannot guess format of input '{}'", input_path.display()))?,
    };
    let output_format = match opts.to {
        Some(format) => format,
        None => guess_file_format(output_path)
            .ok_or_else(|| anyhow!("cannot guess format of output '{}'", output_path.display()))?,
    };

    use FileFormat as F;

    match (input_format, output_format) {
        (F::LibpclPcd, F::NewslabPcd) => {
            libpcl_pcd_to_newslab_pcd(input_path, output_path)?;
        }
        (F::NewslabPcd, F::LibpclPcd) => {
            newslab_pcd_to_libpcl_pcd(input_path, output_path)?;
        }
        (F::VelodynePcap, F::LibpclPcd) => {
            let velodyne_model = opts
                .velodyne_model
                .ok_or_else(|| anyhow!("--velodyne-mode must be set"))?;
            let velodyne_return_mode = opts
                .velodyne_return_mode
                .ok_or_else(|| anyhow!("--velodyne-return-mode must be set"))?;

            velodyne_pcap_to_libpcl_pcd(
                input_path,
                output_path,
                velodyne_model,
                velodyne_return_mode,
            )?;
        }
        (F::VelodynePcap, F::NewslabPcd) => {
            bail!("converting from pcap.velodyne file to pcd.newslab is not supported");
        }
        (F::LibpclPcd | F::NewslabPcd, F::VelodynePcap) => {
            bail!("converting to pcap.velodyne is not supported");
        }
        (F::LibpclPcd, F::LibpclPcd)
        | (F::NewslabPcd, F::NewslabPcd)
        | (F::VelodynePcap, F::VelodynePcap) => {
            // Simply copy the file
            fs::copy(input_path, output_path)?;
        }
    }

    Ok(())
}

fn libpcl_pcd_to_newslab_pcd<PI, PO>(input_path: PI, output_path: PO) -> Result<()>
where
    PI: AsRef<Path>,
    PO: AsRef<Path>,
{
    let mut reader = pcd_rs::Reader::open(input_path)?;
    let pcd_rs::PcdMeta {
        width,
        height,
        ref viewpoint,
        data,
        ..
    } = *reader.meta();

    let mut writer = pcd_rs::WriterInit {
        width,
        height,
        viewpoint: viewpoint.clone(),
        data_kind: data,
        schema: None,
    }
    .create(output_path)?;

    reader.try_for_each(|point| -> Result<_> {
        let LibpclPoint { x, y, z, .. } = point?;
        let x = x as f64;
        let y = y as f64;
        let z = z as f64;
        let distance = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();

        let point = if abs_diff_eq!(distance, 0.0) {
            NewslabV1Point {
                x,
                y,
                z,
                distance,
                azimuthal_angle: 0.0,
                vertical_angle: 0.0,
                intensity: 0.0,
                laser_id: 0,
                timestamp_ns: 0,
            }
        } else {
            let polar_angle = if abs_diff_eq!(z, 0.0) {
                FRAC_PI_2
            } else {
                let planar_dist = (x.powi(2) + y.powi(2)).sqrt();
                planar_dist.atan2(z) + if z > 0.0 { 0.0 } else { PI }
            };
            let azimuthal_angle = match (abs_diff_eq!(x, 0.0), abs_diff_eq!(y, 0.0)) {
                (true, true) => 0.0,
                (true, false) => {
                    if y > 0.0 {
                        FRAC_PI_2
                    } else {
                        -FRAC_PI_2
                    }
                }
                (false, _) => {
                    y.atan2(x)
                        + if x > 0.0 {
                            0.0
                        } else if y >= 0.0 {
                            PI
                        } else {
                            -PI
                        }
                }
            };
            let vertical_angle = FRAC_PI_2 - polar_angle;

            NewslabV1Point {
                x,
                y,
                z,
                distance,
                azimuthal_angle,
                vertical_angle,
                intensity: 0.0,
                laser_id: 0,
                timestamp_ns: 0,
            }
        };

        writer.push(&point)?;
        Ok(())
    })?;

    writer.finish()?;

    Ok(())
}

fn newslab_pcd_to_libpcl_pcd<PI, PO>(input_path: PI, output_path: PO) -> Result<()>
where
    PI: AsRef<Path>,
    PO: AsRef<Path>,
{
    let mut reader = pcd_rs::Reader::open(input_path)?;
    let pcd_rs::PcdMeta {
        width,
        height,
        ref viewpoint,
        data,
        ..
    } = *reader.meta();

    let mut writer = pcd_rs::WriterInit {
        width,
        height,
        viewpoint: viewpoint.clone(),
        data_kind: data,
        schema: None,
    }
    .create(output_path)?;

    reader.try_for_each(|point| -> Result<_> {
        let NewslabV1Point { x, y, z, .. } = point?;
        let x = x as f32;
        let y = y as f32;
        let z = z as f32;
        let point = LibpclPoint { x, y, z, rgb: 0 };

        writer.push(&point)?;
        Ok(())
    })?;

    writer.finish()?;

    Ok(())
}

fn velodyne_pcap_to_libpcl_pcd<I, O>(
    input_file: I,
    output_dir: O,
    model: ProductID,
    mode: VelodyneReturnMode,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    use FormatKind as F;
    use ReturnMode as R;

    // closures
    let map_measurement = |measurement: Measurement| {
        let [x, y, z] = measurement.xyz;
        [
            x.as_meters() as f32,
            y.as_meters() as f32,
            z.as_meters() as f32,
        ]
    };

    let map_point_single = |point: PointS| map_measurement(point.measurement);
    let map_point_dual = |point: PointD| {
        let MeasurementDual {
            strongest: strongest_measure,
            last: last_measure,
        } = point.measurements;
        let strongest_point = map_measurement(strongest_measure);
        let last_point = map_measurement(last_measure);
        (strongest_point, last_point)
    };

    // create the velodyne-lidar config
    let config = build_velodyne_config(model, mode.0)?;

    // Create output directories
    let output_dir = output_dir.as_ref();
    let strongest_output_dir = output_dir.join("strongest");
    let last_output_dir = output_dir.join("last");
    fs::create_dir(output_dir)?;

    match mode.0 {
        R::Strongest => {
            fs::create_dir(&strongest_output_dir)?;
        }
        R::Last => {
            fs::create_dir(&last_output_dir)?;
        }
        R::Dual => {
            fs::create_dir(&strongest_output_dir)?;
            fs::create_dir(&last_output_dir)?;
        }
    }

    let frames = frame_xyz_iter_from_file(config, input_file)?;

    match mode.0 {
        R::Strongest => {
            frames.enumerate().try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file = strongest_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_pcd_file_single(points, pcd_file, width, 16)?;
                    }
                    F::Single32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_pcd_file_single(points, pcd_file, width, 32)?;
                    }
                    _ => unreachable!(),
                }

                anyhow::Ok(())
            })?;
        }
        R::Last => {
            frames.enumerate().try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file = last_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_pcd_file_single(points, pcd_file, width, 16)?;
                    }
                    F::Single32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_pcd_file_single(points, pcd_file, width, 32)?;
                    }
                    _ => unreachable!(),
                }

                anyhow::Ok(())
            })?;
        }
        R::Dual => {
            frames.enumerate().try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file_strongest = strongest_output_dir.join(&file_name);
                let pcd_file_last = last_output_dir.join(&file_name);

                match frame? {
                    F::Dual16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_pcd_file_dual(points, pcd_file_strongest, pcd_file_last, width, 16)?;
                    }
                    F::Dual32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_pcd_file_dual(points, pcd_file_strongest, pcd_file_last, width, 32)?;
                    }
                    _ => unreachable!(),
                }

                anyhow::Ok(())
            })?;
        }
    }

    Ok(())
}

fn create_pcd_file_single<P, I>(points: I, pcd_file: P, width: usize, height: usize) -> Result<()>
where
    P: AsRef<Path>,
    I: IntoIterator<Item = [f32; 3]>,
{
    let mut writer = pcd_rs::WriterInit {
        width: width as u64,
        height: height as u64,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
    }
    .create(pcd_file)?;

    points
        .into_iter()
        .map(|[x, y, z]| LibpclPoint { x, y, z, rgb: 0 })
        .try_for_each(|point| -> Result<_> {
            writer.push(&point)?;
            Ok(())
        })?;
    writer.finish()?;

    Ok(())
}

fn create_pcd_file_dual<P1, P2, I>(
    points: I,
    pcd_file1: P1,
    pcd_file2: P2,
    width: usize,
    height: usize,
) -> Result<()>
where
    P1: AsRef<Path>,
    P2: AsRef<Path>,
    I: IntoIterator<Item = ([f32; 3], [f32; 3])>,
{
    let data_kind = DataKind::Binary;

    let mut writer1 = pcd_rs::WriterInit {
        width: width as u64,
        height: height as u64,
        viewpoint: Default::default(),
        data_kind,
        schema: None,
    }
    .create(pcd_file1)?;
    let mut writer2 = pcd_rs::WriterInit {
        width: width as u64,
        height: height as u64,
        viewpoint: Default::default(),
        data_kind,
        schema: None,
    }
    .create(pcd_file2)?;

    let map_point = |[x, y, z]: [f32; 3]| LibpclPoint { x, y, z, rgb: 0 };
    points
        .into_iter()
        .map(|(p1, p2)| (map_point(p1), map_point(p2)))
        .try_for_each(|(p1, p2)| -> Result<_> {
            writer1.push(&p1)?;
            writer2.push(&p2)?;
            Ok(())
        })?;
    writer2.finish()?;

    Ok(())
}
