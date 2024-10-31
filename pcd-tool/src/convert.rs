use crate::{
    io::{
        count_frames_in_velodyne_pcap, create_libpcl_pcd_file_dual, create_libpcl_pcd_file_single,
        create_pcd_reader, create_raw_bin_file_dual, create_raw_bin_file_single, load_bin_iter,
        RawBinWriter,
    },
    opts::{Convert, EndFrame, StartFrame, VelodyneReturnMode},
    types::{BinPoint, FileFormat},
    utils::{build_velodyne_config, guess_file_format},
};
use approx::abs_diff_eq;
use eyre::{bail, ensure, format_err, Context, Result};
use itertools::Itertools;
use nalgebra as na;
use pcd_format::{LibpclPoint, NewslabV1Point};
use rayon::prelude::*;
use std::{
    f64::{
        self,
        consts::{FRAC_PI_2, PI},
    },
    fs::{self, File},
    io::BufReader,
    path::Path,
};
use tf_format::MaybeTransform;
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

    let tf: Option<na::Isometry3<f32>> = match (&opts.transform_file, &opts.transform) {
        (None, None) => None,
        (Some(file), None) => {
            let reader = BufReader::new(File::open(file)?);
            let tf: MaybeTransform = serde_json::from_reader(reader)?;
            Some(tf.to_na_isometry3())
        }
        (None, Some(text)) => {
            let tf: MaybeTransform = serde_json::from_str(text)?;
            Some(tf.to_na_isometry3())
        }
        (Some(_), Some(_)) => bail!("--transform and --transform-file cannot be both specified"),
    };

    let input_format = match opts.from {
        Some(format) => format,
        None => guess_file_format(input_path).ok_or_else(|| {
            format_err!("cannot guess format of input '{}'", input_path.display())
        })?,
    };
    let output_format = match opts.to {
        Some(format) => format,
        None => guess_file_format(output_path).ok_or_else(|| {
            format_err!("cannot guess format of output '{}'", output_path.display())
        })?,
    };

    use FileFormat as F;

    match (input_format, output_format) {
        (F::LibpclPcd, F::NewslabPcd) => {
            libpcl_pcd_to_newslab_pcd(input_path, output_path, tf)?;
        }
        (F::NewslabPcd, F::LibpclPcd) => {
            newslab_pcd_to_libpcl_pcd(input_path, output_path, tf)?;
        }
        (F::VelodynePcap, F::LibpclPcd) => {
            let velodyne_model = opts
                .velodyne_model
                .ok_or_else(|| format_err!("--velodyne-mode must be set"))?;
            let velodyne_return_mode = opts
                .velodyne_return_mode
                .ok_or_else(|| format_err!("--velodyne-return-mode must be set"))?;

            velodyne_pcap_to_libpcl_pcd(
                input_path,
                output_path,
                velodyne_model,
                velodyne_return_mode,
                opts.start,
                opts.end,
                tf,
            )?;
        }
        (F::VelodynePcap, F::NewslabPcd) => {
            bail!("converting from pcap.velodyne file to pcd.newslab is not supported");
        }
        (F::LibpclPcd | F::NewslabPcd, F::VelodynePcap) => {
            bail!("converting to pcap.velodyne is not supported");
        }
        (F::LibpclPcd | F::NewslabPcd, F::RawBin) => {
            if is_file(input_path)? {
                pcd_file_raw_bin_file(input_path, output_path, tf)?;
            } else {
                pcd_dir_raw_bin_dir(input_path, output_path, tf)?;
            }
        }
        (F::VelodynePcap, F::RawBin) => {
            let velodyne_model = opts
                .velodyne_model
                .ok_or_else(|| format_err!("--velodyne-mode must be set"))?;
            let velodyne_return_mode = opts
                .velodyne_return_mode
                .ok_or_else(|| format_err!("--velodyne-return-mode must be set"))?;

            velodyne_pcap_to_raw_bin(
                input_path,
                output_path,
                velodyne_model,
                velodyne_return_mode,
                opts.start,
                opts.end,
                tf,
            )?;
        }
        (F::RawBin, F::LibpclPcd) => {
            if is_file(input_path)? {
                bin_file_to_libpcl_pcd_file(input_path, output_path, tf)?;
            } else {
                bin_dir_to_libpcl_pcd_dir(input_path, output_path, tf)?;
            }
        }
        (F::RawBin, F::NewslabPcd) => {
            bail!("conversion from raw.bin to pcd.newslab is not supported");
        }
        (F::RawBin, F::VelodynePcap) => {
            bail!("converting to pcap.velodyne is not supported");
        }
        (F::LibpclPcd, F::LibpclPcd) => {
            libpcl_pcd_to_libpcl_pcd(input_path, output_path, tf)?;
        }
        (F::NewslabPcd, F::NewslabPcd)
        | (F::VelodynePcap, F::VelodynePcap)
        | (F::RawBin, F::RawBin) => {
            match (opts.start, opts.end) {
                (StartFrame::Forward(1), EndFrame::Backward(1)) => {}
                _ => {
                    bail!("--start and --end are not supported ");
                }
            }

            if tf.is_some() {
                bail!("--transform and --transform-file are not supported ");
            }

            // Simply copy the file
            fs::copy(input_path, output_path)?;
        }
    }

    Ok(())
}

fn libpcl_pcd_to_libpcl_pcd<PI, PO>(
    input_path: PI,
    output_path: PO,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    PI: AsRef<Path>,
    PO: AsRef<Path>,
{
    let Some(tf) = tf else {
        // Simply copy the file
        fs::copy(input_path, output_path)?;
        return Ok(());
    };
    let input_path = input_path.as_ref();
    let mut reader = create_pcd_reader(input_path)?;
    let pcd_rs::PcdMeta {
        width,
        height,
        ref viewpoint,
        data,
        ref field_defs,
        ..
    } = *reader.meta();

    let mut writer = pcd_rs::WriterInit {
        width,
        height,
        viewpoint: viewpoint.clone(),
        data_kind: data,
        schema: Some(field_defs.clone()),
    }
    .create(output_path)?;

    let find_field = |name| {
        let field = field_defs
            .fields
            .iter()
            .enumerate()
            .find(|(_, field)| field.name == name);

        match field {
            Some((idx, _)) => Ok(idx),
            None => bail!(r#""{name}" field is required but is not found"#),
        }
    };

    let x_idx = find_field("x")?;
    let y_idx = find_field("y")?;
    let z_idx = find_field("z")?;

    let set_value = |field: &mut pcd_rs::Field, value: f32| {
        match field {
            pcd_rs::Field::F32(vec) => vec[0] = value,
            pcd_rs::Field::F64(vec) => vec[0] = value as f64,
            _ => bail!("transforming non-floating point coordinates is not supported"),
        }
        Ok(())
    };

    reader.try_for_each(|point| -> Result<_> {
        let mut point = point?;
        let Some([x, y, z]) = point.to_xyz::<f32>() else {
            bail!(
                "the file {} misses one of x, y or z field",
                input_path.display()
            );
        };

        let [x, y, z] = transform_point([x, y, z], Some(tf));
        set_value(&mut point.0[x_idx], x)?;
        set_value(&mut point.0[y_idx], y)?;
        set_value(&mut point.0[z_idx], z)?;

        writer.push(&point)?;
        Ok(())
    })?;

    writer.finish()?;

    Ok(())
}

fn libpcl_pcd_to_newslab_pcd<PI, PO>(
    input_path: PI,
    output_path: PO,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    PI: AsRef<Path>,
    PO: AsRef<Path>,
{
    let input_path = input_path.as_ref();
    let mut reader = create_pcd_reader(input_path)?;
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
        let Some([x, y, z]) = point?.to_xyz::<f32>() else {
            bail!(
                "the file {} misses one of x, y or z field",
                input_path.display()
            );
        };

        let [x, y, z] = transform_point([x, y, z], tf);

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

fn newslab_pcd_to_libpcl_pcd<PI, PO>(
    input_path: PI,
    output_path: PO,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
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

        // Transform points
        let [x, y, z] = transform_point([x, y, z], tf);

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
    start: StartFrame,
    end: EndFrame,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    use FormatKind as F;
    use ReturnMode as R;

    let num_frames = count_frames_in_velodyne_pcap(input_file.as_ref(), model, mode)?;

    let start = match start {
        StartFrame::Forward(count) => count - 1,
        StartFrame::Backward(count) => {
            let Some(end) = num_frames.checked_sub(count) else {
                bail!("--start position is out of bound");
            };
            end
        }
    };
    let end = match end {
        EndFrame::Forward(count) => {
            ensure!(count <= num_frames, "--end position is out of bound");
            count
        }
        EndFrame::Backward(count) => {
            let Some(end) = (num_frames + 1).checked_sub(count) else {
                bail!("--end position is out of bound");
            };
            end
        }
        EndFrame::Count(count) => {
            let end = start + count;
            ensure!(count <= num_frames, "--end position is out of bound");
            end
        }
    };
    let Some(count) = end.checked_sub(start) else {
        bail!("--start position must go before --end position");
    };

    // closures
    let map_measurement = |measurement: Measurement| {
        let [x, y, z] = measurement.xyz;
        [
            x.as_meters() as f32,
            y.as_meters() as f32,
            z.as_meters() as f32,
        ]
    };

    let map_point_single = |point: PointS| transform_point(map_measurement(point.measurement), tf);
    let map_point_dual = |point: PointD| {
        let MeasurementDual {
            strongest: strongest_measure,
            last: last_measure,
        } = point.measurements;
        let strongest_point = transform_point(map_measurement(strongest_measure), tf);
        let last_point = transform_point(map_measurement(last_measure), tf);
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

    let mut frames = frame_xyz_iter_from_file(config, input_file)?
        .enumerate()
        .skip(start)
        .take(count);

    match mode.0 {
        R::Strongest => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file = strongest_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_libpcl_pcd_file_single(points, pcd_file, width, 16)?;
                    }
                    F::Single32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_libpcl_pcd_file_single(points, pcd_file, width, 32)?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
        R::Last => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file = last_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_libpcl_pcd_file_single(points, pcd_file, width, 16)?;
                    }
                    F::Single32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_single);
                        create_libpcl_pcd_file_single(points, pcd_file, width, 32)?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
        R::Dual => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.pcd", index);
                let pcd_file_strongest = strongest_output_dir.join(&file_name);
                let pcd_file_last = last_output_dir.join(&file_name);

                match frame? {
                    F::Dual16(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_libpcl_pcd_file_dual(
                            points,
                            pcd_file_strongest,
                            pcd_file_last,
                            width,
                            16,
                        )?;
                    }
                    F::Dual32(frame) => {
                        let width = frame.firings.len();
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_libpcl_pcd_file_dual(
                            points,
                            pcd_file_strongest,
                            pcd_file_last,
                            width,
                            32,
                        )?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
    }

    Ok(())
}

fn velodyne_pcap_to_raw_bin<I, O>(
    input_file: I,
    output_dir: O,
    model: ProductID,
    mode: VelodyneReturnMode,
    start: StartFrame,
    end: EndFrame,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    use FormatKind as F;
    use ReturnMode as R;

    let num_frames = count_frames_in_velodyne_pcap(input_file.as_ref(), model, mode)?;

    let start = match start {
        StartFrame::Forward(count) => count - 1,
        StartFrame::Backward(count) => {
            let Some(end) = num_frames.checked_sub(count) else {
                bail!("--start position is out of bound");
            };
            end
        }
    };
    let end = match end {
        EndFrame::Forward(count) => {
            ensure!(count <= num_frames, "--end position is out of bound");
            count
        }
        EndFrame::Backward(count) => {
            let Some(end) = (num_frames + 1).checked_sub(count) else {
                bail!("--end position is out of bound");
            };
            end
        }
        EndFrame::Count(count) => {
            let end = start + count;
            ensure!(count <= num_frames, "--end position is out of bound");
            end
        }
    };
    let Some(count) = end.checked_sub(start) else {
        bail!("--start position must go before --end position");
    };

    // closures
    let map_measurement = |measurement: Measurement| {
        let [x, y, z] = measurement.xyz;
        [
            x.as_meters() as f32,
            y.as_meters() as f32,
            z.as_meters() as f32,
        ]
    };

    let map_point_single = |point: PointS| transform_point(map_measurement(point.measurement), tf);
    let map_point_dual = |point: PointD| {
        let MeasurementDual {
            strongest: strongest_measure,
            last: last_measure,
        } = point.measurements;
        let strongest_point = transform_point(map_measurement(strongest_measure), tf);
        let last_point = transform_point(map_measurement(last_measure), tf);
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

    let mut frames = frame_xyz_iter_from_file(config, input_file)?
        .enumerate()
        .skip(start)
        .take(count);

    match mode.0 {
        R::Strongest => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.bin", index);
                let bin_file = strongest_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let points = frame.into_point_iter().map(map_point_single);
                        create_raw_bin_file_single(points, bin_file)?;
                    }
                    F::Single32(frame) => {
                        let points = frame.into_point_iter().map(map_point_single);
                        create_raw_bin_file_single(points, bin_file)?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
        R::Last => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.bin", index);
                let bin_file = last_output_dir.join(file_name);

                match frame? {
                    F::Single16(frame) => {
                        let points = frame.into_point_iter().map(map_point_single);
                        create_raw_bin_file_single(points, bin_file)?;
                    }
                    F::Single32(frame) => {
                        let points = frame.into_point_iter().map(map_point_single);
                        create_raw_bin_file_single(points, bin_file)?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
        R::Dual => {
            frames.try_for_each(|(index, frame)| {
                let file_name = format!("{:06}.bin", index);
                let bin_file_strongest = strongest_output_dir.join(&file_name);
                let bin_file_last = last_output_dir.join(&file_name);

                match frame? {
                    F::Dual16(frame) => {
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_raw_bin_file_dual(points, bin_file_strongest, bin_file_last)?;
                    }
                    F::Dual32(frame) => {
                        let points = frame.into_point_iter().map(map_point_dual);
                        create_raw_bin_file_dual(points, bin_file_strongest, bin_file_last)?;
                    }
                    _ => unreachable!(),
                }

                eyre::Ok(())
            })?;
        }
    }

    Ok(())
}

fn pcd_file_raw_bin_file<I, O>(
    input_file: I,
    output_file: O,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let input_file = input_file.as_ref();
    let reader = create_pcd_reader(input_file)?;
    let mut writer = RawBinWriter::from_path(output_file)?;

    let intensity_field = reader
        .meta()
        .field_defs
        .fields
        .iter()
        .enumerate()
        .find(|(_, field)| field.name == "intensity");

    let intensity_idx = match intensity_field {
        Some((idx, field)) => {
            if field.count == 1 {
                Some(idx)
            } else {
                eprintln!("the intensity field is not a single number");
                None
            }
        }
        None => None,
    };

    for point in reader {
        let point = point?;

        let intensity = match intensity_idx {
            Some(idx) => {
                let val = match &point.0[idx] {
                    pcd_rs::Field::I8(vec) => vec[0] as f32,
                    pcd_rs::Field::I16(vec) => vec[0] as f32,
                    pcd_rs::Field::I32(vec) => vec[0] as f32,
                    pcd_rs::Field::U8(vec) => vec[0] as f32,
                    pcd_rs::Field::U16(vec) => vec[0] as f32,
                    pcd_rs::Field::U32(vec) => vec[0] as f32,
                    pcd_rs::Field::F32(vec) => vec[0],
                    pcd_rs::Field::F64(vec) => vec[0] as f32,
                };
                Some(val)
            }
            None => None,
        };
        let intensity = intensity.unwrap_or(0.0);

        let Some([x, y, z]) = point.to_xyz::<f32>() else {
            bail!(
                "the file {} misses one of x, y or z field",
                input_file.display()
            );
        };

        let [x, y, z] = transform_point([x, y, z], tf);
        writer.push([x, y, z, intensity])?;
    }
    writer.finish()?;

    Ok(())
}

fn pcd_dir_raw_bin_dir<I, O>(
    input_dir: I,
    output_dir: O,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let output_dir = output_dir.as_ref();
    fs::create_dir(output_dir)
        .with_context(|| format!("unable to create directory {}", output_dir.display()))?;

    let input_paths: Vec<_> = input_dir
        .as_ref()
        .read_dir()?
        .filter_map(|entry| {
            macro_rules! skip {
                () => {
                    {
                        return None;
                    }
                };
                ($($tokens:tt)*) => {
                    {
                        eprintln!("Error: {}", format_args!($($tokens)*));
                        return None
                    }
                };
            }

            let path = match entry {
                Ok(entry) => entry.path(),
                Err(err) => skip!("{err}"),
            };

            match path.extension() {
                Some(ext) => {
                    if ext != "pcd" {
                        skip!();
                    }
                }
                None => skip!(),
            }

            match path.canonicalize() {
                Ok(path) => {
                    if !path.is_file() {
                        skip!();
                    }
                }
                Err(err) => skip!("Unable to read {}: {err}", path.display()),
            };

            Some(path)
        })
        .collect();

    input_paths.par_iter().for_each(|input_file| {
        macro_rules! skip {
            () => {
                {
                    return;
                }
            };
            ($($tokens:tt)*) => {
                {
                    eprintln!("Error: {}", format_args!($($tokens)*));
                    return;
                }
            };
        }

        let Some(stem) = input_file.file_stem() else {
            skip!("unable to convert {}", input_file.display());
        };
        let Some(stem) = stem.to_str() else {
            skip!("unable to convert {}", input_file.display());
        };

        let output_file = output_dir.join(format!("{stem}.bin"));

        if let Err(err) = pcd_file_raw_bin_file(input_file, &output_file, tf) {
            skip!("unable to write {}: {err}", output_file.display());
        }
    });
    Ok(())
}

fn bin_file_to_libpcl_pcd_file<I, O>(
    input_file: I,
    output_file: O,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let points: Vec<_> = load_bin_iter(input_file)?
        .map(|p| -> Result<_> {
            let BinPoint { x, y, z, .. } = p?;
            let [x, y, z] = transform_point([x, y, z], tf);
            Ok([x, y, z])
        })
        .try_collect()?;

    let num_points = points.len();
    create_libpcl_pcd_file_single(points, output_file, num_points, 1)?;
    Ok(())
}

fn bin_dir_to_libpcl_pcd_dir<I, O>(
    input_dir: I,
    output_dir: O,
    tf: Option<na::Isometry3<f32>>,
) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let output_dir = output_dir.as_ref();
    fs::create_dir(output_dir)
        .with_context(|| format!("unable to create directory {}", output_dir.display()))?;

    let input_paths: Vec<_> = input_dir
        .as_ref()
        .read_dir()?
        .filter_map(|entry| {
            macro_rules! skip {
                () => {
                    {
                        return None;
                    }
                };
                ($($tokens:tt)*) => {
                    {
                        eprintln!("Error: {}", format_args!($($tokens)*));
                        return None
                    }
                };
            }

            let path = match entry {
                Ok(entry) => entry.path(),
                Err(err) => skip!("{err}"),
            };

            match path.extension() {
                Some(ext) => {
                    if ext != "bin" {
                        skip!();
                    }
                }
                None => skip!(),
            }

            match path.canonicalize() {
                Ok(path) => {
                    if !path.is_file() {
                        skip!();
                    }
                }
                Err(err) => skip!("Unable to read {}: {err}", path.display()),
            };

            Some(path)
        })
        .collect();

    input_paths.par_iter().for_each(|input_file| {
        macro_rules! skip {
            () => {
                {
                    return;
                }
            };
            ($($tokens:tt)*) => {
                {
                    eprintln!("Error: {}", format_args!($($tokens)*));
                    return;
                }
            };
        }

        let points = match load_bin_iter(input_file) {
            Ok(points) => points,
            Err(err) => skip!("unable to read {}: {err}", input_file.display()),
        };

        let points: Result<Vec<_>> = points
            .map(|p| -> Result<_> {
                let BinPoint { x, y, z, .. } = p?;
                let [x, y, z] = transform_point([x, y, z], tf);
                Ok([x, y, z])
            })
            .collect();
        let points = match points {
            Ok(points) => points,
            Err(err) => skip!("unable to read {}: {err}", input_file.display()),
        };

        let Some(stem) = input_file.file_stem() else {
            skip!("unable to convert {}", input_file.display());
        };
        let Some(stem) = stem.to_str() else {
            skip!("unable to convert {}", input_file.display());
        };

        let output_file = output_dir.join(format!("{stem}.pcd"));

        let num_points = points.len();
        if let Err(err) = create_libpcl_pcd_file_single(points, &output_file, num_points, 1) {
            skip!("unable to write {}: {err}", output_file.display());
        };
    });

    Ok(())
}

fn transform_point<T>(point: [T; 3], tf: Option<na::Isometry3<T>>) -> [T; 3]
where
    T: na::RealField,
{
    match tf {
        Some(tf) => {
            let input = na::Point3::from(point);
            let output = tf * &input;
            output.into()
        }
        None => point,
    }
}

fn is_file<P>(path: P) -> Result<bool>
where
    P: AsRef<Path>,
{
    Ok(path.as_ref().canonicalize()?.is_file())
}
