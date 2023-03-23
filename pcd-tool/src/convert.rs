use crate::types::{FileFormat, LidarType};
use anyhow::{anyhow, bail, Context, Result};
use console::Term;
use dialoguer::{Input, Select};
use iterator_ext::IteratorExt;
use lidar_utils::velodyne::{self, FrameConverter as _};
use num_traits::ToPrimitive;
use pcd_format::{LibpclPoint, NewslabV1Point};
use std::{f64, fs, iter, path::Path};

pub fn convert<PI, PO>(input_path: PI, output_path: PO) -> Result<()>
where
    PI: AsRef<Path>,
    PO: AsRef<Path>,
{
    let input_path = input_path.as_ref();
    let output_path = output_path.as_ref();

    let input_format = guess_file_format(input_path).ok_or_else(|| {
        anyhow!(
            "cannot guess format of input file '{}'",
            input_path.display()
        )
    })?;

    let output_format = guess_file_format(output_path);

    match (input_format, output_format) {
        (FileFormat::LibpclPcd, Some(FileFormat::NewslabPcd)) => {
            libpcl_pcd_to_newslab_pcd(input_path, output_path)?;
        }
        (FileFormat::NewslabPcd, Some(FileFormat::LibpclPcd)) => {
            newslab_pcd_to_libpcl_pcd(input_path, output_path)?;
        }
        (FileFormat::Pcap, _) => {
            if output_format.is_some() {
                eprintln!(
                    "Warning: the output path '{}' is treated as a directory",
                    output_path.display()
                );
            }
            pcap_to_pcd(input_path, output_path)?;
        }
        (FileFormat::LibpclPcd, None) | (FileFormat::NewslabPcd, None) => {
            bail!("You must specify a output file when transforming pcd/newslab-pcd");
        }

        (FileFormat::LibpclPcd, Some(FileFormat::LibpclPcd))
        | (FileFormat::NewslabPcd, Some(FileFormat::NewslabPcd)) => {
            bail!("Nothing to be done");
        }
        (_, Some(FileFormat::Pcap)) => {
            bail!("converting to pcap file is not supported");
        }
    }

    Ok(())
}

fn guess_file_format<P>(file: P) -> Option<FileFormat>
where
    P: AsRef<Path>,
{
    let file = file.as_ref();
    let file_name = file.file_name()?.to_str()?;

    let format = if file_name.ends_with(".newslab.pcd") {
        FileFormat::NewslabPcd
    } else if file_name.ends_with(".pcd") {
        FileFormat::LibpclPcd
    } else if file_name.ends_with(".pcap") {
        FileFormat::Pcap
    } else {
        return None;
    };

    Some(format)
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
        let azimuthal_angle = y.atan2(x);
        let polar_angle = (x.powi(2) + y.powi(2) / z).atan();
        let vertical_angle = f64::consts::FRAC_PI_2 - polar_angle;

        let point = NewslabV1Point {
            x,
            y,
            z,
            distance,
            azimuthal_angle,
            vertical_angle,
            intensity: 0.0,
            laser_id: 0,
            timestamp_ns: 0,
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

pub fn pcap_to_pcd<I, O>(input_file: I, output_dir: O) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let input_file = input_file.as_ref();
    let output_dir = output_dir.as_ref();

    // Get some user informations
    let term = Term::stdout();
    let lidar_type = {
        let choice = Select::new()
            .with_prompt("What kind of lidar was used to collect this pcap?")
            .items(&["Vlp16", "Vlp32"])
            .default(1)
            .interact_on(&term)?;

        match choice {
            0 => LidarType::Vlp16,
            1 => LidarType::Vlp32,
            _ => unreachable!(),
        }
    };
    let target_type = {
        let choice = Select::new()
            .with_prompt("What type of file do you want to convert?")
            .items(&["standard pcd", "newslab pcd"])
            .default(0)
            .interact_on(&term)?;

        match choice {
            0 => FileFormat::LibpclPcd,
            1 => FileFormat::NewslabPcd,
            _ => unreachable!(),
        }
    };
    let start_number: usize = Input::new()
        .with_prompt("Which frame number do you want to transform from?")
        .default(0)
        .interact()?;
    let number_of_frames: usize = Input::new()
        .with_prompt("How many frames do you want to transform (0 for all)?")
        .default(0)
        .interact()?;

    // prepare pcap capture handlers
    let mut cap = pcap::Capture::from_file(input_file)
        .with_context(|| format!("unable to open file '{}'", input_file.display()))?;
    let mut frame_converter = match lidar_type {
        LidarType::Vlp16 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::puck_hires_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
        LidarType::Vlp32 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::vlp_32c_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
    };

    fs::create_dir_all(output_dir)
        .with_context(|| format!("unable to create directory {}", output_dir.display()))?;

    match target_type {
        FileFormat::LibpclPcd => {
            use velodyne::DynamicReturnPoints as DP;

            // Create a packet iterator
            let packet_iter = iter::from_fn(|| -> Option<Option<_>> {
                let raw_packet = cap.next().ok()?;
                let velodyne_packet = velodyne::DataPacket::from_pcap(&raw_packet).ok();
                Some(velodyne_packet)
            })
            .flatten();

            // Cconvert packets to frames
            let frame_iter = packet_iter
                .map(Ok)
                .try_flat_map(|packet| frame_converter.convert::<velodyne::DataPacket>(packet))
                .enumerate()
                .map(|(index, frame)| anyhow::Ok((index, frame?)));

            // Restrict the range of frame indices
            let frame_iter = frame_iter.skip(start_number);
            let mut frame_iter: Box<dyn Iterator<Item = Result<(usize, DP)>> + Sync + Send> =
                if number_of_frames > 0 {
                    Box::new(frame_iter.take(number_of_frames))
                } else {
                    Box::new(frame_iter)
                };

            frame_iter.try_for_each(|args| -> Result<_> {
                use uom::si::length::meter;

                let (index, frame) = args?;
                println!("transforming frame number {}", index);

                let points: Vec<_> = match frame {
                    DP::Single(points) => points
                        .into_iter()
                        .map(|point| point.data.position)
                        .map(|[x, y, z]| [x.get::<meter>(), y.get::<meter>(), z.get::<meter>()])
                        .collect(),
                    DP::Dual(points) => points
                        .into_iter()
                        .map(|point| point.strongest_return_data.position)
                        .map(|[x, y, z]| [x.get::<meter>(), y.get::<meter>(), z.get::<meter>()])
                        .collect(),
                };

                let pcd_file = output_dir.join(format!("{:06}.pcd", index));
                save_pcd(points, &pcd_file, pcd_rs::DataKind::Ascii).with_context(|| {
                    format!("failed to create the pcd file '{}'", pcd_file.display())
                })?;

                Ok(())
            })?;
        }
        FileFormat::NewslabPcd => {
            todo!()
        }
        _ => unreachable!(),
    }
    Ok(())
}

fn save_pcd<P, T>(points: Vec<[T; 3]>, pcd_file: P, data_kind: pcd_rs::DataKind) -> Result<()>
where
    P: AsRef<Path>,
    T: ToPrimitive,
{
    let mut writer = pcd_rs::WriterInit {
        width: points.len() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind,
        schema: None,
    }
    .create(pcd_file)?;

    points.into_iter().try_for_each(|[x, y, z]| -> Result<_> {
        let point = LibpclPoint {
            x: x.to_f32().unwrap(),
            y: y.to_f32().unwrap(),
            z: z.to_f32().unwrap(),
            rgb: 0,
        };
        writer.push(&point)?;
        Ok(())
    })?;

    writer.finish()?;

    Ok(())
}
