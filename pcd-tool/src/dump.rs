mod tui;

use self::tui::{run_tui, Record, Value};
use crate::{
    opts::{Dump, VelodyneReturnMode},
    types::FileFormat,
    utils::{build_velodyne_config, guess_file_format},
};
use anyhow::{anyhow, Result};
use itertools::{chain, izip, Itertools};
use pcd_rs::{Field, FieldDef};
use std::path::Path;
use velodyne_lidar::{ProductID, ReturnMode};

pub fn dump(args: Dump) -> Result<()> {
    let Dump {
        input,
        format,
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
        F::LibpclPcd | F::NewslabPcd => dump_pcd(&input)?,
        F::VelodynePcap => {
            let velodyne_model =
                velodyne_model.ok_or_else(|| anyhow!("--velodyne-mode must be set"))?;
            let velodyne_return_mode = velodyne_return_mode
                .ok_or_else(|| anyhow!("--velodyne-return-mode must be set"))?;

            dump_velodyne_pcap(&input, velodyne_model, velodyne_return_mode)?
        }
    }

    Ok(())
}

fn dump_pcd<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let reader = pcd_rs::DynReader::open(path)?;

    let header: Vec<_> = reader
        .meta()
        .field_defs
        .iter()
        .flat_map(|field| {
            let FieldDef {
                ref name, count, ..
            } = *field;

            if count == 1 {
                vec![name.clone()]
            } else {
                (1..=count).map(|idx| format!("{name}#{idx}")).collect()
            }
        })
        .collect();

    let data: Vec<_> = reader
        .map(|record| -> Result<_> {
            let record = record?;

            let values: Vec<Value> = record
                .0
                .iter()
                .flat_map(|field| {
                    let values: Vec<_> = match field {
                        Field::I8(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::I16(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::I32(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::U8(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::U16(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::U32(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::F32(values) => values.iter().cloned().map(Value::from).collect(),
                        Field::F64(values) => values.iter().cloned().map(Value::from).collect(),
                    };

                    values
                })
                .collect();

            Ok(Record(values))
        })
        .try_collect()?;

    run_tui(header, data)?;
    Ok(())
}

fn dump_velodyne_pcap<P>(path: P, model: ProductID, mode: VelodyneReturnMode) -> Result<()>
where
    P: AsRef<Path>,
{
    let config = build_velodyne_config(model, mode.0)?;
    let frames = velodyne_lidar::iter::frame_xyz_iter_from_file(config, path)?;

    let header: Vec<String> = {
        let prefix = &[
            "frame",
            "laser_id",
            "time",
            "azimuth (deg)",
            "distance (m)",
            "intensity",
        ];
        let suffix: &[_] = match mode.0 {
            ReturnMode::Strongest => &["x (m, strongest)", "y (m, strongest)", "z (m, strongest)"],
            ReturnMode::Last => &["x (m, last)", "y (m, last)", "z (m, last)"],
            ReturnMode::Dual => &[
                "x (m, strongest)",
                "y (m, strongest)",
                "z (m, strongest)",
                "x (m, last)",
                "y (m, last)",
                "z (m, last)",
            ],
        };

        chain!(prefix, suffix)
            .map(|title| title.to_string())
            .collect()
    };

    let data: Vec<Record> = izip!(1.., frames)
        .map(|(frame_id, frame)| -> Result<_> {
            let frame = frame?;

            let points: Vec<Record> = frame
                .into_firing_iter()
                .flat_map(|firing| {
                    firing.into_point_iter().map(|point| -> Vec<Value> {
                        use velodyne_lidar::types::{
                            measurements::{Measurement, MeasurementDual},
                            point::{Point as P, PointD, PointS},
                        };

                        match point {
                            P::Single(point) => {
                                let PointS {
                                    laser_id,
                                    time,
                                    azimuth,
                                    measurement:
                                        Measurement {
                                            distance,
                                            intensity,
                                            xyz: [x, y, z],
                                        },
                                } = point;

                                vec![
                                    frame_id.into(),
                                    laser_id.into(),
                                    format!("{time:?}").into(),
                                    azimuth.as_degrees().into(),
                                    distance.as_meters().into(),
                                    intensity.into(),
                                    x.as_meters().into(),
                                    y.as_meters().into(),
                                    z.as_meters().into(),
                                ]
                            }
                            P::Dual(point) => {
                                let PointD {
                                    laser_id,
                                    time,
                                    azimuth,
                                    measurements: MeasurementDual { strongest, last },
                                } = point;

                                vec![
                                    frame_id.into(),
                                    laser_id.into(),
                                    format!("{time:?}").into(),
                                    azimuth.as_degrees().into(),
                                    strongest.distance.as_meters().into(),
                                    strongest.intensity.into(),
                                    strongest.xyz[0].as_meters().into(),
                                    strongest.xyz[1].as_meters().into(),
                                    strongest.xyz[2].as_meters().into(),
                                    last.distance.as_meters().into(),
                                    last.intensity.into(),
                                    last.xyz[0].as_meters().into(),
                                    last.xyz[1].as_meters().into(),
                                    last.xyz[2].as_meters().into(),
                                ]
                            }
                        }
                    })
                })
                .map(Record)
                .collect();

            Ok(points)
        })
        .flatten_ok()
        .try_collect()?;

    run_tui(header, data)?;
    Ok(())
}
