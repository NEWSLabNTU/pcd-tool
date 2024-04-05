use crate::{opts::VelodyneReturnMode, types::BinPoint, utils::build_velodyne_config};
use anyhow::{Context, Result};
use byteorder::{LittleEndian, ReadBytesExt};
use pcd_format::LibpclPoint;
use pcd_rs::DataKind;
use std::{
    fs::File,
    io::{self, prelude::*, BufReader, BufWriter},
    path::Path,
};
use velodyne_lidar::{iter::frame_xyz_iter_from_file, ProductID};

pub struct RawBinWriter {
    writer: Option<BufWriter<File>>,
}

impl RawBinWriter {
    pub fn from_path(path: impl AsRef<Path>) -> io::Result<Self> {
        let writer = BufWriter::new(File::create(path)?);
        Ok(Self {
            writer: Some(writer),
        })
    }

    pub fn push(&mut self, point: [f32; 4]) -> io::Result<()> {
        let writer = self.writer.as_mut().unwrap();

        for val in point {
            let buf = val.to_le_bytes();
            writer.write_all(&buf)?;
        }
        Ok(())
    }

    pub fn finish(mut self) -> io::Result<()> {
        self.writer.take().unwrap().flush()
    }
}

impl Drop for RawBinWriter {
    fn drop(&mut self) {
        if let Some(mut writer) = self.writer.take() {
            writer.flush().unwrap();
        }
    }
}

pub fn load_bin<P>(path: P) -> Result<Vec<BinPoint>>
where
    P: AsRef<Path>,
{
    let pcd_path = path.as_ref();

    let mut input = BufReader::new(
        File::open(pcd_path)
            .with_context(|| format!("Failed to open file {}", pcd_path.display()))?,
    );

    macro_rules! read_f32 {
        () => {{
            input.read_f32::<LittleEndian>()
        }};
    }

    macro_rules! try_read_f32 {
        () => {{
            let mut buf = [0u8; 4];
            let cnt = input.read(&mut buf)?;

            match cnt {
                4 => Ok(Some(f32::from_le_bytes(buf))),
                0 => Ok(None),
                cnt => Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    format!("Truncated f32 found. Expect 4 bytes, but read {cnt} bytes."),
                )),
            }
        }};
    }

    let mut points = vec![];

    loop {
        let Some(x) = try_read_f32!()? else {
            break;
        };
        let y = read_f32!()?;
        let z = read_f32!()?;
        let intensity = read_f32!()?;

        let point = BinPoint { x, y, z, intensity };
        points.push(point);
    }

    Ok(points)
}

pub fn create_pcd_file_single<P, I>(
    points: I,
    pcd_file: P,
    width: usize,
    height: usize,
) -> Result<()>
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

pub fn create_pcd_file_dual<P1, P2, I>(
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

pub fn count_frames_in_velodyne_pcap<P>(
    path: P,
    model: ProductID,
    mode: VelodyneReturnMode,
) -> Result<usize>
where
    P: AsRef<Path>,
{
    let config = build_velodyne_config(model, mode.0)?;
    let count = frame_xyz_iter_from_file(config, path)?.count();
    Ok(count)
}

pub fn create_pcd_reader<P>(
    input_path: P,
) -> Result<pcd_rs::Reader<pcd_rs::DynRecord, BufReader<File>>>
where
    P: AsRef<Path>,
{
    pcd_rs::Reader::open(input_path)
}
