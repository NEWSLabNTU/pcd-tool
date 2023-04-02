use crate::{opts::Dump, types::FileFormat, utils::guess_file_format};
use anyhow::{anyhow, Result};
use std::path::Path;

pub fn dump(args: Dump) -> Result<()> {
    let Dump { input, format } = args;

    let format = match format {
        Some(format) => format,
        None => guess_file_format(&input)
            .ok_or_else(|| anyhow!("unable to guess file format of '{}'", input.display()))?,
    };

    use FileFormat as F;
    match format {
        F::LibpclPcd | F::NewslabPcd => dump_pcd(&input)?,
        F::VelodynePcap => dump_velodyne_pcap(&input)?,
    }

    Ok(())
}

fn dump_pcd<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let reader = pcd_rs::DynReader::open(path)?;

    for record in reader {
        let record = record?;
        println!("{record:?}");
    }

    Ok(())
}

fn dump_velodyne_pcap<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    todo!();
}
