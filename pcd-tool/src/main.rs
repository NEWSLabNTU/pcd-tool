mod convert;
mod info;
mod opts;
mod types;
mod utils;

use anyhow::Result;
use clap::Parser;
use opts::{Convert, Info, Opts};

fn main() -> Result<()> {
    let opts = Opts::parse();

    match opts {
        Opts::Info(Info { file }) => {
            crate::info::info(file)?;
        }
        Opts::Convert(Convert {
            input_path,
            output_path,
        }) => {
            crate::convert::convert(input_path, output_path)?;
        } /* Opts::DeviceTime { file } => {
           *     crate::utils::time(file)?;
           * } */
    }

    Ok(())
}
