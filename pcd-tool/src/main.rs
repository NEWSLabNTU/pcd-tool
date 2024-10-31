mod convert;
mod dump;
mod info;
mod io;
mod opts;
mod show;
mod types;
mod utils;

use clap::Parser;
use eyre::Result;
use opts::{Info, Opts};

fn main() -> Result<()> {
    let opts = Opts::parse();

    match opts {
        Opts::Info(Info { file }) => {
            crate::info::info(file)?;
        }
        Opts::Convert(args) => {
            crate::convert::convert(args)?;
        }
        Opts::Dump(args) => {
            crate::dump::dump(args)?;
        }
        Opts::Show(args) => {
            crate::show::show(args)?;
        }
    }

    Ok(())
}
