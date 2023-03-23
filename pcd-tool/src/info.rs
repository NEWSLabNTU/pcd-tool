use anyhow::{ensure, Result};
use std::path::Path;

pub fn info(file: impl AsRef<Path>) -> Result<()> {
    let file = file.as_ref();

    ensure!(
        file.extension().map(|ext| ext == "pcd").unwrap_or(false),
        "file name must ends with '.pcd', but get '{}'",
        file.display()
    );

    let reader = pcd_rs::DynReader::open(file)?;
    let fields = &reader.meta().field_defs;

    println!("name\ttype\tcount");
    fields.iter().for_each(|field| {
        let pcd_rs::FieldDef {
            ref name,
            kind,
            count,
        } = *field;

        println!("{}\t{:?}\t{}", name, kind, count);
    });

    Ok(())
}
