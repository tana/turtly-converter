mod dewarp;
mod gcode;
mod tessellation;
mod transform;
mod utils;
mod warp;

use anyhow::Result;
use clap::{Parser, Subcommand};
use dewarp::DewarpArgs;
use warp::WarpArgs;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Warp(WarpArgs),
    Dewarp(DewarpArgs),
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Warp(args) => warp::command_main(args),
        Commands::Dewarp(args) => dewarp::command_main(args),
    }
}
