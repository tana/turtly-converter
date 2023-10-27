mod warp;
mod tessellation;
mod vector;

use clap::{Parser, Subcommand};
use warp::WarpArgs;
use anyhow::Result;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    Warp(WarpArgs),
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Warp(args) => warp::command_main(args),
    }
}