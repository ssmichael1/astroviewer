/// Generate a tetra3 solver database for the viewer.
///
/// Run with:
///   cargo test --release --features starsolve --test generate_db -- --nocapture
///
/// Reads:  ../tetra3rs/data/gaia_merged.bin  (Gaia DR3 catalog)
/// Writes: data/solver_5_40.rkyv              (solver database, 5°–40° FOV)
#[cfg(feature = "starsolve")]
#[test]
fn generate_solver_database() {
    use std::time::Instant;
    use tetra3::{GenerateDatabaseConfig, SolverDatabase};

    let catalog_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tetra3rs/data/gaia_merged.bin"
    );
    let output_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/data");
    let output_path = format!("{}/solver_5_40.rkyv", output_dir);

    std::fs::create_dir_all(output_dir).expect("create data dir");

    let config = GenerateDatabaseConfig {
        max_fov_deg: 40.0,
        min_fov_deg: Some(5.0),
        epoch_proper_motion_year: Some(2025.0),
        verification_stars_per_fov: 500,
        ..Default::default()
    };

    println!("Generating database: FOV 5°–40°, Gaia catalog");
    println!("  catalog: {}", catalog_path);
    println!("  output:  {}", output_path);

    let t0 = Instant::now();
    let db = SolverDatabase::generate_from_gaia(catalog_path, &config)
        .expect("database generation failed");
    let gen_time = t0.elapsed();

    println!("Generation took {:.1}s", gen_time.as_secs_f64());
    println!("  patterns: {}", db.props.num_patterns);
    println!("  stars:    {}", db.star_vectors.len());

    let t1 = Instant::now();
    db.save_to_file(&output_path).expect("save failed");
    let save_time = t1.elapsed();

    let file_size = std::fs::metadata(&output_path).unwrap().len();
    println!(
        "Saved in {:.1}s, file size: {:.1} MB",
        save_time.as_secs_f64(),
        file_size as f64 / 1e6
    );
}
