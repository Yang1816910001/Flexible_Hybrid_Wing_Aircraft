# Coupling Model V1.2 — Minimal Delivery

This folder is a self-contained runtime package for the current flexible-aircraft coupling model. It intentionally excludes structural-model generation, XFOIL generation, diagnostics, historical models, cached trim results and generated outputs.

## Requirements

- MATLAB R2025b (25.2) or a compatible release
- Simulink
- Control System Toolbox
- Optimization Toolbox
- Simulink Control Design

The package was dependency-checked against MATLAB 25.2. A different release may upgrade the SLX files when they are opened; commit such upgrades separately.

## Quick verification after cloning

Open MATLAB, change to this repository folder, and run:

```matlab
run_delivery_smoke_test
```

The script checks all required data files, runs `initial_main_V1_1`, resolves the linked libraries and advances `aerodynamic_model_V1_2_Wrench` for 0.02 s without saving the model. A successful run ends with `PASS`.

## Main workflows

Single-condition sinusoidal pull-up/load visualization:

```matlab
cfg = sine_load_case_settings();
cfg.output.figure_visible = true;
result = generate_pullup_load_cloud(cfg);
```

Elevator frequency sweep for the four monitored root sections:

```matlab
sweep_result = generate_section_load_frequency_sweep();
```

Direct rigid-flexible trim and optional explicit Wagner restoration:

```matlab
mdl = 'aerodynamic_model_V1_2_Wrench';
trim_result = trim_flexible_direct_findop(0.3, struct(), mdl);
wagner_result = restore_wagner_from_flexible_direct(0.3, mdl);
```

`generate_pullup_load_cloud` already performs the trim/Wagner workflow according to `sine_load_case_settings`; the explicit calls above are for manual inspection.

## Configuration locations

- `sine_load_case_settings.m`: single-condition flight condition, initialization, solver, Wagner switch, control command and output settings.
- `generate_section_load_frequency_sweep.m`: sweep frequencies, cycles, amplitude and per-case output settings.
- `initial_main_V1_1.m`: low-level model initialization and default trim seeds.

Generated plots, videos, MAT results and CSV summaries are written under `outputs/`, which is ignored by Git.

## Delivered structural data

Exactly four structural CSV files are included in `Library/`:

1. `nodes.csv` — node geometry, mass and inertia data.
2. `elements.csv` — structural connectivity.
3. `frequencies.csv` — retained modal frequencies.
4. `modal_matrix_15.csv` — retained modal matrix.

These are runtime inputs. The scripts and tools that generated them are deliberately not included.

## Other runtime data

- `data/propeller/CFD_HOVER_2D.mat` and `CFD_DATA_3D.mat`: propulsion lookup tables copied into the repository so no sibling project is required.
- `polars/naca2412_Re6p81e6_Ncrit9.pol`: clean airfoil polar.
- `polars/flap_polars.mat`: control-surface increment tables.
- `Library/*.slx`: linked Simulink libraries required by the plant model.

## Repository policy

Do not commit `outputs/`, `slprj/`, `*.slxc`, autosave files or run logs. The supplied `.gitignore` already excludes them. Do commit changes to the four structural CSV files, lookup MAT files, model, scripts and libraries when those changes are intentional.
