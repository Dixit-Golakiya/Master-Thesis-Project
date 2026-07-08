# Simulation — LTspice

Pre-PCB verification of the critical signal conditioning circuits, done in
**LTspice v24.1.5**, before schematic capture in KiCad (thesis Section 3.2).

## Contents (place your exported .asc/.plt files here)

```
ltspice/
├── capacitor_bank.asc          # 5-parallel × 8-series supercap bank model
├── voltage_divider.asc         # 8-node resistive divider network
├── dac_current_sink.asc        # DAC-driven constant-current sink
├── esr_discharge_model.asc     # First-order RC discharge model (ESR + C)
└── results/                    # exported plots / screenshots (optional)
```

> `.asc` files (schematic + values) are the important artifact and should be
> committed. `.raw` simulation output files are typically large binaries —
> regenerate on demand rather than commit, unless exact result reproduction
> is needed.

## What each simulation verifies

- **Capacitor bank** (`capacitor_bank.asc`): 5P-8S topology of 40× Vishay 25 F
  cells with passive balancing resistors (470 Ω). Confirms series/parallel
  scaling: `Cseries = 3.125 F`, `Ctotal = 15.625 F`, `ESRtotal = 54.4 mΩ`,
  `Vmax = 24 V`.
- **Voltage divider** (`voltage_divider.asc`): Confirms max nonlinearity error
  of 0.0037 mV across all 7 divider nodes (well within ADC resolution).
- **DAC-driven current sink** (`dac_current_sink.asc`): Confirms stable
  ≈2 A discharge current with ~300 µs settling time and negligible current
  ripple (ΔI = 2.5 µA → ΔESR error ≈ 41.25 nΩ, i.e. 0.0001% of nominal ESR).
- **ESR/discharge model** (`esr_discharge_model.asc`): First-order equivalent
  circuit (series ESR + ideal C) reproducing the ohmic-drop + linear-decline
  discharge waveform used to validate the firmware's extraction algorithm.

## Tooling

- **LTspice XVII**, version 24.1.5 or later (Windows/macOS; runs under Wine
  on Linux).
