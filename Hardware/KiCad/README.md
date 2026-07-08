# Hardware — KiCad Project

Custom 2-layer PCB (192 mm × 166 mm) built around a NUCLEO-G474RE evaluation
board, providing all sensing and excitation circuitry for the supercapacitor
health monitoring system.

## Contents (place your exported KiCad project here)

```
kicad/
├── Super_Capacitor_measurement.kicad_pro
├── Super_Capacitor_measurement.kicad_sch   (3 sheets: power/input, measurement, MCU/comms)
├── Super_Capacitor_measurement.kicad_pcb
├── Super_Capacitor_measurement.kicad_prl
├── gerbers/        # exported fab files (optional, generate before final commit)
├── 3d-renders/     # isometric top/bottom views
└── bom/            # bill of materials (csv/xlsx export)
```

> Commit the whole KiCad project folder as-is — KiCad manages internal file
> relationships (`.kicad_pro`, `.kicad_sch`, `.kicad_pcb`, `.kicad_prl`)
> and cherry-picking files can break the project when reopened.

## Key subsystems (see thesis Ch. 3 for full derivations)

- **Voltage divider network** — 8 series-cell nodes scaled into the
  0–3.3 V ADC range (node 8 direct, no divider needed).
- **Current sensing** — 2× INA241A1 (shunt-based, 20 V/V gain, 33 mΩ shunt)
  + 1× TMCS1108A3 (isolated Hall-effect, 0.2 V/A).
- **Controlled current sink** — DAC-driven IRL540N/FQA140N10 MOSFET,
  closed-loop regulated via a two-comparator (TLV3541) feedback network.
- **Status/UI** — 5 status LEDs, 2 toggle switches (mode select), 5 push
  buttons (S3–S7: emergency stop, start measurement, calibrate, arm passive
  discharge, start galvanostatic discharge).

## Layout notes

- RC filters on ADC inputs and op-amp/Hall sensor supply pins for noise
  rejection.
- Bottom-layer ground plane to reduce EMI coupling into high-impedance
  divider inputs.
- Kelvin connections at both shunt resistors.
- Short ADC input traces to minimise parasitic capacitance/crosstalk.

## Tooling

- **KiCad 9.0** or later.
- 3D renders exported via KiCad's built-in 3D viewer.

## Bill of Materials

See `bom/` for the full BOM export. Key parts: STM32G474RE (Nucleo-64),
Vishay 25 F supercapacitors ×40, INA241A1 ×2, TMCS1108A3, TLV3541 ×2,
IRL540N + FQA140N10 MOSFETs.
