# traffic-and-vehicle-simulator

Small hobby project for playing around with traffic simulation in C++20.

Cars drive either on a simple four-way intersection or on road networks
imported from OpenStreetMap. Runs are deterministic (same seed, same result),
can be recorded and replayed, and there's an SDL2 viewer to watch them.
Everything runs on the CPU for now, GPU acceleration is something I want to
try later.

## Build

On Windows with the Visual Studio C++ build tools:

```powershell
.\scripts\build.ps1            # add -Headless to skip the viewer
```

CMake downloads the dependencies (SDL2, nlohmann/json, tinyxml2, stb_truetype).
The programs end up in `build/release/bin`.

## Run

```powershell
.\scripts\run.ps1                             # viewer with the default intersection
.\scripts\run.ps1 --map maps/demo_grid.osm    # small OSM test map
.\scripts\run.ps1 -Headless --csv out.csv     # no window, write vehicle states to CSV
```

`--help` shows all options, and `H` shows the controls in the viewer.
`sim_bench` runs batch benchmarks and `sim_validate` checks the vehicle
dynamics against the traces in `references/`.
