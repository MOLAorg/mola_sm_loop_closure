# mola_sm_loop_closure
Offline tool for loop-closure on simple-maps

## Georeferencing a map

```bash
# Create the mm:
sm2mm -i INPUT_WITH_GPS.simplemap \
 -o MAP.mm \
 -p pipeline.yaml

# georeference it:
mola-sm-georeferencing -i INPUT_WITH_GPS.simplemap --write-into MAP.mm
```

# Loop closure detection

Example usage:

```bash
mola-sm-lc-cli \
 --pipeline src/mola_sm_loop_closure/pipelines/loop-closure-lidar3d.yaml \
 -i map_KAIST01_gps.simplemap \
 -o map_KAIST01_corrected.simplemap
```

# License
Copyright (C) 2018-2025 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).
