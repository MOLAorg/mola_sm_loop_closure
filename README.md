# mola_sm_loop_closure
Offline tool for loop-closure on simple-maps


Example usage:

```bash
mola-sm-lc-cli \
 --pipeline src/mola_sm_loop_closure/pipelines/loop-closure-lidar3d.yaml \
 -i map_KAIST01_gps.simplemap \
 -o map_KAIST01_corrected.simplemap
```

```bash
# Create the mm:
sm2mm -i INPUT_WITH_GPS.simplemap \
 -o MAP.mm \
 -p pipeline.yaml

# georeference it:
mola-sm-georeferencing -i INPUT_WITH_GPS.simplemap --write-into MAP.mm
```
