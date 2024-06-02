## Quick Start
You can run this project with the following command:

```shell
./run.sh
```

Then `rviz` will start, and you can use `2D Nav Goal` to set the position of points (independent of direction). After using `2D Nav Goal` twice, it will generate a point cloud wall.



## Features

### Save

After finishing the map drawing, you can save the map with the following order,

```shell
./save.sh
```

then the pcd file will save to `~/res.pcd`

### undo
```shell
./undo.sh
```

### clear
```shell
./clear.sh
```

### reference
it is hard to loacate the coordinate of the point, so you can modify `reference_points.py` and then run

```
python3 reference_points.py
```

to publish the reference points




## Parameter

you can modify the parameter in the `~/map_gen/src/map_generator/launch/map_generator.launch`

```launch
<!-- pc wall param -->
<param name="~width" value="0.15"/>
<param name="~height" value="2.4"/>

<!-- pc step -->
<param name="~step_length" value="0.05"/>
<param name="~step_width" value="0.05"/>
<param name="~step_height" value="0.02"/>

<param name="frame" value="world" />
<remap from="/click_map" to="/map_generator/click_map"/>
```

## How to use
<img src="./misc/how_to_use_it.gif" alt="how_to_use" width="100%">


## Reference 

This project was inspired by [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL.git).
