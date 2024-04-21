## Quick Start
You can run this project with the following command:

```shell
./run.sh
```

Then `rviz` will start, and you can use `2D Nav Goal` to set the position of points (independent of direction). After using `2D Nav Goal` twice, it will generate a point cloud wall.



## Save the cloud

After finishing the map drawing, you can save the map with the following order,

```shell
./save.sh
```

then the pcd file will save to `~/tmp.pcd`



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
=======
>>>>>>> 353da63e9ba3a072d3bdaee412d54af8055b9857
```

