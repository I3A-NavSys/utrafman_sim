#!/bin/bash

for i in $(seq 0 1 $1)
do
	for j in $(seq 0 1 $1)
	do
		sdf="<sdf version ='1.4'> <model name ='sphere8'> <pose>$i $j 0 0 0 0</pose> <link name ='link'> <pose>0 0 .5 0 0 0</pose> <collision name ='collision'> <geometry> <sphere> <radius>0.5</radius> </sphere> </geometry> </collision> <visual name ='visual'> <geometry> <sphere> <radius>0.5</radius> </sphere> </geometry> <material> <ambient>80 0 0 0</ambient> </material> </visual> </link> </model> </sdf>"
		rostopic pub /god/insert std_msgs/String "$sdf" -1 &
	done
done
