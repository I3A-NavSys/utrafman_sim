#!/bin/bash

for i in $(seq 1 1 $1)
do
	rostopic pub /god/remove std_msgs/String "'$i'" -1 &
done
