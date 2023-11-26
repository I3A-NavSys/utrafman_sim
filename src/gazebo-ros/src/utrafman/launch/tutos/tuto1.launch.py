<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true" />
    <arg name="paused" value="true" />
    <arg name="debug" value="false" />
    <arg name="world_name" value="$(find utrafman)/worlds/tutos/tuto1.world" />
  </include>
</launch>
