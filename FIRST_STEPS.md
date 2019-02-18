# First Steps

This readme contains some basic tasks in order to show basic use cases of the simulation.

First you need to start the simulation by launching the main launch file:

`catkin_ws$ roslaunch simulation_initialization_ros_tool _whole_framework.launch`

You will notice two windows popping up:
* the rviz window for 3d visualization
* the rqt_reconfigure window for online parameter changing

# Tasks

The tasks are divided into three categories:

### Dynamic Reconfigure
  * Let vehicle1 move faster
  * Accelerate the simulation time
  * Change the localization update frequency to 20Hz

### Changes in the launchfiles
  * Change the object ID of vehicle1
  * Changing the initialization path of vehicle1
  * Add a third vehicle

### Changes in the code
  * Convert the exact_loc_sensor to a GPS sensor (add noise)
  * Insert a ghost object into the abstract_object_sensor


# Solutions

### Dynamic Reconfigure

In the dynamic reconfigure screen you can change various parameters of the simulation framework.

* If you want to change the speed of an object, for example change the speed of vehicle1, you can expand v1, expand planning and click on the planner. There you can change the v_desired to the value you want.

* Under time_mgmt you can let the simulation time run faster.

* Under loacalization_mgmt you can change the update frequency to for example 20Hz.


### Changes in the launchfiles

The .launch files used for the roslaunch command are written in XML format. For a detailed description you can visit http://wiki.ros.org/roslaunch/XML

#### Change the object ID of object 1

First, open the file `simulation_initialization_ros_tool/launch/_whole_framework.launch`

There we can change for example the Object ID of vehicle1 by changing the arguments value
```xml
<arg name="vehicle1_id" value="1" />
```
to
```xml
<arg name="vehicle1_id" value="4" />
```
for example.

#### Changing the initialization path of object 1

Again in the file `_whole_framework.launch`, change the `trajectory_file` of vehicle1, for example from
```xml
<arg name="trajectory_file" value="$(find simulation_initialization_ros_tool)/res/traj1.osm" />
...
<arg name="s_start" value="0.0" />
```
to
```xml
<arg name="trajectory_file" value="$(find simulation_initialization_ros_tool)/res/traj2.osm" />
...
<arg name="s_start" value="20.0" />
```


#### Add a third vehicle

In order to add a third vehicle, you have to launch another sample_vehicle by including `sample_vehicle.launch` once more. This can be done via copying the initialization of vehicle2 with the following changes:

```xml
<arg name="vehicle3_id" value="3" /> <!-- new -->
<arg name="vehicle3_ns" value="v$(arg vehicle3_id)" /> <!-- new -->
<include file="$(find simulation_initialization_ros_tool)/launch/vehicle_launchfiles/sample_vehicle.launch" ns="$(arg vehicle3_ns)" > <!-- changed -->
        <arg name="vehicle_id" value="$(arg vehicle3_id)" /> <!-- changed -->
        <arg name="vehicle_ns" value="$(arg vehicle3_ns)" /> <!-- changed -->

        <arg name="objects_ground_truth_topic_with_ns" value="$(arg objects_ground_truth_topic_with_ns)" />
        <arg name="desired_motion_topic_with_ns" value="$(arg desired_motion_topic_with_ns)" />
        <arg name="global_communication_ns" value="$(arg global_communication_ns)" />
        <arg name="perc_pred_obj_topic" value="$(arg perc_pred_obj_topic)" />
        <arg name="perc_egomotion_topic" value="$(arg perc_egomotion_topic)" />
        <arg name="pred_plan_obj_topic" value="$(arg pred_plan_obj_topic)" />
        <arg name="internal_communication_subns" value="$(arg internal_communication_subns)" />
        <arg name="BSM_topic" value="$(arg BSM_topic)" />

        <arg name="object_initialization_topic_with_ns" value="$(arg object_initialization_topic_with_ns)" />
        <arg name="trajectory_file" value="$(find simulation_initialization_ros_tool)/res/traj2.osm" />
        <arg name="initial_v" value="5.0" />
        <arg name="hull_file" value="$(find simulation_initialization_ros_tool)/res/sample_hull.xml" />
        <arg name="object_type" value="car" />
        <arg name="s_start" value="20.0" /> <!-- changed -->
        <arg name="frame_id_loc_mgmt" value="$(arg loc_mgmt_frame)" />
    </include>
```


### Changes in the code

#### Convert the exact_loc_sensor to a GPS sensor (add noise)

The position is provided by the node `exact_localization_sensor` of `sim_sample_perception_ros_tool`. This node computes velocity and acceleration from the ground truth data via finite differences in the callback `subCallback` defined in `exact_localization_sensor.cpp`:

```cpp
void ExactLocalizationSensor::subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {
  //...
}
```
After the `latestMotionState_` is saved (for finite differences), but before it is published, you can add noise, or for example a constant offset:
```cpp

    // add uncertainty to ego-motionstate here, if desired
    egoObjectState.motion_state.pose.pose.position.x =
        egoObjectState.motion_state.pose.pose.position.x + 2.;
    egoObjectState.motion_state.pose.pose.position.y =
        egoObjectState.motion_state.pose.pose.position.y + 1.;
    // change covariance also
    egoObjectState.motion_state.pose.covariance[0] = 4.;
    egoObjectState.motion_state.pose.covariance[7] = 4.;

    // publish new motion state
    percEgoMotionPub_.publish(egoObjectState.motion_state);

```
You can visualize this wrong ego position by adding a MotionState-plugin to rviz and setting the topic to `/v1/ego_motion_state` for example

#### Insert a ghost object into the abstract_object_sensor

The list of objectStates of other objects is provided by the node `abstract_object_sensor` of `sim_sample_perception_ros_tool`. This node computes velocity and acceleration from the ground truth data for all objects (but the ego object) via finite differences in the callback `subCallback` defined in `abstract_object_sensor.cpp`:

```cpp
void AbstractObjectSensor::subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {
  //...
}
```

In order to insert a ghost object, you can add the object after the `latestPerceivedObjects_` are saved but before the objects are published:

```cpp

    // add uncertainty to other objects' motionstates here, if desired

    // insert ghost object
    automated_driving_msgs::ObjectState ghostObjectState = perceivedObjects.objects[0];
    ghostObjectState.object_id = 999;
    ghostObjectState.motion_state.pose.pose.position.x = ghostObjectState.motion_state.pose.pose.position.x + 20.;
    perceivedObjects.objects.push_back(ghostObjectState);

    // publish perceived objects
    percObjectsPub_.publish(perceivedObjects);

```
You can visualize this ghost object by setting the topic of the ObjectStateArray-plugin in rviz to `/v1/perceived_objects` for example
