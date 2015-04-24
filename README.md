# TF Pose Rebroadcaster

ROS package for rebroadcasting transforms in the TF tree as PoseStamped.

## Nodes

* **broadcast_pose** - broadcasts the tf as a PoseStamped
* **broadcast_point** - broadcasts only the origin of the tf as a PointStamped

### Parameters

Both nodes need the following parameters:

* `~tf_parent` - the parent frame of the transform to look up
* `~tf_child` - the frame to look up 
* `~output_topic` - the topic to publish the pose or point on

### Launch files

Each node has it's own default launch file, including this in your project is the recommended usage.
