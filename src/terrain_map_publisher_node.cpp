#include <ros/ros.h>
#include "global_body_planner/terrain_map_publisher.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "terrain_map_publisher_node");
	ros::NodeHandle nh;

	TerrainMapPublisher terrain_map_publisher(nh);
	terrain_map_publisher.spin();

	return 0;
}
