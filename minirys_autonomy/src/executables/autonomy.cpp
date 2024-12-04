#include "nodes/AutonomyNode.hpp"

int main(int argc, char const* argv[]) {
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;

	auto autonomyNode = AutonomyNode::make_shared(options);
	executor.add_node(autonomyNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}
