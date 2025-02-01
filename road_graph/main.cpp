#include "RoadGraph/RoadGraph.h"

int main(int argc, char* argv[]) {

	if (argc < 5) {
		std::cerr << "Usage: " << argv[0]
			<< " <road_segments_file> <coordinates_file> <start_id> <end_id> [-Rp]\n";
		std::cerr << "Example:  " << argv[0]
			<< " manhattan_edges.bin manhattan_nodes.bin 42429058 1332843647 -Rp\n";
		return 1;
	}
	try {
	    std::string road_segments_file = argv[1];
		std::string coordinates_file = argv[2];
		uint64_t start_id = std::strtoull(argv[3], nullptr, 10);
		uint64_t end_id = std::strtoull(argv[4], nullptr, 10);

		bool show_reachable_nodes = false;
		if (argc == 6 && std::string(argv[5]) == "-Rp") {
			show_reachable_nodes = true;
		}

		rgp::RoadNetwork road_network;
		auto roadsTask = road_network.loadRoadsAsync(road_segments_file);
		auto pointsTask = road_network.loadPointsAsync(coordinates_file);
		roadsTask.wait();
		pointsTask.wait();

		if (roadsTask.get() && pointsTask.get()) {
			if (!show_reachable_nodes) {
				auto task = road_network.findShotFastIfReachable(start_id, end_id);
				task.wait();
			}
			else {
				auto reachable_nodes = road_network.getReachableNodes(start_id);
				if (reachable_nodes.empty()) {
					std::cout << "No reachable points from " << start_id << ".\n";
				}
				else {
					std::cout << "Reachable points from " << start_id << ": ";
					for (uint64_t node : reachable_nodes) {
						std::cout << node << " ";
					}
					std::cout << std::endl;
				}
			}
		}
		else {
			std::cerr << "File opening error\n";
			return 1;
		}
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << '\n';
		return 1;
	}
	catch (...) {
		std::cerr << "Unknown error!" << std::endl;
		return 1;
	}

	return 0;
}
