#include "RoadGraph.h"

using namespace rgp;

CoroutineTaskT<bool> RoadNetwork::loadRoadsAsync(const std::string& file_path)
{
	auto future = std::async(std::launch::async, [this, file_path]() {
		std::ifstream file(file_path, std::ios::binary);
		if (!file) {
			std::cerr << "Error: file failed to open: " << file_path << std::endl;
			file.close();
			std::cerr << "Roads error loaded!\n";
			return false;
		}
		else {
			while (file) {
				RoadSegment segment;
				file.read(reinterpret_cast<char*>(&segment), sizeof(RoadSegment));
				if (file && segment.length > 0 && segment.speed > 0) {
					adjacency_list_distance[segment.start_id].push_back(segment);
				}
			}
			file.close();
			std::cout<<"Roads have been successfully loaded\n";
			return true;
		}
		});
	co_return future.get();
}
//----------------------------------------------------------------
CoroutineTaskT<bool> RoadNetwork::loadPointsAsync(const std::string& file_path)
{
	auto future = std::async(std::launch::async, [this, file_path]() {
		std::ifstream file(file_path, std::ios::binary);
		if (!file) {
			std::cerr << "Error: file failed to open: " << file_path << std::endl;
			file.close();
			std::cerr << "Points error loaded!\n";
			return false;
		}
		else
		{
			while (file) {
				Point pnt;
				file.read(reinterpret_cast<char*>(&pnt), sizeof(Point));
				if (file) {
					points_[pnt.id] = { pnt.id, pnt.x, pnt.y };
				}
			}
			file.close();
			std::cout<<"Points successfully loaded\n";
			return true; 
		}
		});
	co_return future.get();
}

//------------------------------------------------
std::pair<std::vector<uint64_t>, std::pair<double, double>> RoadNetwork::getShortestPath(const uint64_t& start_id, const uint64_t& end_id) {

	if (adjacency_list_distance.find(start_id) == adjacency_list_distance.end()) {
		return { {}, { std::numeric_limits<double>::infinity(), 0.0 } };
	}
	auto heuristic1 = [&](uint64_t current_id, uint64_t end_id) {
		return haversine_m(current_id, end_id);
		};
	std::unordered_map<uint64_t, double> distances;  
	std::unordered_map<uint64_t, double> times;     
	std::unordered_map<uint64_t, uint64_t> previous; 
	std::priority_queue<std::pair<double, uint64_t>, std::vector<std::pair<double, uint64_t>>, std::greater<>> pq;

	for (const auto& [node, _] : points_) { 
		distances[node] = std::numeric_limits<double>::infinity();
		times[node] = std::numeric_limits<double>::infinity();
	}
	distances[start_id] = 0.0;
	times[start_id] = 0.0;
	pq.push({ 0.0 + heuristic1(start_id, end_id), start_id });

	while (!pq.empty()) {
		auto [_, current] = pq.top();
		pq.pop();
		if (current == end_id) break;
		for (const auto& segment : adjacency_list_distance[current]) {
			uint64_t neighbor = segment.end_id;
			double new_dist = distances[current] + segment.length;;// current_dist + segment.length;
			double travel_time = times[current] + segment.time();

			if (new_dist < distances[neighbor]) {
				distances[neighbor] = new_dist;
				times[neighbor] = travel_time;
				previous[neighbor] = current;
				pq.push({ new_dist+heuristic1(neighbor, end_id), neighbor });
			}
		}
	}

	std::vector<uint64_t> path;
	double total_distance = distances[end_id];
	double total_time = times[end_id];

	if (total_distance == std::numeric_limits<double>::infinity()) {
		return { {}, { total_distance, total_time } };
	}
	for (uint64_t at = end_id; at != start_id; at = previous[at]) {
		if (previous.find(at) == previous.end()) {
			break;
		}
		path.push_back(at);
	}
	path.push_back(start_id);
	std::reverse(path.begin(), path.end());
	return { path, { total_distance, total_time } };
}
//-------------------------------------------------------------------------
std::pair<std::vector<uint64_t>, std::pair<double, double>> RoadNetwork::getFastestPath(const uint64_t& start_id, const uint64_t& end_id) {
	if (adjacency_list_distance.find(start_id) == adjacency_list_distance.end()) {
		return { {}, { 0.0, std::numeric_limits<double>::infinity() } };
	}
	double max_speed = getMaxSpeed();
	auto heuristic1 = [&](uint64_t current_id, uint64_t end_id) {
		double distance = haversine_m(current_id, end_id);
		return (distance / 1000.0) / max_speed; 
		};
	std::unordered_map<uint64_t, double> times;
	std::unordered_map<uint64_t, double> distances;
	std::unordered_map<uint64_t, uint64_t> previous;
	std::priority_queue<std::pair<double, uint64_t>, std::vector<std::pair<double, uint64_t>>, std::greater<>> pq;
	for (const auto& [node, _] : points_) {  
		distances[node] = std::numeric_limits<double>::infinity();
		times[node] = std::numeric_limits<double>::infinity();
	}
	times[start_id] = 0.0;
	distances[start_id] = 0.0;
	pq.push({ 0.0 + heuristic1(start_id, end_id), start_id });
	
	while (!pq.empty()) {
		auto [_, current] = pq.top();
		pq.pop();
		if (current == end_id) break;
		for (const auto& segment : adjacency_list_distance[current]) {
			uint64_t neighbor = segment.end_id;
			double travel_time = segment.time();
			double new_time = times[current] + travel_time;
			double new_distance = distances[current] + segment.length;

			if (new_time < times[neighbor]) {
				times[neighbor] = new_time;
				distances[neighbor] = new_distance;
				previous[neighbor] = current;
				pq.push({ new_time + heuristic1(neighbor, end_id), neighbor });
			}
		}
	}
	std::vector<uint64_t> path;
	double total_time = times[end_id];
	double total_distance = distances[end_id];
	if (total_time == std::numeric_limits<double>::infinity()) {
		return { {}, { total_distance, total_time } };
	}
	for (uint64_t at = end_id; at != start_id; at = previous[at]) {
		if (previous.find(at) == previous.end()) {
			break;
		}
		path.push_back(at);
	}
	path.push_back(start_id);
	std::reverse(path.begin(), path.end());
	return { path, { total_distance, total_time } };
}
//-------------------------------------------------
std::vector<uint64_t> RoadNetwork::getReachableNodes(const uint64_t& start_id) {

	if (adjacency_list_distance.find(start_id) == adjacency_list_distance.end()) {
		return {};  
	}
	std::unordered_map<uint64_t, bool> visited; 
	std::vector<uint64_t> reachable_nodes;     
	std::queue<uint64_t> q;
	q.push(start_id);
	visited[start_id] = true;
	while (!q.empty()) {
		uint64_t current = q.front();
		q.pop();
		reachable_nodes.push_back(current);  
		for (const auto& segment : adjacency_list_distance[current]) {
			uint64_t neighbor = segment.end_id;
			if (!visited[neighbor]) {
				visited[neighbor] = true;
				q.push(neighbor);
			}
		}
	}
	return reachable_nodes;
}
//--------------------------------------------------------------------------------------------------------------
CoroutineTaskT<bool> RoadNetwork::findShotFastIfReachable(const uint64_t& start_id, const uint64_t& end_id)
{
	auto res = co_await isReachableAsync(start_id, end_id);
	if (res) {
		auto [path, dist_time] = getShortestPath(start_id, end_id);
		if (path.empty()) {
			std::cout << "Error: point is reachable, but no path is found!\n";
			co_return false;
		}
		std::cout << "The shortest distance is found: \n";
		for (uint64_t node : path) {
			std::cout << node << " ";
		}
		std::cout << "\nPath length: " << dist_time.first << " m.";
		std::cout << "\nTracking time: " << dist_time.second << " hours.\n";
		auto [fastest_path, dist_time1] = getFastestPath(start_id, end_id);
		std::cout << "\nThe fastest way: \n";
		for (uint64_t node : fastest_path) {
			std::cout << node << " ";
		}
		std::cout << "\nPath length: " << dist_time1.first << " m. ";
		std::cout << "\nTracking time: " << dist_time1.second << " hours.\n";
		co_return true;
	}
	else co_return false;
}
//---------------------------------------------------------------
CoroutineTaskT<bool> RoadNetwork::isReachableAsync(const uint64_t& start_id, const uint64_t& end_id)
{
	if (points_.find(start_id) == points_.end()) {
		std::cout << "Starting point not found in points.\n";
		co_return false;
	}
	if (points_.find(end_id) == points_.end()) {
		std::cout << "End point not found in points.\n";
		co_return false;
	}
	if (adjacency_list_distance.find(start_id) == adjacency_list_distance.end() ||
		adjacency_list_distance[start_id].empty()) {
		std::cout << "Starting point not connected to any road.\n";
		co_return false;
	}
	bool end_connected = false;
	for (const auto& entry : adjacency_list_distance) {
		for (const auto& segment : entry.second) {
			if (segment.end_id == end_id) {
				end_connected = true;
				break;
			}
		}
		if (end_connected) break;
	}
	if (!end_connected) {
		std::cout << "End point not connected to any road.\n";
		co_return false;
	}
	std::unordered_map<uint64_t, bool> visited;
	std::queue<uint64_t> q;
	q.push(start_id);
	visited[start_id] = true;
	while (!q.empty()) {
		uint64_t current = q.front();
		q.pop();
		if (current == end_id) {
			std::cout << "The point is reachable\n";
			co_return true;
		}
		for (const auto& segment : adjacency_list_distance[current]) {
			uint64_t neighbor = segment.end_id;
			if (!visited[neighbor]) {
				visited[neighbor] = true;
				q.push(neighbor);
				co_await std::suspend_always{};
			}
		}
	}
	co_return true;
}



