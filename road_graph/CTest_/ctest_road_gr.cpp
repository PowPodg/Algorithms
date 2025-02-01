#include "../RoadGraph/RoadGraph.h"  

std::unordered_map<uint64_t, rgp::Point> points = {
	{1, {1, 40.730610, -73.935242}},
	{2, {2, 40.731610, -73.934242}},
	{3, {3, 40.732610, -73.933242}},
	{4, {4, 40.733610, -73.932242}},
	{5, {5, 40.734610, -73.931242}},
	{6, {6, 40.735610, -73.930242}},
	{7, {7, 40.736610, -73.929242}},
};
//---------------------
std::unordered_map<uint64_t, std::vector<rgp::RoadSegment>> segments = {
	{1, {
		{1, 2, 500, 30}, 
		{1, 3, 1000, 40},
		{1, 4, 1500, 10},
		{1, 5, 2000, 15},
	}},
	{2, {
		{2, 4, 1500, 30}, 
		{2, 5, 1000, 50}, 
		{2, 6, 700, 20}, 
	}},
	{3, {
		{3, 4, 2000, 40},
		{3, 5, 500, 70}, 
		{3, 6, 1500, 25}, 
	}},
	{4, {
		{4, 5, 1500, 90},
		{4, 7, 500, 10}, 
	}},
	{5, {
		{5, 6, 2000, 60}, 
		{5, 7, 800, 35}, 
	}},
	{6, {
		{6, 7, 1000, 80}, 
	}},
};
//------------------
int TEST_reachability_of_poin(uint64_t start_id, uint64_t end_id)
{
	rgp::RoadNetwork graph(points, segments);
	auto res = graph.isReachableAsync(start_id, end_id);
	res.wait();
	return !res.get();
}
//-------------------
int TEST_Short_Fast(uint64_t start_id, uint64_t end_id)
{
	rgp::RoadNetwork graph(points, segments);
	int result = 1;

	auto [path, dist_time] = graph.getShortestPath(start_id, end_id);
	std::cout << "\nThe shortest distance is found: \n";
	for (uint64_t node : path) {
		std::cout << node << " ";
	}
	std::cout << "\nPath length: " << dist_time.first << " m.";
	std::cout << "\nTracking time: " << dist_time.second << " hours.\n";

	auto [fastest_path, dist_time1] = graph.getFastestPath(start_id, end_id);
	std::cout << "\nThe fastest way: \n";
	for (uint64_t node : fastest_path) {
		std::cout << node << " ";
	}
	std::cout << "\nPath length: " << dist_time1.first << " m. ";
	std::cout << "\nTracking time: " << dist_time1.second << " hours.\n";

	if (dist_time.first <= dist_time1.first && dist_time.second >= dist_time1.second)
	{
		result = 0;
	}

	return result;
}
//------------
int main(int num_arg, char* arg[]) {
	std::string s_expected1 = arg[1];
	std::string s_expected2 = arg[2];
	std::string type_test = arg[3];
	uint64_t point1 = std::stoull(s_expected1);
	uint64_t point2 = std::stoull(s_expected2);

	int res = 0;
	if (type_test == "Test_reachability_of_poin")
	{
		res = TEST_reachability_of_poin(point1, point2);
	}
	else
		if (type_test == "Test_Short_Fast")
		{
			res = TEST_Short_Fast(point1, point2);
		}
	return res;
}
