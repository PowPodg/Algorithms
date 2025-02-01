#pragma once

#include <cstdint>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <coroutine>
#include <future>
#include <numbers>


namespace rgp {
	template <typename T>
	struct CoroutineTaskT {
		struct promise_type {
			T result = std::conditional_t<std::is_same_v<T, bool>, std::true_type, T>{}();
			CoroutineTaskT get_return_object() {
				return CoroutineTaskT{ std::coroutine_handle<promise_type>::from_promise(*this) };
			}
			std::suspend_never initial_suspend() noexcept { return {}; }
			std::suspend_always final_suspend() noexcept { return {}; }
			void return_value(T res) { result = std::move(res); }  
			void unhandled_exception() { std::terminate(); }
		};
		std::coroutine_handle<promise_type> handle;
		explicit CoroutineTaskT(std::coroutine_handle<promise_type> h) : handle(h) {}
		~CoroutineTaskT() {
			if (handle) handle.destroy();
		}
		bool await_ready() { return false; }
		void await_suspend(std::coroutine_handle<> h) {
			if (handle && !handle.done()) handle.resume();
		}
		T await_resume() {
			return handle ? std::move(handle.promise().result) : T{};
		}
		T get() {
			if (handle && !handle.done()) handle.resume();
			return std::move(handle.promise().result);
		}
		void wait() {
			if (handle && !handle.done())  handle.resume();
		}
	};
	//------------
	struct RoadSegment {
		uint64_t start_id;
		uint64_t end_id;
		double length; 
		double speed; 

		double time() const {
			return (length / 1000.0) / speed;  // Time in hours
		}
	};
	//------------
	struct Point {
		uint64_t id;
		double x, y;
	};
	//-----------------
	class RoadNetwork {
	public:
		CoroutineTaskT<bool>loadRoadsAsync(const std::string& file_path);
		CoroutineTaskT<bool>loadPointsAsync(const std::string& file_path);
		std::pair<std::vector<uint64_t>, std::pair<double, double>> getFastestPath(const uint64_t& start_id, const uint64_t& end_id);
		std::pair<std::vector<uint64_t>, std::pair<double, double>> getShortestPath(const uint64_t& start_id, const uint64_t& end_id);
		std::vector<uint64_t>getReachableNodes(const uint64_t& start_id);
		RoadNetwork() = default;
		RoadNetwork(const std::unordered_map<uint64_t, Point>& points, const std::unordered_map<uint64_t, std::vector<RoadSegment>>& data)
			:points_(points), adjacency_list_distance(data) {
		}
		CoroutineTaskT<bool> findShotFastIfReachable(const uint64_t& start_id, const uint64_t& end_id);
		CoroutineTaskT<bool> isReachableAsync(const uint64_t& start_id, const uint64_t& end_id);
	private:
		double haversine_m(const uint64_t& start_id, const uint64_t& end_id) {
			const double R = 6371000.0;  
			const double DEG_TO_RAD = std::numbers::pi / 180.0;
			const Point& start = points_[start_id];
			const Point& end = points_[end_id];
			double lat1 = start.y * DEG_TO_RAD; 
			double lon1 = start.x * DEG_TO_RAD; 
			double lat2 = end.y * DEG_TO_RAD;    
			double lon2 = end.x * DEG_TO_RAD;    
			double dlat = lat2 - lat1;
			double dlon = lon2 - lon1;
			double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
				std::cos(lat1) * std::cos(lat2) *
				std::sin(dlon / 2) * std::sin(dlon / 2);
			double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
			return R * c / 1000;
		}
		//--------------------
		double getMaxSpeed() {
			double max_speed = 0.0; 
			for (const auto& [start_id, segments] : adjacency_list_distance) {
				for (const auto& segment : segments) {
					max_speed = std::max(max_speed, segment.speed);  
				}
			}
			return max_speed; 
		}
		std::unordered_map<uint64_t, Point> points_;
		std::unordered_map<uint64_t, std::vector<RoadSegment>> adjacency_list_distance;
	};

}