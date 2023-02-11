#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <utility>
#include <algorithm>
#include <numeric>
#include "Untils.h"
#include <iomanip>

namespace hsh {
	namespace mvirp {
		using Str = std::string;
		using Capacity = int;
		using InvenLevel = int;
        extern InvenLevel INVEN_LEVEL_MAX;
		using InvenCost = double;
        using TravelCost = int;
        using Cost = double;
		using ProdCons = int;
		template<typename T> using Vec = std::vector<T>;
		using Coordinate = double;
		using Distance = int;
        extern Distance DISTANCE_MAX;
        extern InvenCost INVEN_COST_MAX;
		extern Cost COST_MAX;
        using Quantity = int;
        extern Quantity EMPTY;
        using Visit = bool;
		using ID = int;
		using TimeSeed = time_t;
        using Second = unsigned int;
		template<typename K, typename V> using HashMap = std::unordered_map<K, V>;
		template<typename K> using HashSet = std::unordered_set<K>;
		extern Logger Log;
		using UL = unsigned long;
		using ULL = unsigned long long;
		template<typename T1, typename T2> using Pair = std::pair<T1, T2>;
	}
}