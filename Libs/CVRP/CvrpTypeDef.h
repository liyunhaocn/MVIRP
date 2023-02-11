#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hsh {
    namespace cvrp {
        using String = std::string;
        template<typename T> using Vec = std::vector<T>;
        using IFStream = std::ifstream;
        using InvalidArg = std::invalid_argument;
        using TimeSeed = time_t;
        extern double Inf;
        template<typename T> using Pair = std::pair<T, T>;
        template<typename T, typename U> using UnorderedMap = std::unordered_map<T, U>;
        template<typename T> using UnorderedSet = std::unordered_set<T>;
        using Cycle = std::vector<int>;
        using Second = unsigned int;
    }
}