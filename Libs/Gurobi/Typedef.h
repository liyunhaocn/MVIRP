////////////////////////////////
/// usage : 1.	type aliases for simple types.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef CN_HUST_GOAL_COMMON_TYPEDEF_H
#define CN_HUST_GOAL_COMMON_TYPEDEF_H


#include <string>
#include <vector>
#include <set>
#include <map>


namespace goal {

// zero-based consecutive integer identifier.
using ID = int;

using Millisecond = int;

using Iteration = int;

using PopulationSize = int;

using String = std::string;

template<typename T>
using List = std::vector<T>;

template<typename T>
using Set = std::set<T>;

template<typename Key, typename Value>
using Map = std::map<Key, Value>;


template<typename T>
const char* toBytes(const T *ptr) { return reinterpret_cast<const char*>(ptr); }

template<typename T>
char* toBytes(T *ptr) { return reinterpret_cast<char*>(ptr); }

}


#endif // CN_HUST_GOAL_COMMON_TYPEDEF_H
