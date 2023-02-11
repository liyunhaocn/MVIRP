#include "Solution.h"

void hsh::mvirp::Solution::init(Parameters & params) {
	if (paramsPtr == nullptr) { paramsPtr = &params; }
    visits = Vec<Vec<Visit>>(params.numPeriods, Vec<Visit>(params.numNodes, false));
    deliveries = Vec<Vec<Quantity>>(params.numPeriods, Vec<Quantity>(params.numNodes));
    routings = Vec<Routes>(params.numPeriods, Routes(params.numNodes));
    allocated = true;
}

hsh::mvirp::InvenCost hsh::mvirp::Solution::calcInvCost(ID ndId, bool check)
{
    using namespace std;
    auto &params = *paramsPtr;
    int numPer = params.numPeriods;
	Quantity curLv = params.nodes[ndId].initLevel;
    InvenCost sum = 0;
    for (ID perId = 0; perId < numPer; ++perId) {
        curLv += deliveries[perId][ndId];
		if (check && (curLv > params.nodes[ndId].maxLevel + params.error)) {
            cout << "Sln Check: curLv > maxLv, "
                << "nd " << ndId
                << ", curLv " << curLv
                << ", maxLv " << params.nodes[ndId].maxLevel
                << ", at period " << perId << endl;
            system("pause");
        }
        curLv -= params.nodes[ndId].prodCons;
        if (check && (curLv + params.error < params.nodes[ndId].minLevel)) {
            cout << "Sln Check: curLv < minLv, "
                << "nd " << ndId
                << ", curLv " << curLv
                << ", minLv " << params.nodes[ndId].minLevel
                << ", at period " << perId << endl;
            system("pause");
        }
        sum += curLv * params.nodes[ndId].holdingCost;
    }
    return sum;
}

hsh::mvirp::InvenCost hsh::mvirp::Solution::calcInvCost(bool check)
{
    auto &params = *paramsPtr;
    int numNd = params.numNodes;
    InvenCost sum = 0;
    for (ID ndId = 0; ndId < numNd; ++ndId) {
        sum += calcInvCost(ndId, check);
    }
    return sum;
}

hsh::mvirp::TravelCost hsh::mvirp::Solution::calcTraCost(ID perId, ID rouId, bool setDist, bool check)
{
    auto &params = *paramsPtr;
    TravelCost sum = 0.;
    Quantity totLoad = 0;
    //struct P { int id, quantity; };
    //Vec<P> loads;
    for (ID start = routings[perId].depots[rouId].id, cur = start, succ = routings[perId].depots[rouId].links[1];
        ;
        cur = succ, succ = routings[perId].nodes[cur].links[1])
    {
        sum += params.distances[cur][succ];
        if (succ == start) { break; }
        totLoad += deliveries[perId][succ];
        //loads.push_back({ succ,deliveries[perId][succ] });
    }
    if (setDist) { routings[perId].dist[rouId] = sum; }
    if (check && totLoad > params.vehCapacity) {
		std::cout << "Period: " << perId 
            << ", Load: " << totLoad << ", VehCapa: " << params.vehCapacity << std::endl;
        system("pause"); // TODO[dbg]
    }
    return sum;
}

hsh::mvirp::TravelCost hsh::mvirp::Solution::calcTraCost(ID perId, bool setDist, bool check)
{
    auto &params = *paramsPtr;
    TravelCost sum = 0.;
    for (ID rouId = 0; rouId < routings[perId].numRoutes; ++rouId) {
		sum += calcTraCost(perId, rouId, setDist, check);
    }
    if (setDist) { routings[perId].totDist = sum; }
    return sum;
}

hsh::mvirp::TravelCost hsh::mvirp::Solution::calcTraCost(bool setDist, bool check)
{
    auto &params = *paramsPtr;
    int numPer = params.numPeriods;
    TravelCost sum = 0.;
    for (ID perId = 0; perId < numPer; ++perId) {
		sum += calcTraCost(perId, setDist, check);
    }
    return sum;
}

void hsh::mvirp::Solution::outputDimacsFmt(Str path, bool check)
{
    using namespace std;
    static int outId = 0;
    //path = "yes.txt";
	std::ofstream of(path);
    of << fixed << setprecision(4);
    // 路线和配送量.
    auto &params = *paramsPtr;
    int numPer = params.numPeriods;
    int numVeh = params.numVehicles;
    for (ID perId = 0; perId < numPer; ++perId) {
        of << "Day " << perId + 1 << endl;
        for (ID rouId = 0; rouId < numVeh; ++rouId) {
            of << "Route " << rouId + 1 << ": ";
            if (rouId < routings[perId].numRoutes) {
                ID start = routings[perId].depots[rouId].id;
				for (ID cur = start, succ = routings[perId].depots[rouId].links[1];
					;
					cur = succ, succ = routings[perId].nodes[cur].links[1]) {
                    of << cur;
                    if (cur != start) { of << " ( " << deliveries[perId][cur] << " )"; }
                    of << " - ";
					if (succ == start) { break; }
				}
				of << start << endl;
            }
            else {
                of << "0 - 0" << endl;
            }
        }
    }
    auto tCost = calcTraCost(true,true);
    auto dCost = calcInvCost(0,true);
    auto cCost = calcInvCost(true) - dCost;
    of << tCost << endl;
    of << cCost << endl;
    of << dCost << endl;
    of << tCost + dCost + cCost << endl;
    of << "Intel Xeon E5-2698 v3 @ 2.30GHz" << endl;
    of << time << endl;
	
    ++outId;
}
