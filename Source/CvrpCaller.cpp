#include "CvrpCaller.h"

bool hsh::mvirp::CvrpCaller::solve(ID period, Solution & sln, Mode mode, int specNumRou, double replacedThres) {
    subgraphToInstance(period, sln, specNumRou);
    if (!nodeMap.empty() && nodeMap[0] != 0) { throw "cvrp.solve(): dismiss depot."; }
    switch (mode) {
    case Mode::GenInit : {
        return runGenInit(period, sln, specNumRou, replacedThres);
    }
    case Mode::AsInit: {
        return runAsInitSln(period, sln, specNumRou, replacedThres);
    }
    case Mode::Pop: {
		return runWithPop(period, sln, specNumRou, replacedThres);
    }
    }
}

void hsh::mvirp::CvrpCaller::subgraphToInstance(ID periodId, Solution & sln, int specNumRou) {
    /*
        `visits` 给出该周期访问的客户.
        `deliveries` 给出该周期应该给每个客户配送的量.
    */

    auto &subGraph = sln.visits[periodId];

    /* 节点映射 */
    nodeMap.clear();
    revNdMap.resize(mvirpParams.numNodes);
    for (ID id = 0; id < subGraph.size(); ++id) {
        if (subGraph[id] == true) {
            nodeMap.push_back(id);
			revNdMap[id] = (int)nodeMap.size() - 1;
        }
        else {
            revNdMap[id] = -1;
        }
    }
    
    /* 基本参数 */
    cvrpParams.numClients = nodeMap.size() - 1;
    cvrpParams.numNodes = nodeMap.size();
    cvrpParams.critical.maxNumRoutes = mvirpParams.numVehicles;
    cvrpParams.critical.numRoutes = specNumRou;

    /* 直接覆盖之前的参数 */
    for (ID id = 0; id < cvrpParams.numNodes; ++id) {
        cvrpParams.nodeInfo[id].ID = id;
        cvrpParams.nodeInfo[id].coordX = mvirpParams.nodes[nodeMap[id]].x;
        cvrpParams.nodeInfo[id].coordY = mvirpParams.nodes[nodeMap[id]].y;
        cvrpParams.nodeInfo[id].demand = nodeMap[id] == 0 ? 0. : sln.deliveries[periodId][nodeMap[id]];
    }
    for (ID id1 = 0; id1 < cvrpParams.numNodes; ++id1) {
        for (ID id2 = 0; id2 < cvrpParams.numNodes; ++id2) {
            cvrpParams.distanceMat[id1][id2] = mvirpParams.distances[nodeMap[id1]][nodeMap[id2]];
        }
    }
    // OPTIMIZE[hsh][5]: 可由完整图的接近关系直接提取出来.
    Vec<std::pair<double, int>> orderNear;
    for (int i = 1; i < cvrpParams.numNodes; ++i) {
        orderNear.clear();
        for (int j = 1; j < cvrpParams.numNodes; ++j) {
            // 接近关系只考虑客户节点
            if (j == i) { continue; }
            orderNear.push_back({ cvrpParams.distanceMat[i][j],cvrpParams.nodeInfo[j].ID });
        }
        sort(orderNear.begin(), orderNear.end());

        for (int k = 0; k < cvrpParams.numClients - 1; ++k) {
            cvrpParams.proximityMat[i][k].ID = orderNear[k].second;
            cvrpParams.proximityMat[i][k].distance = orderNear[k].first;
        }
    }
    for (int i = 1; i < cvrpParams.numNodes; ++i) {
        for (int k = 0; k < cvrpParams.numClients - 1; ++k) {
            int j = cvrpParams.proximityMat[i][k].ID;
            for (int kk = 0; kk < cvrpParams.numClients - 1; ++kk) {
                if (cvrpParams.proximityMat[j][kk].ID == i) { cvrpParams.proximityMat[i][k].reversedProximity = kk; break; }
            }
        }
    }

    cvrpParams.lookupProximityMat = Vec<Vec<int>>(cvrpParams.numNodes, Vec<int>(cvrpParams.numNodes));
    for (int i = 1; i < cvrpParams.numNodes; ++i) {
        for (int k = 0; k < cvrpParams.numClients - 1; ++k) {
            cvrpParams.lookupProximityMat[i][cvrpParams.proximityMat[i][k].ID] = k;
        }
    }
}

bool hsh::mvirp::CvrpCaller::runGenInit(ID periodId, Solution &sln, int specNumRou, double replacedThres) {
    // TODO[hsh][0]: 优化这里的求解过程, 目前只有局部搜索.

    using namespace std;
    if (nodeMap.empty()) {
        sln.routings[periodId].reset();
        return true;
    }
    cvrp::NagataLocalSearch nagataLS(cvrpParams);
    cvrp::Repair repair(cvrpParams);
    cvrp::Individual indiv(cvrpParams);
    cvrp::Eax eax(cvrpParams);
    cvrp::Population pop(cvrpParams, eax, nagataLS, repair);
    
    double base = cvrpParams.critical.penaltyCapacityExcess;
    int cnt = 0, maxCnt = 10;
	while (cnt < maxCnt) {
		cvrpParams.critical.penaltyCapacityExcess = base * pow(2, cnt);
        //cout << "Try penalCapa: " << cvrpParams.critical.penaltyCapacityExcess << endl;
        indiv.reset();
        bool stat = pop.generateInitialIndiv(indiv, nagataLS, repair);
        if (stat && doubleLess(indiv.distance, replacedThres, mvirpParams.error)) {
            indivToRoutes(indiv, sln.routings[periodId], sln.visits[periodId]);
            break;
        }
        ++cnt;
    }
    cvrpParams.critical.penaltyCapacityExcess = base;
    //cout << "is " << indiv.isFeasible << ", distance " << indiv.distance << ", route num" << indiv.numRoutes << endl;
    return indiv.isFeasible && indiv.numRoutes <= cvrpParams.critical.maxNumRoutes;
}

bool hsh::mvirp::CvrpCaller::runAsInitSln(ID periodId, Solution &sln, int specNumRou, double replacedThres)
{
	if (nodeMap.empty() || nodeMap.size() == 1) {
        sln.routings[periodId].reset();
        return true;
    }
    //cvrpParams.nagataLS.proximity = cvrpParams.numClients;
    cvrp::NagataLocalSearch ngtLs(cvrpParams);
    cvrp::Repair rp(cvrpParams);
    cvrp::Individual indiv(cvrpParams);
    //cvrp::Eax eax(cvrpParams);
    //cvrp::Population pop(cvrpParams, eax, ngtLs, rp);

    for (ID id = 0; id < cvrpParams.numNodes; ++id) { indiv.nodes[id].ID = id; }

    ID triedCount = 10;
    while (triedCount-- > 0) {
        indiv.reset();
        auto &routes = sln.routings[periodId];
        // clients
        for (ID n = 1; n < mvirpParams.numNodes; ++n) {
            if (sln.visits[periodId][n]) {
                auto &iNd = indiv.nodes[revNdMap[n]];
                auto &rNd = routes.nodes[n];
                iNd.routeID = rNd.routeId;
                iNd.links[0] = revNdMap[rNd.links[0]];
                iNd.links[1] = revNdMap[rNd.links[1]];
            }
        }
        // depots
        for (ID r = 0; r < routes.numRoutes; ++r) {
            auto &iDep = indiv.depots[r];
            auto &rDep = routes.depots[r];
            iDep.ID = rDep.id;
            iDep.routeID = rDep.routeId;
            iDep.links[0] = revNdMap[rDep.links[0]];
            iDep.links[1] = revNdMap[rDep.links[1]];
        }
        indiv.numRoutes = indiv.numDepots = routes.numRoutes;
        indiv.distance = std::accumulate(routes.dist.begin(), routes.dist.end(), 0);
        //Log[Logger::Type::Cvrp] << "Before LS: " << indiv.distance << std::endl;
        indiv.isFeasible = true;
        ngtLs.setSearchMode(cvrp::NagataLocalSearch::SearchMode::FEASIBLE).run(indiv);
		if (indiv.isFeasible && doubleLess(indiv.distance, replacedThres, mvirpParams.error)) {
            //if (!indiv.inspectLinks(cvrpParams.dbg.lk)) { system("pause"); } // TODO[dbg]
            //Log[Logger::Type::Cvrp] << "After LS: " << indiv.distance << std::endl;
            indivToRoutes(indiv, sln.routings[periodId], sln.visits[periodId]);
            break;
        }
    }
   /* Log[Logger::Type::Cvrp]
        << "is " << indiv.isFeasible << ", distance " << indiv.distance << ", route num" << indiv.numRoutes << std::endl;*/
    return indiv.isFeasible && indiv.numRoutes <= cvrpParams.critical.numRoutes;
}

bool hsh::mvirp::CvrpCaller::runWithPop(ID periodId, Solution &sln, int specNumRou, double replacedThres)
{
	if (nodeMap.empty() || nodeMap.size() == 1) {
        sln.routings[periodId].reset();
        return true;
    }
    //if (cvrpParams.dbg.lk) { std::cout << "pop1" << std::endl; }
    cvrp::NagataLocalSearch nagataLS(cvrpParams);
    cvrp::Repair repair(cvrpParams);
    cvrp::Eax eax(cvrpParams);
    cvrp::Population pop(cvrpParams, eax, nagataLS, repair);
    cvrp::PathRelinking pr(cvrpParams, eax);
    cvrp::PathRelinkingWithEax prEax(cvrpParams, pop, eax, pr, nagataLS, repair);
    //if (cvrpParams.dbg.lk) { std::cout << "pop2" << std::endl; }
    // 若不指定路径数, 将尝试所有的路径数
    auto minRoutes = [&]()->int {
        double sumDema = 0;
        for (ID ndId = 1; ndId < cvrpParams.numNodes; ++ndId) { sumDema += cvrpParams.nodeInfo[ndId].demand; }
        return std::ceil(sumDema / cvrpParams.capacityLimit);
    };
    for (int r = minRoutes(); r <= cvrpParams.critical.maxNumRoutes; ++r) {
        if (specNumRou != -1 && r != specNumRou) { continue; }
        cvrpParams.critical.numRoutes = r;
        //Log[Logger::Type::Cvrp] << "Try route num: " << cvrpParams.critical.numRoutes << std::endl;
        prEax.run();
        //if (cvrpParams.dbg.lk) { std::cout << "pop3" << std::endl; }
        auto &indiv = prEax.overallBest;
        if (indiv.isFeasible && doubleLess(indiv.distance, replacedThres, mvirpParams.error)) {
            //if (!indiv.inspectLinks(cvrpParams.dbg.lk)) { system("pause"); }; // TODO[dbg]
            //if (cvrpParams.dbg.lk) { std::cout << "pop4" << std::endl; }
            indivToRoutes(prEax.overallBest, sln.routings[periodId], sln.visits[periodId]);
            //if (cvrpParams.dbg.lk) { std::cout << "pop5" << std::endl; }
            break;
        }
		//else { std::cout << "pop overbest dist: " << indiv.distance << ", thres: " << replacedThres << std::endl; }
    }
    /*Log[Logger::Type::Cvrp]
        << "is " << prEax.overallBest.isFeasible 
        << ", distance " << prEax.overallBest.distance 
        << ", route num" << prEax.overallBest.numRoutes << std::endl;*/
    return prEax.overallBest.isFeasible && prEax.overallBest.numRoutes <= cvrpParams.critical.numRoutes;
}

void hsh::mvirp::CvrpCaller::indivToRoutes(cvrp::Individual &indiv, Routes & routes, Vec<Visit> &visits) {
    // 清空之前的路径信息.
    for (ID nodeId = 0; nodeId < mvirpParams.numNodes; ++nodeId) { 
        routes.nodes[nodeId].reset(); 
        if (visits[nodeId]) { routes.nodes[nodeId].id = nodeId; } // 只保留被访问节点信息
    }

    // 填入 CVRP 求解结果.
    for (ID n = 1; n < cvrpParams.numNodes; ++n) {
        ID nodeId = nodeMap[n];
        routes.nodes[nodeId].id = nodeMap[indiv.nodes[n].ID];
        routes.nodes[nodeId].routeId = indiv.nodes[n].routeID;
        routes.nodes[nodeId].links[0] = nodeMap.at(indiv.nodes[n].links[0]);
        routes.nodes[nodeId].links[1] = nodeMap.at(indiv.nodes[n].links[1]);
    }
    routes.numRoutes = indiv.numRoutes;
    routes.totDist = 0;
    std::fill(routes.dist.begin(), routes.dist.end(), 0);
    for (ID r = 0; r < indiv.numRoutes; ++r) {
        routes.depots[r].id = 0;
        routes.depots[r].links[0] = nodeMap.at(indiv.depots[r].links[0]);
        routes.depots[r].links[1] = nodeMap.at(indiv.depots[r].links[1]);
        routes.depots[r].routeId = r;
        routes.dist[r] = indiv.getDistanceOf(r);
        routes.totDist += routes.dist[r];
    }
}

hsh::mvirp::CvrpCaller::CvrpCaller(Parameters &params):mvirpParams(params) {
    /* 节点映射分配空间 */
    nodeMap.reserve(mvirpParams.numNodes);

    /* CVRP 参数分配空间, 并初始化 */
    cvrpParams.critical.maxNumRoutes = mvirpParams.numVehicles;
    cvrpParams.maxClients = mvirpParams.numClients;
    cvrpParams.maxNodes = cvrpParams.maxClients + 1;
    cvrpParams.serviceTime = 0.;
    cvrpParams.durationLimit = cvrp::Inf;
    cvrpParams.capacityLimit = mvirpParams.vehCapacity;
    cvrpParams.nodeInfo = Vec<cvrp::NodeInfo>(cvrpParams.maxNodes);
    cvrpParams.distanceMat = Vec<Vec<double>>(cvrpParams.maxNodes, Vec<double>(cvrpParams.maxNodes));
    cvrpParams.proximityMat = Vec<Vec<cvrp::Near>>(cvrpParams.maxNodes, Vec<cvrp::Near>(cvrpParams.maxNodes));
    cvrpParams.lookupProximityMat = Vec<Vec<int>>(cvrpParams.maxNodes, Vec<int>(cvrpParams.maxNodes));
}
