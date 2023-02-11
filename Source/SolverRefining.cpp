#include "Solver.h"

void hsh::mvirp::Solver::structureRefining(Solution& sln) {
    Solution tmpSln = sln;
    hasher.setBase(tmpSln);
    int steps = 0, maxSteps = 20;
    Vec<Move> moves;
    while (!timer.isTimeOut()) {
        while (steps < maxSteps) {
            if (timer.isTimeOut()) { return; }
            ++steps;
            //Log[Logger::Type::Rf] << "make moves." << std::endl;
            makeMoves(tmpSln, moves);
            if (moves.empty()) { break; } // 当前解的所有邻域解都被禁忌
            //Log[Logger::Type::Rf] << "eval moves." << std::endl;
            evalMoves(tmpSln, moves);
            //Log[Logger::Type::Rf] << "done moves." << std::endl;
            if (moves.empty()) { break; } // 当前解的所有非禁忌邻域解都不可行
            while (moves.size() > 1 && moves.front().totCost + params.error < moves.back().totCost) { moves.pop_back(); }
			int pIdx = rand() % moves.size();
            Move &m = moves[pIdx];
            //Log[Logger::Type::Rf] << "accept feasible move." << std::endl;
            //Log[Logger::Type::Rf] << "take move ty: " << m.type << std::endl;
            takeMove(tmpSln, m);
            //Log[Logger::Type::Rf] << "taked moves." << std::endl;
            hasher.updBase(m).stBaseTaboo();
            /*Log[Logger::Type::Rf]
                << hasher.hashValBase[0]
                << "," << hasher.hashValBase[1]
                << "," << hasher.hashValBase[2] << std::endl;*/
            //Log[Logger::Type::Rf] << "Cur Cost: " << tmpSln.cost << ", move size: " << moves.size() << std::endl;
            if (doubleLess(tmpSln.cost, sln.cost, params.error)) {
                sln = tmpSln;
                sln.calcInvCost(true); // TODO[dbg]
                sln.calcTraCost(true, true);
                sln.time = timer.curTime();
                steps = 0;
				Log[Logger::Type::Bf] << "Best Cost: " << sln.cost << ", time: " << sln.time << std::endl;
            }
        }
        perturb(tmpSln, sln);
        tmpSln.calcInvCost(true); // TODO[dbg]
        tmpSln.calcTraCost(true, true);
        steps = 0;
        ++params.refining.timesPtb;
    }
}

void hsh::mvirp::Solver::makeMoves(Solution &sln, Vec<Move> &moves)
{
	auto dtDistDelNd = [&](ID prevId, ID succId, ID curId)->Distance {
        auto &d = params.distances;
        return d[prevId][succId] - (d[prevId][curId] + d[curId][succId]);
    };
    auto dtDistInsNd = [&](ID prevId, ID succId, ID curId)->Distance {
        return -dtDistDelNd(prevId, succId, curId);
    };
    // auto backup = sln; // TODO[dbg]
    moves.clear();

    int numPer = params.numPeriods;
    int numNd = params.numNodes;
    auto &vis = sln.visits;
    auto &routings = sln.routings;
    auto &dist = params.distances;
    /*auto backupRoutings = routings;
    auto checkRoutes = [&]() {
        using namespace std;
        for (ID perId = 0; perId < numPer; ++perId) {
            auto &bRoutes = backupRoutings[perId];
            auto &routes = routings[perId];
            for (ID ndId = 0; ndId < numNd; ++ndId) {
                if (bRoutes.nodes[ndId].links[0] != routes.nodes[ndId].links[0]) {
                    Log[Logger::Type::Debug] << "Not match links." << endl;
                }
                if (bRoutes.nodes[ndId].links[1] != routes.nodes[ndId].links[1]) {
                    Log[Logger::Type::Debug] << "Not match links." << endl;
                }
                if (bRoutes.depots[ndId].links[0] != routes.depots[ndId].links[0]) {
                    Log[Logger::Type::Debug] << "Not match links." << endl;
                }
                if (bRoutes.depots[ndId].links[1] != routes.depots[ndId].links[1]) {
                    Log[Logger::Type::Debug] << "Not match links." << endl;
                }
            }
        }
    };*/
    Vec<ID> orderPer(numPer), orderNds(numNd - 1);
    for (ID perId = 0; perId < numPer; ++perId) { orderPer[perId] = perId; }
    for (ID cliId = 0; cliId < numNd - 1; ++cliId) { orderNds[cliId] = cliId + 1; }
    std::random_shuffle(orderPer.begin(), orderPer.end());
    std::random_shuffle(orderNds.begin(), orderNds.end());
    for (ID perId : orderPer) {
        for (ID ndId : orderNds) {
            //// 删除访问
            //if (vis[perId][ndId]) {
            //    Move m(Move::Type::Del);
            //    auto &nd = routings[perId].nodes[ndId];
            //    ID prevNdId = nd.links[0], succNdId = nd.links[1];
            //    m.periodIdA = perId;
            //    m.origNdA = nd;
            //    if (!hasher.isNbrTaboo(m)) {
            //        m.dtDistDelA = dtDistDelNd(prevNdId, succNdId, ndId);
            //        m.dtTraCost = m.dtDistDelA;
            //        m.taboo = false;
            //    }
            //    if (!m.taboo) { moves.push_back(m); }
            //}
            // 增加访问, 当前周期所有路径都尝试, 找到最好的插入位置
            if (!vis[perId][ndId]) {
                auto &perRoutes = routings[perId];
                Move m(Move::Type::Add);
                m.periodIdB = perId;
                m.curNdA.assignId(ndId);
				if (!hasher.isNbrTaboo(m)) {
                    for (ID rouId = 0; rouId < perRoutes.numRoutes; ++rouId) {
                        auto &dep = perRoutes.depots[rouId];
                        auto &nd = perRoutes.nodes;
                        for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                            ;
                            curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                            auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                            if (dt < m.dtDistAddA) { //TODO: 水池抽样
                                m.curNdA.assignRoute(rouId);
                                m.curNdA.assignLinks(curNdId, succNdId);
                                m.dtDistAddA = dt;
                            }
                            if (succNdId == staNdId) { break; }
                        }
                    }
                    if (perRoutes.numRoutes == 0) { // 单客户路径
                        m.curNdA.assignRoute(-2);
                        m.dtDistAddA = dist[0][ndId] + dist[ndId][0];
                    }
                    m.dtTraCost = m.dtDistAddA;
                    m.taboo = false;
                }
                if (!m.taboo) { moves.push_back(m); }
            }
            // 移动访问, 原周期删除访问, 其他周期所有路径都尝试, 找到最好的插入位置
            if (vis[perId][ndId]) {
                auto &nd = routings[perId].nodes[ndId];
                ID origPrevNdId = nd.links[0], origSuccNdId = nd.links[1];
                Move m(Move::Type::Shift);
                m.periodIdA = perId;
                m.origNdA = nd;
                m.curNdA.assignId(ndId);
                m.dtDistDelA = dtDistDelNd(origPrevNdId, origSuccNdId, ndId);
                for (ID newPerId = 0; newPerId < numPer; ++newPerId) {
                    if (newPerId == perId || vis[newPerId][ndId]) { continue; }
                    m.dtDistAddA = DISTANCE_MAX; m.taboo = true; // 本轮比较
                    m.periodIdB = newPerId;
                    auto &newPerRoutes = routings[newPerId];
                    if (!hasher.isNbrTaboo(m)) {
                        for (ID rouId = 0; rouId < newPerRoutes.numRoutes; ++rouId) {
                            auto &dep = newPerRoutes.depots[rouId];
                            auto &nd = newPerRoutes.nodes;
                            for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                                ;
                                curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                                auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                                if (dt < m.dtDistAddA) {
                                    m.curNdA.assignRoute(rouId);
                                    m.curNdA.assignLinks(curNdId, succNdId);
                                    m.dtDistAddA = dt;
                                }
                                if (succNdId == staNdId) { break; }
                            }
                        }
                        if (newPerRoutes.numRoutes == 0) { // 单客户路径
                            m.curNdA.assignRoute(-2);
                            m.dtDistAddA = dist[0][ndId] + dist[ndId][0];
                        }
                        m.dtTraCost = m.dtDistDelA + m.dtDistAddA;
                        m.taboo = false;
                    }
                    if (!m.taboo) { moves.push_back(m); }
                }
            }
            // 交换访问, 两周期都删除访问并临时删除连接关系, 各周期分别找到最好的插入位置
            if (vis[perId][ndId]) {
                auto &perRoutesA = routings[perId];
                auto &ndA = perRoutesA.nodes[ndId];
                ID prevNdIdA = ndA.links[0], succNdIdA = ndA.links[1];
                Move m(Move::Type::Swap);
                m.periodIdA = perId;
                m.origNdA = ndA;
                m.curNdA.assignId(ndA.id);
				m.dtDistDelA = dtDistDelNd(prevNdIdA, succNdIdA, ndId);
				perRoutesA.unlinkPrevSucc(prevNdIdA, succNdIdA, m.origNdA.id, m.origNdA.routeId);
				for (ID perIdB = perId + 1; perIdB < numPer; ++perIdB) {
                    auto &perRoutesB = routings[perIdB];
                    for (ID ndIdB = 0; ndIdB < numNd; ++ndIdB) {
						if (ndIdB == ndId || !(!vis[perId][ndIdB] && vis[perIdB][ndIdB] && !vis[perIdB][ndId])) { continue; }
                        auto &ndB = perRoutesB.nodes[ndIdB];
                        ID prevNdIdB = ndB.links[0], succNdIdB = ndB.links[1];
                        m.dtDistAddA = m.dtDistAddB = DISTANCE_MAX; m.taboo = true; // 本轮比较
                        m.periodIdB = perIdB;
                        m.origNdB = ndB;
                        m.curNdB.assignId(ndB.id);
                        if (!hasher.isNbrTaboo(m)) {
                            m.dtDistDelB = dtDistDelNd(prevNdIdB, succNdIdB, ndIdB);
                            perRoutesB.unlinkPrevSucc(prevNdIdB, succNdIdB, m.origNdB.id, m.origNdB.routeId);
                            for (ID rouId = 0; rouId < perRoutesB.numRoutes; ++rouId) {
                                auto &dep = perRoutesB.depots[rouId];
                                auto &nd = perRoutesB.nodes;
                                for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                                    ;
                                    curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                                    auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                                    if (dt < m.dtDistAddA) { //TODO: 水池抽样
                                        m.curNdA.assignRoute(rouId);
                                        m.curNdA.assignLinks(curNdId, succNdId);
                                        m.dtDistAddA = dt;
                                    }
                                    if (succNdId == staNdId) { break; }
                                }
                            }
                            for (ID rouId = 0; rouId < perRoutesA.numRoutes; ++rouId) {
                                auto &dep = perRoutesA.depots[rouId];
                                auto &nd = perRoutesA.nodes;
                                for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                                    ;
                                    curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                                    auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                                    if (dt < m.dtDistAddB) { //TODO: 水池抽样
                                        m.curNdB.assignRoute(rouId);
                                        m.curNdB.assignLinks(curNdId, succNdId);
                                        m.dtDistAddB = dt;
                                    }
                                    if (succNdId == staNdId) { break; }
                                }
                            }
                            m.dtTraCost = m.dtDistDelA + m.dtDistDelB + m.dtDistAddA + m.dtDistAddB;
                            m.taboo = false;
                            perRoutesB.linkPrevSucc(prevNdIdB, succNdIdB, m.origNdB.id, m.origNdB.routeId); // 恢复连接
                        }
                        if(!m.taboo){ moves.push_back(m); }
                    }
                }
                perRoutesA.linkPrevSucc(prevNdIdA, succNdIdA, m.origNdA.id, m.origNdA.routeId); // 恢复连接
                //checkRoutes();
            }
            // 改变车辆?是否需要?
        }
    }
    int k = 100;
}

void hsh::mvirp::Solver::evalMoves(Solution &sln, Vec<Move> &moves)
{
    if (moves.empty()) { return; }

    params.refining.szLayer1 = std::sqrt(params.numNodes) * params.numPeriods * std::sqrt(params.numVehicles);
    // 双层筛选
    auto screenRule1 = [&](Move &m1, Move &m2) {
        return m1.dtTraCost < m2.dtTraCost;
    };
    std::sort(moves.begin(), moves.end(), screenRule1);
    // 为第二层筛选足够的可行解, 保留第一层全部邻域动作
    /*if ((int)moves.size() > params.refining.szLayer1) {
        moves.erase(moves.begin() + params.refining.szLayer1, moves.end());
    }*/

    NetflowCaller nf(params);
    auto &vis = sln.visits;
    auto &routings = sln.routings;
    auto &deliv = sln.deliveries;
    int numPer = params.numPeriods;
    int numNd = params.numNodes;
    auto copyDeliv = [&](Vec<Vec<Quantity>> &src, Vec<Vec<Quantity>> &dst) {
        if (dst.size() != src.size() || dst.front().size() != src.front().size()) {
            dst = Vec<Vec<Quantity>>(src.size(), Vec<Quantity>(src.front().size(), EMPTY));
        }
        for (ID perId = 0; perId < numPer; ++perId) {
            for (ID ndId = 0; ndId < numNd; ++ndId) { dst[perId][ndId] = src[perId][ndId]; }
        }
    };
    //auto backupRoutings = routings; // TODO[dbg]
    //auto checkRoutes = [&]() {
    //    using namespace std;
    //    for (ID perId = 0; perId < numPer; ++perId) {
    //        auto &bRoutes = backupRoutings[perId];
    //        auto &routes = routings[perId];
    //        for (ID ndId = 0; ndId < numNd; ++ndId) {
    //            if (bRoutes.nodes[ndId].links[0] != routes.nodes[ndId].links[0]) {
    //                Log[Logger::Type::Debug] << "Not match links." << endl;
    //            }
    //            if (bRoutes.nodes[ndId].links[1] != routes.nodes[ndId].links[1]) {
    //                Log[Logger::Type::Debug] << "Not match links." << endl;
    //            }
    //            if (bRoutes.depots[ndId].links[0] != routes.depots[ndId].links[0]) {
    //                Log[Logger::Type::Debug] << "Not match links." << endl;
    //            }
    //            if (bRoutes.depots[ndId].links[1] != routes.depots[ndId].links[1]) {
    //                Log[Logger::Type::Debug] << "Not match links." << endl;
    //            }
    //        }
    //    }
    //};
    auto evalDistDelDelivZero = [&](Move &m) {
        // 评估CI方式删除网络流重新分配后配送量为0的已访问节点路径长度变化
        // 记录到move.dtDist5中, 并更新move.dtTraCost
        auto &d = params.distances;
        m.dtDistDelZero = 0;
        for (ID perId = 0; perId < numPer; ++perId) {
            auto &perRoutes = routings[perId];
            for (ID rouId = 0; rouId < perRoutes.numRoutes; ++rouId) {
                auto &dep = perRoutes.depots[rouId];
                auto &nd = perRoutes.nodes;
                ID notEmpNdId = dep.id;
                Distance completed = 0, saved = 0;
                for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                    ;
                    curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                    completed += d[curNdId][succNdId];
                    if (deliv[perId][succNdId] != 0) {
                        saved += d[notEmpNdId][succNdId];
                        notEmpNdId = succNdId;
                    }
					if (succNdId == staNdId) { break; }
                }
                m.dtDistDelZero += (saved - completed);
            }
        }
        m.dtTraCost += m.dtDistDelZero;
    };

    copyDeliv(sln.deliveries, origDeliveries); // 复制, 评估动作需覆盖sln.deliveries
	for (int i = 0, feasibleCnt = 0; i < (int)moves.size() && feasibleCnt < params.refining.szLayer1; ++i) { // 找到满足个数的合法邻域动作
        Move &m = moves[i];
        //hasher.stNbrTaboo(m); // 对于评估过网络流的动作所对应的visits, 采取禁忌
        switch (m.type)
        {
        case Move::Type::Del: {
            chgVisLk(sln, m, false);
			m.invCost = nf.run(sln, true);
            if (m.invCost != INVEN_COST_MAX) {
                if (params.refining.delDelivZero) { evalDistDelDelivZero(m); }
                m.totCost = m.invCost + m.dtTraCost;
                m.feasible = true;
                ++feasibleCnt;
            }
            rstVisLk(sln, m);
            //checkRoutes(); // TODO[dbg]
            break;
        }
        case Move::Type::Add: {
            chgVisLk(sln, m, false);
            m.invCost = nf.run(sln, true);
            if (m.invCost != INVEN_COST_MAX) {
                if (params.refining.delDelivZero) { evalDistDelDelivZero(m); }
                m.totCost = m.invCost + m.dtTraCost;
                m.feasible = true;
                ++feasibleCnt;
            }
            rstVisLk(sln, m);
            //checkRoutes();
            break;
        }
        case Move::Type::Shift: {
            chgVisLk(sln, m, false);
            m.invCost = nf.run(sln, true);
            if (m.invCost != INVEN_COST_MAX) {
                if (params.refining.delDelivZero) { evalDistDelDelivZero(m); }
                m.totCost = m.invCost + m.dtTraCost;
                m.feasible = true;
                ++feasibleCnt;
            }
            rstVisLk(sln, m);
            //checkRoutes();
            break;
        }
        case Move::Type::Swap: {
            chgVisLk(sln, m, false);
            m.invCost = nf.run(sln, true);
            if (m.invCost != INVEN_COST_MAX) {
                if (params.refining.delDelivZero) { evalDistDelDelivZero(m); }
                m.totCost = m.invCost + m.dtTraCost;
                m.feasible = true;
                ++feasibleCnt;
            }
            rstVisLk(sln, m);
            //checkRoutes();
            break;
        }
        default:
            break;
        }
    }
    copyDeliv(origDeliveries, sln.deliveries);

    params.refining.szLayer2 = params.refining.szLayer1 * 0.25;
    auto screenRule2 = [&](Move &m1, Move &m2) {
        if (m1.feasible != m2.feasible) { return m1.feasible == true; } // 非法动作放在最后
		return m1.totCost < m2.totCost;
    };
    std::sort(moves.begin(), moves.end(), screenRule2);
    
    // TODO: 从前面选择前k个进行CVRP优化, 选择出最好的动作, 配送量要重新计算一次网络流获得
    /*int pivot = -1;
    for (int i = 0; i < (int)moves.size(); ++i) {
        if (!moves[i].feasible && pivot == -1) { pivot = i; }
        if (moves[i].feasible && pivot != -1) {
            system("pause");
        }
    }*/
    // 禁忌计算过网络流的不可行解和尚未计算的路径增量较差解
	while (!moves.empty() && !moves.back().feasible) { hasher.stNbrTaboo(moves.back()); moves.pop_back(); }
}

void hsh::mvirp::Solver::takeMove(Solution &sln, Move &move)
{
    const bool setDist = true;
    NetflowCaller nf(params);
    CvrpCaller cvrp(params);
    Move &m = move;
    //cvrp.cvrpParams.dbg.lk = true;
    // 修改访问状态, 确定配送量, 优化路径长度
    auto &vis = sln.visits;
    auto &routings = sln.routings;
    auto &deliv = sln.deliveries;
    int numPer = params.numPeriods;
    int numNd = params.numNodes;
    auto delVisLkDelivZero = [&]() {
        // 删除网络流重新分配后配送量为0的已访问节点的访问和连接关系
        for (ID perId = 0; perId < numPer; ++perId) {
            auto &perRoutes = routings[perId];
            for (ID ndId = 1; ndId < numNd; ++ndId) {
                if (vis[perId][ndId] && deliv[perId][ndId] == 0) {
                    auto &nd = perRoutes.nodes[ndId];
                    auto &prevNdId = nd.links[0], succNdId = nd.links[1];
                    vis[perId][ndId] = false;
                    perRoutes.unlinkPrevSucc(prevNdId, succNdId, ndId, nd.routeId);
                }
            }
        }
    };
    TravelCost nodelzero = -1, delzero= -1, cvrpopt = -1, cvrppreax = -1;
    switch (m.type)
    {
    case Move::Type::Del: {
        //std::cout << "del" << std::endl;
        chgVisLk(sln, m, true);
        //std::cout << "1" << std::endl;
		nf.run(sln, true);
        //std::cout << "2" << std::endl;
        nodelzero = sln.calcTraCost(setDist, true);
        //std::cout << "3" << std::endl;
        if (params.refining.delDelivZero) { delVisLkDelivZero(); }
        //std::cout << "4" << std::endl;
        delzero = sln.calcTraCost(setDist, true);
        //std::cout << "5" << std::endl;
		cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdA].numRoutes, sln.routings[m.periodIdA].totDist);
        //std::cout << "6" << std::endl;
        cvrpopt = sln.calcTraCost(setDist, true);
        //std::cout << "7" << std::endl;
		cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdA].totDist);
        //std::cout << "8" << std::endl;
        cvrppreax = sln.calcTraCost(setDist, true);
        //std::cout << "9" << std::endl;
        break;
    }
    case Move::Type::Add: {
        //std::cout << "add" << std::endl;
        chgVisLk(sln, m, true);
        //std::cout << "1" << std::endl;
        nf.run(sln, true);
        //std::cout << "2" << std::endl;
        nodelzero = sln.calcTraCost(setDist, true);
        //std::cout << "3" << std::endl;
        if (params.refining.delDelivZero) { delVisLkDelivZero(); }
        //std::cout << "4" << std::endl;
        delzero = sln.calcTraCost(setDist, true);
        //std::cout << "5" << std::endl;
		cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdB].numRoutes, sln.routings[m.periodIdB].totDist);
        //std::cout << "6" << std::endl;
        cvrpopt = sln.calcTraCost(setDist, true);
        //std::cout << "7" << std::endl;
		cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdB].totDist);
        //std::cout << "8" << std::endl;
        cvrppreax = sln.calcTraCost(setDist, true);
        //std::cout << "9" << std::endl;
        break;
    }
    case Move::Type::Shift: {
        //std::cout << "shift" << std::endl;
        chgVisLk(sln, m, true);
        //std::cout << "1" << std::endl;
        nf.run(sln, true);
        //std::cout << "1" << std::endl;
        nodelzero = sln.calcTraCost(setDist, true);
        //std::cout << "2" << std::endl;
        if (params.refining.delDelivZero) { delVisLkDelivZero(); }
        //std::cout << "3" << std::endl;
        delzero = sln.calcTraCost(setDist, true);
        //std::cout << "4" << std::endl;
		cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdA].numRoutes, sln.routings[m.periodIdA].totDist);
        //std::cout << "5" << std::endl;
        cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdB].numRoutes, sln.routings[m.periodIdB].totDist);
        //std::cout << "6" << std::endl;
        cvrpopt = sln.calcTraCost(setDist, true);
        //std::cout << "7" << std::endl;
        cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdA].totDist);
        //std::cout << "8" << std::endl;
        cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdB].totDist);
        //std::cout << "9" << std::endl;
        cvrppreax = sln.calcTraCost(setDist, true);
        //std::cout << "10" << std::endl;
        break;
    }
    case Move::Type::Swap: {
        //std::cout << "swap" << std::endl;
        chgVisLk(sln, m, true);
        //std::cout << "1" << std::endl;
        nf.run(sln, true);
        //std::cout << "2" << std::endl;
        nodelzero = sln.calcTraCost(setDist, true);
        //std::cout << "3" << std::endl;
        if (params.refining.delDelivZero) { delVisLkDelivZero(); }
        //std::cout << "4" << std::endl;
        delzero = sln.calcTraCost(setDist, true);
        //std::cout << "5" << std::endl;
        cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdA].numRoutes, sln.routings[m.periodIdA].totDist);
        //std::cout << "6" << std::endl;
        cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::AsInit,
            sln.routings[m.periodIdB].numRoutes, sln.routings[m.periodIdB].totDist);
        //std::cout << "7" << std::endl;
        cvrpopt = sln.calcTraCost(setDist, true);
        //std::cout << "8" << std::endl;
		cvrp.solve(m.periodIdA, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdA].totDist);
        //std::cout << "9" << std::endl;
		cvrp.solve(m.periodIdB, sln, CvrpCaller::Mode::Pop, -1, sln.routings[m.periodIdB].totDist);
        //std::cout << "10" << std::endl;
        cvrppreax = sln.calcTraCost(setDist, true);
        //std::cout << "11" << std::endl;
        break;
    }
    default:
        break;
    }
    nodelzero += 0;
    sln.invenCost = sln.calcInvCost(true);
    sln.travelCost = sln.calcTraCost(setDist, true);
    sln.cost = sln.invenCost + sln.travelCost;
    /*for (ID p = 0; p < numPer; ++p) { // TODO[dbg]
        for (ID n = 1; n < numNd; ++n) {
            if (vis[p][n] && routings[p].nodes[n].id != n) {
                int k = 100;
                system("pause");
            }
            if (vis[p][n] && !vis[p][0]) {
                int k = 100;
                system("pause");
            }
        }
    }*/
}

// 对访问状态和节点的连接状态进行修改
void hsh::mvirp::Solver::chgVisLk(Solution &sln, Move &move, bool cleanSelfLoop)
{
    Move &m = move;
    auto &routings = sln.routings;
    auto &vis = sln.visits;
    switch (m.type)
    {
    case Move::Type::Del: {
        auto &perRoutes = routings[m.periodIdA];
        vis[m.periodIdA][m.origNdA.id] = false;
        perRoutes.unlinkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        if (cleanSelfLoop) { perRoutes.cleanDepSelfLoop(); }
        break;
    }
    case Move::Type::Add: {
        auto &perRoutes = routings[m.periodIdB];
        vis[m.periodIdB][0] = true;
        vis[m.periodIdB][m.curNdA.id] = true;
        if (m.curNdA.routeId != -2) {
            perRoutes.linkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        }
        else {
            perRoutes.addOneCliRouAtLast(m.curNdA.id);
        }
        if (cleanSelfLoop) { perRoutes.cleanDepSelfLoop(); }
        break;
    }
    case Move::Type::Shift: {
        auto &perRoutes1 = routings[m.periodIdA], &perRoutes2 = routings[m.periodIdB];
        vis[m.periodIdA][m.origNdA.id] = false;
        vis[m.periodIdB][0] = true;
        vis[m.periodIdB][m.curNdA.id] = true;
        perRoutes1.unlinkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        if (m.curNdA.routeId != -2) {
            perRoutes2.linkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        }
        else {
            perRoutes2.addOneCliRouAtLast(m.curNdA.id);
        }
        if (cleanSelfLoop) { perRoutes1.cleanDepSelfLoop(); perRoutes2.cleanDepSelfLoop(); }
        break;
    }
    case Move::Type::Swap: {
        auto &perRoutes1 = routings[m.periodIdA], &perRoutes2 = routings[m.periodIdB];
        vis[m.periodIdA][m.origNdA.id] = false; vis[m.periodIdA][m.curNdB.id] = true;
        vis[m.periodIdB][m.curNdA.id] = true; vis[m.periodIdB][m.origNdB.id] = false;
        perRoutes1.unlinkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        perRoutes2.unlinkPrevSucc(m.origNdB.links[0], m.origNdB.links[1], m.origNdB.id, m.origNdB.routeId);
        perRoutes2.linkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        perRoutes1.linkPrevSucc(m.curNdB.links[0], m.curNdB.links[1], m.curNdB.id, m.curNdB.routeId);
        if (cleanSelfLoop) { perRoutes1.cleanDepSelfLoop(); perRoutes2.cleanDepSelfLoop(); } // 同样需要清理{0,0}
        break;
    }
    default:
        break;
    }
    /*for (ID p = 0; p < params.numPeriods; ++p) {
        for (ID n = 1; n < params.numNodes; ++n) {
            if (vis[p][n] && routings[p].nodes[n].id != n) {
                int k = 100;
                system("pause");
            }
            if (vis[p][n] && !vis[p][0]) {
                int k = 100;
                system("pause");
            }
        }
    }*/
}

// 对访问状态和节点的连接状态进行恢复
void hsh::mvirp::Solver::rstVisLk(Solution &sln, Move &move)
{
    Move &m = move;
    auto &routings = sln.routings;
    auto &vis = sln.visits;
    switch (m.type)
    {
    case Move::Type::Del: {
        auto &perRoutes = routings[m.periodIdA];
        vis[m.periodIdA][m.origNdA.id] = true;
        perRoutes.linkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        break;
    }
    case Move::Type::Add: {
        auto &perRoutes = routings[m.periodIdB];
        vis[m.periodIdB][m.curNdA.id] = false;
        if (m.curNdA.routeId != -2) {
            perRoutes.unlinkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        }
        else {
            perRoutes.delOneCliRouAtLast();
        }
        break;
    }
    case Move::Type::Shift: {
        auto &perRoutes1 = routings[m.periodIdA], &perRoutes2 = routings[m.periodIdB];
        vis[m.periodIdA][m.origNdA.id] = true;
        vis[m.periodIdB][m.curNdA.id] = false;
        if (m.curNdA.routeId != -2) {
            perRoutes2.unlinkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        }
        else {
            perRoutes2.delOneCliRouAtLast();
        }
        perRoutes1.linkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        break;
    }
    case Move::Type::Swap: {
        auto &perRoutes1 = routings[m.periodIdA], &perRoutes2 = routings[m.periodIdB];
        vis[m.periodIdA][m.origNdA.id] = true; vis[m.periodIdA][m.curNdB.id] = false;
        vis[m.periodIdB][m.curNdA.id] = false; vis[m.periodIdB][m.origNdB.id] = true;
        perRoutes1.unlinkPrevSucc(m.curNdB.links[0], m.curNdB.links[1], m.curNdB.id, m.curNdB.routeId);
        perRoutes2.unlinkPrevSucc(m.curNdA.links[0], m.curNdA.links[1], m.curNdA.id, m.curNdA.routeId);
        perRoutes2.linkPrevSucc(m.origNdB.links[0], m.origNdB.links[1], m.origNdB.id, m.origNdB.routeId);
        perRoutes1.linkPrevSucc(m.origNdA.links[0], m.origNdA.links[1], m.origNdA.id, m.origNdA.routeId);
        break;
    }
    default:
        break;
    }
}

void hsh::mvirp::Solver::perturb(Solution &cur, Solution &base)
{
    Log[Logger::Type::Rf] << " .P ";
    auto dtDistDelNd = [&](ID prevId, ID succId, ID curId)->Distance {
        auto &d = params.distances;
        return d[prevId][succId] - (d[prevId][curId] + d[curId][succId]);
    };
    auto dtDistInsNd = [&](ID prevId, ID succId, ID curId)->Distance {
        return -dtDistDelNd(prevId, succId, curId);
    };
    auto &params = *cur.paramsPtr;
    int numPer = params.numPeriods;
    int numNd = params.numNodes;
    auto &vis = cur.visits;
    auto &routings = cur.routings;
    auto &dist = params.distances;
    int numAdd = 4 + rand() % 2,
        numDel = 0 + rand() % 3,
        numShift = 4 + rand() % 2;
    NetflowCaller nf(params);

    do {
        if (timer.isTimeOut()) { return; }
        cur = base; // 每次扰动以历史最优解为出发点
        Vec<ID> addIdx, delIdx;
        Vec<Pair<ID,ID>> shiftIdx;
        // 增加访问
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 1; n < numNd; ++n) {
                if (!vis[p][n]) { addIdx.push_back(p * numNd + n); }
            }
        }
        Sampler::pool(addIdx, numAdd);
        for (ID i : addIdx) {
            ID perId = i / numNd;
            ID ndId = i % numNd;
            auto &perRoutes = routings[perId];
            Move m(Move::Type::Add);
            m.periodIdB = perId;
            m.curNdA.assignId(ndId);
            for (ID rouId = 0; rouId < perRoutes.numRoutes; ++rouId) {
                auto &dep = perRoutes.depots[rouId];
                auto &nd = perRoutes.nodes;
                for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                    ;
                    curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                    auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                    if (dt < m.dtDistAddA) { //TODO: 水池抽样
                        m.curNdA.assignRoute(rouId);
                        m.curNdA.assignLinks(curNdId, succNdId);
                        m.dtDistAddA = dt;
                    }
                    if (succNdId == staNdId) { break; }
                }
            }
            if (perRoutes.numRoutes == 0) { // 单客户路径
                m.curNdA.assignRoute(-2);
                m.dtDistAddA = dist[0][ndId] + dist[ndId][0];
            }
            m.dtTraCost = m.dtDistAddA;
            /*Log[Logger::Type::Rf] << "Add visit of p: " << perId
                << ", n: " << ndId
                << ", at route: " << m.periodIdB << ", num route: " << perRoutes.numRoutes
                << ", prev: " << m.curNdA.links[0] << ", succ: " << m.curNdA.links[1]
                << std::endl;*/
            chgVisLk(cur, m, true);
        }

        // 移动访问
        // 对于新增访问不作移动
        //auto bvis = vis;
        //auto brou = routings;
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 1; n < numNd; ++n) {
                if (vis[p][n]) {
                    int i = p * numNd + n;
                    if (find(addIdx.begin(), addIdx.end(), i) == addIdx.end()) {
                        for (ID pp = 0; pp < numPer; ++pp) {
                            if (!vis[pp][n]) {
								shiftIdx.push_back({ p * numNd + n,pp * numNd + n });
                            }
                        }
                    }
                }
            }
        }
        Sampler::pool(shiftIdx, numShift);
        for (auto &pair : shiftIdx) {
			ID perId = pair.first / numNd, newPerId = pair.second / numNd;
            ID ndId = pair.first % numNd;
			if (vis[newPerId][ndId] || vis[perId][ndId]) { continue; } // 新的访问被其他移动操作或原访问已操作
            auto& nd = routings[perId].nodes[ndId];
            ID origPrevNdId = nd.links[0], origSuccNdId = nd.links[1];
            Move m(Move::Type::Shift);
            m.periodIdA = perId;
            m.origNdA = nd;
            m.curNdA.assignId(ndId);
            m.dtDistDelA = dtDistDelNd(origPrevNdId, origSuccNdId, ndId);
            m.periodIdB = newPerId;
            auto& newPerRoutes = routings[newPerId];
            for (ID rouId = 0; rouId < newPerRoutes.numRoutes; ++rouId) {
                auto& dep = newPerRoutes.depots[rouId];
                auto& nd = newPerRoutes.nodes;
                for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                    ;
                    curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                    auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                    if (dt < m.dtDistAddA) {
                        m.curNdA.assignRoute(rouId);
                        m.curNdA.assignLinks(curNdId, succNdId);
                        m.dtDistAddA = dt;
                    }
                    if (succNdId == staNdId) { break; }
                }
            }
            if (newPerRoutes.numRoutes == 0) { // 单客户路径
                m.curNdA.assignRoute(-2);
                m.dtDistAddA = dist[0][ndId] + dist[ndId][0];
            }
            m.dtTraCost = m.dtDistDelA + m.dtDistAddA;
            chgVisLk(cur, m, true);
        }

        // 删除访问
        /*Vec<int> visCnt(numNd, 0);
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 1; n < numNd; ++n) { visCnt[n] += (vis[p][n] ? 1 : 0); }
        }*/
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 1; n < numNd; ++n) {
                int i = p * numNd + n;
                if (std::find(addIdx.begin(), addIdx.end(), i) != addIdx.end()) { continue; }
                bool isShifted = false;
                for (auto& pair : shiftIdx) { if (i == pair.second) { isShifted = true; } }
                if (isShifted) { continue; }
				if (vis[p][n]) { delIdx.push_back(i); }
            }
        }
        Sampler::pool(delIdx, numDel);
        for (ID i : delIdx) {
            ID perId = i / numNd;
            ID ndId = i % numNd;
            Move m(Move::Type::Del);
            auto &nd = routings[perId].nodes[ndId];
            ID prevNdId = nd.links[0], succNdId = nd.links[1];
            m.periodIdA = perId;
            m.origNdA = nd;
            m.dtDistDelA = dtDistDelNd(prevNdId, succNdId, ndId);
            m.dtTraCost = m.dtDistDelA;
            //Log[Logger::Type::Rf] << "Del visit of p: " << perId << ", n: " << ndId << std::endl;
            chgVisLk(cur, m, true);
        }

        bool isTaboo = hasher.setBase(cur).isBaseTaboo();
        if (isTaboo) { continue; }
        // 调用网络流检查是否可行
        cur.invenCost = nf.run(cur, true);
        bool isFeasible = cur.invenCost != INVEN_COST_MAX;
        if (isFeasible) {
            InvenCost ic = cur.invenCost;
            cur.invenCost = cur.calcInvCost(true);
            cur.travelCost = cur.calcTraCost(true, true);
            cur.cost = cur.invenCost + cur.travelCost;
            break;
        }
		else {
            /*Log[Logger::Type::Rf]
				<< "Failed perturbation, feasible: " << isFeasible << ", taboo: " << isTaboo << std::endl;*/
        }
    } while (1);
    hasher.stBaseTaboo();
}

void hsh::mvirp::Solver::periodPtb(Solution &cur, Solution &base)
{
    Log[Logger::Type::Rf] << " .P ";
    auto dtDistDelNd = [&](ID prevId, ID succId, ID curId)->Distance {
        auto &d = params.distances;
        return d[prevId][succId] - (d[prevId][curId] + d[curId][succId]);
    };
    auto dtDistInsNd = [&](ID prevId, ID succId, ID curId)->Distance {
        return -dtDistDelNd(prevId, succId, curId);
    };
    auto &params = *cur.paramsPtr;
    int numPer = params.numPeriods;
    int numNd = params.numNodes;
    auto &vis = cur.visits;
    auto &routings = cur.routings;
    auto &dist = params.distances;
    NetflowCaller nf(params);

    do {
        if (timer.isTimeOut()) { return; }
        cur = base; // 每次扰动以历史最优解为出发点
        for (ID p = 0; p < numPer; ++p) {
            int numAdd = 2 + rand() % 2,
                numDel = 0 + rand() % 2;
            Vec<ID> addIdx, delIdx;
            // 增加访问
			for (ID n = 1; n < numNd; ++n) {
				if (!vis[p][n]) { addIdx.push_back(p * numNd + n); }
			}
            Sampler::pool(addIdx, numAdd);
            for (ID i : addIdx) {
                ID perId = i / numNd;
                ID ndId = i % numNd;
                auto &perRoutes = routings[perId];
                Move m(Move::Type::Add);
                m.periodIdB = perId;
                m.curNdA.assignId(ndId);
                for (ID rouId = 0; rouId < perRoutes.numRoutes; ++rouId) {
                    auto &dep = perRoutes.depots[rouId];
                    auto &nd = perRoutes.nodes;
                    for (ID staNdId = dep.id, curNdId = staNdId, succNdId = dep.links[1];
                        ;
                        curNdId = succNdId, succNdId = nd[curNdId].links[1]) {
                        auto dt = dtDistInsNd(curNdId, succNdId, ndId);
                        if (dt < m.dtDistAddA) { //TODO: 水池抽样
                            m.curNdA.assignRoute(rouId);
                            m.curNdA.assignLinks(curNdId, succNdId);
                            m.dtDistAddA = dt;
                        }
                        if (succNdId == staNdId) { break; }
                    }
                }
                if (perRoutes.numRoutes == 0) { // 单客户路径
                    m.curNdA.assignRoute(-2);
                    m.dtDistAddA = dist[0][ndId] + dist[ndId][0];
                }
                m.dtTraCost = m.dtDistAddA;
                chgVisLk(cur, m, true);
            }

            // 删除访问
			for (ID n = 1; n < numNd; ++n) {
				int i = p * numNd + n;
				if (std::find(addIdx.begin(), addIdx.end(), i) != addIdx.end()) { continue; }
				if (vis[p][n]) { delIdx.push_back(i); }
			}
            Sampler::pool(delIdx, numDel);
            for (ID i : delIdx) {
                ID perId = i / numNd;
                ID ndId = i % numNd;
                Move m(Move::Type::Del);
                auto &nd = routings[perId].nodes[ndId];
                ID prevNdId = nd.links[0], succNdId = nd.links[1];
                m.periodIdA = perId;
                m.origNdA = nd;
                m.dtDistDelA = dtDistDelNd(prevNdId, succNdId, ndId);
                m.dtTraCost = m.dtDistDelA;
                chgVisLk(cur, m, true);
            }
        }

        bool isTaboo = hasher.setBase(cur).isBaseTaboo();
        if (isTaboo) { continue; }
        // 调用网络流检查是否可行
        cur.invenCost = nf.run(cur, true);
        bool isFeasible = cur.invenCost != INVEN_COST_MAX;
        if (isFeasible) {
            InvenCost ic = cur.invenCost;
            cur.invenCost = cur.calcInvCost(true);
            cur.travelCost = cur.calcTraCost(true, true);
            cur.cost = cur.invenCost + cur.travelCost;
            break;
        }
        else {
            /*Log[Logger::Type::Rf]
                << "Failed perturbation, feasible: " << isFeasible << ", taboo: " << isTaboo << std::endl;*/
        }
    } while (1);
    hasher.stBaseTaboo();
}
