#include "Solver.h"

void hsh::mvirp::Solver::structureAdjusting(Solution &sln){
    auto &vis = sln.visits;
    int numPer = params.numPeriods;
	Vec<int> lens = (params.numPeriods == 3 ? Vec<int>{ 2, 1 } : Vec<int>{ 2, 2 });
	for (int len : lens) {
		for (int pStart = 0, pLast = pStart + len - 1; pLast < numPer; ++pStart, ++pLast) {
            if (timer.isTimeOut()) { return; }
			Log[Logger::Type::Aj] << "Adjusting periods: " << pStart << " to " << pLast << std::endl;
            adjustPartial(sln, pStart, pLast);
		}
	}
    Log[Logger::Type::Aj] << "Structure adjusting done." << std::endl;
}

bool hsh::mvirp::Solver::adjustPartial(Solution &sln, int perStart, int perLast)
{
    using namespace std;
    using MpSolverGurobi = goal::MpSolverGurobi;
    MpSolverGurobi grb;
    grb.setSeed(params.timeSeed);
    grb.setMaxThread(params.adjusting.maxThreads);
    grb.setTimeLimit(params.adjusting.timeOnce);
    grb.setOutput(Log.stat[(size_t)Logger::Type::Aj]);

    auto isFixedPer = [&](ID id) {return id<perStart || id>perLast; };
    auto &vis = sln.visits;
    auto &routings = sln.routings;
    int perLen = perLast - perStart + 1;
    int numPer = params.numPeriods;
    int numNds = params.numNodes;
    int numVeh = params.numVehicles;

    // deliveries[p][n], 决策变量, 周期 p 时给节点 n 的配送量
    Vec<Vec<GRBVar>> deliveries(numPer, Vec<GRBVar>(numNds));
    // inventLevel[p][n], 表达式, 周期 p 时节点 n 处库存水平
    Vec<Vec<GRBLinExpr>> invenLevels(numPer, Vec<GRBLinExpr>(numNds));
    // degees[p][v][n], 表达式, 周期 p 时车辆 v 对节点 n 的度
    Vec<Vec<Vec<GRBLinExpr>>> degrees(numPer, Vec<Vec<GRBLinExpr>>(numVeh, Vec<GRBLinExpr>(numNds)));
    // x[p][v][n][m], 决策变量, 周期 p 时车辆 v 是否经过无向边 (n,m), n < m
    Vec<Vec<Vec<Vec<GRBVar>>>> x(numPer,
        Vec<Vec<Vec<GRBVar>>>(numVeh, Vec<Vec<GRBVar>>(numNds, Vec<GRBVar>(numNds))));
    // y[p][v][n], 决策变量, 周期 p 时是否存在车辆 v 访问客户 n
    Vec<Vec<Vec<GRBVar>>> y(numPer, Vec<Vec<GRBVar>>(numVeh, Vec<GRBVar>(numNds)));
    // z[p][v][n], 决策变量, 周期 p 时是否存在车辆 v 只服务客户 n 的路径
    Vec<Vec<Vec<GRBVar>>> z(numPer, Vec<Vec<GRBVar>>(numVeh, Vec<GRBVar>(numNds)));
    // vehLoads[p][v], 表达式, 车辆v于周期p所有配送量之和
    Vec<Vec<GRBLinExpr>> vehLoads(numPer, Vec<GRBLinExpr>(numVeh));

    // 添加决策变量, 根据给定取值范围
    for (ID p = 0; p < numPer; ++p) {
        for (ID n = 0; n < numNds; ++n) {
            if (n == 0) {
                // 仓库上限是: 车辆总容量 * 给定使用率
                Quantity q = std::min(
                    InvenLevel(params.vehCapacity * params.numVehicles - params.statistics.maxCons),
                    params.nodes[n].maxLevel
                );
                deliveries[p][n] = grb.addVar(MpSolverGurobi::Integer, -q, 0); // 仓库配送量取负
            }
            else {
                int coef = !isFixedPer(p) ? 1 : (vis[p][n] ? 1 : 0); // 固定周期由访问状态决定
                Quantity q = std::min(params.vehCapacity, params.nodes[n].maxLevel);
                deliveries[p][n] = grb.addVar(MpSolverGurobi::Integer, 0, coef * q);
            }
        }
        if (!isFixedPer(p)) {
            for (ID v = 0; v < numVeh; ++v) {
                auto &xpv = x[p][v];
                for (ID n = 0; n < numNds; ++n) {
                    z[p][v][n] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1);
                    y[p][v][n] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1);
                    for (ID m = n + 1; m < numNds; ++m) {
                        xpv[n][m] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1); // 无向边, (n,m) 中 n < m
                    }
                }
            }
        }
    }

    // 添加约束
    // 所有周期的配送量约束
    for (ID n = 0; n < numNds; ++n) {
        GRBLinExpr invenLevel = params.nodes[n].initLevel; // 逐周期的库存水平要求
        for (ID p = 0; p < numPer; ++p) {
            invenLevel += deliveries[p][n]; // 仓库的配送量是所有的客户配送量之和
            grb.addConstraint(invenLevel - params.nodes[n].maxLevel <= 0); // 配送时不超过库存水平上限
            invenLevel -= params.nodes[n].prodCons; // 仓库产生库存, 客户消费库存
            grb.addConstraint(params.nodes[n].minLevel - invenLevel <= 0); // 生产/消费后不少于库存水平下限
            invenLevels[p][n] = invenLevel;
        }
    }
    for (ID p = 0; p < numPer; ++p) {
        GRBLinExpr deliveryMatch;
        for (ID n = 0; n < numNds; ++n) {
            deliveryMatch += deliveries[p][n];
        }
        grb.addConstraint(deliveryMatch == 0); // 每周期的仓库送出和客户收到的配送量一致
    }

    // 未固定周期的度约束
    for (ID p = 0; p < numPer; ++p) {
        if (!isFixedPer(p)) {
            for (ID v = 0; v < numVeh; ++v) {
                auto &xpv = x[p][v];
                for (ID n = 0; n < numNds; ++n) {
                    GRBLinExpr degree;
                    if (n == 0) {
                        for (ID m = 1; m < numNds; ++m) {
                            degree += xpv[n][m];
                        }
                        for (ID m = 1; m < numNds; ++m) {
                            degree += (2 * z[p][v][m]);
                        }
                    }
                    else {
                        for (ID m = 0; m < n; ++m) {
                            degree += xpv[m][n];
                        }
                        for (ID m = n + 1; m < numNds; ++m) {
                            degree += xpv[n][m];
                        }
                        degree += (2 * z[p][v][n]);
                    }
                    degrees[p][v][n] = degree;
                    grb.addConstraint(degree - 2 * y[p][v][n] == 0);
                }
            }
        }
    }

    // 未固定周期各节点的车辆访问量之和二次限制该周期总配送量上限
    for (ID p = 0; p < numPer; ++p) {
        if (!isFixedPer(p)) {
            for (ID n = 0; n < numNds; ++n) {
                GRBLinExpr allVisits;
                for (ID v = 0; v < numVeh; ++v) {
                    allVisits += y[p][v][n];
                }
                if (n == 0) {
                    Quantity q = std::min(params.vehCapacity * numVeh, params.nodes[n].maxLevel);
                    double qCoeff = -1;
                    grb.addConstraint(qCoeff * deliveries[p][0] - q * allVisits <= 0);
                }
                else {
                    grb.addConstraint(allVisits <= 1);
                    Quantity q = std::min(params.vehCapacity, params.nodes[n].maxLevel);
                    double qCoeff = 1;
                    grb.addConstraint(qCoeff * deliveries[p][n] - q * allVisits <= 0);
                }
            }
        }
        
    }

    // 固定周期内车辆载荷不超过其容量上限, 客户的配送量系数由访问状态和车辆ID共同确定
    // 只为已使用的车辆添加该约束
    for (ID p = 0; p < numPer; ++p) {
        if (isFixedPer(p)) {
            auto &routes = routings[p];
            for (ID v = 0; v < routes.numRoutes; ++v) {
                GRBLinExpr loads;
                for (ID n = 1; n < numNds; ++n) {
                    int coef = (vis[p][n] && routes.nodes[n].routeId == v) ? 1 : 0;
                    loads += (coef * deliveries[p][n]);
                }
                vehLoads[p][v] = loads;
                grb.addConstraint(loads <= params.vehCapacity);
            }
        }
    }

    GRBLinExpr objective;
    GRBLinExpr invenCost; // 不包括初始库存产生的成本
    for (ID n = 0; n < params.numNodes; ++n) {
        for (ID p = 0; p < params.numPeriods; ++p) {
            invenCost += (params.nodes[n].holdingCost * invenLevels[p][n]);
        }
    }
    GRBLinExpr travelCost;
    for (ID p = 0; p < numPer; ++p) {
        if (!isFixedPer(p)) {
            for (ID v = 0; v < numVeh; ++v) {
                auto &xpv = x[p][v];
                for (ID n = 0; n < params.numNodes; ++n) {
                    for (ID m = n + 1; m < params.numNodes; ++m) {
                        travelCost += (params.distances[n][m] * xpv[n][m]);
                    }
                    if (n != 0) {
                        travelCost += (2 * params.distances[0][n] * z[p][v][n]);
                    }
                }
            }
        }
    }
    objective = invenCost + travelCost;
    grb.setObjective(objective, MpSolverGurobi::OptimaDirection::Minimize);

    // 子回路消除
    // 不会存在2个点的子回路, 约束要求如果访问该节点度为2, 满足时必然是连接2个不同的节点
    auto subtourHandler = [&](MpSolverGurobi::MpEvent &e)
    {
        using EliminationPolicy = Parameters::EliminationPolicy;
        EliminationPolicy policy = params.adjusting.policy;
        Vec<ID> bestTour;
        Vec<ID> curTour;
        Vec<ID> pickedTour;
        ID noVeh = -1;
        bestTour.reserve(params.numNodes);
        curTour.reserve(params.numNodes);
        pickedTour.reserve(params.numNodes);
        Vec<bool> visited(params.numNodes);

        // 只对未固定周期内的子回路进行统计
        for (ID p = 0; p < numPer; ++p) {
            if (!isFixedPer(p)) {
                SamplerOne so;
                for (ID v = 0; v < numVeh; ++v) { // 每个车辆 v 的路线都检测子回路
                    if (policy == EliminationPolicy::MinSubTour || policy == EliminationPolicy::SVRandSubTour) { so.rstCnt(); }
                    auto &xpv = x[p][v];
                    curTour.clear();
                    //std::cout << p << ", v " << v << std::endl;
                    for (ID i = 0; i < visited.size(); ++i) { visited[i] = false; }
                    for (ID s = 0; s < params.numNodes; ++s) {
                        if (visited[s] == true) { continue; }
                        //std::cout << "start " << s << std::endl;
                        //cout << "delivery " << lround(e.getValue(deliveries[p][s])) << endl;
                        ID prev = s;
                        ID prevPrev = s;
                        do {
                            // z 变量对应的路径不是子回路, 不需要处理
                            for (ID n = 1; n < params.numNodes; ++n) {
                                if (e.isTrue(z[p][v][n])) {
                                    //cout << "one-customer tour for " << n << endl;
                                }
                            }
                            for (ID n = 0; n < params.numNodes; ++n) {
                                if (n == s) { continue; }
                                if (e.isTrue(n < s ? xpv[n][s] : xpv[s][n])) {
                                    //cout << "connect " << n << endl;
                                }
                            }
                            for (ID n = 0; n < params.numNodes; ++n) {
                                //cout << "test " << n << endl;
                                if (prev == n || prevPrev == n || visited[n]) { continue; } // 至少存在3个节点
                                if (!e.isTrue(n < prev ? xpv[n][prev] : xpv[prev][n])) { continue; } // 无向图
                                //cout << "delivery " << lround(e.getValue(deliveries[p][n])) << endl;
                                if (s != 0) {
                                    curTour.push_back(n);
                                    //std::cout << "push " << n << std::endl;
                                } // 包含仓库的路径不需要加入
                                //std::cout << s << ", prev " << prev << ", n " << n << ", tour size " << tour.size() << std::endl;
                                prevPrev = prev;
                                prev = n;
                                visited[n] = true;
                                break;
                            }
                        } while (prev != s);
                        if (curTour.empty()) { continue; }

                        if (policy == EliminationPolicy::FirstSubTour) {
                            GRBLinExpr eliminated;
                            ID prev = curTour.back();
                            for (auto n = curTour.begin(); n != curTour.end(); prev = *n, ++n) {
                                eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // 无向图
                            }
                            e.addLazy(eliminated <= static_cast<double>(curTour.size() - 1));
                            break; // 该模式添加一个子回路即退出
                        }

                        if (policy == EliminationPolicy::MinSubTour || policy == EliminationPolicy::MinMinSubTour) {
                            if (bestTour.empty() || curTour.size() < bestTour.size()) {
                                swap(bestTour, curTour);
                                noVeh = v;
                            }
                            else if (curTour.size() == bestTour.size() && so.isPicked()) {
                                swap(bestTour, curTour);
                                noVeh = v;
                            }
                        }

                        if (policy == EliminationPolicy::SVRandSubTour || policy == EliminationPolicy::MVRandSubTour) {
                            if (pickedTour.empty() || curTour.size() < params.finding.szRandSubTour) {
                                swap(pickedTour, curTour);
                                noVeh = v;
                            }
                            else if (curTour.size() < params.finding.szRandSubTour && so.isPicked()) {
                                swap(pickedTour, curTour);
                                noVeh = v;
                            }
                        }
                    }
                    if ((policy == EliminationPolicy::MinSubTour) && !bestTour.empty()) {
                        GRBLinExpr eliminated;
                        ID prev = bestTour.back();
                        for (auto n = bestTour.begin(); n != bestTour.end(); prev = *n, ++n) {
                            eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // 无向图
                        }
                        e.addLazy(eliminated <= static_cast<double>(bestTour.size() - 1));
                        bestTour.clear();
                    }
                    if ((policy == EliminationPolicy::SVRandSubTour) && !pickedTour.empty()) {
                        GRBLinExpr eliminated;
                        ID prev = pickedTour.back();
                        for (auto n = pickedTour.begin(); n != pickedTour.end(); prev = *n, ++n) {
                            eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // 无向图
                        }
                        e.addLazy(eliminated <= static_cast<double>(pickedTour.size() - 1));
                        pickedTour.clear();
                    }
                }
                if ((policy == EliminationPolicy::MinMinSubTour) && !bestTour.empty()) {
                    auto &xpv = x[p][noVeh];
                    GRBLinExpr eliminated;
                    ID prev = bestTour.back();
                    for (auto n = bestTour.begin(); n != bestTour.end(); prev = *n, ++n) {
                        eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // 无向图
                    }
                    e.addLazy(eliminated <= static_cast<double>(bestTour.size() - 1));
                    bestTour.clear();
                }
                if ((policy == EliminationPolicy::MVRandSubTour) && !pickedTour.empty()) {
                    auto &xpv = x[p][noVeh];
                    GRBLinExpr eliminated;
                    ID prev = pickedTour.back();
                    for (auto n = pickedTour.begin(); n != pickedTour.end(); prev = *n, ++n) {
                        eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // 无向图
                    }
                    e.addLazy(eliminated <= static_cast<double>(pickedTour.size() - 1));
                    pickedTour.clear();
                }
            }
        }
        //std::cout << "!!!" << std::endl;
    };

    CvrpCaller cvrpCaller(params);
    Solution tmpSln(params);
    tmpSln = sln;
    Cost bestFound = sln.cost;
    auto nodeSetHandler = [&](MpSolverGurobi::MpEvent &e) {
        tmpSln.resetCost();
        bool isFeasible = true;
        // 所有周期节点的配送量
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 0; n < numNds; ++n) {
                tmpSln.deliveries[p][n] = lround(e.getValue(deliveries[p][n]));
            }
        }
        // 未固定周期内的访问和路径重新规划
        for (ID p = 0; p < numPer; ++p) {
            if (!isFixedPer(p)) {
                for (ID n = 0; n < numNds; ++n) {
                    tmpSln.visits[p][n] = false;
                    for (ID v = 0; v < params.numVehicles; ++v) {
                        for (ID m = 0; m < params.numNodes; ++m) {
                            if (m == n) { continue; }
                            if (e.isTrue(m < n ? x[p][v][m][n] : x[p][v][n][m]) &&
                                abs(lround(e.getValue(deliveries[p][n]))) > 0) {
                                tmpSln.visits[p][0] = true;
                                tmpSln.visits[p][n] = true;
                                Log[Logger::Type::Db]
                                    << "x:p[" << p << "],v[" << v << "],deliv[" << n << "] " << tmpSln.deliveries[p][n] <<
                                    ", m" << m << ", " << tmpSln.deliveries[p][m] << ", " << tmpSln.deliveries[p][0] <<
                                    ", actually " << e.getValue(deliveries[p][m]) << ", " << e.getValue(deliveries[p][0]) <<
                                    endl;
                                break;
                            }
                        }
                        if (tmpSln.visits[p][n] == true) { break; }
                        if (n != 0 && e.isTrue(z[p][v][n]) &&
                            abs(lround(e.getValue(deliveries[p][n]))) > 0) {
                            tmpSln.visits[p][0] = true;
                            tmpSln.visits[p][n] = true;
                            Log[Logger::Type::Db]
                                << "z:p[" << p << "],v[" << v << "],deliv[" << n << "] " << tmpSln.deliveries[p][n] <<
                                ", actually " << e.getValue(deliveries[p][n]) << ", " << e.getValue(deliveries[p][0]) <<
                                endl;
                            break;
                        }
                    }
                }
                Log[Logger::Type::Db]
                    << "p " << p << ", visit " << tmpSln.visits[p][0] << ", delivery " << tmpSln.deliveries[p][0] << endl;
                bool depotVisit = false;
                for (ID v = 0; v < params.numVehicles; ++v) {
                    Log[Logger::Type::Db] << "v" << v << ": " << lround(e.getValue(deliveries[p][0])) << " ";
                    if (abs(lround(e.getValue(deliveries[p][0]))) > 0) { depotVisit = true; }
                }Log[Logger::Type::Db] << endl;
                if (tmpSln.visits[p][0] != depotVisit) {
                    Log[Logger::Type::Info] << "Clients delivered while depot not visited." << endl;
                }

                if (cvrpCaller.solve(p, tmpSln, CvrpCaller::Mode::GenInit, -1, DISTANCE_MAX)) {
                    tmpSln.travelCost += std::accumulate(tmpSln.routings[p].dist.begin(), tmpSln.routings[p].dist.end(), 0);
                }
                else {
                    tmpSln.travelCost += DISTANCE_MAX; // 不可行 CVRP 子问题
                    isFeasible = false;
                    break;
                }
            }
        }

        subtourHandler(e);
        if (isFeasible) {
            // 固定周期内只路径长度重新计算
            for (ID p = 0; p < numPer; ++p) {
                if (isFixedPer(p)) {
                    tmpSln.travelCost += tmpSln.calcTraCost(p, true, true);
                }
            }

            tmpSln.invenCost = e.getValue(invenCost);
            tmpSln.cost = tmpSln.invenCost + tmpSln.travelCost;
            Log[Logger::Type::Nf]
                << "current found " << tmpSln.cost << ", inven " << tmpSln.invenCost << ", travel " << tmpSln.travelCost << endl;

            if (tmpSln.cost < bestFound) {
                bestFound = tmpSln.cost;
                Log[Logger::Type::Bf] << "best found now " << bestFound << endl;
                sln = tmpSln;
                //for (ID p = 0; p < params.numPeriods; ++p) { cvrpCaller.solve(p, sln, CvrpCaller::Mode::Pop, -1, sln.routings[p].totDist); }
                sln.calcInvCost(true); // TODO[dbg]
                sln.calcTraCost(true, true);
                //Log[Logger::Type::Bf] << "after pop, best found now " << sln.cost << endl;
                sln.time = timer.curTime();
            }
        }
    };

    grb.setMipSlnEvent(nodeSetHandler);
    Log[Logger::Type::Aj] << "structure adjusting model init done." << endl;

    grb.optimize();

    return true;
}


