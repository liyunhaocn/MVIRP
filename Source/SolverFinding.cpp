#include "Solver.h"
#include "Delaunator/delaunator.hpp"

void hsh::mvirp::Solver::structureFinding(Solution &sln) {
    using namespace std;
    using MpSolverGurobi = goal::MpSolverGurobi;
    MpSolverGurobi grb;
    grb.setSeed(params.timeSeed);
    grb.setTimeLimit(params.finding.timeOnce);
    grb.setMaxThread(1);
    grb.setOutput(Log.stat[(size_t)Logger::Type::Fd]);

    // deliveries[p][n], 决策变量, 周期 p 时给节点 n 的配送量
    Vec<Vec<GRBVar>> deliveries(params.numPeriods, Vec<GRBVar>(params.numNodes));
    // inventLevel[p][n], 表达式, 周期 p 时节点 n 处库存水平
    Vec<Vec<GRBLinExpr>> invenLevels(params.numPeriods, Vec<GRBLinExpr>(params.numNodes));
    // degees[p][v][n], 表达式, 周期 p 时车辆 v 对节点 n 的度
    Vec<Vec<Vec<GRBLinExpr>>> degrees(params.numPeriods, Vec<Vec<GRBLinExpr>>(params.numVehicles, Vec<GRBLinExpr>(params.numNodes)));
    // x[p][v][n][m], 决策变量, 周期 p 时车辆 v 是否经过无向边 (n,m), n < m
    Vec<Vec<Vec<Vec<GRBVar>>>> x(params.numPeriods,
        Vec<Vec<Vec<GRBVar>>>(params.numVehicles, Vec<Vec<GRBVar>>(params.numNodes, Vec<GRBVar>(params.numNodes))));
    // y[p][v][n], 决策变量, 周期 p 时是否存在车辆 v 访问客户 n
    Vec<Vec<Vec<GRBVar>>> y(params.numPeriods, Vec<Vec<GRBVar>>(params.numVehicles, Vec<GRBVar>(params.numNodes)));
    // z[p][v][n], 决策变量, 周期 p 时是否存在车辆 v 只服务客户 n 的路径
    Vec<Vec<Vec<GRBVar>>> z(params.numPeriods, Vec<Vec<GRBVar>>(params.numVehicles, Vec<GRBVar>(params.numNodes)));

    // 规模缩减
    Vec<Coordinate> coords(params.numNodes * 2);
    for (ID n = 0; n < params.numNodes; ++n) {
        coords[2 * n] = params.nodes[n].x;
        coords[2 * n + 1] = params.nodes[n].y;
    }
    delaunator::Delaunator d(coords);
    Vec<Vec<bool>> exist(params.numNodes, Vec<bool>(params.numNodes, false));
    Vec<bool> cover(params.numNodes, false);
    for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
        auto n1 = d.triangles[i], n2 = d.triangles[i + 1], n3 = d.triangles[i + 2];
        exist[n1][n2] = exist[n2][n1] = true;
        exist[n2][n3] = exist[n3][n2] = true;
        exist[n3][n1] = exist[n1][n3] = true;
        cover[n1] = cover[n2] = cover[n3] = true;
        //cout << "vtx[1] " << n1 << ", vtx[2] " << n2 << ", vtx[3] " << n3 << endl;
    }
    for (ID n = 0; n < params.numNodes; ++n) {
        if (cover[n] == false) { Log[Logger::Type::Fd] << n << " not covered" << endl; }
    }Log[Logger::Type::Fd] << "all covered" << endl;
    auto hasEdge = [&](ID n, ID m) {
        return exist[n][m];
    };

    Vec<Distance> radiuses(params.numNodes);
    for (ID n = 0; n < params.numNodes; ++n) {
        auto tmp(params.distances[n]);
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        radiuses[n] = tmp[tmp.size() / 2];

    }
    auto skipEdge = [&](ID n, ID m) {
        return false;
        /*if (n > m) { swap(n, m); }
        if (params.distances[n][m] <= radiuses[n] || params.distances[n][m] <= radiuses[m]) {
            return false;
        }
        else { return true; }*/
        /*if (hasEdge(n, m)) {
            return false;
        }
        else {
            return true;
        }*/
    };

    // 添加决策变量, 给定取值范围
    for (ID p = 0; p < params.numPeriods; ++p) {
        for (ID n = 0; n < params.numNodes; ++n) {
            if (n == 0) {
                // 仓库上限是: 车辆总容量 * 给定使用率
                Quantity q = std::min(
                    InvenLevel(params.vehCapacity * params.numVehicles - params.statistics.maxCons),
                    params.nodes[n].maxLevel
                );
				Log[Logger::Type::Fd]
                    << "Set capa: " << q
                    << ", single veh: " << params.vehCapacity
                    << ", maxCons: " << params.statistics.maxCons
                    << std::endl;
                deliveries[p][n] = grb.addVar(MpSolverGurobi::Integer, -q, 0); // 仓库配送量取负
            }
            else {
                Quantity q = std::min(params.vehCapacity, params.nodes[n].maxLevel);
                deliveries[p][n] = grb.addVar(MpSolverGurobi::Integer, 0, q);
            }
        }
        for (ID v = 0; v < params.numVehicles; ++v) {
            auto &xpv = x[p][v];
            for (ID n = 0; n < params.numNodes; ++n) {
                z[p][v][n] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1);
                y[p][v][n] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1);
                for (ID m = n + 1; m < params.numNodes; ++m) {
                    if (skipEdge(n, m)) { continue; }
                    //cout << "add <" << n << ", " << m << ">" << endl;
                    xpv[n][m] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1); // 无向边, (n,m) 中 n < m
                }
            }
        }
    }

    // 添加约束
    for (ID n = 0; n < params.numNodes; ++n) {
        GRBLinExpr invenLevel = params.nodes[n].initLevel; // 逐周期的库存水平要求
        for (ID p = 0; p < params.numPeriods; ++p) {
            invenLevel += deliveries[p][n]; // 仓库的配送量是所有的客户配送量之和
            grb.addConstraint(invenLevel - params.nodes[n].maxLevel <= 0); // 配送时不超过库存水平上限
            invenLevel -= params.nodes[n].prodCons; // 仓库产生库存, 客户消费库存
            grb.addConstraint(params.nodes[n].minLevel - invenLevel <= 0); // 生产/消费后不少于库存水平下限
            invenLevels[p][n] = invenLevel;
        }
    }
    for (ID p = 0; p < params.numPeriods; ++p) {
        GRBLinExpr deliveryMatch;
        for (ID n = 0; n < params.numNodes; ++n) {
            deliveryMatch += deliveries[p][n];
        }
        grb.addConstraint(deliveryMatch == 0);
    }

    for (ID p = 0; p < params.numPeriods; ++p) {
        for (ID v = 0; v < params.numVehicles; ++v) {
            auto &xpv = x[p][v];
            for (ID n = 0; n < params.numNodes; ++n) {
                //cout << n << " ";
                GRBLinExpr degree;
                if (n == 0) {
                    for (ID m = 1; m < params.numNodes; ++m) {
                        if (skipEdge(n, m)) { continue; }
                        degree += xpv[n][m];
                    }
                    for (ID m = 1; m < params.numNodes; ++m) {
                        degree += (2 * z[p][v][m]);
                    }
                }
                else {
                    //int count1 = 0, count2 = 0;
                    for (ID m = 0; m < n; ++m) {
                        if (skipEdge(m, n)) { continue; }
                        //cout << "m " << m << endl;
                        degree += xpv[m][n];
                        //++count1;
                    }
                    for (ID m = n + 1; m < params.numNodes; ++m) {
                        if (skipEdge(n, m)) { continue; }
                        //cout << "m " << m << endl;
                        degree += xpv[n][m];
                        //++count2;
                    }
                    degree += (2 * z[p][v][n]);
                    //cout << count1 << ", " << count2 << endl;
                }
                degrees[p][v][n] = degree;
                //cout << p << ", " << v << ", " << n << endl;
                grb.addConstraint(degree - 2 * y[p][v][n] == 0);
            }
        }
    }

    // 每个周期各节点的车辆访问量之和二次限制该周期总配送量上限
    for (ID p = 0; p < params.numPeriods; ++p) {
        for (ID n = 0; n < params.numNodes; ++n) {
            GRBLinExpr allVisits;
            for (ID v = 0; v < params.numVehicles; ++v) {
                allVisits += y[p][v][n];
            }
            if (n == 0) {
                Quantity q = std::min(params.vehCapacity * params.numVehicles, params.nodes[n].maxLevel);
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

    GRBLinExpr objective;
    GRBLinExpr invenCost; // 不包括初始库存产生的成本
    for (ID n = 0; n < params.numNodes; ++n) {
        for (ID p = 0; p < params.numPeriods; ++p) {
            invenCost += (params.nodes[n].holdingCost * invenLevels[p][n]);
        }
    }
    GRBLinExpr travelCost;
    for (ID p = 0; p < params.numPeriods; ++p) {
        for (ID v = 0; v < params.numVehicles; ++v) {
            auto &xpv = x[p][v];
            for (ID n = 0; n < params.numNodes; ++n) {
                for (ID m = n + 1; m < params.numNodes; ++m) {
                    if (skipEdge(n, m)) { continue; }
                    travelCost += (params.distances[n][m] * xpv[n][m]);
                }
                if (n != 0) {
                    travelCost += (2 * params.distances[0][n] * z[p][v][n]);
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
        EliminationPolicy policy = params.finding.policy;
        Vec<ID> bestTour;
        Vec<ID> curTour;
        Vec<ID> pickedTour;
        ID noVeh = -1;
        bestTour.reserve(params.numNodes);
        curTour.reserve(params.numNodes);
        pickedTour.reserve(params.numNodes);
        Vec<bool> visited(params.numNodes);
        
        for (ID p = 0; p < params.numPeriods; ++p) {
            SamplerOne so;
            for (ID v = 0; v < params.numVehicles; ++v) { // 每个车辆 v 的路线都检测子回路
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
                            if (skipEdge(s, n)) { continue; }
                            if (n == s) { continue; }
                            if (e.isTrue(n < s ? xpv[n][s] : xpv[s][n])) {
                                //cout << "connect " << n << endl;
                            }
                        }
                        for (ID n = 0; n < params.numNodes; ++n) {
                            //cout << "test " << n << endl;
                            if (prev == n || prevPrev == n || visited[n]) { continue; } // 至少存在3个节点
                            if (skipEdge(prev, n)) { continue; }
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
                        else if (curTour.size() <params.finding.szRandSubTour && so.isPicked()) {
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
        //std::cout << "!!!" << std::endl;
    };

    CvrpCaller cvrpCaller(params);
    NetflowCaller netflow(params);
    Solution tmpSln(params);
    Cost bestFound = COST_MAX;
    auto nodeSetHandler = [&](MpSolverGurobi::MpEvent &e) {
        // 统计周期内客户的访问情况, 配送量作为客户的需求, 交给CVRP进行求解
        // 并消除子回路

        tmpSln.resetCost();
        bool isFeasible = true;
        for (ID p = 0; p < params.numPeriods; ++p) {
            for (ID n = 0; n < params.numNodes; ++n) {
                tmpSln.visits[p][n] = false;
                tmpSln.deliveries[p][n] = EMPTY;
                for (ID v = 0; v < params.numVehicles; ++v) {
                    for (ID m = 0; m < params.numNodes; ++m) {
                        if (skipEdge(m, n)) { continue; }
                        if (m == n) { continue; }
                        if (e.isTrue(m < n ? x[p][v][m][n] : x[p][v][n][m]) &&
                            abs(lround(e.getValue(deliveries[p][n]))) > 0) {
                            tmpSln.visits[p][0] = true;
                            tmpSln.deliveries[p][0] = lround(e.getValue(deliveries[p][0]));
                            tmpSln.visits[p][n] = true;
                            tmpSln.deliveries[p][n] = lround(e.getValue(deliveries[p][n]));
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
                        tmpSln.deliveries[p][0] = lround(e.getValue(deliveries[p][0]));
                        tmpSln.visits[p][n] = true;
                        tmpSln.deliveries[p][n] = lround(e.getValue(deliveries[p][n]));
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
                Log[Logger::Type::Cri] << "Clients delivered while depot not visited." << endl;
            }
            
			if (cvrpCaller.solve(p, tmpSln, CvrpCaller::Mode::GenInit, -1, DISTANCE_MAX)) {
                tmpSln.travelCost += std::accumulate(tmpSln.routings[p].dist.begin(), tmpSln.routings[p].dist.end(), 0);
            }
            else {
                tmpSln.travelCost += DISTANCE_MAX;
                isFeasible = false;
                break; // 不可行CVRP子问题, 直接中止
            }
        }

        subtourHandler(e);
        if (isFeasible) {
            tmpSln.invenCost = e.getValue(invenCost);
            tmpSln.cost = tmpSln.invenCost + tmpSln.travelCost;

            Log[Logger::Type::Nf]
                << "new found " << tmpSln.cost << ", inven " << tmpSln.invenCost << ", travel " << tmpSln.travelCost << endl;
            
            if (tmpSln.cost < bestFound) {
                bestFound = tmpSln.cost;
                Log[Logger::Type::Bf] << "best found now " << bestFound << endl;
                std::swap(tmpSln, sln);
                //for (ID p = 0; p < params.numPeriods; ++p) { cvrpCaller.solve(p, sln, CvrpCaller::Mode::Pop, -1, sln.routings[p].totDist); }
                sln.calcInvCost(true); // TODO[dbg]
                sln.calcTraCost(true, true);
                //Log[Logger::Type::Bf] << "after pop, best found now " << sln.cost << endl;
            }
        }
    };

    grb.setMipSlnEvent(nodeSetHandler);
    grb.optimize();


    Log[Logger::Type::Fd] << "structure finding done." << endl;
}
