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

    // deliveries[p][n], ���߱���, ���� p ʱ���ڵ� n ��������
    Vec<Vec<GRBVar>> deliveries(numPer, Vec<GRBVar>(numNds));
    // inventLevel[p][n], ���ʽ, ���� p ʱ�ڵ� n �����ˮƽ
    Vec<Vec<GRBLinExpr>> invenLevels(numPer, Vec<GRBLinExpr>(numNds));
    // degees[p][v][n], ���ʽ, ���� p ʱ���� v �Խڵ� n �Ķ�
    Vec<Vec<Vec<GRBLinExpr>>> degrees(numPer, Vec<Vec<GRBLinExpr>>(numVeh, Vec<GRBLinExpr>(numNds)));
    // x[p][v][n][m], ���߱���, ���� p ʱ���� v �Ƿ񾭹������ (n,m), n < m
    Vec<Vec<Vec<Vec<GRBVar>>>> x(numPer,
        Vec<Vec<Vec<GRBVar>>>(numVeh, Vec<Vec<GRBVar>>(numNds, Vec<GRBVar>(numNds))));
    // y[p][v][n], ���߱���, ���� p ʱ�Ƿ���ڳ��� v ���ʿͻ� n
    Vec<Vec<Vec<GRBVar>>> y(numPer, Vec<Vec<GRBVar>>(numVeh, Vec<GRBVar>(numNds)));
    // z[p][v][n], ���߱���, ���� p ʱ�Ƿ���ڳ��� v ֻ����ͻ� n ��·��
    Vec<Vec<Vec<GRBVar>>> z(numPer, Vec<Vec<GRBVar>>(numVeh, Vec<GRBVar>(numNds)));
    // vehLoads[p][v], ���ʽ, ����v������p����������֮��
    Vec<Vec<GRBLinExpr>> vehLoads(numPer, Vec<GRBLinExpr>(numVeh));

    // ��Ӿ��߱���, ���ݸ���ȡֵ��Χ
    for (ID p = 0; p < numPer; ++p) {
        for (ID n = 0; n < numNds; ++n) {
            if (n == 0) {
                // �ֿ�������: ���������� * ����ʹ����
                Quantity q = std::min(
                    InvenLevel(params.vehCapacity * params.numVehicles - params.statistics.maxCons),
                    params.nodes[n].maxLevel
                );
                deliveries[p][n] = grb.addVar(MpSolverGurobi::Integer, -q, 0); // �ֿ�������ȡ��
            }
            else {
                int coef = !isFixedPer(p) ? 1 : (vis[p][n] ? 1 : 0); // �̶������ɷ���״̬����
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
                        xpv[n][m] = grb.addVar(MpSolverGurobi::VariableType::Bool, 0, 1); // �����, (n,m) �� n < m
                    }
                }
            }
        }
    }

    // ���Լ��
    // �������ڵ�������Լ��
    for (ID n = 0; n < numNds; ++n) {
        GRBLinExpr invenLevel = params.nodes[n].initLevel; // �����ڵĿ��ˮƽҪ��
        for (ID p = 0; p < numPer; ++p) {
            invenLevel += deliveries[p][n]; // �ֿ�������������еĿͻ�������֮��
            grb.addConstraint(invenLevel - params.nodes[n].maxLevel <= 0); // ����ʱ���������ˮƽ����
            invenLevel -= params.nodes[n].prodCons; // �ֿ�������, �ͻ����ѿ��
            grb.addConstraint(params.nodes[n].minLevel - invenLevel <= 0); // ����/���Ѻ����ڿ��ˮƽ����
            invenLevels[p][n] = invenLevel;
        }
    }
    for (ID p = 0; p < numPer; ++p) {
        GRBLinExpr deliveryMatch;
        for (ID n = 0; n < numNds; ++n) {
            deliveryMatch += deliveries[p][n];
        }
        grb.addConstraint(deliveryMatch == 0); // ÿ���ڵĲֿ��ͳ��Ϳͻ��յ���������һ��
    }

    // δ�̶����ڵĶ�Լ��
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

    // δ�̶����ڸ��ڵ�ĳ���������֮�Ͷ������Ƹ�����������������
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

    // �̶������ڳ����غɲ���������������, �ͻ���������ϵ���ɷ���״̬�ͳ���ID��ͬȷ��
    // ֻΪ��ʹ�õĳ�����Ӹ�Լ��
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
    GRBLinExpr invenCost; // ��������ʼ�������ĳɱ�
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

    // �ӻ�·����
    // �������2������ӻ�·, Լ��Ҫ��������ʸýڵ��Ϊ2, ����ʱ��Ȼ������2����ͬ�Ľڵ�
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

        // ֻ��δ�̶������ڵ��ӻ�·����ͳ��
        for (ID p = 0; p < numPer; ++p) {
            if (!isFixedPer(p)) {
                SamplerOne so;
                for (ID v = 0; v < numVeh; ++v) { // ÿ������ v ��·�߶�����ӻ�·
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
                            // z ������Ӧ��·�������ӻ�·, ����Ҫ����
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
                                if (prev == n || prevPrev == n || visited[n]) { continue; } // ���ٴ���3���ڵ�
                                if (!e.isTrue(n < prev ? xpv[n][prev] : xpv[prev][n])) { continue; } // ����ͼ
                                //cout << "delivery " << lround(e.getValue(deliveries[p][n])) << endl;
                                if (s != 0) {
                                    curTour.push_back(n);
                                    //std::cout << "push " << n << std::endl;
                                } // �����ֿ��·������Ҫ����
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
                                eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // ����ͼ
                            }
                            e.addLazy(eliminated <= static_cast<double>(curTour.size() - 1));
                            break; // ��ģʽ���һ���ӻ�·���˳�
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
                            eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // ����ͼ
                        }
                        e.addLazy(eliminated <= static_cast<double>(bestTour.size() - 1));
                        bestTour.clear();
                    }
                    if ((policy == EliminationPolicy::SVRandSubTour) && !pickedTour.empty()) {
                        GRBLinExpr eliminated;
                        ID prev = pickedTour.back();
                        for (auto n = pickedTour.begin(); n != pickedTour.end(); prev = *n, ++n) {
                            eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // ����ͼ
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
                        eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // ����ͼ
                    }
                    e.addLazy(eliminated <= static_cast<double>(bestTour.size() - 1));
                    bestTour.clear();
                }
                if ((policy == EliminationPolicy::MVRandSubTour) && !pickedTour.empty()) {
                    auto &xpv = x[p][noVeh];
                    GRBLinExpr eliminated;
                    ID prev = pickedTour.back();
                    for (auto n = pickedTour.begin(); n != pickedTour.end(); prev = *n, ++n) {
                        eliminated += (*n < prev ? xpv[*n][prev] : xpv[prev][*n]); // ����ͼ
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
        // �������ڽڵ��������
        for (ID p = 0; p < numPer; ++p) {
            for (ID n = 0; n < numNds; ++n) {
                tmpSln.deliveries[p][n] = lround(e.getValue(deliveries[p][n]));
            }
        }
        // δ�̶������ڵķ��ʺ�·�����¹滮
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
                    tmpSln.travelCost += DISTANCE_MAX; // ������ CVRP ������
                    isFeasible = false;
                    break;
                }
            }
        }

        subtourHandler(e);
        if (isFeasible) {
            // �̶�������ֻ·���������¼���
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


