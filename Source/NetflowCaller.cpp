#include "NetflowCaller.h"

void hsh::mvirp::NetflowCaller::init()
{
    // 映射关系
    auto sNd = g.nodeFromId(0);
    auto tNd = g.nodeFromId(1);
    auto depNd = [&](ID perId) {return g.nodeFromId(2 + perId); };
    auto vehNd = [&](ID perId, ID vehId) {return g.nodeFromId(2 + numPer + numVeh * perId + vehId); };
    auto cliNd = [&](ID perId, ID cliId) {return g.nodeFromId(2 + numPer + totVeh + numCli * perId + cliId); };

    g.reserveNode(totNd);
    for (int i = 0; i < totNd; ++i) { g.addNode(); }
    g.reserveArc(totArc);

    /* 构造完整网络流 */
    costPtr = new CostMap(g);
    capaPtr = new CapaMap(g);
    demaPtr = new DemaMap(g);
    flowAmount = 0;
    for (ID ndId = 0; ndId < numNd; ++ndId) { flowAmount += params.nodes[ndId].initLevel; }
    flowAmount += -params.nodes[0].prodCons * numPer;

	auto &cost = *costPtr;
    auto &capa = *capaPtr;
    auto &dema = *demaPtr;
    ID arcId = -1;
    // 每个周期仓库的生产量
    for (ID perId = 0; perId < numPer; ++perId) {
        arcId = (ID)g.id(g.addArc(sNd, depNd(perId)));
        cost[g.arcFromId(arcId)] = 0;
        capa[g.arcFromId(arcId)] = -params.nodes[0].prodCons;
    }
    // 初始库存
    arcId = (ID)g.id(g.addArc(sNd, depNd(0)));
    cost[g.arcFromId(arcId)] = 0;
    capa[g.arcFromId(arcId)] = params.nodes[0].initLevel;
    for (ID cliId = 0; cliId < numCli; ++cliId) {
        arcId = (ID)g.id(g.addArc(sNd, cliNd(0, cliId)));
        cost[g.arcFromId(arcId)] = 0;
        capa[g.arcFromId(arcId)] = params.nodes[cliId + 1].initLevel;
    }
    // 每周期库存
    for (ID perId = 0; perId < numPer; ++perId) {
        auto nxtNdDep = (perId == numPer - 1 ? tNd : depNd(perId + 1));
        arcId = (ID)g.id(g.addArc(depNd(perId), nxtNdDep));
        cost[g.arcFromId(arcId)] = params.nodes[0].holdingCost * precise;
        capa[g.arcFromId(arcId)] = flowAmount;
        dema[g.arcFromId(arcId)] = params.nodes[0].minLevel;
        for (ID cliId = 0; cliId < numCli; ++cliId) {
            auto nxtNdCli = (perId == numPer - 1 ? tNd : cliNd(perId + 1, cliId));
            arcId = (ID)g.id(g.addArc(cliNd(perId, cliId), nxtNdCli));
            cost[g.arcFromId(arcId)] = params.nodes[cliId + 1].holdingCost * precise;
            capa[g.arcFromId(arcId)] = params.nodes[cliId + 1].maxLevel - params.nodes[cliId + 1].prodCons;
            dema[g.arcFromId(arcId)] = params.nodes[cliId + 1].minLevel;
        }
    }
    // 每周期仓库到车辆
    for (ID perId = 0; perId < numPer; ++perId) {
        for (ID vehId = 0; vehId < numVeh; ++vehId) {
            arcId = (ID)g.id(g.addArc(depNd(perId), vehNd(perId, vehId)));
            cost[g.arcFromId(arcId)] = 0;
            capa[g.arcFromId(arcId)] = params.vehCapacity;
        }
    }
    // 每辆车的配送量
    for (ID perId = 0; perId < numPer; ++perId) {
        for (ID vehId = 0; vehId < numVeh; ++vehId) {
            for (ID cliId = 0; cliId < numCli; ++cliId) {
                arcId = (ID)g.id(g.addArc(vehNd(perId, vehId), cliNd(perId, cliId)));
                cost[g.arcFromId(arcId)] = 0;
                capa[g.arcFromId(arcId)] = params.vehCapacity;
            }
        }
    }
    // 每周期客户的消耗量
    for (ID perId = 0; perId < numPer; ++perId) {
        for (ID cliId = 0; cliId < numCli; ++cliId) {
            arcId = (ID)g.id(g.addArc(cliNd(perId, cliId), tNd));
            cost[g.arcFromId(arcId)] = 0;
            capa[g.arcFromId(arcId)] = params.nodes[cliId + 1].prodCons;
            dema[g.arcFromId(arcId)] = params.nodes[cliId + 1].prodCons;
        }
    }

    netSplexPtr = new NetSplex(g);
}

hsh::mvirp::InvenCost hsh::mvirp::NetflowCaller::run(Solution& sln, bool assignDeliv)
{
    //double clk = clock();
	//std::ofstream of("netflowtakes.txt", std::ios_base::app);
    auto &cost = *costPtr;
    auto &capa = *capaPtr;
    auto &dema = *demaPtr;
    ID arcId = -1;

    /* 根据访问状态修改每周期车辆到客户的配送量上限 */
    for (ID perId = 0; perId < numPer; ++perId) {
        for (ID vehId = 0; vehId < numVeh; ++vehId) {
            for (ID cliId = 0; cliId < numCli; ++cliId) {
                arcId = delivOffset + perId * numVeh * numCli + vehId * numCli + cliId;
                bool visByVeh = sln.visits[perId][cliId + 1]
                    && (sln.routings[perId].nodes[cliId + 1].routeId == vehId);
                if (!visByVeh) {
                    capa[g.arcFromId(arcId)] = 0;
                    restoreList.insert(arcId);
                }
            }
        }
    }

    // 源点s和宿点t的供应值
    auto sNd = g.nodeFromId(0), tNd = g.nodeFromId(1);
    auto &netSplex = *netSplexPtr;
    if (firstRun) {
        netSplex.costMap(cost).upperMap(capa).lowerMap(dema).stSupply(sNd, tNd, flowAmount);
    }
    else { netSplex.costMap(cost); firstRun = false; }

    // 执行
    NetSplex::ProblemType stat = netSplex.run();
    InvenCost invCost = INVEN_COST_MAX;
    if (stat == NetSplex::OPTIMAL) {
        using namespace std;
		//of << "Netflow takes " << (clock() - clk) / CLOCKS_PER_SEC << std::endl;
        if (check) {
            // 配送量
            Log[Logger::Type::Db] << "Deliveries:" << endl;
            for (ID perId = 0; perId < numPer; ++perId) {
                Log[Logger::Type::Db] << "----- Period " << perId << endl;
                for (ID vehId = 0; vehId < numVeh; ++vehId) {
                    arcId = numPer + numNd + numNd * numPer + perId * numVeh + vehId;
                    auto vehDeliv = netSplex.flow(g.arcFromId(arcId));
                    cout << "Veh " << vehId << ": ";
                    for (ID cliId = 0; cliId < numCli; ++cliId) {
                        arcId = delivOffset + perId * numVeh * numCli + vehId * numCli + cliId;
                        auto qt = netSplex.flow(g.arcFromId(arcId));
                        Log[Logger::Type::Db]
                            << "[" << cliId + 1 << ":" << sln.routings[perId].nodes[cliId + 1].routeId << "](" << qt << ") ";
                    }
                    Log[Logger::Type::Db] << " Deliv " << vehDeliv << endl;
                }
            }
            // 每周期每客户的消耗量检查
            Log[Logger::Type::Db] << "Consumptions:" << endl;
            for (ID perId = 0; perId < numPer; ++perId) {
                for (ID cliId = 0; cliId < numCli; ++cliId) {
                    arcId = consOffset + perId * numCli + cliId;
                    auto cs = netSplex.flow(g.arcFromId(arcId));
                    Log[Logger::Type::Db]
                        << "[" << cliId + 1 << "](" << cs << ":" << params.nodes[cliId + 1].prodCons << ")";
                    if (cs != params.nodes[cliId + 1].prodCons) {
                        Log[Logger::Type::Db] << "Cons not match" << endl;
                        system("pause");
                    }
                }
                Log[Logger::Type::Db] << endl;
            }
            // 初始库存
            {
                Log[Logger::Type::Db] << "Init inven:" << endl;
                arcId = numPer;
                auto ini = netSplex.flow(g.arcFromId(arcId));
                Log[Logger::Type::Db] << "[" << 0 << "](" << ini << ":" << params.nodes[0].initLevel << ") ";
                for (ID cliId = 0; cliId < numCli; ++cliId) {
                    arcId = numPer + 1 + cliId;
                    ini = netSplex.flow(g.arcFromId(arcId));
                    Log[Logger::Type::Db]
                        << "[" << cliId + 1 << "](" << ini << ":" << params.nodes[cliId + 1].initLevel << ")";
                }
                Log[Logger::Type::Db] << endl;
            }
            // 每周期生产量
            Log[Logger::Type::Db] << "Period products:" << endl;
            for (ID perId = 0; perId < numPer; ++perId) {
                arcId = perId;
                auto prod = netSplex.flow(g.arcFromId(arcId));
                Log[Logger::Type::Db] << "[" << perId << "](" << prod << ":" << -params.nodes[0].prodCons << ")";
            }
            Log[Logger::Type::Db] << endl;
            // 每周期配送量比较
            Log[Logger::Type::Db] << "Deliveries Compare:" << endl;
            for (ID perId = 0; perId < numPer; ++perId) {
                Log[Logger::Type::Db] << "----- Period " << perId << endl;
                auto qtDep = 0;
                for (ID cliId = 0; cliId < numCli; ++cliId) {
                    auto qtCli = 0;
                    int fromVeh = -1;
                    for (ID vehId = 0; vehId < numVeh; ++vehId) {
                        arcId = delivOffset + perId * numVeh * numCli + vehId * numCli + cliId;
                        auto q = netSplex.flow(g.arcFromId(arcId));
                        if (q != 0) {
                            qtCli += q;
                            if (fromVeh != -1) {
                                system("pause");
                            }
                            fromVeh = vehId;
                        }
                    }
                    qtDep -= qtCli;
                    Log[Logger::Type::Db]
                        << "[" << cliId + 1 << ":" << fromVeh << "](" << qtCli << ":" 
                        << (sln.deliveries[perId][cliId + 1] == -1 ? 0 : sln.deliveries[perId][cliId + 1]) << ") ";
                    if (assignDeliv) { sln.deliveries[perId][cliId + 1] = qtCli; }
                    if (qtCli == 0 && sln.visits[perId][cliId + 1]) {
                        Log[Logger::Type::Db]
                            << "No need to visit a client deliveried 0." << endl;
                    }
                }
                sln.deliveries[perId][0] = qtDep;
                Log[Logger::Type::Db] << endl;
            }
            // 每个节点的每周期剩余库存
            Log[Logger::Type::Db] << "Inven Level:" << endl;
            InvenCost tmpInvCost = 0;
            for (ID perId = 0; perId < numPer; ++perId) {
                Log[Logger::Type::Db] << "----- Period " << perId << endl;
                arcId = numPer + numNd + perId * numNd;
                auto invLv = netSplex.flow(g.arcFromId(arcId));
                auto invCostDep = invLv * params.nodes[0].holdingCost;
                tmpInvCost += invCostDep;
                Log[Logger::Type::Db] << "[" << 0 << "](" << invLv << ":" << invCostDep << ") ";
                for (ID cliId = 0; cliId < numCli; ++cliId) {
                    arcId = numPer + numNd + perId * numNd + 1 + cliId;
                    auto invLv = netSplex.flow(g.arcFromId(arcId));
                    auto invCostCli = invLv * params.nodes[cliId + 1].holdingCost;
                    tmpInvCost += invCostCli;
                    Log[Logger::Type::Db] << "[" << cliId + 1 << "](" << invLv << ":" << invCostCli << ") ";
                }
                Log[Logger::Type::Db] << endl;
            }
        }
        // 保存配送量
        if (assignDeliv) {
            for (ID perId = 0; perId < numPer; ++perId) {
                auto qtDep = 0;
                for (ID cliId = 0; cliId < numCli; ++cliId) {
                    auto qtCli = 0;
                    int fromVeh = -1;
                    for (ID vehId = 0; vehId < numVeh; ++vehId) {
                        arcId = delivOffset + perId * numVeh * numCli + vehId * numCli + cliId;
                        auto q = netSplex.flow(g.arcFromId(arcId));
                        if (q != 0) {
                            qtCli += q;
                            if (fromVeh != -1) {
                                Log[Logger::Type::Netflow] << "Visited more than once." << endl;
                                system("pause");
                            }
                            fromVeh = vehId;
                        }
                    }
                    qtDep -= qtCli;
                    sln.deliveries[perId][cliId + 1] = qtCli;
                }
                sln.deliveries[perId][0] = qtDep;
            }
        }
        invCost = netSplex.totalCost<double>() / precise;
        Log[Logger::Type::Db] << "Total netflow cost: " << invCost << endl;
    }
    else {
        using namespace std;
        Log[Logger::Type::Db] << "Netflow failed." << endl;
    }
    // 恢复到完整图
    for (ID reArcId : restoreList) {
        capa[g.arcFromId(reArcId)] = params.vehCapacity;
    }
    restoreList.clear();
    
    return invCost;
}
