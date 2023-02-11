#pragma once
#include "TypeDef.h"
#include "Parameters.h"

namespace hsh {
    namespace mvirp {

        struct RouteNode {
			ID id = -1;
			ID routeId = -1;
            ID links[2] = { -1,-1 };

            void reset(){ id = -1; routeId = -1; links[0] = links[1] = -1; }
			void copyLinks(RouteNode &src) { links[0] = src.links[0]; links[1] = src.links[1]; }
            void assignId(ID id) { this->id = id; }
            void assignRoute(ID id) { routeId = id; }
            void assignLinks(ID prevId, ID succId) { links[0] = prevId; links[1] = succId; }
            void assign(RouteNode &src) {
                id = src.id; routeId = src.routeId;
                links[0] = src.links[0]; links[1] = src.links[1];
            }
        };

        struct Routes {
            Vec<RouteNode> nodes;
            Vec<RouteNode> depots;
            Vec<Distance> dist;
            int numRoutes;
            Distance totDist;

            Routes(int numNodes) {
                nodes = Vec<RouteNode>(numNodes);
                depots = Vec<RouteNode>(numNodes);
                dist = Vec<Distance>(numNodes);
                numRoutes = 0;
                totDist = DISTANCE_MAX;
            }

            void reset() {
                for (auto &node : nodes) { node.reset(); }
                for (auto &depot : depots) { depot.reset(); }
                for (auto &d : dist) { d = 0; }
                numRoutes = 0;
                totDist = DISTANCE_MAX;
            }

            void unlinkPrevSucc(ID prevNdId, ID succNdId, ID curNdId, ID rouId) {
                auto &prevNd = prevNdId == 0 ? depots[rouId] : nodes[prevNdId],
                    &succNd = succNdId == 0 ? depots[rouId] : nodes[succNdId],
                    &curNd = nodes[curNdId];
                prevNd.links[1] = succNdId;
                succNd.links[0] = prevNdId;
                curNd.links[0] = curNd.links[1] = -1;
                curNd.routeId = -1;
                curNd.id = -1;
            };

            void linkPrevSucc(ID prevNdId, ID succNdId, ID curNdId, ID rouId) {
                auto &prevNd = prevNdId == 0 ? depots[rouId] : nodes[prevNdId],
                    &succNd = succNdId == 0 ? depots[rouId] : nodes[succNdId],
                    &curNd = nodes[curNdId];
                prevNd.links[1] = curNdId;
                succNd.links[0] = curNdId;
                curNd.links[0] = prevNdId; curNd.links[1] = succNdId;
                curNd.routeId = rouId;
                curNd.id = curNdId;
            };

            void addOneCliRouAtLast(ID ndId) {
                // 在末尾添加一条单客户路径
                auto &dep = depots[numRoutes];
                auto &cli = nodes[ndId];
				dep.id = 0; dep.routeId = numRoutes;
                dep.links[0] = dep.links[1] = ndId;
                cli.id = ndId; cli.routeId = numRoutes;
                cli.links[0] = cli.links[1] = 0;
                ++numRoutes;
            }

            void delOneCliRouAtLast() {
                // 删除在末尾的一条单客户路径
                auto &dep = depots[numRoutes - 1];
                auto &cli = nodes[dep.links[1]];
                dep.id = -1; dep.routeId = -1;
                dep.links[0] = dep.links[1] = -1;
                cli.id = -1; cli.routeId = -1;
                cli.links[0] = cli.links[1] = -1;
                --numRoutes;
            }

            void cleanDepSelfLoop() {
                for (ID rouId = 0; rouId < numRoutes; ++rouId) {
                    auto &dep = depots[rouId];
                    if (dep.links[0] == dep.links[1] && dep.links[1] == 0) {
                        for (ID nxtRouId = rouId + 1; nxtRouId < numRoutes; ++nxtRouId) {
                            depots[nxtRouId].routeId = nxtRouId - 1;
							std::swap(depots[nxtRouId - 1], depots[nxtRouId]);
                        }
                        auto& lastDep = depots[numRoutes - 1];
                        lastDep.id = 0; lastDep.routeId = -1;
                        lastDep.links[0] = lastDep.links[1] = -1;
                        dist[--numRoutes] = 0;
                        --rouId; // 从当前位置开始继续处理
                    }
                }
                for (ID rouId = 0; rouId < numRoutes; ++rouId) {
                    auto& dep = depots[rouId];
                    for (int start = dep.id, cur = start, succ = dep.links[1];
                        ;
                        cur = succ, succ = nodes[cur].links[1]) {
                        auto& nd = (cur == 0 ? dep : nodes[cur]);
                        nd.routeId = dep.routeId;
                        if (succ == start) { break; }
                    }
                }
            }
        };

        class Solution {
        public:
            Parameters *paramsPtr = nullptr;
            Vec<Vec<Visit>> visits; // 每个周期每个客户的访问状态
            Vec<Vec<Quantity>> deliveries; // 每个周期每个客户的配送量
            Vec<Routes> routings; // 每个周期的车队路线, 包含每条路线的信息
            bool allocated;

            InvenCost invenCost = 0;
            TravelCost travelCost = 0;
            Cost cost = COST_MAX;
            double time = INFINITY;

        public:
            Solution():allocated(true) {}
            Solution(Parameters &params):paramsPtr(&params) { init(params); }

            void init(Parameters &params);
            void resetCost() {
                invenCost = 0;
                travelCost = 0;
                cost = COST_MAX;
            }
			void calcCost(bool invCheck = false, bool traCheck = false) {
                invenCost = calcInvCost(invCheck);
				travelCost = calcTraCost(true, traCheck);
                cost = invenCost + travelCost;
            }
            InvenCost calcInvCost(ID ndId, bool check = false);
            InvenCost calcInvCost(bool check = false);
			TravelCost calcTraCost(ID perId, ID rouId, bool setDist = false, bool check = false);
            TravelCost calcTraCost(ID perId, bool setDist = false, bool check = false);
            TravelCost calcTraCost(bool setDist = false, bool check = false);

			void outputDimacsFmt(Str path, bool check = false);
        };
    }
}