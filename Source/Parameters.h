#pragma once
#include "CmdLineParser.h"
#include "json.h"

using Json = nlohmann::json;

namespace hsh {
	namespace mvirp {
        struct Node {
			ID id;
			Coordinate x, y;
			ProdCons prodCons; // 每个节点的生产量/消费量, 消费量为负值, 生产量为正值
			InvenCost holdingCost;
			InvenLevel initLevel, minLevel, maxLevel;
		};

		class Parameters {
		public:
			int numNodes;
			int numClients;
			int numPeriods;
			Capacity vehCapacity;
			int numVehicles;
			Vec<Node> nodes;
			Vec<Vec<Distance>> distances;

			int totTimeLimit = INT_MAX;
			TimeSeed randomSeed = 0;
			double error = 0.00001; // 误差

			Str instancePath;
			Str outputDir;
			Str parametersPath;

			enum  class EliminationPolicy {
				NoSubTour = 0,
				AllSubTours,
				FirstSubTour,
				MinSubTour,
				MinMinSubTour,
				SVRandSubTour, // 随机添加给定长度范围内的单车辆的子回路
				MVRandSubTour // 随机添加给定长度范围内的多车辆的子回路
			};

            struct StageOne{
                Second timeOnce = 300;
                int maxThreads = 1;
				double capaUsage = 0.9;
				int szRandSubTour = 10;
				EliminationPolicy policy = EliminationPolicy::MVRandSubTour;
				
				void from_json(const Json& j, StageOne& cl) {
					j.at("timeOnce").get_to(cl.timeOnce);
					j.at("maxThreads").get_to(cl.maxThreads);
					j.at("capaUsage").get_to(cl.capaUsage);
					j.at("szRandSubTour").get_to(cl.szRandSubTour);
					j.at("policy").get_to(cl.policy);
				}

            }finding;

            struct StageTwo{
                Second timeOnce = 60;
				int maxThreads = 1;
				double capaUsage = 0.9;
				int szRandSubTour = 10;
				EliminationPolicy policy = EliminationPolicy::MVRandSubTour;
				
				void from_json(const Json& j, StageTwo& cl) {
					j.at("timeOnce").get_to(cl.timeOnce);
					j.at("maxThreads").get_to(cl.maxThreads);
					j.at("capaUsage").get_to(cl.capaUsage);
					j.at("szRandSubTour").get_to(cl.szRandSubTour);
					j.at("policy").get_to(cl.policy);
				}

            }adjusting;

            struct StageThree {
				int szLayer1 = 0;
				int szLayer2 = 0;
				bool delDelivZero = true;
				//int maxTimesPtb = 100, timesPtb = 0;

				void from_json(const Json& j, StageThree& cl) {
					j.at("szLayer1").get_to(cl.szLayer1);
					j.at("szLayer2").get_to(cl.szLayer2);
					j.at("delDelivZero").get_to(cl.delDelivZero);
					//j.at("maxTimesPtb").get_to(cl.maxTimesPtb);
					//j.at("timesPtb").get_to(cl.timesPtb);
				}
            }refining;

			struct {
				ProdCons maxCons = 0;
			}statistics;

		public:
			bool loadInstance(Str instPath);
			bool setTimeSeed(TimeSeed timeSeed = 0);
			void loadCommandParameters(int argc, char* argv[]);
			Parameters();

			void to_json(Json& j, const Parameters& cl) {
				j = Json{
					{"randomSeed", cl.randomSeed},
					{"totalTimeLimit", cl.totTimeLimit}
				};

			}

			void from_json(const Json& j, Parameters& cl) {

				j.at("randomSeed").get_to(cl.randomSeed);
				j.at("totalTimeLimit").get_to(cl.totTimeLimit);

				Json stage1 = j["finding"];
				finding.from_json(stage1, finding);
				Json stage2 = j["adjusting"];
				adjusting.from_json(stage2, adjusting);
				Json stage3 = j["refining"];
				refining.from_json(stage3, refining);
			}

		};
	}
}