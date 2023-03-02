
#include "Parameters.h"
#include <cmath>
#include <sstream>

bool hsh::mvirp::Parameters::loadInstance(Str instPath) {
	using namespace std;

	ifstream inst(instPath);
	if (inst.is_open()) {
		Str content;

		// 节点数, 周期数, 车辆容量, 车队大小
		inst >> content;
		numNodes = stoi(content);
		numClients = numNodes - 1;
		inst >> content;
		numPeriods = stoi(content);
		inst >> content;
		vehCapacity = stod(content);
		inst >> content;
		numVehicles = stoi(content);
        //vehCapacity *= numVehicles;
		// 分配空间
		nodes = Vec<Node>(numNodes);
		distances = Vec<Vec<Distance>>(numNodes, Vec<Distance>(numNodes, 0));

		// 仓库 ID, 坐标信息, 初始库存, 日生产量/消费量, 库存成本
		inst >> content;
		nodes[0].id = stoi(content);
		inst >> content;
		nodes[0].x = stod(content);
		inst >> content;
		nodes[0].y = stod(content);
		inst >> content;
		nodes[0].initLevel = stoi(content);
		inst >> content;
		nodes[0].prodCons = -stoi(content); // 仓库产量取负数.
		nodes[0].minLevel = 0;
		nodes[0].maxLevel = INVEN_LEVEL_MAX;
		inst >> content;
		nodes[0].holdingCost = stod(content);

		// 客户 ID, 坐标信息, 初始库存, 最大库存水平, 最小库存水平, 日生产量/消费量, 库存成本
		for (inst >> content; !inst.eof(); inst >> content) {
			int nID = stoi(content);
			nodes[nID].id = nID;
			inst >> content;
			nodes[nID].x = stod(content);
			inst >> content;
			nodes[nID].y = stod(content);
			inst >> content;
			nodes[nID].initLevel = stoi(content);
			inst >> content;
            nodes[nID].maxLevel = stoi(content);
            inst >> content;
			nodes[nID].minLevel = stoi(content);
			inst >> content;
            nodes[nID].prodCons = stoi(content);
            inst >> content;
			nodes[nID].holdingCost = stod(content);
			if (nodes[nID].prodCons > statistics.maxCons) {
				statistics.maxCons = nodes[nID].prodCons;
			}
		}

		// 每两个节点之间距离
		for (ID nID1 = 0; nID1 < numNodes; ++nID1) {
			for (ID nID2 = 0; nID2 < numNodes; ++nID2) {
				if (nID1 == nID2) { distances[nID1][nID2] = 0; }
				else {
					distances[nID1][nID2] = int(
						sqrt(
						(nodes[nID1].x - nodes[nID2].x)*(nodes[nID1].x - nodes[nID2].x) +
						(nodes[nID1].y - nodes[nID2].y)*(nodes[nID1].y - nodes[nID2].y)
						) + 0.5);
				}
			}
		}
        cout << "load instance \"" << instPath << "\" done." << endl;
        cout << "vechile " << numVehicles << ", node " << numNodes << ", capacity " << vehCapacity << endl;
	}
	else {
		cout << "Cannot open this file. file path:" << instPath << endl;
	}
	return true;
}

bool hsh::mvirp::Parameters::setTimeSeed(TimeSeed timeSeed) {
    if (timeSeed == 0) {
        this->randomSeed = time(nullptr);
        srand(this->randomSeed);
    }
    else {
        this->randomSeed = timeSeed;
    }
    std::cout << "time seed " << this->randomSeed << std::endl;
    return true;
}

void hsh::mvirp::Parameters::loadCommandParameters(int argc, char* argv[]) {
	
	std::stringstream ss;

	ss << "Usage: ./Solver instance output parameters						" << std::endl;
	ss << "Positionals:														" <<std::endl;
	ss << "  instance    string  required    the instance file path			" <<std::endl;
	ss << "  output      string  required    the solution saved directory	" <<std::endl;
	ss << "  parameters  string  required    the parameters file path		" <<std::endl;
	ss << "																	" << std::endl;
	ss << "Options:															" <<std::endl;
	ss << "  -h,--help						 print help message and exit	" << std::endl;
	

	if (argc != 4) {
		std::cout << ss.str() << std::endl;
		exit(0);
	}

	this->instancePath = argv[1];
	this->outputDir = argv[2];
	this->parametersPath = argv[3];

}

hsh::mvirp::Parameters::Parameters() {}