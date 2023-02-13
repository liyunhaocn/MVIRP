
#include "Parameters.h"
#include <cmath>

bool hsh::mvirp::Parameters::loadInstance(Str instPath) {
	using namespace std;

	ifstream inst(instPath);
	if (inst.is_open()) {
		Str content;

		// �ڵ���, ������, ��������, ���Ӵ�С
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
		// ����ռ�
		nodes = Vec<Node>(numNodes);
		distances = Vec<Vec<Distance>>(numNodes, Vec<Distance>(numNodes, 0));

		// �ֿ� ID, ������Ϣ, ��ʼ���, ��������/������, ���ɱ�
		inst >> content;
		nodes[0].id = stoi(content);
		inst >> content;
		nodes[0].x = stod(content);
		inst >> content;
		nodes[0].y = stod(content);
		inst >> content;
		nodes[0].initLevel = stoi(content);
		inst >> content;
		nodes[0].prodCons = -stoi(content); // �ֿ����ȡ����.
		nodes[0].minLevel = 0;
		nodes[0].maxLevel = INVEN_LEVEL_MAX;
		inst >> content;
		nodes[0].holdingCost = stod(content);

		// �ͻ� ID, ������Ϣ, ��ʼ���, �����ˮƽ, ��С���ˮƽ, ��������/������, ���ɱ�
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

		// ÿ�����ڵ�֮�����
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
		cout << "Cannot open this file." << endl;
	}
	return true;
}

bool hsh::mvirp::Parameters::setTimeSeed(TimeSeed timeSeed) {
    if (timeSeed == 0) {
        this->timeSeed = time(nullptr);
        srand(this->timeSeed);
    }
    else {
        this->timeSeed = timeSeed;
    }
    std::cout << "time seed " << this->timeSeed << std::endl;
    return true;
}

hsh::mvirp::Parameters::Parameters() {}