/*
	��ɽ���ͼת���� CVRP �����������, 200����ľ��������ڽ�������Ҫ0.009s���
*/

#pragma once
#include "TypeDef.h"
#include "Solution.h"
#include "../Libs/CVRP/CvrpParameters.h"
#include "../Libs/CVRP/NagataLocalSearch.h"
#include "../Libs/CVRP/Repair.h"
#include "../Libs/CVRP/Population.h"
#include "../Libs/CVRP/Framework.h"

namespace hsh {
	namespace mvirp {

		class CvrpCaller {
		public:
            mvirp::Parameters &mvirpParams;
            cvrp::Parameters cvrpParams;

            Vec<int> nodeMap; // ���ڵ� Id ��С�����˳����, ������Ϊ�����еĽڵ� Id
            Vec<int> revNdMap; // �������

            enum class Mode { GenInit, AsInit, Pop };
            
            // TODO[hsh][5]: ��������ʱ������.

        public:
            /*���������ڵ� CVRP ������ */
			bool solve(ID period, Solution &sln, Mode mode, int specNumRou, double replacedThres);
            /* ���������ڵ���ͼת��Ϊ CVRP ���� */
            void subgraphToInstance(ID periodId, Solution &sln, int specNumRou);
            /* ������� */
            bool runGenInit(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            // �Ե�ǰ���·����ΪCVRP��ʼ��
            bool runAsInitSln(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            // ������Ⱥ����CVRP�Ż�
            bool runWithPop(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            /* ����������� Routes. */
            void indivToRoutes(cvrp::Individual &indiv, Routes &routes, Vec<Visit> &visits);
		    
            /* ���� CVRP ������, ��ʼ��������� */
            CvrpCaller(mvirp::Parameters &params);
        };
	}
}