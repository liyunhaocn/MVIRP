#pragma once
#include "NagataBase.h"

namespace hsh
{
	namespace cvrp
	{
		class NagataLocalSearch :public NagataBase
		{
		public:

			enum SearchMode { FEASIBLE , PENAL };

				/* �ֲ�����������Ϣ */
			SearchMode searchMode = FEASIBLE; // ������������ģʽ
			//int rangeProximity = 10; // ����ͻ��ڵ�����ʱ�����ǵĽӽ���Χ
			Move firstImproving; // ��¼�״θĽ���������
			Vec<int> clientOrder; // ��������Ľڵ����
			Vec<Move::Type> typeOrder; // ��������Ķ������ʹ���
			Vec<int> orderProximity;

			/* ����������, ���ض����Ƿ�Ľ����� */
			bool evaluateMove( Move &move , Individual &indiv ) override;

			/* ִ�оֲ�����, ָ������ģʽ */
			void run( Individual & indiv );

			/* ��������ģʽ */
			NagataLocalSearch & setSearchMode( SearchMode searchMode );

			NagataLocalSearch( Parameters &params );
		};
	}
}
