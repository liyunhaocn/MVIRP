/*
	Ŀǰ��� `NagataLocalSearch` �ڲ���������޸�
*/

#pragma once
#include "NagataBase.h"

namespace hsh
{
	namespace cvrp
	{
		class Repair :private NagataBase
		{
		public:

			enum ProximityMode { FULL , GRANULAR };

				/* �޸�������Ϣ */
			ProximityMode proximityMode;
			int rangeProximity; // ����ǽӽ��ֿ�Ŀͻ��ڵ�����ʱ�����ǵĽӽ���Χ
			int rangeProximityDepot; // ����ӽ��ֿ�Ŀͻ��ڵ�����ʱ�����ǵĽӽ���Χ
			Vec<int> infeasibleRoutes; // ������·������, ��·�� ID ��ʾ��Ӧ·��
			Vec<Move> bestImprovingMoves; // �ܹ����ٳͷ�Ŀ�꺯���ĳͷ�����������������
			double bestImprobingValue; // �ܹ����ٳͷ�Ŀ�꺯���ĳͷ���������������Ŀ��ֵ
			Vec<Move::Type> typeOrder; // ��������Ķ������ʹ���

			/* �ϲ��ӻ�· */
			void mergeSubtours( Individual &indiv );

			/* ����Υ�� */
			bool eliminateExcess( Individual &indiv );

			/* ����������, �����Ƿ�Ľ�����ֵ��Ϊ false, �������������� */
			virtual bool evaluateMove( Move &move , Individual &indiv ); // �麯��

			/* ��ϵ�ǰ�ӽ���Ϣ��չ�ӽ���Χ, �����ÿ������Ӻ����½� */
			int extendRangeWithCurrentProximity( int V , double looseFactor , int minRangeProximity , int maxRangeProximity, Individual &indiv );

			/* ���ýӽ���Χģʽ */
			Repair & setProximityMode( ProximityMode proximityMode );

			/* ���캯�� */
			Repair( Parameters &params ) :NagataBase( params )
			{ }
		};
	}
}