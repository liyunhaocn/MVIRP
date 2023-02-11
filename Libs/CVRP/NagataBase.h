/*
	�������, ������ Nagata �������ᵽ������, �Լ���Ӧ�����ݽṹ�ͺ���, �����������������������ʵ�־���
*/

#pragma once
#include "LocalSearch.h"
#include "Individual.h"

namespace hsh
{
	namespace cvrp
	{
		class NagataBase :private LocalSearch
		{
		public:

			struct Move
			{
				enum Type
				{
					EMPTY = 0 ,
					OPT2KIND1 , OPT2KIND2 , OPT2KIND3 , OPT2KIND4 ,
					INSKIND1 , INSKIND2 , INSKIND3 , INSKIND4 ,
					SWAPKIND1 , SWAPKIND2 ,
					OPT2STARTYPE1 , OPT2STARTYPE2
				};
				int type;
				int V , W; // �Զ�Ӧ ID ��ʾ�ڵ�
				int routeV , routeW; // �Զ�Ӧ ID ��ʾ·��

				Move() :type( Move::Type::EMPTY ) { }

				Move( Type type , int V , int W , int routeV , int routeW )
					:type( type ) , V( V ) , W( W ) , routeV( routeV ) , routeW( routeW ) { }
			};

			struct AdditionalInfo
			{
				int ID; // �ڵ� ID
				int routeID; // ����·�� ID
				int pos; // ��·���ڲ���λ��
				double distanceFromDepot , distanceToDepot; // ���������ǰ�ڵ�ľ���
				double loadFromDepot , loadToDepot; // ���ذ�����ǰ�ڵ������
				double durationFromDepot , durationToDepot; // ����ʱ��������ǰ�ڵ�ķ���ʱ��
				int prevProximity;
				int reversedPrevProximity;
				int succProximity;
				int reversedSuccProximity;
			};

			Parameters &params;

				/* ��¼����ڵ�ĸ�����Ϣ */
			Vec<AdditionalInfo> nodeAdditionalInfo;
			Vec<AdditionalInfo> depotAdditionalInfo;

				/* ��¼����·���ĸ�����Ϣ */
			Vec<double> distances;
			Vec<double> loads;
			Vec<double> capacityExcesses;
			Vec<double> durations;
			Vec<double> durationExcesses;
			Vec<int> sizes;


			 /* ����ڵ����� */
			void constructNeighborhoodOf( int V , int rangeProximity , Vec<Move::Type> &typeOrder , Individual &indiv );
			void constructNeighborhoodOf( int V , int rangeProximityStart , int rangeProximityEnd , Vec<Move::Type> &typeOrder , Individual &indiv );
			void constructNeighborhoodOf( int V , Vec<int> &orderProximity , Vec<Move::Type> &typeOrder , Individual &indiv );

			/* ������������ӺϷ��� */
			bool inspectMoveLinks( Move &move , Individual &indiv );

			/* �������·�������� */
			int deltaNumRoutes( Move &move , Individual &indiv );

			/* �������������� */
			double deltaDistance( Move &move , Individual &indiv );

			/* �����������Υ�������� */
			double deltaCapacityExcess( Move &move , Individual &indiv );

			/* ����������ʱ��Υ��������, ���ṩ�������ĸ���������� */
			double deltaDurationExcess( Move &move , double moveDeltaDistance , Individual &indiv );

			/* ����������, �����Ƿ�Ľ� */
			virtual bool evaluateMove( Move &move , Individual &indiv ) = 0; // ������

			/* ִ�������� */
			void performMove( Move &move , Individual &indiv );

			/* ��������������������Ϣ */
			void updateLinks( Move &move , Individual &indiv );

			/* ׼��������Ϣ */
			void prepareAdditionalInfo( Individual &indiv );

			/* ִ��������֮����¸�����Ϣ */
			void updateAdditionalInfo( Individual &indiv , int routeID , bool deleted );

			/* ���캯�� */
			NagataBase( Parameters &params );
		};
	}
}