/*
	抽象基类, 定义了 Nagata 论文中提到的邻域, 以及相应的数据结构和函数, 评估邻域动作由派生类的搜索实现决定
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
				int V , W; // 以对应 ID 表示节点
				int routeV , routeW; // 以对应 ID 表示路径

				Move() :type( Move::Type::EMPTY ) { }

				Move( Type type , int V , int W , int routeV , int routeW )
					:type( type ) , V( V ) , W( W ) , routeV( routeV ) , routeW( routeW ) { }
			};

			struct AdditionalInfo
			{
				int ID; // 节点 ID
				int routeID; // 所在路径 ID
				int pos; // 在路径内部的位置
				double distanceFromDepot , distanceToDepot; // 距离包括当前节点的距离
				double loadFromDepot , loadToDepot; // 载重包括当前节点的需求
				double durationFromDepot , durationToDepot; // 服务时长包括当前节点的服务时间
				int prevProximity;
				int reversedPrevProximity;
				int succProximity;
				int reversedSuccProximity;
			};

			Parameters &params;

				/* 记录个体节点的附加信息 */
			Vec<AdditionalInfo> nodeAdditionalInfo;
			Vec<AdditionalInfo> depotAdditionalInfo;

				/* 记录个体路径的附加信息 */
			Vec<double> distances;
			Vec<double> loads;
			Vec<double> capacityExcesses;
			Vec<double> durations;
			Vec<double> durationExcesses;
			Vec<int> sizes;


			 /* 构造节点邻域 */
			void constructNeighborhoodOf( int V , int rangeProximity , Vec<Move::Type> &typeOrder , Individual &indiv );
			void constructNeighborhoodOf( int V , int rangeProximityStart , int rangeProximityEnd , Vec<Move::Type> &typeOrder , Individual &indiv );
			void constructNeighborhoodOf( int V , Vec<int> &orderProximity , Vec<Move::Type> &typeOrder , Individual &indiv );

			/* 检查邻域动作连接合法性 */
			bool inspectMoveLinks( Move &move , Individual &indiv );

			/* 计算个体路径数增量 */
			int deltaNumRoutes( Move &move , Individual &indiv );

			/* 计算个体距离增量 */
			double deltaDistance( Move &move , Individual &indiv );

			/* 计算个体载重违反量增量 */
			double deltaCapacityExcess( Move &move , Individual &indiv );

			/* 计算个体服务时长违反量增量, 需提供邻域动作的个体距离增量 */
			double deltaDurationExcess( Move &move , double moveDeltaDistance , Individual &indiv );

			/* 评估邻域动作, 返回是否改进 */
			virtual bool evaluateMove( Move &move , Individual &indiv ) = 0; // 抽象函数

			/* 执行邻域动作 */
			void performMove( Move &move , Individual &indiv );

			/* 根据邻域动作更新连接信息 */
			void updateLinks( Move &move , Individual &indiv );

			/* 准备附加信息 */
			void prepareAdditionalInfo( Individual &indiv );

			/* 执行邻域动作之后更新附加信息 */
			void updateAdditionalInfo( Individual &indiv , int routeID , bool deleted );

			/* 构造函数 */
			NagataBase( Parameters &params );
		};
	}
}