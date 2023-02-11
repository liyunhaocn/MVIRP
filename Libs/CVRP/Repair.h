/*
	目前结合 `NagataLocalSearch` 内部函数完成修复
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

				/* 修复过程信息 */
			ProximityMode proximityMode;
			int rangeProximity; // 构造非接近仓库的客户节点邻域时所考虑的接近范围
			int rangeProximityDepot; // 构造接近仓库的客户节点邻域时所考虑的接近范围
			Vec<int> infeasibleRoutes; // 不可行路径集合, 以路径 ID 表示对应路径
			Vec<Move> bestImprovingMoves; // 能够减少惩罚目标函数的惩罚项的最好邻域动作集合
			double bestImprobingValue; // 能够减少惩罚目标函数的惩罚项的最好邻域动作的目标值
			Vec<Move::Type> typeOrder; // 构造邻域的动作类型次序

			/* 合并子回路 */
			void mergeSubtours( Individual &indiv );

			/* 消除违反 */
			bool eliminateExcess( Individual &indiv );

			/* 评估邻域动作, 无论是否改进返回值均为 false, 考虑所有邻域动作 */
			virtual bool evaluateMove( Move &move , Individual &indiv ); // 虚函数

			/* 结合当前接近信息拓展接近范围, 并设置宽松因子和上下界 */
			int extendRangeWithCurrentProximity( int V , double looseFactor , int minRangeProximity , int maxRangeProximity, Individual &indiv );

			/* 设置接近范围模式 */
			Repair & setProximityMode( ProximityMode proximityMode );

			/* 构造函数 */
			Repair( Parameters &params ) :NagataBase( params )
			{ }
		};
	}
}