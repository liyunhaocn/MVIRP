#pragma once
#include "CvrpTypeDef.h"
#include "CvrpParameters.h"

namespace hsh
{
	namespace cvrp
	{

		struct Node
		{
			int ID; // 节点 ID, 关联到节点信息 `nodeInfo[ID]`
			int routeID;
			int links[2]; // `links[0]`记录前驱, `links[1]`记录后继

			void reset() { ID = -1; routeID = -1; links[0] = links[1] = -1; }
		};

		class Individual
		{
		public:

			Parameters &params;

				/* 连接信息 */
			Vec<Node> nodes; //TODO[修改为clients?]
			Vec<Node> depots; // 仓库 ID 代表路径或子回路 ID; 存储上路径仓库在前, 子回路仓库在后
			int numRoutes = 0;
			int numSubtours = 0;
			int numDepots = 0;

				/* 基本信息 */
			double distance = Inf;
			double capacityExcess = 0.;
			double durationExcess = 0.;
			bool isFeasible = false;

				/* Path relinking 信息 */
			int cycleID = -1; // 本次选择的 AB-Cycle ID
			int numAccumulation = 0; // 从 base 到当前个体累积的差异边数量
			int numPrivateEdges = 0; // base 与 goal 之间的私有边数量
			int baseIndivID = -1;
			int goalIndivID = -1;
			int tGenID = -1;

			/* 清空个体信息 */
			void reset();

			/* 增加一个路径仓库(一条路径) */
			Node & addRouteDepot();

			/* 增加一个子回路仓库(一条子回路), 在预期数量的路径仓库存储位置之后 */
			Node & addSubtDepot( int expectedNumRoutes );

			/* 完成个体的修改, 检查路径数 */
			void done( int expectedNumRoutes );

			/* 删除一个仓库(一条路径)或一个子回路仓库(一条子回路) */
			void delDepot( int deletedDepotID );

			/* 复制个体的所有信息 */
			void copyAllFrom( Individual &src );

			/* 复制个体的客户节点连接信息 */
			void copyLinksFrom( const Individual &src );

			/* 计算所有基本信息 */
			void calculateCompletely();

			/* 路径或子回路距离 */
			double getDistanceOf( int depotID );

			/* 个体总距离, 包括可能的子回路 */
			double getDistance();
			
			/* 路径或子回路载重 */
			double getLoadOf(int depotID );

			/* 路径或子回路载重违反量 */
			double getCapacityExcessOf( int depotID );

			/* 个体载重违反量, 包括可能的子回路 */
			double getCapacityExcess();

			/* 路径或子回路时长, 涉及节点的服务时间与距离之和 */
			double getDurationOf(int depotID );

			/* 路径或子回路时长违反量 */
			double getDurationExcessOf( int depotID );

			/* 个体服务时长违反量, 包括可能的子回路 */
			double getDurationExcess();

			/* 提供已知数据计算违反量, 误差由 `params` 决定 */
			double getExcessOf( double limit , double value);

			/* 个体可行状态 */
			bool getFeasibleStatus();

			/* 检查 `nodes` 及 `depots` 的连接是否合法 */
			bool inspectLinks(bool showErrorMsg = false);

			/* 打印连接信息 */
			void print();

			/* 输出到 CVRPLib 格式 */
			void exportCVRPLibFormat( const String& pathExport );

			/* 构造个体 */
			Individual( Parameters &params ) :params( params )
			{
				nodes = Vec<Node>( params.numNodes );
				depots = Vec<Node>( params.numNodes );
			}
		};
	}
}