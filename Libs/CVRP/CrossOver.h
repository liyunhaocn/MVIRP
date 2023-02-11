/*
	目前支持的节点数设置为 1000, 该数值可以针对大算例放大, 主要作用在边的 hash;
	GAB 中客户节点最多 4 个邻接节点, 存储和遍历边所需信息的复杂度为 O( 4 · numClients + numRoutes · 2 )
*/

#pragma once
#include "CvrpParameters.h"
#include "Individual.h"

namespace hsh
{
	namespace cvrp
	{

		class CrossOver
		{

		};

		class Eax :public CrossOver
		{
		public:

			Parameters &params;

				/* 可支持节点数 */
			const int supportableNumNodes = 1000;

				/* 存储及遍历过程所需数据结构 */
			enum Owner { UNKNOWN , BASE , GOAL , COMMON };
			bool isFirstTimeEax = true;
			struct RichEdge;
			int maxNumRichEdges; // 最多所需存储的边数
			int allocatedEnd = -1; // `richEdges` 下一次分配的 ID
			Vec<RichEdge> richEdges; // 存储双亲所有边信息
			Vec<Vec<int>> adjNodeTable; // 当前 GAB 中每个节点的邻接节点列表
			Vec<int> adjNodeSizes; // `adjNodeTable` 中每个节点邻接列表的大小
			Vec<Vec<int>> arrivalSteps; // 遍历到达每个节点的步数栈, 防止过早形成奇数 Cycle
			Vec<int> traversalPath; // 差异边的遍历路径, 记录 `RichEdge` ID
			UnorderedMap<int , int> edgesMap , privateEdgesMap , commonEdgesMap , baseEdgesMap , goalPrivateEdgesMap; // 映射 `code` 到 `richEdges` 的 ID
			Vec<Cycle> single; // 一次分解 GAB 得到的 AB-Cycle 集合
			Vec<Cycle> cycles; // 多次分解 GAB 得到的数量最多的 AB-Cycle 集合
			Vec<bool> visited; // 标记一个节点是否已访问

				/* GAB 分解为 AB-Cycle 所采用的数据结构 */
			struct RichEdge
			{
				int ID; // 在存储空间的 ID
				Pair<int> nodeIDs; // 对应的边
				int code; // 边的哈希码
				Owner owner; // 边的所有权属性
				bool unique; // 如果边 <i, j> 和边 <j, i> 不同时存在, 则唯一; 只出现在服务单个客户的路径中
				int baseRouteID , goalRouteID; // 在双亲中的路径 ID
				bool turn; // 替换默认按照唯一 (即 i< j) 进行; 如果不唯一, 则首次访问 <i, j>, 下一次访问 <j, i>
				bool usable; // 是否可以遍历
				bool visited; // 是否已访问
				int cycleID; // 该边被划分到的 AB-Cycle 的 ID

				RichEdge() :
					ID( -1 ) , nodeIDs( { -1,-1 } ) , code( -1 ) , owner( Owner::UNKNOWN ) ,
					unique( true ) , baseRouteID( -1 ) , goalRouteID( -1 ) ,
					turn( false ) , usable( false ) , visited( false ) , cycleID( -1 )
				{ }

				/* 设置数据 */
				void set( int ID , int i , int j , int code , Owner owner , bool unique , int routeID );
			};

			/* 重置映射 */
			void resetMap( UnorderedMap<int , int>& map , int size );

			/* 重置集合 */
			void resetSet( UnorderedSet<int>& set , int size );

			/* 重置所有节点遍历到达的步数栈 */
			void resetArrivalSteps();

			/* 从边到哈希码 */
			int toCode( int i , int j , bool unique );

			/* 从哈希码到边 */
			Pair<int> toEdge( int code );

			/* 设置边的访问状态 */
			void setVisited( UnorderedMap<int , int>& mp , bool stat );

			/* 设置边的可用状态 */
			void setUsable( UnorderedMap<int , int>& mp , bool stat );

			/* 加入新的边 */
			void addRichEdge( int i , int j , Owner owner , bool unique , int routeID );

			/* 准备交叉算符所需的数据结构, 并使上一次交叉的结果无效 */
			void prepareDataStructures();

			/* 双亲边集分类 */
			void classifyEdges( Individual &base , Individual &goal );

			/* 分解 GAB, 获得 AB-Cycle */
			void generateCycles( int repeatTimes = 5 );

			/* 仅复制个体的客户节点连接信息 */

			/* 对个体应用给定 AB-Cycle; 目标路径数为 `params.preprocess.numRoutes` */
			void applyCycle( Cycle &cycle , Individual &indiv );

			/* 对个体应用给定 AB-Cycle 集合; 目标路径数为 `params.preprocess.numRoutes` */
			void applyCycles( Vec<Cycle> &cycles , Individual &indiv );

			Eax( Parameters &params ) :params( params )
			{ }
		};
	}
}