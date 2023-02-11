#include "CrossOver.h"

inline void hsh::cvrp::Eax::resetMap( UnorderedMap<int , int>& map , int size )
{
	map.clear(); map.reserve( size );
}

inline void hsh::cvrp::Eax::resetSet( UnorderedSet<int>& set , int size )
{
	set.clear(); set.reserve( size );
}

inline void hsh::cvrp::Eax::resetArrivalSteps()
{
	for ( auto &row : arrivalSteps )
	{
		row.clear();
	}
}

inline int hsh::cvrp::Eax::toCode( int i , int j , bool unique )
{
	if ( unique && i > j )
	{
		std::swap( i , j );
	}
	return ( i * supportableNumNodes ) + j;
}

inline hsh::cvrp::Pair<int> hsh::cvrp::Eax::toEdge( int code )
{
	return { ( code / supportableNumNodes ),( code % supportableNumNodes ) };
}

inline void hsh::cvrp::Eax::setVisited( UnorderedMap<int , int>& mp , bool stat )
{
	for ( auto& item : mp )
	{
		richEdges[item.second].visited = stat;
	}
}

inline void hsh::cvrp::Eax::setUsable( UnorderedMap<int , int>& mp , bool stat )
{
	for ( auto& item : mp )
	{
		richEdges[item.second].usable = stat;
	}
}

inline void hsh::cvrp::Eax::addRichEdge( int i , int j , Owner owner , bool unique , int routeID )
{
	if ( unique && i > j ) { std::swap( i , j ); } // 优先视为唯一
	int code = toCode( i , j , unique );

	if ( edgesMap.insert( { code, allocatedEnd } ).second == false )
	{
		/* 共有 */
		auto& rEdge = richEdges[edgesMap[code]];
		rEdge.owner = Owner::COMMON;
		if ( !unique ) { rEdge.unique = false; }
		rEdge.goalRouteID = routeID;
	}
	else
	{
		/* 私有 */
		auto& rEdge = richEdges[allocatedEnd];
		rEdge.set( allocatedEnd , i , j , code , owner , unique , routeID );
		allocatedEnd++;
	}
}

void hsh::cvrp::Eax::prepareDataStructures()
{
	/* 第一次调用时分配数据结构所需的存储空间 */
	if ( params.numNodes > supportableNumNodes ) { throw String( "Excess of supportable nodes." ); }
	if ( isFirstTimeEax )
	{
		isFirstTimeEax = false;
		maxNumRichEdges = 4 * params.numNodes;
		richEdges = Vec<RichEdge>( maxNumRichEdges );
		adjNodeTable = Vec<Vec<int>>( params.numNodes , Vec<int>( params.numNodes * 2 ) );
		adjNodeSizes = Vec<int>( params.numNodes );
		traversalPath.reserve( maxNumRichEdges );
		arrivalSteps = Vec<Vec<int>>( params.numNodes , Vec<int>( params.numNodes ) );
		visited = Vec<bool>( params.numNodes );
	}

	/* 重置所有数据结构 */
	allocatedEnd = 0;
	for ( auto& size : adjNodeSizes ) { size = 0; }
	resetMap( edgesMap , maxNumRichEdges );
	resetMap( privateEdgesMap , maxNumRichEdges );
	resetMap( commonEdgesMap , maxNumRichEdges );
	resetMap( baseEdgesMap , maxNumRichEdges );
	resetMap( goalPrivateEdgesMap , maxNumRichEdges );
}

void hsh::cvrp::Eax::classifyEdges( Individual & base , Individual & goal )
{
	//if (params.dbg.lk)std::cout << "classify edges 0.1" << std::endl;
	if ( base.numRoutes != goal.numRoutes ) {
		std::cout << "base num routes " << base.numRoutes << std::endl;
		std::cout << "goal num routes " << goal.numRoutes << std::endl;
		std::cout << "pop size " << params.population.size << std::endl;
		system("pause"); // TODO[dbg]
		throw String( "UNMATCH NUMBER OF ROUTES." );
	}
	//if (params.dbg.lk)std::cout << "classify edges 0.2" << std::endl;
	/* 对双亲边进行分类, 并通过 `edgesMap` 进行映射 */
	for ( int routeID = 0; routeID < base.numDepots; ++routeID )
	{
		//if (params.dbg.lk)std::cout << "classify edges 1.0" << std::endl;
		auto &depot = base.depots[routeID];
		bool unique = depot.links[0] != depot.links[1] ? true : false;
		int startNodeId = depot.ID , predNodeId = startNodeId , curNodeId = depot.links[1];
		//if (params.dbg.lk)std::cout << "classify edges 1.1" << std::endl;
		while ( curNodeId != startNodeId )
		{
			//if (params.dbg.lk)std::cout << "classify edges 1.2" << std::endl;
			addRichEdge( predNodeId , curNodeId , Owner::BASE , unique , routeID );
			predNodeId = curNodeId;
			curNodeId = base.nodes[curNodeId].links[1];
			//if (params.dbg.lk)std::cout << "classify edges 1.3" << std::endl;
		}
		//if (params.dbg.lk)std::cout << "classify edges 1.4" << std::endl;
		addRichEdge( predNodeId , curNodeId , Owner::BASE , unique , routeID );
		//if (params.dbg.lk)std::cout << "classify edges 1.5" << std::endl;
	}
	//if (params.dbg.lk)std::cout << "classify edges 1" << std::endl;
	for ( int routeID = 0; routeID < goal.numDepots; ++routeID )
	{
		auto &depot = goal.depots[routeID];
		bool unique = depot.links[0] != depot.links[1] ? true : false;
		int startNodeId = depot.ID , predNodeId = startNodeId , curNodeId = depot.links[1];
		while ( curNodeId != startNodeId )
		{
			addRichEdge( predNodeId , curNodeId , Owner::GOAL , unique , routeID );
			predNodeId = curNodeId;
			curNodeId = goal.nodes[curNodeId].links[1];
		}
		addRichEdge( predNodeId , curNodeId , Owner::GOAL , unique , routeID );
	}
	//if (params.dbg.lk)std::cout << "classify edges 2" << std::endl;
	/* 构建双亲边的邻接表 */
	for ( auto& kv : edgesMap )
	{
		auto& rEdge = richEdges[kv.second];

		// 不唯一的边会在双方的邻接列表中出现两次; 即 0->v->0 存在时, `adjNodeTable[0]` 会有两个邻接点 v, `adjNodeTable[v]` 也会有两个邻接点 0
		int i = rEdge.nodeIDs.first , j = rEdge.nodeIDs.second;
		auto& iSize = adjNodeSizes[i];
		adjNodeTable[i][iSize++] = j; // j 邻接于 i
		auto& jSize = adjNodeSizes[j];
		adjNodeTable[j][jSize++] = i; // i 邻接于 j
		//if (params.dbg.lk)std::cout << "classify edges 3" << std::endl;
		// 处理其余映射
		if ( rEdge.owner != Owner::COMMON )
		{
			privateEdgesMap[rEdge.code] = rEdge.ID;
			if ( rEdge.owner == Owner::BASE )
			{
				baseEdgesMap[rEdge.code] = rEdge.ID;
			}
			else
			{
				goalPrivateEdgesMap[rEdge.code] = rEdge.ID;
			}
		}
		else
		{
			commonEdgesMap[rEdge.code] = rEdge.ID;
			baseEdgesMap[rEdge.code] = rEdge.ID;
		}
	}
	//if (params.dbg.lk)std::cout << "classify edges 4" << std::endl;
	/* 将邻接列表中的节点 ID 均替换为对应 `RichEdge` ID, 加速后续遍历 */
	for ( int ID = 0; ID < params.numNodes; ++ID )
	{
		for ( int k = 0; k < adjNodeSizes[ID]; ++k )
		{
			int i = ID , j = adjNodeTable[i][k];
			int code = toCode( i , j , true );
			auto &rEdge = richEdges[edgesMap[code]];

			// 替换时默认按照唯一 (即 i < j) 进行; 如果不唯一, 则首次替换为 <i, j> 的 `RichEdge` ID, 下一次替换为 <j, i> 的 `RichEdge` ID
			if ( !rEdge.unique )
			{
				if ( rEdge.turn )
				{
					if ( i < j ) { std::swap( i , j ); }
					code = toCode( i , j , false );
					adjNodeTable[ID][k] = edgesMap[code];
					rEdge.turn = false; // 为另一端点重置; 即当前节点 0 替换完成之后, 对端节点 v 也需要同样地替换
				}
				else
				{
					adjNodeTable[ID][k] = rEdge.ID;
					rEdge.turn = true; // 下一次转到 <j, i> 进行替换
				}
			}
			else { adjNodeTable[ID][k] = rEdge.ID; }
		}
	}
}

void hsh::cvrp::Eax::generateCycles( int repeatTimes )
{
	struct AdjInfo
	{
		enum Status { EVEN_HOOP , ODD_HOOP , EXPLORING , UNKNOWN };
		Status stat = UNKNOWN;
		int ID = -1;
		int nodeID = -1; // 另一端节点 ID
		int formedSize = -1; // 已形成闭环的大小

		AdjInfo()
		{ }

		AdjInfo( Status stat , int ID , int nodeID , int size ) :
			stat( stat ) , ID( ID ) , nodeID( nodeID ) , formedSize( size )
		{ }
	};

	cycles.clear();
	if ( privateEdgesMap.size() == 0 )
	{
		return;
	}

	for ( int times = 0; times < repeatTimes; ++times )
	{
		/* 设置所有双亲边状态 */
		single.clear();
		setUsable( commonEdgesMap , false ); // 设置所有共有边不可用
		setUsable( privateEdgesMap , true ); // 设置所有私有边可用
		setVisited( privateEdgesMap , false ); // 设置所有私有边未访问

		/* 开始分解 GAB */
		int visitedCount = 0; // 已遍历私有边的计数
		int requiredCount = privateEdgesMap.size(); // 需遍历私有边的总数

		auto keyValStartTraversal = privateEdgesMap.begin();
		auto keyValEnd = privateEdgesMap.end();
		auto keyValIt = privateEdgesMap.begin(); // 顺序遍历, 以发现尚未访问的私有边, 作为下一个 AB-Cycle 的起始

		int	startID; // 每次遍历开始的 `RichEdeg` ID
		int startNodeID; // 对应的节点 ID

		int curNodeID; // 当前遍历到的节点 ID, 作为下一条差异边的开始节点 ID
		int currentSteps; // 当前遍历的步数
		RichEdge *cur; // 当前遍历到的差异边

		for ( int offset = rand() % ( privateEdgesMap.size() ); offset >= 1; --offset ) { keyValStartTraversal++; } // 随机选择一个私有边作为遍历的开始
		currentSteps = 0;
		traversalPath.clear();
		resetArrivalSteps(); // 清空遍历到达每个节点的步数栈
		startID = keyValStartTraversal->second;  // std::cout << "StartID " << startID << ", visited " << richEdges[startID].visited << std::endl;
		startNodeID = richEdges[startID].nodeIDs.first;
		arrivalSteps[startNodeID].push_back( currentSteps );

		cur = &richEdges[startID];
		curNodeID = cur->nodeIDs.second;
		cur->visited = true; // std::cout << "marked " << cur->ID << std::endl;
		traversalPath.push_back( startID );
		++visitedCount; // 回到起点时再对起点计数
		++currentSteps;
		arrivalSteps[curNodeID].push_back( currentSteps );

		Vec<AdjInfo> valid , alternative;
		valid.reserve( params.numNodes );
		alternative.reserve( params.numNodes );

		while ( visitedCount < requiredCount )
		{
			/* 为下一步寻找符合遍历规则的邻接边集合 */
			valid.clear();
			bool incluEvenCycle = false , incluExploring = false;
			for ( int k = 0; k < adjNodeSizes[curNodeID]; ++k )
			{
				int adjID = adjNodeTable[curNodeID][k];
				auto &adj = richEdges[adjID];

				if ( !adj.usable || ( adj.owner == cur->owner ) || adj.visited )
				{
					continue;
				}
				else
				{
					int adjNodeID = ( ( adj.nodeIDs ).first != curNodeID ? ( adj.nodeIDs ).first : ( adj.nodeIDs ).second );
					if ( arrivalSteps[adjNodeID].size() != 0 )
					{
						// 以前访问过 `adjNodeID` 对应的节点, 形成闭环
						int prevSteps = arrivalSteps[adjNodeID].back();
						int	length = ( currentSteps + 1 ) - prevSteps;
						if ( length % 2 != 0 )
						{
							// 奇数闭环, 需进一步判断
							if ( arrivalSteps[adjNodeID].size() > 1 )
							{
								// 上一次访问时也必定是奇数闭环, 组成一个 AB-Cycle
								int prevPrevSteps = arrivalSteps[adjNodeID][( arrivalSteps[adjNodeID].size() - 2 )];
								int combinedLength = ( currentSteps + 1 ) - prevPrevSteps;
								valid.push_back( AdjInfo( AdjInfo::Status::EVEN_HOOP , adjID , adjNodeID , combinedLength ) );
								incluEvenCycle = true;
							}
							else
							{
								// 等待另一奇数闭环
								valid.push_back( AdjInfo( AdjInfo::Status::ODD_HOOP , adjID , adjNodeID , length ) );
							}
						}
						else
						{
							// 偶数闭环
							valid.push_back( AdjInfo( AdjInfo::Status::EVEN_HOOP , adjID , adjNodeID , length ) );
							incluEvenCycle = true;
						}
					}
					else
					{
						// 该节点尚未访问过, 继续遍历
						valid.push_back( AdjInfo( AdjInfo::Status::EXPLORING , adjID , adjNodeID , -1 ) );
						incluExploring = true;
					}
				}
			}

			/* 有效邻接边排序 */
			alternative.clear();
			for ( auto &next : valid )
			{
				if ( incluEvenCycle )
				{
					// 优先形成偶数闭环
					if ( next.stat == AdjInfo::Status::EVEN_HOOP ) { alternative.push_back( next ); }
				}
				else
				{
					if ( incluExploring )
					{
						if ( next.stat == AdjInfo::Status::EXPLORING ) { alternative.push_back( next ); }
					}
					else
					{
						alternative.push_back( next ); // 避免奇数闭环
					}
				}
			}

			if ( !alternative.empty() )
			{
				int picked = rand() % alternative.size();
				auto &nextInfo = alternative[picked];
				int nextID = nextInfo.ID;
				auto &next = richEdges[nextID];

				cur = &next;
				curNodeID = alternative[picked].nodeID;
				cur->visited = true; // std::cout << "marked " << cur->ID << std::endl;
				traversalPath.push_back( cur->ID );
				++visitedCount;
				++currentSteps;
				arrivalSteps[curNodeID].push_back( currentSteps );

				if ( nextInfo.stat != AdjInfo::Status::EVEN_HOOP )
				{
					// 未形成 AB-Cycle, 继续遍历
					continue;
				}
				else
				{
					// 形成 AB-Cycle, 判断是否需要寻找新的遍历起点
					if ( curNodeID == startNodeID && traversalPath.size() == nextInfo.formedSize )
					{
						// 整个遍历路径是 AB-Cycle, 寻找新的遍历起点
						single.push_back( Cycle( traversalPath.begin() , traversalPath.end() ) ); // std::cout << "formed final size " << traversalPath.size() << std::endl;
						// for ( auto ID : single.back() ) { std::cout << ID << " "; }std::cout << std::endl;
					}
					else
					{
						// 遍历路径的部分是 AB-Cycle
						int length = nextInfo.formedSize; // std::cout << "formed size " << length << std::endl;
						single.push_back( {} );
						do
						{
							arrivalSteps[curNodeID].pop_back();
							single.back().push_back( traversalPath.back() );
							traversalPath.pop_back();
							length--;

							int prevNodeID = ( ( cur->nodeIDs ).first != curNodeID ? ( cur->nodeIDs ).first : ( cur->nodeIDs ).second );
							curNodeID = prevNodeID;
							cur = &richEdges[traversalPath.back()];
						} while ( length != 0 );
						currentSteps = arrivalSteps[curNodeID].back();
						// for ( auto ID : single.back() ) { std::cout << ID << " "; }std::cout << std::endl;
						continue; // 从其余遍历路径继续
					}
				}
			}
			else
			{
				using namespace std;
				cout << "start it " << keyValStartTraversal->first << ", " << keyValStartTraversal->second << endl;
				cout << curNodeID << "no legal next edge" << endl;
				system( "pause" );
			}

			/* 寻找新的遍历起点 */
			for ( ; keyValIt != keyValEnd; keyValIt++ ) //TODO[这里每次寻找新的起点都是 ID 小的优先, 是否影响?]
			{
				auto& rEdge = richEdges[keyValIt->second];
				if ( !rEdge.visited )
				{
					keyValStartTraversal = keyValIt;
					currentSteps = 0;
					traversalPath.clear();
					resetArrivalSteps(); // 清空遍历到达每个节点的步数栈
					startID = keyValStartTraversal->second; // std::cout << "StartID " << startID << ", visited " << richEdges[startID].visited << std::endl;
					startNodeID = richEdges[startID].nodeIDs.first;
					arrivalSteps[startNodeID].push_back( currentSteps );

					cur = &richEdges[startID];
					curNodeID = cur->nodeIDs.second;
					cur->visited = true;
					traversalPath.push_back( startID );
					++visitedCount; // 回到起点时再对起点计数
					++currentSteps;
					arrivalSteps[curNodeID].push_back( currentSteps );

					break;
				}
			}
		}


		/* 记录数量最多的 AB-Cycle 集合, 并更新所有 `RichEdge` 的 `cycleID` 字段 */
		if ( single.size() > cycles.size() )
		{
			std::swap( single , cycles);
			for ( int cycleID = 0; cycleID < cycles.size(); ++cycleID )
			{
				auto &cycle = cycles[cycleID];
				for ( auto & ID : cycle )
				{
					auto &rEdge = richEdges[ID];
					rEdge.cycleID = cycleID;
				}
			}
		}
	}

}

void hsh::cvrp::Eax::applyCycle( Cycle & cycle , Individual & indiv )
{
	/* 标记本次选择的 AB-Cycle ID */
	if ( cycle.empty() ) { return; }
	indiv.cycleID = richEdges[cycle.front()].cycleID;

	/* 将 `cycle` 中的 base 边删除 */
	for ( auto ID : cycle )
	{
		auto &rEdge = richEdges[ID];
		if ( rEdge.owner != Owner::GOAL )
		{
			int i = rEdge.nodeIDs.first , j = rEdge.nodeIDs.second;
			int( &iLink )[2] = indiv.nodes[i].links , // 暂时将 `node[0]` 作为删除/连接的操作对象, 取消额外的条件判断, 但是并不考虑可能的 `node[0]` 多次操作而引起冲突,
				( &jLink )[2] = indiv.nodes[j].links; // 客户节点记录了完整的连接信息, 后续只从客户节点开始遍历, 重新组织个体
			int &i_j = ( iLink[0] != j ? iLink[1] : iLink[0] ) ,
				&j_i = ( jLink[0] != i ? jLink[1] : jLink[0] );

			i_j = -2; j_i = -2;
		}
	}

	/* 将断点按 `cycle` 中的 goal 边进行连接 */
	for ( auto ID : cycle )
	{
		auto &rEdge = richEdges[ID];
		if ( rEdge.owner != Owner::BASE )
		{
			int i = rEdge.nodeIDs.first , j = rEdge.nodeIDs.second;
			int( &iLink )[2] = indiv.nodes[i].links ,
				( &jLink )[2] = indiv.nodes[j].links;
			int &i_break = ( iLink[0] != -2 ? iLink[1] : iLink[0] ) ,
				&j_break = ( jLink[0] != -2 ? jLink[1] : jLink[0] );

			i_break = j; j_break = i;
		}
	}

	/* 重新开始遍历个体 */
	for ( int nodeID = 1; nodeID < params.numNodes; ++nodeID ) { visited[nodeID] = false; }
	for ( int nodeID = 1; nodeID < params.numNodes; ++nodeID )
	{
		if ( visited[nodeID] ) { continue; }
		// 只从客户节点开始遍历
		int startNodeID = nodeID;
		bool isRoute = false , isSubtour = false;

		/* 根据开始客户节点的连接方向重新组织 */
		visited[startNodeID] = true;
		for ( int prevNodeID = startNodeID , curNodeID = indiv.nodes[prevNodeID].links[1];
			  ;
			  prevNodeID = curNodeID , curNodeID = indiv.nodes[prevNodeID].links[1] )
		{
			if ( curNodeID != 0 )
			{
				if ( curNodeID != startNodeID )
				{
					int( &curNodeLink )[2] = indiv.nodes[curNodeID].links;

					if ( curNodeLink[0] != prevNodeID ) { std::swap( curNodeLink[1] , curNodeLink[0] ); }
					visited[curNodeID] = true;
				}
				else
				{
					// 遍历未遇到仓库节点 0, 且遍历结束, 属于子回路
					isSubtour = true;
					break;
				}
			}
			else
			{
				// 遍历遇到仓库节点, 闭环形成时即是路径, 后半段遍历另外完成
				isRoute = true;
				indiv.nodes[0].links[0] = prevNodeID; // 暂时使用 `node[0]` 进行连接
				break;
			}
		}

		if ( isRoute )
		{
			/* 完成后半段路径, 逆着连接状态进行遍历 */
			for ( int succNodeID = startNodeID , curNodeID = indiv.nodes[succNodeID].links[0];
				  ;
				  succNodeID = curNodeID , curNodeID = indiv.nodes[succNodeID].links[0] )
			{
				if ( curNodeID != 0 )
				{
					int( &curNodeLink )[2] = indiv.nodes[curNodeID].links;

					if ( curNodeLink[1] != succNodeID ) { std::swap( curNodeLink[0] , curNodeLink[1] ); }
					visited[curNodeID] = true;
				}
				else
				{
					/* 路径形成 */
					indiv.nodes[0].links[1] = succNodeID; // 暂时使用 `node[0]` 进行连接
					Node &routeDepot = indiv.addRouteDepot();
					routeDepot.ID = 0;
					routeDepot.routeID = indiv.numRoutes - 1;
					routeDepot.links[0] = indiv.nodes[0].links[0];
					routeDepot.links[1] = indiv.nodes[0].links[1];
					break;
				}
			}
		}
		else
		{
			/* 子回路形成 */
			Node &subtDepot = indiv.addSubtDepot( params.critical.numRoutes );
			subtDepot.ID = startNodeID;
			subtDepot.routeID = indiv.numSubtours - 1 + params.critical.numRoutes;
			subtDepot.links[0] = indiv.nodes[startNodeID].links[0];
			subtDepot.links[1] = indiv.nodes[startNodeID].links[1];
		}

		indiv.nodes[0].reset();
	}

	/* 完成个体的重新组织, 并检查路径数 */
	indiv.done( params.critical.numRoutes );

	/* 评估可能带子回路个体的基本信息 */
	indiv.distance = indiv.getDistance();
	for ( int routeID = 0; routeID < params.critical.numRoutes; ++routeID )
	{
		indiv.capacityExcess += indiv.getCapacityExcessOf( routeID );
		indiv.durationExcess += indiv.getDurationExcessOf( routeID );
	}
	if ( indiv.numSubtours != 0 )
	{
		double subtLoads = 0. , subtDurations = 0.;

		for ( int subtID = 0; subtID < indiv.numSubtours; ++subtID )
		{
			subtLoads += indiv.getLoadOf( subtID + params.critical.numRoutes );
			subtDurations += indiv.getDurationOf( subtID + params.critical.numRoutes );
		}
		// 所有子回路的载重之和 ( 服务时长之和 ) 视为一条路径去评估违反量
		indiv.capacityExcess += indiv.getExcessOf( params.capacityLimit , subtLoads );
		indiv.durationExcess += indiv.getExcessOf( params.durationLimit , subtDurations );
	}
}

void hsh::cvrp::Eax::applyCycles( Vec<Cycle>& cycles , Individual & indiv )
{
	for ( auto &cycle : cycles )
	{
		applyCycle( cycle , indiv );
	}
}

inline void hsh::cvrp::Eax::RichEdge::set( int ID , int i , int j , int code , Owner owner , bool unique , int routeID )
{
	this->ID = ID;
	nodeIDs.first = i;
	nodeIDs.second = j;
	this->code = code;
	this->owner = owner;
	this->unique = unique;
	if ( owner == Owner::BASE ) { baseRouteID = routeID; goalRouteID = -1; }
	else { goalRouteID = routeID; baseRouteID = -1; }
	turn = false;
	usable = true;
	visited = false;
	cycleID = -1;
}
