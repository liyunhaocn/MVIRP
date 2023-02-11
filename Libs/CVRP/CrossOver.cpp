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
	if ( unique && i > j ) { std::swap( i , j ); } // ������ΪΨһ
	int code = toCode( i , j , unique );

	if ( edgesMap.insert( { code, allocatedEnd } ).second == false )
	{
		/* ���� */
		auto& rEdge = richEdges[edgesMap[code]];
		rEdge.owner = Owner::COMMON;
		if ( !unique ) { rEdge.unique = false; }
		rEdge.goalRouteID = routeID;
	}
	else
	{
		/* ˽�� */
		auto& rEdge = richEdges[allocatedEnd];
		rEdge.set( allocatedEnd , i , j , code , owner , unique , routeID );
		allocatedEnd++;
	}
}

void hsh::cvrp::Eax::prepareDataStructures()
{
	/* ��һ�ε���ʱ�������ݽṹ����Ĵ洢�ռ� */
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

	/* �����������ݽṹ */
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
	/* ��˫�ױ߽��з���, ��ͨ�� `edgesMap` ����ӳ�� */
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
	/* ����˫�ױߵ��ڽӱ� */
	for ( auto& kv : edgesMap )
	{
		auto& rEdge = richEdges[kv.second];

		// ��Ψһ�ı߻���˫�����ڽ��б��г�������; �� 0->v->0 ����ʱ, `adjNodeTable[0]` ���������ڽӵ� v, `adjNodeTable[v]` Ҳ���������ڽӵ� 0
		int i = rEdge.nodeIDs.first , j = rEdge.nodeIDs.second;
		auto& iSize = adjNodeSizes[i];
		adjNodeTable[i][iSize++] = j; // j �ڽ��� i
		auto& jSize = adjNodeSizes[j];
		adjNodeTable[j][jSize++] = i; // i �ڽ��� j
		//if (params.dbg.lk)std::cout << "classify edges 3" << std::endl;
		// ��������ӳ��
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
	/* ���ڽ��б��еĽڵ� ID ���滻Ϊ��Ӧ `RichEdge` ID, ���ٺ������� */
	for ( int ID = 0; ID < params.numNodes; ++ID )
	{
		for ( int k = 0; k < adjNodeSizes[ID]; ++k )
		{
			int i = ID , j = adjNodeTable[i][k];
			int code = toCode( i , j , true );
			auto &rEdge = richEdges[edgesMap[code]];

			// �滻ʱĬ�ϰ���Ψһ (�� i < j) ����; �����Ψһ, ���״��滻Ϊ <i, j> �� `RichEdge` ID, ��һ���滻Ϊ <j, i> �� `RichEdge` ID
			if ( !rEdge.unique )
			{
				if ( rEdge.turn )
				{
					if ( i < j ) { std::swap( i , j ); }
					code = toCode( i , j , false );
					adjNodeTable[ID][k] = edgesMap[code];
					rEdge.turn = false; // Ϊ��һ�˵�����; ����ǰ�ڵ� 0 �滻���֮��, �Զ˽ڵ� v Ҳ��Ҫͬ�����滻
				}
				else
				{
					adjNodeTable[ID][k] = rEdge.ID;
					rEdge.turn = true; // ��һ��ת�� <j, i> �����滻
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
		int nodeID = -1; // ��һ�˽ڵ� ID
		int formedSize = -1; // ���γɱջ��Ĵ�С

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
		/* ��������˫�ױ�״̬ */
		single.clear();
		setUsable( commonEdgesMap , false ); // �������й��б߲�����
		setUsable( privateEdgesMap , true ); // ��������˽�б߿���
		setVisited( privateEdgesMap , false ); // ��������˽�б�δ����

		/* ��ʼ�ֽ� GAB */
		int visitedCount = 0; // �ѱ���˽�бߵļ���
		int requiredCount = privateEdgesMap.size(); // �����˽�бߵ�����

		auto keyValStartTraversal = privateEdgesMap.begin();
		auto keyValEnd = privateEdgesMap.end();
		auto keyValIt = privateEdgesMap.begin(); // ˳�����, �Է�����δ���ʵ�˽�б�, ��Ϊ��һ�� AB-Cycle ����ʼ

		int	startID; // ÿ�α�����ʼ�� `RichEdeg` ID
		int startNodeID; // ��Ӧ�Ľڵ� ID

		int curNodeID; // ��ǰ�������Ľڵ� ID, ��Ϊ��һ������ߵĿ�ʼ�ڵ� ID
		int currentSteps; // ��ǰ�����Ĳ���
		RichEdge *cur; // ��ǰ�������Ĳ����

		for ( int offset = rand() % ( privateEdgesMap.size() ); offset >= 1; --offset ) { keyValStartTraversal++; } // ���ѡ��һ��˽�б���Ϊ�����Ŀ�ʼ
		currentSteps = 0;
		traversalPath.clear();
		resetArrivalSteps(); // ��ձ�������ÿ���ڵ�Ĳ���ջ
		startID = keyValStartTraversal->second;  // std::cout << "StartID " << startID << ", visited " << richEdges[startID].visited << std::endl;
		startNodeID = richEdges[startID].nodeIDs.first;
		arrivalSteps[startNodeID].push_back( currentSteps );

		cur = &richEdges[startID];
		curNodeID = cur->nodeIDs.second;
		cur->visited = true; // std::cout << "marked " << cur->ID << std::endl;
		traversalPath.push_back( startID );
		++visitedCount; // �ص����ʱ�ٶ�������
		++currentSteps;
		arrivalSteps[curNodeID].push_back( currentSteps );

		Vec<AdjInfo> valid , alternative;
		valid.reserve( params.numNodes );
		alternative.reserve( params.numNodes );

		while ( visitedCount < requiredCount )
		{
			/* Ϊ��һ��Ѱ�ҷ��ϱ���������ڽӱ߼��� */
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
						// ��ǰ���ʹ� `adjNodeID` ��Ӧ�Ľڵ�, �γɱջ�
						int prevSteps = arrivalSteps[adjNodeID].back();
						int	length = ( currentSteps + 1 ) - prevSteps;
						if ( length % 2 != 0 )
						{
							// �����ջ�, ���һ���ж�
							if ( arrivalSteps[adjNodeID].size() > 1 )
							{
								// ��һ�η���ʱҲ�ض��������ջ�, ���һ�� AB-Cycle
								int prevPrevSteps = arrivalSteps[adjNodeID][( arrivalSteps[adjNodeID].size() - 2 )];
								int combinedLength = ( currentSteps + 1 ) - prevPrevSteps;
								valid.push_back( AdjInfo( AdjInfo::Status::EVEN_HOOP , adjID , adjNodeID , combinedLength ) );
								incluEvenCycle = true;
							}
							else
							{
								// �ȴ���һ�����ջ�
								valid.push_back( AdjInfo( AdjInfo::Status::ODD_HOOP , adjID , adjNodeID , length ) );
							}
						}
						else
						{
							// ż���ջ�
							valid.push_back( AdjInfo( AdjInfo::Status::EVEN_HOOP , adjID , adjNodeID , length ) );
							incluEvenCycle = true;
						}
					}
					else
					{
						// �ýڵ���δ���ʹ�, ��������
						valid.push_back( AdjInfo( AdjInfo::Status::EXPLORING , adjID , adjNodeID , -1 ) );
						incluExploring = true;
					}
				}
			}

			/* ��Ч�ڽӱ����� */
			alternative.clear();
			for ( auto &next : valid )
			{
				if ( incluEvenCycle )
				{
					// �����γ�ż���ջ�
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
						alternative.push_back( next ); // ���������ջ�
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
					// δ�γ� AB-Cycle, ��������
					continue;
				}
				else
				{
					// �γ� AB-Cycle, �ж��Ƿ���ҪѰ���µı������
					if ( curNodeID == startNodeID && traversalPath.size() == nextInfo.formedSize )
					{
						// ��������·���� AB-Cycle, Ѱ���µı������
						single.push_back( Cycle( traversalPath.begin() , traversalPath.end() ) ); // std::cout << "formed final size " << traversalPath.size() << std::endl;
						// for ( auto ID : single.back() ) { std::cout << ID << " "; }std::cout << std::endl;
					}
					else
					{
						// ����·���Ĳ����� AB-Cycle
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
						continue; // ���������·������
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

			/* Ѱ���µı������ */
			for ( ; keyValIt != keyValEnd; keyValIt++ ) //TODO[����ÿ��Ѱ���µ���㶼�� ID С������, �Ƿ�Ӱ��?]
			{
				auto& rEdge = richEdges[keyValIt->second];
				if ( !rEdge.visited )
				{
					keyValStartTraversal = keyValIt;
					currentSteps = 0;
					traversalPath.clear();
					resetArrivalSteps(); // ��ձ�������ÿ���ڵ�Ĳ���ջ
					startID = keyValStartTraversal->second; // std::cout << "StartID " << startID << ", visited " << richEdges[startID].visited << std::endl;
					startNodeID = richEdges[startID].nodeIDs.first;
					arrivalSteps[startNodeID].push_back( currentSteps );

					cur = &richEdges[startID];
					curNodeID = cur->nodeIDs.second;
					cur->visited = true;
					traversalPath.push_back( startID );
					++visitedCount; // �ص����ʱ�ٶ�������
					++currentSteps;
					arrivalSteps[curNodeID].push_back( currentSteps );

					break;
				}
			}
		}


		/* ��¼�������� AB-Cycle ����, ���������� `RichEdge` �� `cycleID` �ֶ� */
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
	/* ��Ǳ���ѡ��� AB-Cycle ID */
	if ( cycle.empty() ) { return; }
	indiv.cycleID = richEdges[cycle.front()].cycleID;

	/* �� `cycle` �е� base ��ɾ�� */
	for ( auto ID : cycle )
	{
		auto &rEdge = richEdges[ID];
		if ( rEdge.owner != Owner::GOAL )
		{
			int i = rEdge.nodeIDs.first , j = rEdge.nodeIDs.second;
			int( &iLink )[2] = indiv.nodes[i].links , // ��ʱ�� `node[0]` ��Ϊɾ��/���ӵĲ�������, ȡ������������ж�, ���ǲ������ǿ��ܵ� `node[0]` ��β����������ͻ,
				( &jLink )[2] = indiv.nodes[j].links; // �ͻ��ڵ��¼��������������Ϣ, ����ֻ�ӿͻ��ڵ㿪ʼ����, ������֯����
			int &i_j = ( iLink[0] != j ? iLink[1] : iLink[0] ) ,
				&j_i = ( jLink[0] != i ? jLink[1] : jLink[0] );

			i_j = -2; j_i = -2;
		}
	}

	/* ���ϵ㰴 `cycle` �е� goal �߽������� */
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

	/* ���¿�ʼ�������� */
	for ( int nodeID = 1; nodeID < params.numNodes; ++nodeID ) { visited[nodeID] = false; }
	for ( int nodeID = 1; nodeID < params.numNodes; ++nodeID )
	{
		if ( visited[nodeID] ) { continue; }
		// ֻ�ӿͻ��ڵ㿪ʼ����
		int startNodeID = nodeID;
		bool isRoute = false , isSubtour = false;

		/* ���ݿ�ʼ�ͻ��ڵ�����ӷ���������֯ */
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
					// ����δ�����ֿ�ڵ� 0, �ұ�������, �����ӻ�·
					isSubtour = true;
					break;
				}
			}
			else
			{
				// ���������ֿ�ڵ�, �ջ��γ�ʱ����·��, ���α����������
				isRoute = true;
				indiv.nodes[0].links[0] = prevNodeID; // ��ʱʹ�� `node[0]` ��������
				break;
			}
		}

		if ( isRoute )
		{
			/* ��ɺ���·��, ��������״̬���б��� */
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
					/* ·���γ� */
					indiv.nodes[0].links[1] = succNodeID; // ��ʱʹ�� `node[0]` ��������
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
			/* �ӻ�·�γ� */
			Node &subtDepot = indiv.addSubtDepot( params.critical.numRoutes );
			subtDepot.ID = startNodeID;
			subtDepot.routeID = indiv.numSubtours - 1 + params.critical.numRoutes;
			subtDepot.links[0] = indiv.nodes[startNodeID].links[0];
			subtDepot.links[1] = indiv.nodes[startNodeID].links[1];
		}

		indiv.nodes[0].reset();
	}

	/* ��ɸ����������֯, �����·���� */
	indiv.done( params.critical.numRoutes );

	/* �������ܴ��ӻ�·����Ļ�����Ϣ */
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
		// �����ӻ�·������֮�� ( ����ʱ��֮�� ) ��Ϊһ��·��ȥ����Υ����
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
