#include "NagataBase.h"

void hsh::cvrp::NagataBase::constructNeighborhoodOf( int V , int rangeNear , Vec<Move::Type> &typeOrder , Individual &indiv )
{
	/* 以 ID 表示对应节点或路径 */

	if ( V == 0 ) { return; }
	int prevV = indiv.nodes[V].links[0] , succV = indiv.nodes[V].links[1];
	int routeV = indiv.nodes[V].routeID;
	//int moveCount = 0;
	for ( auto curType : typeOrder )
	{
		if ( curType == Move::Type::INSKIND1 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w- and w*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || succV == W || succV == prevW ) { skip = true; }
					}
				}
				/*in the case that vRouteId!=wRouteId, v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND2 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w and w+*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == succW || prevV == W || succV == succW || succV == W ) { skip = true; } //
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND1 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w-*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || prevV == prevPrevW || succV == prevW || succV == prevPrevW )
						{
							skip = true;
						}
					}
				}
				else
				{
					if ( prevW == 0 ) { skip = true; }
				}
				if ( !skip )
				{
					Move move( Move::Type::SWAPKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND2 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w+*/
				{
					bool skip = false;
					if ( routeV == routeW )
					{
						if ( sizes[routeV] < 4 ) { skip = true; }
						else
						{
							if ( prevV == succSuccW || prevV == succW || prevV == W || succV == succW || succV == W )
							{
								skip = true;
							}
						}
					}
					else
					{
						if ( succW == 0 ) { skip = true; }
					}
					if ( !skip )
					{
						Move move( Move::Type::SWAPKIND2 , V , W , routeV , routeW );
						if ( evaluateMove( move , indiv ) ) { return; }
					}//moveCount++;
				}
			}
		}

		if ( curType == Move::Type::INSKIND3 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v-*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == V || prevW == prevV || succW == V || succW == prevV ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND4 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v+*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == succV || prevW == V || succW == succV || succW == V ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND1 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w-, w, ||*/
				bool skip = false;
				if ( routeV == routeW && ( prevV == W || prevV == prevW || V == prevW ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND2 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w, w+, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND3 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w-, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND4 )
		{
			for ( int k = 0; k < rangeNear; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w+, ||*/
				bool skip = false;
				if ( routeV == routeW && ( V == succW || V == W || succV == W ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}
	}
	//std::cout << "move count" << moveCount << std::endl;
}

void hsh::cvrp::NagataBase::constructNeighborhoodOf( int V , int rangeProximityStart , int rangeProximityEnd , Vec<Move::Type>& typeOrder , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */

	if ( V == 0 ) { return; }
	int prevV = indiv.nodes[V].links[0] , succV = indiv.nodes[V].links[1];
	int routeV = indiv.nodes[V].routeID;
	//int moveCount = 0;
	for ( auto curType : typeOrder )
	{
		if ( curType == Move::Type::INSKIND1 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w- and w*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || succV == W || succV == prevW ) { skip = true; }
					}
				}
				/*in the case that vRouteId!=wRouteId, v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND2 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w and w+*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == succW || prevV == W || succV == succW || succV == W ) { skip = true; } //
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND1 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w-*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || prevV == prevPrevW || succV == prevW || succV == prevPrevW )
						{
							skip = true;
						}
					}
				}
				else
				{
					if ( prevW == 0 ) { skip = true; }
				}
				if ( !skip )
				{
					Move move( Move::Type::SWAPKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND2 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w+*/
				{
					bool skip = false;
					if ( routeV == routeW )
					{
						if ( sizes[routeV] < 4 ) { skip = true; }
						else
						{
							if ( prevV == succSuccW || prevV == succW || prevV == W || succV == succW || succV == W )
							{
								skip = true;
							}
						}
					}
					else
					{
						if ( succW == 0 ) { skip = true; }
					}
					if ( !skip )
					{
						Move move( Move::Type::SWAPKIND2 , V , W , routeV , routeW );
						if ( evaluateMove( move , indiv ) ) { return; }
					}//moveCount++;
				}
			}
		}

		if ( curType == Move::Type::INSKIND3 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v-*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == V || prevW == prevV || succW == V || succW == prevV ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND4 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v+*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == succV || prevW == V || succW == succV || succW == V ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND1 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w-, w, ||*/
				bool skip = false;
				if ( routeV == routeW && ( prevV == W || prevV == prevW || V == prevW ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND2 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w, w+, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND3 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w-, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND4 )
		{
			for ( int k = rangeProximityStart; k < rangeProximityEnd; ++k )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w+, ||*/
				bool skip = false;
				if ( routeV == routeW && ( V == succW || V == W || succV == W ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}
	}
	//std::cout << "move count" << moveCount << std::endl;
}

void hsh::cvrp::NagataBase::constructNeighborhoodOf( int V , Vec<int>& orderProximity , Vec<Move::Type>& typeOrder , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */

	if ( V == 0 ) { return; }
	int prevV = indiv.nodes[V].links[0] , succV = indiv.nodes[V].links[1];
	int routeV = indiv.nodes[V].routeID;
	//int moveCount = 0;
	for ( auto curType : typeOrder )
	{
		if ( curType == Move::Type::INSKIND1 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w- and w*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || succV == W || succV == prevW ) { skip = true; }
					}
				}
				/*in the case that vRouteId!=wRouteId, v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND2 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert v between w and w+*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == succW || prevV == W || succV == succW || succV == W ) { skip = true; } //
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND1 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w-*/
				bool skip = false;
				if ( routeV == routeW )
				{
					if ( sizes[routeV] < 4 ) { skip = true; }
					else
					{
						if ( prevV == W || prevV == prevW || prevV == prevPrevW || succV == prevW || succV == prevPrevW )
						{
							skip = true;
						}
					}
				}
				else
				{
					if ( prevW == 0 ) { skip = true; }
				}
				if ( !skip )
				{
					Move move( Move::Type::SWAPKIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::SWAPKIND2 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*just think about the one-route case if wPred/wSucc is the depot, and wPred/wSucc's pred/succ is certain*/
				/*it's no need to consider about wPred/wSucc's multiple preds/succs*/
				/*because wPred/wSucc can't be the deopt in the two-route case here*/
				int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] ) ,
					succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
				/*swap v with w+*/
				{
					bool skip = false;
					if ( routeV == routeW )
					{
						if ( sizes[routeV] < 4 ) { skip = true; }
						else
						{
							if ( prevV == succSuccW || prevV == succW || prevV == W || succV == succW || succV == W )
							{
								skip = true;
							}
						}
					}
					else
					{
						if ( succW == 0 ) { skip = true; }
					}
					if ( !skip )
					{
						Move move( Move::Type::SWAPKIND2 , V , W , routeV , routeW );
						if ( evaluateMove( move , indiv ) ) { return; }
					}//moveCount++;
				}
			}
		}

		if ( curType == Move::Type::INSKIND3 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v-*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == V || prevW == prevV || succW == V || succW == prevV ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::INSKIND4 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
				int routeW = indiv.nodes[W].routeID;
				/*insert w between v and v+*/
				bool skip = false;
				if ( routeW == routeV )
				{
					if ( sizes[routeW] < 4 ) { skip = true; }
					else
					{
						if ( prevW == succV || prevW == V || succW == succV || succW == V ) { skip = true; }
					}
				}
				/*v≠0 is satisfied already*/
				if ( !skip )
				{
					Move move( Move::Type::INSKIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND1 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w-, w, ||*/
				bool skip = false;
				if ( routeV == routeW && ( prevV == W || prevV == prevW || V == prevW ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND1 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND2 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1]; if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v-, v, w, w+, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND2 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND3 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w-, X*/
				bool skip = false;
				if ( routeV == routeW ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND3 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}

		if ( curType == Move::Type::OPT2KIND4 )
		{
			for ( int k : orderProximity )
			{
				int W = params.proximityMat[V][k].ID , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];  if ( W == 0 ) { std::cout << k << "W==0" << V << std::endl; system( "pause" ); }
				int routeW = indiv.nodes[W].routeID;
				/*v, v+, w, w+, ||*/
				bool skip = false;
				if ( routeV == routeW && ( V == succW || V == W || succV == W ) ) { skip = true; }
				if ( !skip )
				{
					Move move( Move::Type::OPT2KIND4 , V , W , routeV , routeW );
					if ( evaluateMove( move , indiv ) ) { return; }
				}//moveCount++;
			}
		}
	}
	//std::cout << "move count" << moveCount << std::endl;
}

int hsh::cvrp::NagataBase::deltaNumRoutes( Move & move , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */
	Node &nodeV = indiv.nodes[move.V] , &nodeW = indiv.nodes[move.W];
	int V = move.V , prevV = nodeV.links[0] , succV = nodeV.links[1] , routeV = nodeV.routeID;
	int W = move.W , prevW = nodeW.links[0] , succW = nodeW.links[1] , routeW = nodeW.routeID;

	switch ( move.type )
	{
	case Move::Type::OPT2KIND1:
		{
			/*only both v- and w- point to the depot in two-route case*/
			if ( routeV != routeW && prevV == prevW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::OPT2KIND2:
		{
			/*only both v- and w+ point to the depot in two-route case*/
			if ( routeV != routeW && prevV == succW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::OPT2KIND3:
		{
			/*only both v+ and w- point to the depot in two-route case*/
			if ( routeV != routeW && succV == prevW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::OPT2KIND4:
		{
			/*only both v+ and w+ point to the depot in two-route case*/
			if ( routeV != routeW && succV == succW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::INSKIND1:
		{
			/*only vRoute is a one-customer route in two-rout case*/
			if ( routeV != routeW && prevV == succV ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::INSKIND2:
		{
			/*only vRoute is a one-customer route in two-rout case*/
			if ( routeV != routeW && prevV == succV ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::INSKIND3:
		{
			/*only wRoute is a one-customer route in two-rout case*/
			if ( routeV != routeW && prevW == succW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::INSKIND4:
		{
			/*only wRoute is a one-customer route in two-rout case*/
			if ( routeV != routeW && prevW == succW ) { return -1; }
			else { return 0; }
			break;
		}
	case Move::Type::SWAPKIND1:
		{
			return 0;
			break;
		}
	case Move::Type::SWAPKIND2:
		{
			return 0;
			break;
		}
	}
	return INT_MAX;
}

double hsh::cvrp::NagataBase::deltaDistance( Move & move , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */
	double deltaDistance = Inf;
	int V = move.V , prevV = indiv.nodes[V].links[0] , succV = indiv.nodes[V].links[1];
	int W = move.W , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1];
	switch ( move.type )
	{
	case Move::Type::OPT2KIND1:
		{
			deltaDistance = ( params.distanceMat[prevV][prevW] + params.distanceMat[V][W] - params.distanceMat[prevV][V] - params.distanceMat[prevW][W] );
			break;
		}
	case Move::Type::OPT2KIND2:
		{
			deltaDistance = ( params.distanceMat[prevV][succW] + params.distanceMat[V][W] - params.distanceMat[prevV][V] - params.distanceMat[W][succW] );
			break;
		}
	case Move::Type::OPT2KIND3:
		{
			deltaDistance = ( params.distanceMat[succV][prevW] + params.distanceMat[V][W] - params.distanceMat[V][succV] - params.distanceMat[prevW][W] );
			break;
		}
	case Move::Type::OPT2KIND4:
		{
			deltaDistance = ( params.distanceMat[V][W] + params.distanceMat[succV][succW] - params.distanceMat[V][succV] - params.distanceMat[W][succW] );
			break;
		}
	case Move::Type::INSKIND1:
		{
			deltaDistance = ( params.distanceMat[prevV][succV] + params.distanceMat[V][prevW] + params.distanceMat[V][W]
							  - params.distanceMat[prevV][V] - params.distanceMat[V][succV] - params.distanceMat[prevW][W] );
			break;
		}
	case Move::Type::INSKIND2:
		{
			deltaDistance = ( params.distanceMat[prevV][succV] + params.distanceMat[V][W] + params.distanceMat[V][succW]
							  - params.distanceMat[prevV][V] - params.distanceMat[V][succV] - params.distanceMat[W][succW] );
			break;
		}
	case Move::Type::INSKIND3:
		{
			deltaDistance = ( params.distanceMat[prevW][succW] + params.distanceMat[W][prevV] + params.distanceMat[W][V]
							  - params.distanceMat[prevW][W] - params.distanceMat[W][succW] - params.distanceMat[prevV][V] );
			break;
		}
	case Move::Type::INSKIND4:
		{
			deltaDistance = ( params.distanceMat[prevW][succW] + params.distanceMat[W][V] + params.distanceMat[W][succV]
							  - params.distanceMat[prevW][W] - params.distanceMat[W][succW] - params.distanceMat[V][succV] );
			break;
		}
	case Move::Type::SWAPKIND1:
		{
			int routeV = move.routeV , routeW = move.routeW;
			int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] );
			deltaDistance = ( params.distanceMat[prevV][prevW] + params.distanceMat[succV][prevW] + params.distanceMat[V][prevPrevW] + params.distanceMat[V][W]
							  - params.distanceMat[prevV][V] - params.distanceMat[V][succV] - params.distanceMat[prevPrevW][prevW] - params.distanceMat[prevW][W] );
			break;
		}
	case Move::Type::SWAPKIND2:
		{
			int routeV = move.routeV , routeW = move.routeW;
			int succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );
			deltaDistance = ( params.distanceMat[prevV][succW] + params.distanceMat[succV][succW] + params.distanceMat[V][W] + params.distanceMat[V][succSuccW]
							  - params.distanceMat[prevV][V] - params.distanceMat[V][succV] - params.distanceMat[W][succW] - params.distanceMat[succW][succSuccW] );
			break;
		}
	case Move::Type::OPT2STARTYPE1:
		{
			int routeW = move.routeW;
			succW = ( W != 0 ? indiv.nodes[W].links[1] : indiv.depots[routeW].links[1] );
			deltaDistance = ( params.distanceMat[V][W] + params.distanceMat[succV][succW] - params.distanceMat[V][succV] - params.distanceMat[W][succW] );
			break;
		}
	case Move::Type::OPT2STARTYPE2:
		{
			int routeW = move.routeW;
			succW = ( W != 0 ? indiv.nodes[W].links[1] : indiv.depots[routeW].links[1] );
			deltaDistance = ( params.distanceMat[V][succW] + params.distanceMat[W][succV] - params.distanceMat[V][succV] - params.distanceMat[W][succW] );
			break;
		}
	}
	return deltaDistance;
}

double hsh::cvrp::NagataBase::deltaCapacityExcess( Move & move , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */
	Node &nodeV = indiv.nodes[move.V] , &nodeW = indiv.nodes[move.W];
	int V = move.V , prevV = nodeV.links[0] , succV = nodeV.links[1] , routeV = nodeV.routeID;
	int W = move.W , prevW = nodeW.links[0] , succW = nodeW.links[1] , routeW = nodeW.routeID;

	switch ( move.type )
	{
	case Move::Type::OPT2KIND1:
		{
			if ( routeV != routeW )
			{
				/* v- or w- may be the depot, but their loadFromDepot are 0 which is the same as any other depotInfo, including nodeAdditionalInfo[0]. */
				double load1 = ( prevV != 0 ? nodeAdditionalInfo[prevV].loadFromDepot : 0. ) + ( prevW != 0 ? nodeAdditionalInfo[prevW].loadFromDepot : 0. );
				double load2 = nodeAdditionalInfo[V].loadToDepot + nodeAdditionalInfo[W].loadToDepot;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::OPT2KIND2:
		{
			/* v- may be the depot, but its loadFromDepot is 0 which is the same as any other depotInfo, including nodeAdditionalInfo[0]. */
			/* w+ may be the depot, its loadToDepot is different to any other depotInfo. */
			double load1 = ( prevV != 0 ? nodeAdditionalInfo[prevV].loadFromDepot : 0. ) + ( succW != 0 ? nodeAdditionalInfo[succW].loadToDepot : 0. );
			double load2 = nodeAdditionalInfo[W].loadFromDepot + nodeAdditionalInfo[V].loadToDepot;
			return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
				- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			break;
		}
	case Move::Type::OPT2KIND3:
		{
			/* w- maybe the depot, but its loadFromDepot is 0 which is the same as any other depotInfo. */
			/* v+ maybe the depot, its loadToDepot is different to any other depotInfo. */
			double load1 = nodeAdditionalInfo[V].loadFromDepot + nodeAdditionalInfo[W].loadToDepot;
			double load2 = ( prevW != 0 ? nodeAdditionalInfo[prevW].loadFromDepot : 0. ) + ( succV != 0 ? nodeAdditionalInfo[succV].loadToDepot : 0. );
			return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
				- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			break;
		}
	case Move::Type::OPT2KIND4:
		{
			if ( routeV != routeW )
			{
				/* v+ or w+ maybe the depot, their loadToDepot are different to any other depotInfo. */
				double load1 = nodeAdditionalInfo[V].loadFromDepot + nodeAdditionalInfo[W].loadFromDepot;
				double load2 = ( succV != 0 ? nodeAdditionalInfo[succV].loadToDepot : 0. ) + ( succW != 0 ? nodeAdditionalInfo[succW].loadToDepot : 0. );
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::INSKIND1:
		{
			if ( routeV != routeW )
			{
				double load1 = loads[routeV] - params.nodeInfo[V].demand;
				double load2 = loads[routeW] + params.nodeInfo[V].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::INSKIND2:
		{
			if ( routeV != routeW )
			{
				double load1 = loads[routeV] - params.nodeInfo[V].demand;
				double load2 = loads[routeW] + params.nodeInfo[V].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::INSKIND3:
		{
			if ( routeW != routeV )
			{
				double load1 = loads[routeW] - params.nodeInfo[W].demand;
				double load2 = loads[routeV] + params.nodeInfo[W].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeW] + capacityExcesses[routeV] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::INSKIND4:
		{
			if ( routeW != routeV )
			{
				double load1 = loads[routeW] - params.nodeInfo[W].demand;
				double load2 = loads[routeV] + params.nodeInfo[W].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeW] + capacityExcesses[routeV] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::SWAPKIND1:
		{
			if ( routeV != routeW )
			{
				double load1 = loads[routeV] - params.nodeInfo[V].demand + params.nodeInfo[prevW].demand;
				double load2 = loads[routeW] - params.nodeInfo[prevW].demand + params.nodeInfo[V].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	case Move::Type::SWAPKIND2:
		{
			if ( routeV != routeW )
			{
				double load1 = loads[routeV] - params.nodeInfo[V].demand + params.nodeInfo[succW].demand;
				double load2 = loads[routeW] - params.nodeInfo[succW].demand + params.nodeInfo[V].demand;
				return ( indiv.getExcessOf( params.capacityLimit , load1 ) + indiv.getExcessOf( params.capacityLimit , load2 ) )
					- ( capacityExcesses[routeV] + capacityExcesses[routeW] );
			}
			else { return 0.; }
			break;
		}
	}
	return Inf;
}

double hsh::cvrp::NagataBase::deltaDurationExcess( Move & move , double moveDeltaDistance , Individual & indiv )
{
	/* 以 ID 表示对应节点或路径 */
	Node &nodeV = indiv.nodes[move.V] , &nodeW = indiv.nodes[move.W];
	int V = move.V , prevV = nodeV.links[0] , succV = nodeV.links[1] , routeV = nodeV.routeID;
	int W = move.W , prevW = nodeW.links[0] , succW = nodeW.links[1] , routeW = nodeW.routeID;

	switch ( move.type )
	{
	case Move::Type::OPT2KIND1:
		{
			if ( routeV != routeW )
			{
				/* v- or w- may be the depot, but their durationFromDepot are 0 which is the same as any other depotInfo, including nodeAdditionalInfo[0]. */
				double duration1 = ( prevV != 0 ? nodeAdditionalInfo[prevV].durationFromDepot : 0. ) + ( prevW != 0 ? nodeAdditionalInfo[prevW].durationFromDepot : 0. )
					+ params.distanceMat[prevV][prevW];
				double duration2 = nodeAdditionalInfo[V].durationToDepot + nodeAdditionalInfo[W].durationToDepot
					+ params.distanceMat[V][W];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeV] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::OPT2KIND2:
		{
			/* v- may be the depot, but its durationFromDepot is 0 which is the same as any other depotInfo, including nodeAdditionalInfo[0]. */
			/* w+ may be the depot, its durationToDepot is different to any other depotInfo. */
			double duration1 = ( prevV != 0 ? nodeAdditionalInfo[prevV].durationFromDepot : 0. ) + ( succW != 0 ? nodeAdditionalInfo[succW].durationToDepot : 0. )
				+ params.distanceMat[prevV][succW];
			double duration2 = nodeAdditionalInfo[W].durationFromDepot + nodeAdditionalInfo[V].durationToDepot
				+ params.distanceMat[V][W];
			return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
				- ( durationExcesses[routeV] + durationExcesses[routeW] );
			break;
		}
	case Move::Type::OPT2KIND3:
		{
			/* w- maybe the depot, but its durationFromDepot is 0 which is the same as any other depotInfo. */
			/* v+ maybe the depot, its durationToDepot is different to any other depotInfo. */
			double duration1 = nodeAdditionalInfo[V].durationFromDepot + nodeAdditionalInfo[W].durationToDepot
				+ params.distanceMat[V][W];
			double duration2 = ( prevW != 0 ? nodeAdditionalInfo[prevW].durationFromDepot : 0. ) + ( succV != 0 ? nodeAdditionalInfo[succV].durationToDepot : 0. )
				+ params.distanceMat[prevW][succV];
			return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
				- ( durationExcesses[routeV] + durationExcesses[routeW] );
			break;
		}
	case Move::Type::OPT2KIND4:
		{
			if ( routeV != routeW )
			{
				/* v+ or w+ maybe the depot, their durationToDepot are different to any other depotInfo. */
				double duration1 = nodeAdditionalInfo[V].durationFromDepot + nodeAdditionalInfo[W].durationFromDepot
					+ params.distanceMat[V][W];
				double duration2 = ( succV != 0 ? nodeAdditionalInfo[succV].durationToDepot : 0. ) + ( succW != 0 ? nodeAdditionalInfo[succW].durationToDepot : 0. )
					+ params.distanceMat[succV][succW];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[V] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::INSKIND1:
		{
			if ( routeV != routeW )
			{
				double duration1 = nodeAdditionalInfo[prevV].durationFromDepot + params.distanceMat[prevV][succV] + nodeAdditionalInfo[succV].durationToDepot;
				double duration2 = durations[routeW] + params.nodeInfo[V].serviceDuration
					+ params.distanceMat[prevW][V] + params.distanceMat[V][W] - params.distanceMat[prevW][W];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeV] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::INSKIND2:
		{
			if ( routeV != routeW )
			{
				double duration1 = nodeAdditionalInfo[prevV].durationFromDepot + params.distanceMat[prevV][succV] + nodeAdditionalInfo[succV].durationToDepot;
				double duration2 = durations[routeW] + params.nodeInfo[V].serviceDuration
					+ params.distanceMat[W][V] + params.distanceMat[V][succW] - params.distanceMat[W][succW];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeV] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::INSKIND3:
		{
			if ( routeW != routeV )
			{
				double duration1 = nodeAdditionalInfo[prevW].durationFromDepot + params.distanceMat[prevW][succW] + nodeAdditionalInfo[succW].durationToDepot;
				double duration2 = durations[routeV] + params.nodeInfo[W].serviceDuration
					+ params.distanceMat[prevV][W] + params.distanceMat[W][V] - params.distanceMat[prevV][V];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeW] + durationExcesses[routeV] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeW] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::INSKIND4:
		{
			if ( routeW != routeV )
			{
				double duration1 = nodeAdditionalInfo[prevW].durationFromDepot + params.distanceMat[prevW][succW] + nodeAdditionalInfo[succW].durationToDepot;
				double duration2 = durations[routeV] + params.nodeInfo[W].serviceDuration
					+ params.distanceMat[V][W] + params.distanceMat[W][succV] - params.distanceMat[V][succV];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeW] + durationExcesses[routeV] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeW] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::SWAPKIND1:
		{
			if ( routeV != routeW )
			{
				// `prevW` 为 0 的情况属于非法连接, 评估之前已被排除
				int prevPrevW = indiv.nodes[prevW].links[0];
				double duration1 = durations[routeV] - params.nodeInfo[V].serviceDuration + params.nodeInfo[prevW].serviceDuration
					+ params.distanceMat[prevV][prevW] + params.distanceMat[succV][prevW] - params.distanceMat[prevV][V] - params.distanceMat[V][succV];
				double duration2 = durations[routeW] - params.nodeInfo[prevW].serviceDuration + params.nodeInfo[V].serviceDuration
					+ params.distanceMat[prevPrevW][V] + params.distanceMat[W][V] - params.distanceMat[prevPrevW][prevW] - params.distanceMat[prevW][W];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeV] + moveDeltaDistance ); }
			break;
		}
	case Move::Type::SWAPKIND2:
		{
			if ( routeV != routeW )
			{
				// `succW` 为 0 的情况属于非法连接, 评估之前已被排除
				int succSuccW = indiv.nodes[succW].links[1];
				double duration1 = durations[routeV] - params.nodeInfo[V].serviceDuration + params.nodeInfo[succW].serviceDuration
					+ params.distanceMat[prevV][succW] + params.distanceMat[succV][succW] - params.distanceMat[prevV][V] - params.distanceMat[V][succV];
				double duration2 = durations[routeW] - params.nodeInfo[succW].serviceDuration + params.nodeInfo[V].serviceDuration
					+ params.distanceMat[W][V] + params.distanceMat[succSuccW][V] - params.distanceMat[W][succW] - params.distanceMat[succW][succSuccW];
				return ( indiv.getExcessOf( params.durationLimit , duration1 ) + indiv.getExcessOf( params.durationLimit , duration2 ) )
					- ( durationExcesses[routeV] + durationExcesses[routeW] );
			}
			else { return indiv.getExcessOf( params.durationLimit , durations[routeV] + moveDeltaDistance ); }
			break;
		}
	}
	return Inf;
}

void hsh::cvrp::NagataBase::performMove( Move &move , Individual &indiv )
{
	//Individual backup( params );
	//backup.copyAllFrom( indiv );
	/* 更新个体的基本信息 */
	double dtDistance = deltaDistance( move , indiv );
	indiv.distance += dtDistance;
	indiv.capacityExcess += deltaCapacityExcess( move , indiv );
	indiv.durationExcess += ( params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0. );
	indiv.isFeasible = indiv.getFeasibleStatus();

	/* 更新个体的连接信息 */
	updateLinks( move , indiv );

	/* 调试信息是否正确 */
	//std::cout << "Move Type " << move.type << std::endl;
	/*if ( indiv.inspectLinks() == false ) { 
		std::cout << "ERROR ON LINKS" << std::endl;
		system( "pause" );
	};
	if ( abs( indiv.distance - indiv.getDistance() ) > params.error || indiv.distance < 0.)
	{
		std::cout << "ERROR ON DISTANCE " << indiv.distance << ", SUPPOSED " << indiv.getDistance() << std::endl;
		system( "pause" );
	}
	if ( abs( indiv.capacityExcess - indiv.getCapacityExcess() ) > params.error || indiv.capacityExcess < 0. )
	{
		std::cout << "ERROR ON CAPACITY EXCESS " << indiv.capacityExcess << ", SUPPOSED " << indiv.getCapacityExcess() << std::endl;
		system( "pause" );
	}
	if ( abs( indiv.durationExcess - indiv.getDurationExcess() ) > params.error || indiv.durationExcess < 0. )
	{
		std::cout << "ERROR ON DURATION EXCESS " << indiv.durationExcess << ", SUPPOSED " << indiv.getDurationExcess() << std::endl;
		system( "pause" );
	}*/
}

void hsh::cvrp::NagataBase::updateLinks( Move & move , Individual & indiv )
{
	/* 翻转部分路径或子回路的连接信息 */
	auto flip = [ ] ( Individual & indiv , int routeID , int startNodeID , int nextNodeID , int endNodeID )
	{
		int predNodeID = startNodeID , curNodeID = nextNodeID;
		while ( curNodeID != endNodeID )
		{
			int( &curNodeLinks )[2] = ( curNodeID != 0 ? indiv.nodes[curNodeID].links : indiv.depots[routeID].links );
			// 将 `startNodeID` 对应节点到 `endNodeID` 对应节点之间 ( 不包括这两个节点 ) 的节点, 翻转连接信息
			if ( curNodeLinks[0] != predNodeID ) { std::swap( curNodeLinks[0] , curNodeLinks[1] ); }
			predNodeID = curNodeID;
			curNodeID = curNodeLinks[1];
		}
	};

	/* 以 ID 代表对应节点或路径 */
	int V = move.V , prevV = indiv.nodes[V].links[0] , succV = indiv.nodes[V].links[1] , routeV = move.routeV;
	int W = move.W , prevW = indiv.nodes[W].links[0] , succW = indiv.nodes[W].links[1] , routeW = move.routeW;
	int( &linksV )[2] = indiv.nodes[V].links , ( &linksW )[2] = indiv.nodes[W].links;

	switch ( move.type )
	{
	case Move::Type::OPT2KIND1:
		{
			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links );

			int &V_prevV = linksV[0] , &W_prevW = linksW[0] ,
				&prevV_V = linksPrevV[1] , &prevW_W = linksPrevW[1];


			//cout << vPredLinks[0] << ", " << vPredLinks[1] << endl;
			//cout << wPredLinks[0] << ", " << wPredLinks[1] << endl;
			V_prevV = W; W_prevW = V; prevV_V = prevW; prevW_W = prevV;
			//cout << vPredLinks[0] << ", " << vPredLinks[1] << endl;
			//cout << wPredLinks[0] << ", " << wPredLinks[1] << endl;

			//cout << vPredLinks[0] << ", " << vPredLinks[1] << endl;
			//cout << wPredLinks[0] << ", " << wPredLinks[1] << endl;

			if ( routeV != routeW )
			{
				std::swap( indiv.depots[routeV].links[0] , indiv.depots[routeW].links[1] );
				flip( indiv , routeW , 0 , indiv.depots[routeW].links[1] , W );
				updateAdditionalInfo( indiv , routeW , false );

				if ( prevW == prevV )
				{
					/* the std::swap() above makes the depots[vRouteId].links = {0,0} while depots[wRouteId].links correct, and wRouteId must be updated. */
					/* step1: delete depot in this sln, step2: update nodeInfo. */
					indiv.delDepot( routeV );
					updateAdditionalInfo( indiv , routeV , true );

					if ( routeV < routeW ) { routeW--; }
					//cout << "after - " << vRouteId << " , wRoute " << wRouteId << endl;
				}
				else
				{
					flip( indiv , routeV , prevV , prevW , 0 );
					updateAdditionalInfo( indiv , routeV , false );
				}
			}
			else
			{
				// TODO[翻转最短的一侧]
				nodeAdditionalInfo[V].pos < nodeAdditionalInfo[W].pos ? flip( indiv , routeV , prevV , prevW , W ) : flip( indiv , routeV , prevW , prevV , V );
				updateAdditionalInfo( indiv , routeV , false );
			}

			break;
		}
	case Move::Type::OPT2KIND2:
		{
			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links );

			int &V_prevV = linksV[0] , &W_succW = linksW[1] ,
				&prevV_V = linksPrevV[1] , &succW_W = linksSuccW[0];

			V_prevV = W; W_succW = V; prevV_V = succW; succW_W = prevV;
			std::swap( indiv.depots[routeV].links[0] , indiv.depots[routeW].links[0] );

			/* only 2-route case. */
			updateAdditionalInfo( indiv , routeW , false );

			if ( prevV == succW )
			{
				/* the std::swap() above makes the depots[vRouteId].links = {0,0} while depots[wRouteId].links correct, and wRouteId must be updated.  */
				indiv.delDepot( routeV );
				updateAdditionalInfo( indiv , routeV , true );

				if ( routeV < routeW ) { routeW--; }
				//cout << "after - " << vRouteId << " , wRoute " << wRouteId << endl;
			}
			else
			{
				updateAdditionalInfo( indiv , routeV , false );
			}

			break;
		}
	case Move::Type::OPT2KIND3:
		{
			int( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links );

			int &V_succV = linksV[1] , &W_prevW = linksW[0] ,
				&succV_V = linksSuccV[0] , &prevW_W = linksPrevW[1];

			V_succV = W; W_prevW = V; succV_V = prevW; prevW_W = succV;
			std::swap( indiv.depots[routeV].links[0] , indiv.depots[routeW].links[0] );

			/* only 2-route case. */
			updateAdditionalInfo( indiv , routeV , false );

			if ( succV == prevW )
			{
				/* the std::swap() above makes the depots[wRouteId].links = {0,0} while depots[vRouteId].links correct, and vRouteId must be updated.  */
				indiv.delDepot( routeW );
				updateAdditionalInfo( indiv , routeW , true );

				if ( routeW < routeV ) { routeV--; }
				//cout << "after - " << wRouteId << " , vRoute " << vRouteId << endl;
			}
			else
			{
				updateAdditionalInfo( indiv , routeW , false );
			}

			break;
		}
	case Move::Type::OPT2KIND4:
		{
			int( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links );

			int &V_succV = linksV[1] , &W_succW = linksW[1] ,
				&succV_V = linksSuccV[0] , &succW_W = linksSuccW[0];

			V_succV = W; W_succW = V; succV_V = succW; succW_W = succV;

			if ( routeV != routeW )
			{
				std::swap( indiv.depots[routeV].links[0] , indiv.depots[routeW].links[1] );
				flip( indiv , routeV , V , W , 0 );
				updateAdditionalInfo( indiv , routeV , false );

				if ( succW == succV )
				{
					/* the std::swap() above makes the depots[wRouteId].links = {0,0} while depots[vRouteId].links correct, and vRouteId must be updated. */
					indiv.delDepot( routeW );
					updateAdditionalInfo( indiv , routeW , true );

					if ( routeW < routeV ) { routeV--; }
					//cout << "after - " << wRouteId << " , vRoute " << vRouteId << endl;
				}
				else
				{
					flip( indiv , routeW , 0 , indiv.depots[routeW].links[1] , succW );
					updateAdditionalInfo( indiv , routeW , false );
				}
			}
			else
			{
				//TODO[Optimize]: reverse the shortest part of this route.[1]
				nodeAdditionalInfo[V].pos < nodeAdditionalInfo[W].pos ? flip( indiv , routeV , V , W , succW ) : flip( indiv , routeV , W , V , succV );
				updateAdditionalInfo( indiv , routeV , false );
			}

			break;
		}
	case Move::Type::INSKIND1:
		{
			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links );

			int &V_prevV = linksV[0] , &V_succV = linksV[1] , &W_prevW = linksW[0] ,
				&prevV_V = linksPrevV[1] , &succV_V = linksSuccV[0] ,
				&prevW_W = linksPrevW[1];

			V_prevV = prevW; V_succV = W; W_prevW = V;
			prevV_V = succV; succV_V = prevV;
			prevW_W = V;

			if ( routeV != routeW )
			{
				updateAdditionalInfo( indiv , routeW , false );

				if ( prevV == succV )
				{
					indiv.delDepot( routeV );
					updateAdditionalInfo( indiv , routeV , true );

					if ( routeV < routeW ) { routeW--; }
					//cout << "after - " << vRouteId << " , wRoute " << wRouteId << endl;
				}
				else
				{
					updateAdditionalInfo( indiv , routeV , false );
				}
			}
			else
			{
				updateAdditionalInfo( indiv , routeV , false );
			}

			break;
		}
	case Move::Type::INSKIND2:
		{
			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links );

			int &V_prevV = linksV[0] , &V_succV = linksV[1] , &W_succW = linksW[1] ,
				&prevV_V = linksPrevV[1] , &succV_V = linksSuccV[0] ,
				&succW_W = linksSuccW[0];

			V_prevV = W; V_succV = succW; W_succW = V;
			prevV_V = succV; succV_V = prevV;
			succW_W = V;

			if ( routeV != routeW )
			{
				updateAdditionalInfo( indiv , routeW , false );

				if ( prevV == succV )
				{
					indiv.delDepot( routeV );
					updateAdditionalInfo( indiv , routeV , true );

					if ( routeV < routeW ) { routeW--; }
					//cout << "after - " << vRouteId << " , wRoute " << wRouteId << endl;
				}
				else
				{
					updateAdditionalInfo( indiv , routeV , false );
				}
			}
			else
			{
				updateAdditionalInfo( indiv , routeV , false );
			}

			break;
		}
	case Move::Type::INSKIND3:
		{
			int( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links ) ,
				( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links );

			int &W_prevW = linksW[0] , &W_succW = linksW[1] , &V_prevV = linksV[0] ,
				&prevW_W = linksPrevW[1] , &succW_W = linksSuccW[0] ,
				&prevV_V = linksPrevV[1];

			W_prevW = prevV; W_succW = V; V_prevV = W;
			prevW_W = succW; succW_W = prevW;
			prevV_V = W;

			if ( routeW != routeV )
			{
				updateAdditionalInfo( indiv , routeV , false );

				if ( prevW == succW )
				{
					indiv.delDepot( routeW );
					updateAdditionalInfo( indiv , routeW , true );

					if ( routeW < routeV ) { routeV--; }
					//cout << "after - " << wRouteId << " , vRoute " << wRouteId << endl;
				}
				else
				{
					updateAdditionalInfo( indiv , routeW , false );
				}
			}
			else
			{
				updateAdditionalInfo( indiv , routeW , false );
			}

			break;
		}
	case Move::Type::INSKIND4:
		{
			int( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links ) ,
				( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links );

			int &W_prevW = linksW[0] , &W_succW = linksW[1] , &V_succV = linksV[1] ,
				&prevW_W = linksPrevW[1] , &succW_W = linksSuccW[0] ,
				&succV_V = linksSuccV[0];

			W_prevW = V; W_succW = succV; V_succV = W;
			prevW_W = succW; succW_W = prevW;
			succV_V = W;

			if ( routeW != routeV )
			{
				updateAdditionalInfo( indiv , routeV , false );

				if ( prevW == succW )
				{
					indiv.delDepot( routeW );
					updateAdditionalInfo( indiv , routeW , true );

					if ( routeW < routeV ) { routeV--; }
					//cout << "after - " << wRouteId << " , vRoute " << vRouteId << endl;
				}
				else
				{
					updateAdditionalInfo( indiv , routeW , false );
				}
			}
			else
			{
				updateAdditionalInfo( indiv , routeW , false );
			}

			break;
		}
	case Move::Type::SWAPKIND1:
		{
			int prevPrevW = ( prevW != 0 ? indiv.nodes[prevW].links[0] : indiv.depots[routeW].links[0] );

			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksPrevW )[2] = ( prevW != 0 ? indiv.nodes[prevW].links : indiv.depots[routeW].links ) ,
				( &linksPrevPrevW )[2] = ( prevPrevW != 0 ? indiv.nodes[prevPrevW].links : indiv.depots[routeW].links );


			int &V_prevV = linksV[0] , &V_succV = linksV[1] , &W_prevW = linksW[0] ,
				&prevV_V = linksPrevV[1] , &succV_V = linksSuccV[0] ,
				&prevPrevW_prevW = linksPrevPrevW[1] , &prevW_prevPrevW = linksPrevW[0] , &prevW_W = linksPrevW[1];

			V_prevV = prevPrevW; V_succV = W; W_prevW = V;
			prevV_V = prevW; succV_V = prevW;
			prevPrevW_prevW = V; prevW_prevPrevW = prevV; prevW_W = succV;

			//cout << vPred << ", " << v << ", " << vSucc << endl;
			//cout << wDuoPred << ", " << wPred << ", " << w << endl;
			//int i = indiv.distance++;
			updateAdditionalInfo( indiv , routeV , false );

			if ( routeV != routeW )
			{
				updateAdditionalInfo( indiv , routeW , false );
			}

			break;
		}
	case Move::Type::SWAPKIND2:
		{
			int succSuccW = ( succW != 0 ? indiv.nodes[succW].links[1] : indiv.depots[routeW].links[1] );

			int( &linksPrevV )[2] = ( prevV != 0 ? indiv.nodes[prevV].links : indiv.depots[routeV].links ) ,
				( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links ) ,
				( &linksSuccSuccW )[2] = ( succSuccW != 0 ? indiv.nodes[succSuccW].links : indiv.depots[routeW].links );


			int &V_prevV = linksV[0] , &V_succV = linksV[1] , &W_succW = linksW[1] ,
				&prevV_V = linksPrevV[1] , &succV_V = linksSuccV[0] ,
				&succW_W = linksSuccW[0] , &succW_succSuccW = linksSuccW[1] , &succSuccW_succW = linksSuccSuccW[0];

			V_prevV = W; V_succV = succSuccW; W_succW = V;
			prevV_V = succW; succV_V = succW;
			succW_W = prevV; succW_succSuccW = succV; succSuccW_succW = V;

			updateAdditionalInfo( indiv , routeV , false );

			if ( routeV != routeW )
			{
				updateAdditionalInfo( indiv , routeW , false );
			}

			break;
		}
	case Move::Type::OPT2STARTYPE1:
		{
			/*
				合并子回路时 `routeV` 为子回路, `routeW` 为路径, `W` 可能为路径仓库;
				`V` 与 `succV` 也可能是子回路仓库, 但其连接信息也存储在 `nodes[V]` 与 `nodes[succV]`中
			*/
			succW = ( W != 0 ? indiv.nodes[W].links[1] : indiv.depots[routeW].links[1] );

			int( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links );

			int &V_succV = linksV[1] , &W_succW = ( W != 0 ? linksW[1] : indiv.depots[routeW].links[1] ) ,
				&succV_V = linksSuccV[0] , &succW_W = linksSuccW[0];

			V_succV = W; W_succW = V; succV_V = succW; succW_W = succV;

			flip( indiv , routeW , W , V , succW );
			// std::cout << "routeV " << routeV << ", routeW" << routeW << std::endl;
			indiv.delDepot( routeV ); // 合并子回路不涉及附加信息

			break;
		}
	case Move::Type::OPT2STARTYPE2:
		{
			/*
				合并子回路时 `routeV` 为子回路, `routeW` 为路径, `W` 可能为路径仓库;
				`V` 与 `succV` 也可能是子回路仓库, 但其连接信息也存储在 `nodes[V]` 与 `nodes[succV]`中
			*/
			succW = W != 0 ? indiv.nodes[W].links[1] : indiv.depots[routeW].links[1];

			int( &linksSuccV )[2] = ( succV != 0 ? indiv.nodes[succV].links : indiv.depots[routeV].links ) ,
				( &linksSuccW )[2] = ( succW != 0 ? indiv.nodes[succW].links : indiv.depots[routeW].links );

			int &V_succV = linksV[1] , &W_succW = ( W != 0 ? linksW[1] : indiv.depots[routeW].links[1] ) ,
				&succV_V = linksSuccV[0] , &succW_W = linksSuccW[0];

			V_succV = succW; W_succW = succV; succV_V = W; succW_W = V;
			// std::cout << "routeV " << routeV << ", routeW" << routeW << std::endl;
			indiv.delDepot( routeV ); // 合并子回路不涉及附加信息

			break;
		}
	}
}

void hsh::cvrp::NagataBase::prepareAdditionalInfo( Individual & indiv )
{
	/* 清空已有附加信息 */
	depotAdditionalInfo.clear(); depotAdditionalInfo.resize( indiv.numRoutes );
	nodeAdditionalInfo.clear(); nodeAdditionalInfo.resize( indiv.nodes.size() );
	distances.clear(); distances.resize( indiv.numRoutes );
	loads.clear(); loads.resize( indiv.numRoutes );
	capacityExcesses.clear(); capacityExcesses.resize( indiv.numRoutes );
	durations.clear(); durations.resize( indiv.numRoutes );
	durationExcesses.clear(); durationExcesses.resize( indiv.numRoutes );
	sizes.clear(); sizes.resize( indiv.numRoutes );

	for ( int routeID = 0; routeID < indiv.numRoutes; ++routeID )
	{
		auto& curDepot = indiv.depots[routeID];
		auto& curDepotAddi = depotAdditionalInfo[routeID];
		double accumulatedDistance = 0;
		double accumulatedload = 0;
		double accumulatedDuration = 0;
		int size = 0;

		/* 处理每个节点 ( 包括路径仓库 ) 附加信息 */
		for ( int pos = 0 , startNodeID = curDepot.ID , curNodeID = startNodeID , succNodeID = curDepot.links[1];
			  ;
			  ++pos , curNodeID = succNodeID , succNodeID = indiv.nodes[curNodeID].links[1] )
		{
			// 从路径仓库开始处理, 需要额外判断 ( 代码可拓展为支持子回路 )
			Node& curNode = ( curNodeID != startNodeID ? indiv.nodes[curNodeID] : curDepot );
			AdditionalInfo& curNodeAddi = ( curNodeID != startNodeID ? nodeAdditionalInfo[curNodeID] : curDepotAddi );

			accumulatedDistance += params.distanceMat[( curNodeID != startNodeID ? curNode.links[0] : startNodeID )][curNodeID];
			accumulatedload += params.nodeInfo[curNodeID].demand;
			accumulatedDuration += params.distanceMat[( curNodeID != startNodeID ? curNode.links[0] : startNodeID )][curNodeID] + params.nodeInfo[curNodeID].serviceDuration;
			size++;
			curNode.ID = curNodeID;
			curNode.routeID = routeID;
			curNodeAddi.pos = pos;
			curNodeAddi.distanceFromDepot = accumulatedDistance;
			curNodeAddi.loadFromDepot = accumulatedload;
			curNodeAddi.durationFromDepot = accumulatedDuration;
			curNodeAddi.prevProximity = params.lookupProximityMat[curNodeID][curNode.links[0]];
			curNodeAddi.succProximity = params.lookupProximityMat[curNodeID][curNode.links[1]];
			curNodeAddi.reversedPrevProximity = params.proximityMat[curNodeID][curNodeAddi.prevProximity].reversedProximity;
			curNodeAddi.reversedSuccProximity = params.proximityMat[curNodeID][curNodeAddi.succProximity].reversedProximity;

			if ( succNodeID == startNodeID )
			{
				// 路径方向上的最后一条边, 只增加距离和服务时长, 起点载重已计算过
				accumulatedDistance += params.distanceMat[curNodeID][startNodeID];
				accumulatedDuration += params.distanceMat[curNodeID][startNodeID];
				break;
			}
		}

		/* 到路径仓库的其余节点附加信息 */
		for ( int pos = 0 , startNodeID = curDepot.ID , curNodeID = startNodeID , succNodeID = curDepot.links[1];
			  pos < size;
			  ++pos , curNodeID = succNodeID , succNodeID = indiv.nodes[curNodeID].links[1] )
		{
			AdditionalInfo& curNodeAddi = ( curNodeID != startNodeID ? nodeAdditionalInfo[curNodeID] : curDepotAddi );

			// 对于路径仓库, `...toDepot` 都是整条路径的值
			curNodeAddi.distanceToDepot = accumulatedDistance - curNodeAddi.distanceFromDepot;
			curNodeAddi.loadToDepot = accumulatedload - curNodeAddi.loadFromDepot + params.nodeInfo[curNodeID].demand;
			curNodeAddi.durationToDepot = accumulatedDuration - curNodeAddi.durationFromDepot + params.nodeInfo[curNodeID].serviceDuration;
		}

		/* 处理路径附加信息 */
		distances[routeID] = curDepotAddi.distanceToDepot;
		loads[routeID] = curDepotAddi.loadToDepot;
		capacityExcesses[routeID] = indiv.getExcessOf( params.capacityLimit , loads[routeID] );
		durations[routeID] = curDepotAddi.durationToDepot;
		durationExcesses[routeID] = indiv.getExcessOf( params.durationLimit , durations[routeID] );
		sizes[routeID] = size;

		/* 如果支持子回路, 则需要将子回路仓库的附加信息复制到对应客户节点 */
	}
}

void hsh::cvrp::NagataBase::updateAdditionalInfo( Individual & indiv , int routeID , bool deleted )
{
	if ( deleted )
	{
		/* 执行邻域动作后路径被删除, 也删除对应的附加信息 */
		// 删除对应 ID 信息
		depotAdditionalInfo.erase( depotAdditionalInfo.begin() + routeID );
		distances.erase( distances.begin() + routeID );
		loads.erase( loads.begin() + routeID );
		capacityExcesses.erase( capacityExcesses.begin() + routeID );
		durations.erase( durations.begin() + routeID );
		durationExcesses.erase( durationExcesses.begin() + routeID );
		sizes.erase( sizes.begin() + routeID );

		// 更新后续路径附加信息
		for ( int ID = routeID; ID < indiv.numRoutes; ++ID )
		{
			auto &curDepot = indiv.depots[ID];
			--curDepot.routeID;
			// 后续路径上每个节点都要更新所在路径 ID
			for ( int startNodeID = curDepot.ID , curNodeID = startNodeID , succNodeID = curDepot.links[1];
				  ;
				  curNodeID = succNodeID , succNodeID = indiv.nodes[curNodeID].links[1] )
			{
				indiv.nodes[curNodeID].routeID = curDepot.routeID;
				if ( succNodeID == startNodeID ) { break; }
			}
		}
	}
	else
	{
		/* 路径未被删除, 更新对应附加信息 */
		// 重新计算
		auto& curDepot = indiv.depots[routeID];
		auto& curDepotAddi = depotAdditionalInfo[routeID];
		double accumulatedDistance = 0;
		double accumulatedload = 0;
		double accumulatedDuration = 0;
		int size = 0;

		// 处理每个节点 ( 包括路径仓库 ) 附加信息
		for ( int pos = 0 , startNodeID = curDepot.ID , curNodeID = startNodeID , succNodeID = curDepot.links[1];
			  ;
			  ++pos , curNodeID = succNodeID , succNodeID = indiv.nodes[curNodeID].links[1] )
		{
			// 从路径仓库开始处理, 需要额外判断 ( 代码可拓展为支持子回路 )
			Node& curNode = ( curNodeID != startNodeID ? indiv.nodes[curNodeID] : curDepot );
			AdditionalInfo& curNodeAddi = ( curNodeID != startNodeID ? nodeAdditionalInfo[curNodeID] : curDepotAddi );

			accumulatedDistance += params.distanceMat[( curNodeID != startNodeID ? curNode.links[0] : startNodeID )][curNodeID];
			accumulatedload += params.nodeInfo[curNodeID].demand;
			accumulatedDuration += params.distanceMat[( curNodeID != startNodeID ? curNode.links[0] : startNodeID )][curNodeID] + params.nodeInfo[curNodeID].serviceDuration;
			size++;
			curNode.ID = curNodeID;
			curNode.routeID = routeID;
			curNodeAddi.distanceFromDepot = accumulatedDistance;
			curNodeAddi.loadFromDepot = accumulatedload;
			curNodeAddi.durationFromDepot = accumulatedDuration;
			curNodeAddi.prevProximity = params.lookupProximityMat[curNodeID][curNode.links[0]];
			curNodeAddi.succProximity = params.lookupProximityMat[curNodeID][curNode.links[1]];
			curNodeAddi.reversedPrevProximity = params.proximityMat[curNodeID][curNodeAddi.prevProximity].reversedProximity;
			curNodeAddi.reversedSuccProximity = params.proximityMat[curNodeID][curNodeAddi.succProximity].reversedProximity;

			if ( succNodeID == startNodeID )
			{
				// 路径方向上的最后一条边, 只增加距离和服务时长, 起点载重已计算过
				accumulatedDistance += params.distanceMat[curNodeID][startNodeID];
				accumulatedDuration += params.distanceMat[curNodeID][startNodeID];
				break;
			}
		}

		// 到路径仓库的其余节点附加信息
		for ( int pos = 0 , startNodeID = curDepot.ID , curNodeID = startNodeID , succNodeID = curDepot.links[1];
			  pos < size;
			  ++pos , curNodeID = succNodeID , succNodeID = indiv.nodes[curNodeID].links[1] )
		{
			AdditionalInfo& curNodeAddi = ( curNodeID != startNodeID ? nodeAdditionalInfo[curNodeID] : curDepotAddi );

			// 对于路径仓库, `...toDepot` 都是整条路径的值
			curNodeAddi.distanceToDepot = accumulatedDistance - curNodeAddi.distanceFromDepot;
			curNodeAddi.loadToDepot = accumulatedload - curNodeAddi.loadFromDepot + params.nodeInfo[curNodeID].demand;
			curNodeAddi.durationToDepot = accumulatedDuration - curNodeAddi.durationFromDepot + params.nodeInfo[curNodeID].serviceDuration;
		}

		// 处理路径附加信息
		distances[routeID] = curDepotAddi.distanceToDepot;
		loads[routeID] = curDepotAddi.loadToDepot;
		capacityExcesses[routeID] = indiv.getExcessOf( params.capacityLimit , loads[routeID] );
		durations[routeID] = curDepotAddi.durationToDepot;
		durationExcesses[routeID] = indiv.getExcessOf( params.durationLimit , durations[routeID] );
		sizes[routeID] = size;

		// 如果支持子回路, 则需要将子回路仓库的附加信息复制到对应客户节点
	}
}

hsh::cvrp::NagataBase::NagataBase( Parameters & params ) :params( params )
{
	/* 为附加信息数据结构分配足够空间 */
	nodeAdditionalInfo = Vec<AdditionalInfo>( params.numNodes );
	depotAdditionalInfo = Vec<AdditionalInfo >( params.numNodes );
	distances = Vec<double>( params.numNodes );
	loads = Vec<double>( params.numNodes );
	capacityExcesses = Vec<double>( params.numNodes );
	durations = Vec<double>( params.numNodes );
	durationExcesses = Vec<double>( params.numNodes );
	sizes = Vec<int>( params.numNodes );
}

