#include "NagataLocalSearch.h"

bool hsh::cvrp::NagataLocalSearch::evaluateMove( Move & move , Individual & indiv )
{
	bool isImproving = false;
	switch ( searchMode )
	{
	case SearchMode::FEASIBLE:
		{
			int dtNumRoutes = deltaNumRoutes( move , indiv );
			if ( dtNumRoutes != 0 ) { break; } // 不考虑改变路径数量的动作

			double dtDistance = deltaDistance( move , indiv );
			if ( dtDistance > -params.error ) { break; } // 不考虑非改进路径长度的动作

			double dtCapacityExcess = deltaCapacityExcess( move , indiv );
			if ( dtCapacityExcess > params.error ) { break; } // 不考虑增加载重违反量的动作

			double dtDuratonExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0.; // 如果算例信息没有服务时长限制, 则不作计算
			if ( dtDuratonExcess > params.error ) { break; } // 不考虑增加服务时长违反量的动作

			firstImproving = move;
			isImproving = true; // 找到改进动作, 终止邻域构造
			break;
		}
	case SearchMode::PENAL:
		{
			double dtDistance = deltaDistance( move , indiv );
			double dtCapacityExcess = deltaCapacityExcess( move , indiv );
			int dtNumRoutes = deltaNumRoutes(move, indiv);
			if (dtNumRoutes + indiv.numRoutes == 1 && dtCapacityExcess + indiv.capacityExcess > params.error) { break; } // 只剩一辆车无法修复容量违反
			double dtDurationExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0.;

			double dtPenaltyFuncValue = dtDistance
				+ params.critical.penaltyCapacityExcess*dtCapacityExcess
				+ params.critical.penaltyDurationExcess*dtDurationExcess;
			if ( dtPenaltyFuncValue > -params.error ) { break; } //不考虑非改进惩罚目标函数值的动作

			firstImproving = move;
			isImproving = true; // 找到改进动作, 终止邻域构造
			break;
		}
	}
	return isImproving;
}

void hsh::cvrp::NagataLocalSearch::run( Individual & indiv )
{
	//double timeStart = clock();
	using namespace std;

	orderProximity.resize(max(0, min(params.nagataLS.proximity, params.numClients - 1)));
	for ( int index = 0; index < (int)orderProximity.size(); ++index ) { orderProximity[index] = index; }

	prepareAdditionalInfo( indiv );
	while ( 1 )
	{
		firstImproving.type = Move::Type::EMPTY;

		/* 构造节点邻域, 寻找改进动作 */
		std::random_shuffle( clientOrder.begin() , clientOrder.end() );
		for ( auto V : clientOrder )
		{
			if ( V == 0 ) { throw String( "Can not construct neighborhood of a depot." ); }

			random_shuffle( typeOrder.begin() , typeOrder.end() ); // 将邻域动作类型随机排序

			random_shuffle( orderProximity.begin() , orderProximity.end() );

			constructNeighborhoodOf( V , orderProximity , typeOrder , indiv );

			if ( firstImproving.type != Move::Type::EMPTY ) { break; }
		}

		/* 循环退出情况判断 */
		if ( firstImproving.type != Move::Type::EMPTY )
		{
			performMove( firstImproving , indiv ); // 更新个体, 更新附加信息
			/*cout << "|NUMBER OF ROUTES: " << indiv.numRoutes << "|";
			cout << "|DISTANCE: " << indiv.distance << "|";
			cout << "|CAPACITY EXCESS: " << indiv.capacityExcess << "|";
			cout << "|DURATION EXCESS: " << indiv.durationExcess << "|" << endl;*/

			// 满足给定路径数即可返回
			if (params.critical.numRoutes != -1 && indiv.numRoutes == params.critical.numRoutes) { break; }
		}
		else { break; }
	}
	//std::cout << "NAGATA LS TAKES " << ( clock() - timeStart ) / CLOCKS_PER_SEC << std::endl;
}

hsh::cvrp::NagataLocalSearch & hsh::cvrp::NagataLocalSearch::setSearchMode( SearchMode searchMode )
{
	this->searchMode = searchMode;
	return *this;
}

hsh::cvrp::NagataLocalSearch::NagataLocalSearch( Parameters & params ) :NagataBase( params )
{
	/* 为局部搜索搜索信息分配空间及初始化 */
	clientOrder = Vec<int>( params.numClients );
	for ( int index = 0; index < params.numClients; ++index ) { clientOrder.at( index ) = index + 1; } // 客户节点 ID 从数字 1 开始
	typeOrder = Vec<Move::Type>
	{
		Move::INSKIND1,Move::INSKIND2,
		Move::SWAPKIND1,Move::SWAPKIND2,
		Move::INSKIND3,Move::INSKIND4,
		Move::OPT2KIND1,Move::OPT2KIND2,Move::OPT2KIND3,Move::OPT2KIND4
	};
	orderProximity = Vec<int>( params.numClients );
}
