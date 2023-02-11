#include "NagataLocalSearch.h"

bool hsh::cvrp::NagataLocalSearch::evaluateMove( Move & move , Individual & indiv )
{
	bool isImproving = false;
	switch ( searchMode )
	{
	case SearchMode::FEASIBLE:
		{
			int dtNumRoutes = deltaNumRoutes( move , indiv );
			if ( dtNumRoutes != 0 ) { break; } // �����Ǹı�·�������Ķ���

			double dtDistance = deltaDistance( move , indiv );
			if ( dtDistance > -params.error ) { break; } // �����ǷǸĽ�·�����ȵĶ���

			double dtCapacityExcess = deltaCapacityExcess( move , indiv );
			if ( dtCapacityExcess > params.error ) { break; } // ��������������Υ�����Ķ���

			double dtDuratonExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0.; // ���������Ϣû�з���ʱ������, ��������
			if ( dtDuratonExcess > params.error ) { break; } // ���������ӷ���ʱ��Υ�����Ķ���

			firstImproving = move;
			isImproving = true; // �ҵ��Ľ�����, ��ֹ������
			break;
		}
	case SearchMode::PENAL:
		{
			double dtDistance = deltaDistance( move , indiv );
			double dtCapacityExcess = deltaCapacityExcess( move , indiv );
			int dtNumRoutes = deltaNumRoutes(move, indiv);
			if (dtNumRoutes + indiv.numRoutes == 1 && dtCapacityExcess + indiv.capacityExcess > params.error) { break; } // ֻʣһ�����޷��޸�����Υ��
			double dtDurationExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0.;

			double dtPenaltyFuncValue = dtDistance
				+ params.critical.penaltyCapacityExcess*dtCapacityExcess
				+ params.critical.penaltyDurationExcess*dtDurationExcess;
			if ( dtPenaltyFuncValue > -params.error ) { break; } //�����ǷǸĽ��ͷ�Ŀ�꺯��ֵ�Ķ���

			firstImproving = move;
			isImproving = true; // �ҵ��Ľ�����, ��ֹ������
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

		/* ����ڵ�����, Ѱ�ҸĽ����� */
		std::random_shuffle( clientOrder.begin() , clientOrder.end() );
		for ( auto V : clientOrder )
		{
			if ( V == 0 ) { throw String( "Can not construct neighborhood of a depot." ); }

			random_shuffle( typeOrder.begin() , typeOrder.end() ); // �������������������

			random_shuffle( orderProximity.begin() , orderProximity.end() );

			constructNeighborhoodOf( V , orderProximity , typeOrder , indiv );

			if ( firstImproving.type != Move::Type::EMPTY ) { break; }
		}

		/* ѭ���˳�����ж� */
		if ( firstImproving.type != Move::Type::EMPTY )
		{
			performMove( firstImproving , indiv ); // ���¸���, ���¸�����Ϣ
			/*cout << "|NUMBER OF ROUTES: " << indiv.numRoutes << "|";
			cout << "|DISTANCE: " << indiv.distance << "|";
			cout << "|CAPACITY EXCESS: " << indiv.capacityExcess << "|";
			cout << "|DURATION EXCESS: " << indiv.durationExcess << "|" << endl;*/

			// �������·�������ɷ���
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
	/* Ϊ�ֲ�����������Ϣ����ռ估��ʼ�� */
	clientOrder = Vec<int>( params.numClients );
	for ( int index = 0; index < params.numClients; ++index ) { clientOrder.at( index ) = index + 1; } // �ͻ��ڵ� ID ������ 1 ��ʼ
	typeOrder = Vec<Move::Type>
	{
		Move::INSKIND1,Move::INSKIND2,
		Move::SWAPKIND1,Move::SWAPKIND2,
		Move::INSKIND3,Move::INSKIND4,
		Move::OPT2KIND1,Move::OPT2KIND2,Move::OPT2KIND3,Move::OPT2KIND4
	};
	orderProximity = Vec<int>( params.numClients );
}
