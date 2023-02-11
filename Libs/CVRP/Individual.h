#pragma once
#include "CvrpTypeDef.h"
#include "CvrpParameters.h"

namespace hsh
{
	namespace cvrp
	{

		struct Node
		{
			int ID; // �ڵ� ID, �������ڵ���Ϣ `nodeInfo[ID]`
			int routeID;
			int links[2]; // `links[0]`��¼ǰ��, `links[1]`��¼���

			void reset() { ID = -1; routeID = -1; links[0] = links[1] = -1; }
		};

		class Individual
		{
		public:

			Parameters &params;

				/* ������Ϣ */
			Vec<Node> nodes; //TODO[�޸�Ϊclients?]
			Vec<Node> depots; // �ֿ� ID ����·�����ӻ�· ID; �洢��·���ֿ���ǰ, �ӻ�·�ֿ��ں�
			int numRoutes = 0;
			int numSubtours = 0;
			int numDepots = 0;

				/* ������Ϣ */
			double distance = Inf;
			double capacityExcess = 0.;
			double durationExcess = 0.;
			bool isFeasible = false;

				/* Path relinking ��Ϣ */
			int cycleID = -1; // ����ѡ��� AB-Cycle ID
			int numAccumulation = 0; // �� base ����ǰ�����ۻ��Ĳ��������
			int numPrivateEdges = 0; // base �� goal ֮���˽�б�����
			int baseIndivID = -1;
			int goalIndivID = -1;
			int tGenID = -1;

			/* ��ո�����Ϣ */
			void reset();

			/* ����һ��·���ֿ�(һ��·��) */
			Node & addRouteDepot();

			/* ����һ���ӻ�·�ֿ�(һ���ӻ�·), ��Ԥ��������·���ֿ�洢λ��֮�� */
			Node & addSubtDepot( int expectedNumRoutes );

			/* ��ɸ�����޸�, ���·���� */
			void done( int expectedNumRoutes );

			/* ɾ��һ���ֿ�(һ��·��)��һ���ӻ�·�ֿ�(һ���ӻ�·) */
			void delDepot( int deletedDepotID );

			/* ���Ƹ����������Ϣ */
			void copyAllFrom( Individual &src );

			/* ���Ƹ���Ŀͻ��ڵ�������Ϣ */
			void copyLinksFrom( const Individual &src );

			/* �������л�����Ϣ */
			void calculateCompletely();

			/* ·�����ӻ�·���� */
			double getDistanceOf( int depotID );

			/* �����ܾ���, �������ܵ��ӻ�· */
			double getDistance();
			
			/* ·�����ӻ�·���� */
			double getLoadOf(int depotID );

			/* ·�����ӻ�·����Υ���� */
			double getCapacityExcessOf( int depotID );

			/* ��������Υ����, �������ܵ��ӻ�· */
			double getCapacityExcess();

			/* ·�����ӻ�·ʱ��, �漰�ڵ�ķ���ʱ�������֮�� */
			double getDurationOf(int depotID );

			/* ·�����ӻ�·ʱ��Υ���� */
			double getDurationExcessOf( int depotID );

			/* �������ʱ��Υ����, �������ܵ��ӻ�· */
			double getDurationExcess();

			/* �ṩ��֪���ݼ���Υ����, ����� `params` ���� */
			double getExcessOf( double limit , double value);

			/* �������״̬ */
			bool getFeasibleStatus();

			/* ��� `nodes` �� `depots` �������Ƿ�Ϸ� */
			bool inspectLinks(bool showErrorMsg = false);

			/* ��ӡ������Ϣ */
			void print();

			/* ����� CVRPLib ��ʽ */
			void exportCVRPLibFormat( const String& pathExport );

			/* ������� */
			Individual( Parameters &params ) :params( params )
			{
				nodes = Vec<Node>( params.numNodes );
				depots = Vec<Node>( params.numNodes );
			}
		};
	}
}