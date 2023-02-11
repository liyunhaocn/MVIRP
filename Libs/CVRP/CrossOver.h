/*
	Ŀǰ֧�ֵĽڵ�������Ϊ 1000, ����ֵ������Դ������Ŵ�, ��Ҫ�����ڱߵ� hash;
	GAB �пͻ��ڵ���� 4 ���ڽӽڵ�, �洢�ͱ�����������Ϣ�ĸ��Ӷ�Ϊ O( 4 �� numClients + numRoutes �� 2 )
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

				/* ��֧�ֽڵ��� */
			const int supportableNumNodes = 1000;

				/* �洢�����������������ݽṹ */
			enum Owner { UNKNOWN , BASE , GOAL , COMMON };
			bool isFirstTimeEax = true;
			struct RichEdge;
			int maxNumRichEdges; // �������洢�ı���
			int allocatedEnd = -1; // `richEdges` ��һ�η���� ID
			Vec<RichEdge> richEdges; // �洢˫�����б���Ϣ
			Vec<Vec<int>> adjNodeTable; // ��ǰ GAB ��ÿ���ڵ���ڽӽڵ��б�
			Vec<int> adjNodeSizes; // `adjNodeTable` ��ÿ���ڵ��ڽ��б�Ĵ�С
			Vec<Vec<int>> arrivalSteps; // ��������ÿ���ڵ�Ĳ���ջ, ��ֹ�����γ����� Cycle
			Vec<int> traversalPath; // ����ߵı���·��, ��¼ `RichEdge` ID
			UnorderedMap<int , int> edgesMap , privateEdgesMap , commonEdgesMap , baseEdgesMap , goalPrivateEdgesMap; // ӳ�� `code` �� `richEdges` �� ID
			Vec<Cycle> single; // һ�ηֽ� GAB �õ��� AB-Cycle ����
			Vec<Cycle> cycles; // ��ηֽ� GAB �õ����������� AB-Cycle ����
			Vec<bool> visited; // ���һ���ڵ��Ƿ��ѷ���

				/* GAB �ֽ�Ϊ AB-Cycle �����õ����ݽṹ */
			struct RichEdge
			{
				int ID; // �ڴ洢�ռ�� ID
				Pair<int> nodeIDs; // ��Ӧ�ı�
				int code; // �ߵĹ�ϣ��
				Owner owner; // �ߵ�����Ȩ����
				bool unique; // ����� <i, j> �ͱ� <j, i> ��ͬʱ����, ��Ψһ; ֻ�����ڷ��񵥸��ͻ���·����
				int baseRouteID , goalRouteID; // ��˫���е�·�� ID
				bool turn; // �滻Ĭ�ϰ���Ψһ (�� i< j) ����; �����Ψһ, ���״η��� <i, j>, ��һ�η��� <j, i>
				bool usable; // �Ƿ���Ա���
				bool visited; // �Ƿ��ѷ���
				int cycleID; // �ñ߱����ֵ��� AB-Cycle �� ID

				RichEdge() :
					ID( -1 ) , nodeIDs( { -1,-1 } ) , code( -1 ) , owner( Owner::UNKNOWN ) ,
					unique( true ) , baseRouteID( -1 ) , goalRouteID( -1 ) ,
					turn( false ) , usable( false ) , visited( false ) , cycleID( -1 )
				{ }

				/* �������� */
				void set( int ID , int i , int j , int code , Owner owner , bool unique , int routeID );
			};

			/* ����ӳ�� */
			void resetMap( UnorderedMap<int , int>& map , int size );

			/* ���ü��� */
			void resetSet( UnorderedSet<int>& set , int size );

			/* �������нڵ��������Ĳ���ջ */
			void resetArrivalSteps();

			/* �ӱߵ���ϣ�� */
			int toCode( int i , int j , bool unique );

			/* �ӹ�ϣ�뵽�� */
			Pair<int> toEdge( int code );

			/* ���ñߵķ���״̬ */
			void setVisited( UnorderedMap<int , int>& mp , bool stat );

			/* ���ñߵĿ���״̬ */
			void setUsable( UnorderedMap<int , int>& mp , bool stat );

			/* �����µı� */
			void addRichEdge( int i , int j , Owner owner , bool unique , int routeID );

			/* ׼�����������������ݽṹ, ��ʹ��һ�ν���Ľ����Ч */
			void prepareDataStructures();

			/* ˫�ױ߼����� */
			void classifyEdges( Individual &base , Individual &goal );

			/* �ֽ� GAB, ��� AB-Cycle */
			void generateCycles( int repeatTimes = 5 );

			/* �����Ƹ���Ŀͻ��ڵ�������Ϣ */

			/* �Ը���Ӧ�ø��� AB-Cycle; Ŀ��·����Ϊ `params.preprocess.numRoutes` */
			void applyCycle( Cycle &cycle , Individual &indiv );

			/* �Ը���Ӧ�ø��� AB-Cycle ����; Ŀ��·����Ϊ `params.preprocess.numRoutes` */
			void applyCycles( Vec<Cycle> &cycles , Individual &indiv );

			Eax( Parameters &params ) :params( params )
			{ }
		};
	}
}