#pragma once
#include "Population.h"

namespace hsh {
	namespace cvrp {

		class PathRelinking {
		public:

			Parameters &params;
			Eax &eax;

				/* ·��������Ϣ */
			int sizeRawIndivs = 0; // ��ȷ����ԭʼ��������
			Vec<Individual> rawIndivs; // ��ȷ����ԭʼ����
			int sizePickedRawIndivs = 0; // ����������ԭʼ��������
			Vec<Individual> pickedRawIndivs; // ����������ԭʼ����
			int sizeTempRawIndivs = 0; // ��̽����������ʱԭʼ��������
			Vec<Individual> tempRawIndivs; // ��̽��������ԭʼ����
			Vec<int> orderPop; // ��Ⱥ���尴�����С��������
			Vec<int> orderPickedRawIndivs; // ����������ԭʼ�������

			void resetRawIndivs();

			void resetPickedRawIndivs();

			void resetTempRawIndivs();

			Individual & addRawIndiv();

			Individual & addTempIndiv();

			Individual & addPickedRawIndiv();

			void core( Individual &base , int baseIndivID , Individual &goal , int goalIndivID );

			const Vec<int> & run( Population &pop );

			PathRelinking( Parameters &params, Eax &eax );
		};
	}
}