#pragma once
#include "Population.h"

namespace hsh {
	namespace cvrp {

		class PathRelinking {
		public:

			Parameters &params;
			Eax &eax;

				/* 路径重连信息 */
			int sizeRawIndivs = 0; // 已确定的原始个体数量
			Vec<Individual> rawIndivs; // 已确定的原始个体
			int sizePickedRawIndivs = 0; // 最终挑出的原始个体数量
			Vec<Individual> pickedRawIndivs; // 最终挑出的原始个体
			int sizeTempRawIndivs = 0; // 试探所产生的临时原始个体数量
			Vec<Individual> tempRawIndivs; // 试探所产生的原始个体
			Vec<int> orderPop; // 种群个体按距离从小到大排序
			Vec<int> orderPickedRawIndivs; // 最终挑出的原始个体次序

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