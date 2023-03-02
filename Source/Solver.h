#pragma once
#include "Parameters.h"
#include "Solution.h"
#include "CvrpCaller.h"
#include "GrbCaller.h"
#include "NetflowCaller.h"

namespace hsh {
    namespace mvirp {
        
        class Solver {
        public:
            Parameters params;

            struct Move {
				enum Type { Del, Add, Shift, Swap, ChgVeh, Empty };
                Type type = Type::Empty;
                
                ID periodIdA = -1, periodIdB = -1;
                RouteNode origNdA, origNdB;
                RouteNode curNdA, curNdB;
                /*ID prevNdId1 = -1, prevNdId2 = -1;
                ID succNdId1 = -1, succNdId2 = -1;
                ID prevRouId1 = -1, prevRouId2 = -1, prevRouId3 = -1, prevRouId4 = -1;*/

                Distance dtDistDelA = DISTANCE_MAX, dtDistAddA = DISTANCE_MAX,
                    dtDistDelB = DISTANCE_MAX, dtDistAddB = DISTANCE_MAX, dtDistDelZero = DISTANCE_MAX;

				InvenCost invCost = INVEN_COST_MAX;
				TravelCost dtTraCost = DISTANCE_MAX;
                Cost totCost = COST_MAX;
                bool taboo = true;
                bool feasible = false;

                Move() {}
                Move(Type ty) :type(ty) {}
            };

            Vec<Vec<Quantity>> origDeliveries;
            Solution bestFinding, bestAdjusting, bestRefining;
            struct Hasher {
                const UL vecLen = 10e8;
                Vec<Vec<bool>> hashVec; // bool数组, 通过位运算减少内存使用
                Vec<double> gamma;
                Vec<UL> hashValBase; // 当前哈希值
                Vec<UL> hashValNbr; // 邻域解的哈希值
                Parameters *paramsPtr = nullptr;
                
                Hasher();
                Hasher& setBase(Solution &sln); // 哈希并禁忌该解
                UL bitWeight(int idx, int no);
                void setNbr(Move &move);
                bool isNbrTaboo(Move &move);
                void stNbrTaboo(Move &move);
                bool isBaseTaboo();
                void stBaseTaboo();
                Hasher& updBase(Move &move);
            }hasher;
            Timer timer;

        public:
            Solver();
            ~Solver();
            int run(int argc, char *argv[]);

            // schedule generation
            void structureFinding(Solution &sln);
            // schedule optimization
            void structureAdjusting(Solution &sln);
            // routing refinement
            void structureRefining(Solution &sln);
            
            void makeMoves(Solution &sln, Vec<Move> &moves);
            void evalMoves(Solution &sln, Vec<Move> &moves);
            void takeMove(Solution &sln, Move &move);
            void perturb(Solution& cur, Solution& base);


            bool adjustPartial(Solution& sln, int perStart, int perLast);

            void chgVisLk(Solution &sln, Move &move, bool cleanSelfLoop);
            void rstVisLk(Solution &sln, Move &move);
            void periodPtb(Solution &cur, Solution &base);

            // 输出
            void saveBest();
        };
    }
}