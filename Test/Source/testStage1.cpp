#include "pch.h"

TEST(TestSolver, Stage1) {

    using namespace hsh::mvirp;

    Log << std::fixed << std::setprecision(3);
    Log.stat[(size_t)Logger::Type::Db] = 0;
    Log.stat[(size_t)Logger::Type::Fd] = 0;
    Log.stat[(size_t)Logger::Type::Aj] = 0;
    Log.stat[(size_t)Logger::Type::Rf] = 0;
    Log.stat[(size_t)Logger::Type::Bf] = 0;
    Log.stat[(size_t)Logger::Type::Nf] = 0;

    int tSeed = 1637917497;
    Solver solver;
    solver.params.setTimeSeed(tSeed); // 1637917497
    solver.params.totTimeLimit = 180;
    solver.params.finding.timeOnce = 30;
    solver.params.adjusting.timeOnce = 30;

    solver.params.setTimeSeed(tSeed); // 1637917497
    solver.params.totTimeLimit = 180;
    solver.params.finding.timeOnce = 30;
    solver.params.adjusting.timeOnce = 30;
    //solver.params.loadInstance("../Instances/LargeH6/L_abs5n200_3_H.dat");
    //solver.params.loadInstance("../../../Instances/LargeL6/L_abs6n200_4_L.dat");
    solver.params.loadInstance("../Instances/SmallL3/S_abs5n15_2_L3.dat");
    //solver.params.loadInstance("../Instances/SmallH3/S_abs1n10_5_H3.dat");
    //solver.params.loadInstance("../Instances/SmallH6/S_abs3n20_5_H6.dat");
    //solver.params.loadInstance("../Instances/SmallL6/S_abs5n5_5_L6.dat");
    //solver.params.loadInstance("../Instances/SmallH6/S_abs1n25_5_H6.dat"); // 该算例可引发 bug, 不要修改.

    solver.timer.setDuration(solver.params.totTimeLimit);
    solver.timer.startCounting();

    solver.bestFinding.init(solver.params);
    solver.structureFinding(solver.bestFinding);

    EXPECT_TRUE(solver.bestFinding.cost != COST_MAX);
    //EXPECT_EQ();

    solver.bestFinding.calcInvCost(true);
    solver.bestFinding.calcTraCost(false, true);


}