#include "pch.h"

TEST(TestSolver, TestName) {

    using namespace hsh::mvirp;

    int tSeed = 1637917497;
    Solver solver;
    solver.params.setTimeSeed(tSeed); // 1637917497
    solver.params.totTimeLimit = 1800;
    solver.params.finding.timeOnce = 120;
    solver.params.adjusting.timeOnce = 60;

    solver.params.setTimeSeed(tSeed); // 1637917497
    solver.params.totTimeLimit = 1800;
    solver.params.finding.timeOnce = 120;
    solver.params.adjusting.timeOnce = 60;
    //solver.params.loadInstance("../Instances/LargeH6/L_abs5n200_3_H.dat");
    solver.params.loadInstance("../../Instances/LargeL6/L_abs6n200_4_L.dat");
    //solver.params.loadInstance("../Instances/SmallL3/S_abs5n15_2_L3.dat");
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

    solver.bestAdjusting.init(solver.params);
    solver.bestAdjusting = solver.bestFinding;
    if (true)solver.structureAdjusting(solver.bestAdjusting);
    solver.bestAdjusting.calcInvCost(true);
    solver.bestAdjusting.calcTraCost(false, true);

    solver.bestRefining.init(solver.params);
    solver.bestRefining = solver.bestAdjusting;
    solver.structureRefining(solver.bestRefining);
    solver.bestRefining.calcInvCost(true);
    solver.bestRefining.calcTraCost(false, true);

}