#include "Solver.h"

hsh::mvirp::Solver::Solver()
{
}

hsh::mvirp::Solver::~Solver()
{
}

int hsh::mvirp::Solver::run(int argc, char *argv[])
{
    // 日志控制
    Log << std::fixed << std::setprecision(3);
    Log.stat[(size_t)Logger::Type::Db] = 0;
    Log.stat[(size_t)Logger::Type::Fd] = 0;
    Log.stat[(size_t)Logger::Type::Aj] = 0;
    Log.stat[(size_t)Logger::Type::Rf] = 0;
    Log.stat[(size_t)Logger::Type::Bf] = 0;
    Log.stat[(size_t)Logger::Type::Nf] = 0;

    // 参数解析, 接受参数:
    // [1]totTimeLimit, [2]timeSeed
    // [3]instDir, [4]instName, [5]instExtName,
    // [6]slnDir, [7]slnName, [8]slnExtName
    if (argc != 9) {
        Log[Logger::Type::Info] << "Size of arguments error." << std::endl;
        return -1;
    }
    int argi = 1;
    params.totTimeLimit = atoi(argv[argi++]);
    params.timeSeed = atoi(argv[argi++]);
    params.instDir = argv[argi++];
    params.instName = argv[argi++];
    params.instExtName = argv[argi++];
    params.slnDir = argv[argi++];
    params.slnName = argv[argi++];
    params.slnExtName = argv[argi++];

    Str path = params.slnDir + "out_" + params.slnName + params.slnExtName;
    Log[Logger::Type::Info] << "Solution will be saved to path: " << path << std::endl;

    params.setTimeSeed(params.timeSeed);
    params.loadInstance(params.instDir + params.instName + params.instExtName);

    timer.setDuration(params.totTimeLimit);
    timer.startCounting();

    params.finding.timeOnce = 60;
    params.adjusting.timeOnce = 60;

    bestFinding.init(params);
    structureFinding(bestFinding);
    if (bestFinding.cost == COST_MAX) { return 0; } // 算例无效, 模型无效
    bestAdjusting.init(params);
    bestAdjusting = bestFinding;
    structureAdjusting(bestAdjusting);
    bestRefining.init(params);
    bestRefining = bestAdjusting;
    structureRefining(bestRefining);

    Log[Logger::Type::Bf]
        << bestFinding.cost << ", "
        << bestAdjusting.cost << ","
        << bestRefining.cost << std::endl;
    saveBest();
    return 0;
}

void hsh::mvirp::Solver::saveBest()
{
	Str path = params.slnDir + "out_" + params.slnName + params.slnExtName;
	bestRefining.outputDimacsFmt(path, true);
    Log[Logger::Type::Cri] << "Solution saved to path: " << path << std::endl;
}