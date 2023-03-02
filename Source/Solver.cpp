#include "Solver.h"
#include "json.h"

using Json = nlohmann::json;

hsh::mvirp::Solver::Solver()
{
}

hsh::mvirp::Solver::~Solver()
{
}

int hsh::mvirp::Solver::run(int argc, char *argv[])
{

    Log << std::fixed << std::setprecision(3);
    Log.stat[(size_t)Logger::Type::Db] = 0;
    Log.stat[(size_t)Logger::Type::Fd] = 0;
    Log.stat[(size_t)Logger::Type::Aj] = 0;
    Log.stat[(size_t)Logger::Type::Rf] = 0;
    Log.stat[(size_t)Logger::Type::Bf] = 0;
    Log.stat[(size_t)Logger::Type::Nf] = 0;

    params.loadCommandParameters(argc, argv);

    std::ifstream ifs(params.parametersPath);
    if (ifs.fail()) {
        std::cout << "can't open parameters file,use the default parameters" << std::endl;
    }
    else {
        Json allJson;
        ifs >> allJson;
        Json apsJson = allJson["AlgorithmParameters"];
        // initial commandLine with Josn
        params.from_json(allJson, params);
    }

    params.setTimeSeed(params.randomSeed);
    params.loadInstance(params.instancePath);

    timer.setDuration(params.totTimeLimit);
    timer.startCounting();

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
    Str outputName =  params.instancePath;
    outputName.erase(remove_if(outputName.begin(), outputName.end(), [=](char c) {return c == '.'; }), outputName.end());
    outputName.erase(remove_if(outputName.begin(), outputName.end(), [=](char c) {return c == '/'; }), outputName.end());

	Str path = params.outputDir + "/out_" + outputName + ".txt";
	bestRefining.outputDimacsFmt(path, true);
    Log[Logger::Type::Cri] << "Solution saved to path: " << path << std::endl;
}