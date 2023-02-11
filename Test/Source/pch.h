//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

#include "../../Source/TypeDef.h"
#include "../../Source/Untils.h"
#include "../../Source/CmdLineParser.h"
#include "../../Source/CvrpCaller.h"
#include "../../Source/GrbCaller.h"
#include "../../Source/NetflowCaller.h"
#include "../../Source/Parameters.h"
#include "../../Source/Solver.h"
#include "../../Source/Solution.h"

#include "../Libs/CVRP/CvrpParameters.h"
#include "../Libs/CVRP/NagataLocalSearch.h"
#include "../Libs/CVRP/Repair.h"
#include "../Libs/CVRP/Population.h"
#include "../Libs/CVRP/Framework.h"
#include "../Libs/CVRP/CrossOver.h"
#include "../Libs/CVRP/CvrpTypeDef.h"
#include "../Libs/CVRP/Individual.h"
#include "../Libs/CVRP/NagataBase.h"