// for gurobi auto-linking.

#include "Flag.h"


#include "Gurobi.h"

#include "Preprocessor.h"

#define GUROBI_VERSION  RESOLVED_STRINGIFY(RESOLVED_CONCAT(GRB_VERSION_MAJOR,GRB_VERSION_MINOR))

#if _DR_DEBUG
#if _LL_DYNAMIC
#define LINK_TYPE  "dd"
#else // _LL_STATIC
#define LINK_TYPE  "td"
#endif // _LL_DYNAMIC
#else // _DR_RELEASE
#if _LL_DYNAMIC
#define LINK_TYPE  "d"
#else // _LL_STATIC
#define LINK_TYPE  "t"
#endif // _LL_DYNAMIC
#endif // _DR_DEBUG

//#pragma message("[auto-linking] " "gurobi" GUROBI_VERSION)
//#pragma message("[auto-linking] " "gurobi_c++m" LINK_TYPE RESOLVED_STRINGIFY(_CC_VERSION))
#pragma comment(lib, "gurobi" GUROBI_VERSION)
#pragma comment(lib, "gurobi_c++m" LINK_TYPE RESOLVED_STRINGIFY(_CC_VERSION))


namespace goal {

GRBEnv MpSolverGurobi::globalEnv(true);

}
