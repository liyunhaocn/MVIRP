#include "TypeDef.h"
#include <algorithm>

namespace hsh {
    namespace mvirp {
        Distance DISTANCE_MAX = 10e7;
        InvenCost INVEN_COST_MAX = std::numeric_limits<int>::max();
        Quantity EMPTY = 0.;
        InvenLevel INVEN_LEVEL_MAX= std::numeric_limits<int>::max();
        Cost COST_MAX = std::numeric_limits<int>::max();
        Logger Log;
    }
}