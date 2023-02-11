#include "Untils.h"

namespace hsh {
	namespace mvirp {
		bool doubleLess(double a, double b, double err) {
			return a < b - err;
		}
	}
}