#pragma once
#include <iostream>
#include <vector>
#include <numeric>
#include <ctime>
#include <limits>

namespace hsh {
    namespace mvirp {
        class Logger {
        public:
			enum class Type { Db, Info, Fd, Aj, Rf, Cvrp, Netflow, Cri, Bf, Nf, Size };
            bool stat[(size_t)Type::Size] = { false };
            using Manipulator = std::ostream &(*)(std::ostream &);

            Logger() : os(std::cout) {
                for (int i = 0; i < (int)Type::Size; ++i) { stat[i] = true; }
            }

            template<typename T>
            Logger &operator<<(const T &obj) {
                if (stat[(size_t)cur]) { os << obj; }
                return *this;
            }
            Logger &operator<<(Manipulator manip) {
                if (stat[(size_t)cur]) { os << manip; }
                return *this;
            }
            Logger &operator[](Type ty) {
                cur = ty;
                return *this;
            }
            Logger &operator()() {
                cur = Type::Info;
                return *this;
            }
        private:
            std::ostream &os;
            Type cur = Type::Info;
        };

        bool doubleLess(double a, double b, double err);

        struct Sampler {
            template <typename T> using Vec = std::vector<T>;
            template <typename T>
            static void pool(Vec<T> &data, int numSaved) {
                if (numSaved == 0) { data.clear(); return; }
                int sz = (int)data.size();
                if (numSaved >= sz) { return; }
                for (int i = numSaved; i < sz; ++i) {
					int j = rand() % (i + 1);
                    if (j < numSaved) { data[j] = data[i]; }
                }
                data.erase(data.begin() + numSaved, data.end());
            }
        };

        struct SamplerOne {
        private:
            int cnt = 0;
        public:
            bool isPicked() {
                ++cnt;
                if (cnt == 1) { return true; }
                else {
                    int j = rand() % cnt;
                    return j < 1;
                }
            }
            void rstCnt() { cnt = 0; }
        };

        struct Timer {
            Timer() {}
            Timer(int sec) :duration(sec) {};
            void setDuration(int sec) { duration = sec; }
            void startCounting() { startClk = clock(); }
            bool isTimeOut() { return (clock() - startClk) / CLOCKS_PER_SEC > duration; }
            double curTime() { return (clock() - startClk) / CLOCKS_PER_SEC; }
        private:
            int startClk = 0;
            int duration = std::numeric_limits<int>::max();
        };
    }
}