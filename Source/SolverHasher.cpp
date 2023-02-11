#include "Solver.h"

hsh::mvirp::Solver::Hasher::Hasher()
{
	hashVec = Vec<Vec<bool>>(3, Vec<bool>(vecLen, false));
	gamma = { 1.8,2.4,3.0 };
	hashValBase = { 0,0,0 };
	hashValNbr = hashValBase;
}

hsh::mvirp::Solver::Hasher& hsh::mvirp::Solver::Hasher::setBase(Solution &sln)
{
	paramsPtr = sln.paramsPtr;
	auto &params = *paramsPtr;
	auto &vis = sln.visits;
	int numPer = params.numPeriods;
	int numNd = params.numNodes;
	int numGm = (int)gamma.size();
	for (ID no = 0; no < numGm; ++no) { hashValBase[no] = 0; }
	for (ID p = 0; p < numPer; ++p) {
		for (ID n = 1; n < numNd; ++n) { // 不哈希仓库的访问, 邻域动作也不会涉及仓库
			for (ID no = 0; no < numGm; ++no) {
				if (vis[p][n]) { (hashValBase[no] += bitWeight(p * numNd + n, no)) %= vecLen; }
			}
		}
	}
	return *this;
}

hsh::mvirp::UL hsh::mvirp::Solver::Hasher::bitWeight(int idx, int no)
{
	return (UL)std::pow(idx, gamma[no]) % vecLen;
}

void hsh::mvirp::Solver::Hasher::setNbr(Move &move)
{
	auto &params = *paramsPtr;
	int numPer = params.numPeriods;
	int numNd = params.numNodes;
	int numGm = (int)gamma.size();
	for (ID no = 0; no < numGm; ++no) { hashValNbr[no] = hashValBase[no]; } // 基于之前的哈希值作增量修改

	switch (move.type)
	{
	case Move::Type::Del: {
		UL idx = move.periodIdA * numNd + move.origNdA.id;
		for (ID no = 0; no < numGm; ++no) { (hashValNbr[no] -= bitWeight(idx, no)) %= vecLen; }
		break;
	}
	case Move::Type::Add: {
		UL idx = move.periodIdB * numNd + move.curNdA.id;
		for (ID no = 0; no < numGm; ++no) { (hashValNbr[no] += bitWeight(idx, no)) %= vecLen; }
		break;
	}
	case Move::Type::Shift: {
		UL idxOrigA = move.periodIdA * numNd + move.origNdA.id,
			idxCurA = move.periodIdB * numNd + move.curNdA.id;
		for (ID no = 0; no < numGm; ++no) {
			(hashValNbr[no] -= bitWeight(idxOrigA, no)) %= vecLen;
			(hashValNbr[no] += bitWeight(idxCurA, no)) %= vecLen;
		}
		break;
	}
	case Move::Type::Swap: {
		UL idxOrigA = move.periodIdA * numNd + move.origNdA.id,
			idxCurA = move.periodIdB * numNd + move.curNdA.id;
		UL idxOrigB = move.periodIdB * numNd + move.origNdB.id,
			idxCurB = move.periodIdA * numNd + move.curNdB.id;
		for (ID no = 0; no < numGm; ++no) {
			(hashValNbr[no] -= bitWeight(idxOrigA, no)) %= vecLen;
			(hashValNbr[no] += bitWeight(idxCurA, no)) %= vecLen;
			(hashValNbr[no] -= bitWeight(idxOrigB, no)) %= vecLen;
			(hashValNbr[no] += bitWeight(idxCurB, no)) %= vecLen;
		}
		break;
	}
	default:
		break;
	}
}

bool hsh::mvirp::Solver::Hasher::isNbrTaboo(Move &move)
{
	setNbr(move);
	int numGm = (int)gamma.size();
	for (ID no = 0; no < numGm; ++no) {
		if (!hashVec[no][hashValNbr[no]]) { return false; }
	}
	return true;
}

void hsh::mvirp::Solver::Hasher::stNbrTaboo(Move &move)
{
	setNbr(move);
	int numGm = (int)gamma.size();
	for (ID no = 0; no < numGm; ++no) {
		hashVec[no][hashValNbr[no]] = 1;
	}
}

bool hsh::mvirp::Solver::Hasher::isBaseTaboo()
{
	int numGm = (int)gamma.size();
	//Vec<UL> b = hashValBase;
	for (ID no = 0; no < numGm; ++no) {
		//b[no] = hashVec[no][hashValBase[no]];
		if (!hashVec[no][hashValBase[no]]) { return false; }
	}
	return true;
}

void hsh::mvirp::Solver::Hasher::stBaseTaboo()
{
	int numGm = (int)gamma.size();
	for (ID no = 0; no < numGm; ++no) { hashVec[no][hashValBase[no]] = 1; }
}

hsh::mvirp::Solver::Hasher &hsh::mvirp::Solver::Hasher::updBase(Move &move)
{
	setNbr(move);
	std::swap(hashValBase, hashValNbr);
	return *this;
}

