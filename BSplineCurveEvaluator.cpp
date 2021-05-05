#include "BSplineCurveEvaluator.h"
#include "vec.h"

Mat4f BSplineCurveEvaluator::BSMatrix = {
	-1, 3, -3, 1,
	3, -6, 3, 0,
	-3, 0, 3, 0,
	1, 4, 1, 0
};



void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {
	ptvEvaluatedCurvePts.clear();
	return noWrap(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength);
}

void BSplineCurveEvaluator::noWrap(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength) const {

	int size = ptvCtrlPts.size();
	if (size == 0) return;

	ptvEvaluatedCurvePts.emplace_back(0, ptvCtrlPts[0].y);

	for (int i = 0; i < size-3; i++) {
		evaluateBSpline(i, ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength);
	}

	ptvEvaluatedCurvePts.emplace_back(fAniLength, ptvCtrlPts[size-1].y);
}

void BSplineCurveEvaluator::evaluateBSpline(int i, const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength) const {

	Vec4f X, Y;
	for (int j = i; j < i + 4; j++) {
		X[j - i] = ptvCtrlPts[j].x;
		Y[j - i] = ptvCtrlPts[j].y;
	}

	int segCount = s_iSegCount * 3;

	for (int i = 0; i < segCount; ++i)
	{
		float t = static_cast<float>(i) / segCount;
		Vec4f T(t * t * t, t * t, t, 1);

		float x = (BSMatrix * T) * X/6.0;
		float y = (BSMatrix * T) * Y/6.0;

		ptvEvaluatedCurvePts.emplace_back(x, y);

	}
}