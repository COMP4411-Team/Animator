#include "BSplineCurveEvaluator.h"
#include "vec.h"

Mat4f BSplineCurveEvaluator::BSMatrix = {
	-1, 3, -3, 1,
	3, -6, 0, 4,
	-3, 3, 3, 1,
	1, 0, 0, 0
};



void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {
	ptvEvaluatedCurvePts.clear();

	if(bWrap) return Wrap(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);

	return noWrap(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}

void BSplineCurveEvaluator::Wrap(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp) const {

	int size = ptvCtrlPts.size();
	if (size == 0) return;

	for (int i = 0; i < size ; i++) {
		evaluateBSpline(i, ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWarp);
	}

}

void BSplineCurveEvaluator::noWrap(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp) const {

	int size = ptvCtrlPts.size();
	if (size == 0) return;

	ptvEvaluatedCurvePts.emplace_back(0, ptvCtrlPts[0].y);

	for (int i = -1; i < size-2; i++) {
		evaluateBSpline(i, ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWarp);
	}

	ptvEvaluatedCurvePts.emplace_back(fAniLength, ptvCtrlPts[size-1].y);
}

void BSplineCurveEvaluator::evaluateBSpline(int i, const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp) const {

	int size = ptvCtrlPts.size();
	Vec4f X, Y;

	if (bWarp) {
		for (int j = i; j < i + 4; j++) {
			if (j > size-1) {
				X[j - i] = ptvCtrlPts[j % size].x + fAniLength;
				Y[j - i] = ptvCtrlPts[j % size].y;
			}
			else {
				X[j - i] = ptvCtrlPts[j].x;
				Y[j - i] = ptvCtrlPts[j].y;
			}

		}

	}
	else {
		for (int j = i; j < i + 4; j++) {
			if (j == -1) {
				X[j - i] = 2 * ptvCtrlPts[0].x - ptvCtrlPts[1].x;
				Y[j - i] = 2 * ptvCtrlPts[0].y - ptvCtrlPts[1].y;
			}
			else if (j == size) {
				X[j - i] = 2 * ptvCtrlPts[size - 1].x - ptvCtrlPts[size - 2].x;
				Y[j - i] = 2 * ptvCtrlPts[size - 1].y - ptvCtrlPts[size - 2].y;
			}
			else {
				X[j - i] = ptvCtrlPts[j].x;
				Y[j - i] = ptvCtrlPts[j].y;
			}

		}
	}

	int segCount = s_iSegCount * 3;

	for (int k = 0; k < segCount; ++k)
	{
		float t = static_cast<float>(k) / segCount;
		Vec4f T(t * t * t, t * t, t, 1);

		float x = 1/6.0*((BSMatrix * T) * X);
		float y = 1/6.0*((BSMatrix * T) * Y);

		if (x > fAniLength) {
			ptvEvaluatedCurvePts.emplace_back(x - fAniLength, y);
		}
		else {
			ptvEvaluatedCurvePts.emplace_back(x, y);
		}

	}
}