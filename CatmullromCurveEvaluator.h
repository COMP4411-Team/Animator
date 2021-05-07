#pragma once

#include "CurveEvaluator.h"
#include "mat.h"

//using namespace std;

class CatmullromCurveEvaluator : public CurveEvaluator {
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const override;

private:
	void Wrap(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		float fAniLength, bool bWarp) const;

	void noWrap(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		float fAniLength, bool bWarp) const;

	void evaluateCSpline(int i, const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		float fAniLength, bool bWarp) const;

	float tension{0.5};
	static Mat4f BazierMatrix;
};