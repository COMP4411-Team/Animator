#include "CatmullromCurveEvaluator.h"
#include "vec.h"

Mat4f CatmullromCurveEvaluator::BazierMatrix =
{
	-1, 3, -3, 1,
	3, -6, 3, 0,
	-3, 3, 0, 0,
	1, 0, 0, 0
};

void CatmullromCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {
	ptvEvaluatedCurvePts.clear();
	

	if (bWrap) return Wrap(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);

	return noWrap(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}

void CatmullromCurveEvaluator::Wrap(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp) const {

	float px = ptvCtrlPts[0].x;
	float  py = ptvCtrlPts[0].y;

	int size = ptvCtrlPts.size();
	if (size == 0) return;

	for (int i = 0; i < size ; i++) {
		evaluateCSpline(i, ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWarp, px, py);
	}
}

void CatmullromCurveEvaluator::noWrap(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp) const {

	float px = ptvCtrlPts[0].x;
	float  py = ptvCtrlPts[0].y;

	int size = ptvCtrlPts.size();
	if (size == 0) return;

	ptvEvaluatedCurvePts.emplace_back(0, ptvCtrlPts[0].y);

	for (int i = 0; i < size-1; i++) {
		evaluateCSpline(i, ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWarp,px, py);
	}

	ptvEvaluatedCurvePts.emplace_back(fAniLength, ptvCtrlPts[size - 1].y);
}

void CatmullromCurveEvaluator::evaluateCSpline(int i, const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	float fAniLength, bool bWarp, float prevX, float prevY) const {
	
	int size = ptvCtrlPts.size();
	Vec4f X, Y;
	Point p0, p1, p2, p3;

	p1 = ptvCtrlPts[i%size];
	p2 = ptvCtrlPts[(i + 1)%size];

	if (p2.x <= p1.x) 
		p2.x += fAniLength;

	X[0] = p1.x;
	Y[0] = p1.y;

	X[3] = p2.x;
	Y[3] = p2.y;

	if (!bWarp) {
		p0= (i-1<0)? ptvCtrlPts[0]: ptvCtrlPts[i-1];
		p3 = (i+2>=size)? ptvCtrlPts[size-1]: ptvCtrlPts[i + 2];
	}
	else {
		p3 = ptvCtrlPts[(i + 2) % size];
		p0 = ptvCtrlPts[(i - 1+size) % size];
		if (p3.x <= p2.x) p3.x += fAniLength;
		if (p0.x >= p1.x) p0.x -= fAniLength;
	}

	X[1] = (p1 + tension / 3.0 * (p2 - p0)).x;
	Y[1] = (p1 + tension / 3.0 * (p2 - p0)).y;

	X[2] = (p2 - tension / 3.0 * (p3 - p1)).x;
	Y[2] = (p2 - tension / 3.0 * (p3 - p1)).y;

	int segCount = s_iSegCount;

	for (int k = 0; k < segCount; ++k)
	{
		float t = static_cast<float>(k) / segCount;
		Vec4f T(t * t * t, t * t, t, 1);

		float x =  (BazierMatrix * T) * X;
		float y =  (BazierMatrix * T) * Y;

		if (x > fAniLength) {
			ptvEvaluatedCurvePts.emplace_back(x - fAniLength, y);
		}
		else {
			if (x < prevX||x>X[3]) {
				x = prevX + s_fFlatnessEpsilon;
				//if (prevY<Y[3] && prevY>y) y = prevY + s_fFlatnessEpsilon;
				//else if(prevY>Y[3] && prevY<y) y = prevY - s_fFlatnessEpsilon;
				
			}
			ptvEvaluatedCurvePts.emplace_back(x, y);
			prevX = x;
			prevY = y;
		}

	}

}