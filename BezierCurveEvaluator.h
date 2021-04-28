#pragma once

#include "curveevaluator.h"
#include "mat.h"

class BezierCurveEvaluator : public CurveEvaluator
{
	using PointIter = std::vector<Point>::const_iterator;
	
public:
	void evaluateCurve(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	                   const float& animation_length, const bool& wrap_control_points) const override;

private:
	void evaluate3k(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	                float animation_length) const;
	
	void evaluateNon3k(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	                   float animation_length) const;
	
	void evaluateNoWrap(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	                    float animation_length) const;
	
	void evaluateSegment(int start, const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	                     float animation_length) const;
	
	static Mat4f basisMatrix;
};

