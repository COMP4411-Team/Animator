#include "C2InterpolatingCurve.h"

#include <algorithm>

#include "BezierCurveEvaluator.h"
#include "linearcurveevaluator.h"

void C2InterpolatingCurve::evaluateCurve(const std::vector<Point>& control_points,
	std::vector<Point>& evaluated_curve_points, const float& animation_length, const bool& wrap_control_points) const
{
	if (control_points.size() < 3)
	{
		LinearCurveEvaluator evaluator;
		return evaluator.evaluateCurve(control_points, evaluated_curve_points, animation_length, wrap_control_points);
	}
	
	std::vector<float> y(control_points.size());
	std::vector<Point> p(control_points.size());
	y[0] = 0.5f;
	p[0] = 3 * y[0] * (control_points[1] - control_points[0]);

	// Forward substitution
	auto size = control_points.size();
	for (int i = 1; i < size - 1; ++i)
	{
		y[i] = 1 / (4 - y[i - 1]);
		p[i] = y[i] * (3 * (control_points[i + 1] - control_points[i - 1]) - p[i - 1]);
	}
	y[size - 1] = 1 / (2 - y[size - 2]);
	p[size - 1] = y[size - 1] * (3 * (control_points[size - 1] - control_points[size - 2]) - p[size - 2]);

	// Backward substitution
	for (int i = size - 2; i >= 0; --i)
	{
		p[i] = p[i] - y[i] * p[i + 1];
	}

	// Compute control points for Bezier curve
	std::vector<Point> bezierCtrlPoints;
	bezierCtrlPoints.push_back(control_points[0]);
	float factor = 1.f / 3.f;
	for (int i = 0; i < size - 1; ++i)
	{
		bezierCtrlPoints.push_back(control_points[i] + factor * p[i]);
		bezierCtrlPoints.push_back(control_points[i + 1] - factor * p[i + 1]);
		bezierCtrlPoints.push_back(control_points[i + 1]);
	}

	BezierCurveEvaluator evaluator;
	evaluator.evaluateCurve(bezierCtrlPoints, evaluated_curve_points, animation_length, wrap_control_points);
	std::sort(evaluated_curve_points.begin(), evaluated_curve_points.end(), 
		[](const Point& p1, const Point& p2){ return p1.x < p2.x; });
}
