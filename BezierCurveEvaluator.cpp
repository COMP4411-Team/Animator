#include "BezierCurveEvaluator.h"

#include "vec.h"

Mat4f BezierCurveEvaluator::basisMatrix = 
{
	-1, 3, -3, 1,
	3, -6, 3, 0,
	-3, 3, 0, 0,
	1, 0, 0, 0
};

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	const float& animation_length, const bool& wrap_control_points) const
{
	evaluated_curve_points.clear();
	if (wrap_control_points)
	{
		if (control_points.size() % 3 == 0)
			return evaluate3k(control_points, evaluated_curve_points, animation_length);
		else
			return evaluateNon3k(control_points, evaluated_curve_points, animation_length);
	}
	return evaluateNoWrap(control_points, evaluated_curve_points, animation_length);
}

void BezierCurveEvaluator::evaluate3k(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	float animation_length) const
{
	auto size = control_points.size();

	for (int i = 0; i < size; i += 3)
	{
		evaluateSegment(i, control_points, evaluated_curve_points, animation_length);
	}
}

void BezierCurveEvaluator::evaluateNon3k(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
	float animation_length) const
{
	auto size = control_points.size();

	float denom = animation_length - control_points[size - 1].x + control_points[0].x;
	float endY = (animation_length - control_points[size - 1].x) / denom * (control_points[0].y - control_points[size - 1].y)
				+ control_points[size - 1].y;
	
	evaluated_curve_points.emplace_back(0, endY);
	for (int i = 0; i < size; i += 3)
	{
		int pointsRemain = size - 1 - i;
		if (pointsRemain >= 3)
			evaluateSegment(i, control_points, evaluated_curve_points, animation_length);
		else
		{
			for (int j = i; j < size; ++j)
				evaluated_curve_points.push_back(control_points[j]);
		}
	}
	evaluated_curve_points.emplace_back(animation_length, endY);
}

void BezierCurveEvaluator::evaluateNoWrap(const std::vector<Point>& control_points, std::vector<Point>& evaluated_curve_points,
                                          float animation_length) const
{
	auto size = control_points.size();
	if (size == 0)
		return;

	evaluated_curve_points.emplace_back(0, control_points[0].y);
	for (int i = 0; i < size; i += 3)
	{
		int pointsRemain = size - 1 - i;
		if (pointsRemain >= 3)
			evaluateSegment(i, control_points, evaluated_curve_points, animation_length);
		else
		{
			for (int j = i; j < size; ++j)
				evaluated_curve_points.push_back(control_points[j]);
		}
	}
	evaluated_curve_points.emplace_back(animation_length, control_points[size - 1].y);
}

void BezierCurveEvaluator::evaluateSegment(int start, const std::vector<Point>& control_points, 
	std::vector<Point>& evaluated_curve_points, float animation_length) const
{
	auto size = control_points.size();
	Vec4f X, Y;
	for (int i = 0; i < 4; ++i)
	{
		X[i] = control_points[(start + i) % size].x;
		Y[i] = control_points[(start + i) % size].y;
	}

	if (X[3] <= X[0])
		X[3] += animation_length;

	float prevX = X[0];
	float prevY = Y[0];

	int segCount = X[3] <= X[0] ? s_iSegCount * 3 : s_iSegCount;
	
	for (int i = 0; i < segCount; ++i)
	{
		float t = static_cast<float>(i) / segCount;
		Vec4f T(t * t * t, t * t, t, 1);

		float x = (basisMatrix * T) * X;
		float y = (basisMatrix * T) * Y;
		
		if (x > animation_length)
		{
			if (prevX < animation_length)
			{
				evaluated_curve_points.emplace_back(x, y);
				evaluated_curve_points.emplace_back(prevX - animation_length, prevY);
			}
			evaluated_curve_points.emplace_back(x - animation_length, y);
		}
		else
			evaluated_curve_points.emplace_back(x, y);

		prevY = y;
		prevX = x;
	}
}
