#pragma once

#include <string>
#include <map>

class Torus {
public:
	Torus(float tubeL, float tubeS, float ringL, float ringS, double px, double py, double pz, double rx, double ry, double rz, int flower, int petal);
	void getPoint(double ringAngle, double tubeAngle);
	void getFlowerPoint(double petalAngel, double tubeAngle);
	void draw();
	void transPoint(double x, double y, double z);
	void pointRX(double rx);
	void pointRY(double ry);
	void pointRZ(double rz);

	void setTubeVertex(int i) {
		tubeVertex = i;
	}
	void setRingVertex(int i){
		ringVertex = i;
	}
	void setTubeLongRadius(float f) {
		tubeLongRadius = f;
	}
	void setTubeShortRadius(float f) {
		tubeShortRadius = f;
	}
	void setRingLongRadius(float f) {
		ringLongRadius = f;
	}
	void setRingShortRadius(float f) {
		ringShortRadius = f;
	}

	int tubeVertex{ 10 };
	int ringVertex{ 40 };
	int petalVertex{ 10 };
	float tubeLongRadius{ 1.0f };
	float tubeShortRadius{ 0.5f };
	float ringLongRadius{ 4.0f };
	float ringShortRadius{ 2.0f };
	double positionX, positionY, positionZ;
	double rotationX, rotationY, rotationZ;
	double tempx, tempy, tempz;
	double petalx, petaly, petalz;
	double petalR;
	bool enableFlower;
	int numPetal;
};