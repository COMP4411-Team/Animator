#include "Torus.h"
#include <FL/gl.h>
#include "modelerdraw.h"
#include "modelerglobals.h"

using namespace std;

Torus::Torus(float tubeL, float tubeS, float ringL, float ringS, double px, double py, double pz, double rx, double ry, double rz, int flower, int petal) {
	tubeLongRadius = tubeL;
	tubeShortRadius = tubeS;
	ringLongRadius = ringL;
	ringShortRadius = ringS;
	positionX = px;
	positionY = py;
	positionZ = pz;
	rotationX = 2 * M_PI / 360 * rx;
	rotationY = 2 * M_PI / 360 * ry;
	rotationZ = 2 * M_PI / 360 * rz;
	enableFlower = flower;
	numPetal = petal;
}

void Torus::getPoint(double ringAngle, double tubeAngle){
	tempy = (ringLongRadius + tubeLongRadius * cos(tubeAngle)) * cos(ringAngle);
	tempz = (ringShortRadius + tubeLongRadius * cos(tubeAngle)) * sin(ringAngle);
	tempx = tubeShortRadius * sin(tubeAngle);
	pointRX(rotationX);
	pointRY(rotationY);
	pointRZ(rotationZ);
	transPoint(positionX, positionY, positionZ);

}

void Torus::getFlowerPoint(double petalAngel, double tubeAngle) {
	tempy = petaly + (petalR + tubeLongRadius * cos(tubeAngle)) * cos(petalAngel);
	tempz = petalz + (petalR + tubeLongRadius * cos(tubeAngle)) * sin(petalAngel);
	tempx = tubeShortRadius * sin(tubeAngle);
}

void Torus::transPoint(double x, double y, double z) {
	tempx += x;
	tempy += y;
	tempz += z;
}

void Torus::pointRX(double rx) {
	double y = tempy * cos(rx) - tempz * sin(rx);
	double z = tempy * sin(rx) + tempz * cos(rx);
	tempy = y;
	tempz = z;
}

void Torus::pointRY(double ry) {
	double x = tempx * cos(ry) + tempz * sin(ry);
	double z = -tempx * sin(ry) + tempz * cos(ry);
	tempx = x;
	tempz = z;
}

void Torus::pointRZ(double rz) {
	double x = tempx * cos(rz) - tempy * sin(rz);
	double y = tempx * sin(rz) + tempy * cos(rz);
	tempx = x;
	tempy = y;
}

void Torus::draw() {
	double ringStep = 2 * M_PI / ringVertex;
	double tubeStep = 2 * M_PI / tubeVertex;
	for (int i = 0; i < ringVertex; i++) {
		for (int j = 0; j < tubeVertex; j++) {
			getPoint(ringStep * i, tubeStep * j);
			auto x1 = tempx;
			auto y1 = tempy;
			auto z1 = tempz;
			getPoint(ringStep * (i + 1), tubeStep * j);
			auto x2 = tempx;
			auto y2 = tempy;
			auto z2 = tempz;
			getPoint(ringStep * i, tubeStep * (j + 1));
			auto x3 = tempx;
			auto y3 = tempy;
			auto z3 = tempz;
			getPoint(ringStep * (i + 1), tubeStep * (j + 1));
			auto x4 = tempx;
			auto y4 = tempy;
			auto z4 = tempz;
			drawSlice(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
		}
	}
	if(enableFlower){
		double flowerStep = 2 * M_PI / numPetal;
		double petalStep = M_PI / petalVertex;
		double tubeStep = 2 * M_PI / tubeVertex;
		for (int i = 0; i < numPetal; i++) {
			petaly = (ringLongRadius * cos(i * flowerStep)+ ringLongRadius * cos((i+1) * flowerStep))/2;
			petalz = (ringShortRadius * sin(i * flowerStep) + ringShortRadius * sin((i + 1) * flowerStep)) / 2;
			double dy = ringLongRadius * cos(i * flowerStep) - petaly;
			double dz = ringShortRadius * sin(i * flowerStep) - petalz;
			petalR = sqrt(dy * dy + dz * dz);
			double iniAngle = atan(dz / dy);
			if (dy < 0) iniAngle += M_PI;
			for (int j = 0; j < petalVertex; j++) {
				for (int k = 0; k < tubeVertex; k++) {
					getFlowerPoint(iniAngle + j * petalStep, tubeStep * k);
					auto x1 = tempx;
					auto y1 = tempy;
					auto z1 = tempz;
					getFlowerPoint(iniAngle + (j+1) * petalStep, tubeStep * k);
					auto x2 = tempx;
					auto y2 = tempy;
					auto z2 = tempz;
					getFlowerPoint(iniAngle + j * petalStep, tubeStep * (k+1));
					auto x3 = tempx;
					auto y3 = tempy;
					auto z3 = tempz;
					getFlowerPoint(iniAngle + (j+1) * petalStep, tubeStep * (k + 1));
					auto x4 = tempx;
					auto y4 = tempy;
					auto z4 = tempz;
					drawSlice(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
				}
			}
		}
	}
}