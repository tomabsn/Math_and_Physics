#pragma once
#include "PxPhysicsAPI.h"
using namespace physx;

enum FilterGroup {
	eBALL = (1 << 0),
	eWALL = (1 << 1),
	eBALLT = (1 << 2),
	eBALLUP = (1 << 3),
	eBALLR = (1 << 4)

};


class MyBall : public PxRigidDynamic {
	FilterGroup ball_type;
public: 
	MyBall() = default;
	void setType(FilterGroup type) {
		ball_type = type;
	}
	FilterGroup getType() {
		return ball_type;
	}

	virtual ~MyBall() = default;
};