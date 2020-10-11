#pragma once
#include "PxPhysicsAPI.h"
#include <vector>
#include <algorithm>
#include "Ball.h"
using namespace physx;


class ContactModif : public PxContactModifyCallback, public PxSimulationEventCallback {

	std::vector<MyBall*> m_balls;
public:

	ContactModif() = default;
	virtual ~ContactModif() = default;

	void emplace_back(MyBall* ball) { m_balls.emplace_back(ball); }
	std::vector<MyBall*>& balls() { return m_balls; }

	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override { PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count) override { PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count) override { PX_UNUSED(actors); PX_UNUSED(count); }
	void onTrigger(PxTriggerPair* pairs, PxU32 count) override { PX_UNUSED(pairs); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody* const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) override {}

	// Implements PxSimulationEventCallback
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override { 
		PX_UNUSED(pairHeader); PX_UNUSED(pairs); PX_UNUSED(nbPairs);
	}

	void onContactModify(PxContactModifyPair* const pairs, PxU32 count) override {
		for (PxU32 i = 0; i < count; i++)
		{
			PxContactModifyPair& pair = pairs[i];
			if (pair.actor[0] !=NULL  && pair.actor[1] != NULL) {

				auto it0 = std::find(m_balls.begin(), m_balls.end(), pair.actor[0]);
				auto it1 = std::find(m_balls.begin(), m_balls.end(), pair.actor[1]);

				if (it0 != m_balls.end())
				{
					if (it1 != m_balls.end())
					{
						for (PxU32 j = 0; j < pair.contacts.size(); ++j)
							pair.contacts.ignore(j);
						continue;
					}

					MyBall* ball = *it0;

					switch (ball->getType())
					{
					case FilterGroup::eBALL:
						break;
					case FilterGroup::eBALLT:
						for (PxU32 j = 0; j < pair.contacts.size(); ++j)
							pair.contacts.ignore(j);
						break;
					case FilterGroup::eBALLR:
					{
						for (PxU32 j = 0; j < pair.contacts.size(); ++j)
							pair.contacts.ignore(j);
						ball->setLinearVelocity(PxVec3{ -25, 0, 0 });
					}
					break;
					case FilterGroup::eBALLUP:
						ball->setLinearVelocity(PxVec3{ 0, 50, 0 });
						break;
						//default:
						//	break;
					}
				}
			}
		}
		
	}
};