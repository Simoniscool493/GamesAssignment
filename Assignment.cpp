#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

#include "Assignment.h"
#include <iostream>
#include <sstream>


using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

std::list <shared_ptr<PhysicsController>> items;
shared_ptr<PhysicsController> myObject;

btHingeConstraint * leftHinge;
btHingeConstraint * rightHinge;

bool Assignment::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	for (int i = 1; i < 20; i++)
	{
		auto pos = RandomPosition(100);
		if (pos.y < 0)
		{
			pos.y = -pos.y;
		}

		shared_ptr<PhysicsController> ball = physicsFactory->CreateSphere(3, pos, glm::quat());
		items.push_back(ball);
	}

	myObject = CreateMyObject(glm::vec3(0, 30, 0));

	return Game::Initialise();
}

glm::vec3 gravityCentre = glm::vec3(0, 0, 0);
int gravityScale = 20;
bool enableBlackhole = false;

void BGE::Assignment::Update(float timeDelta)
{
	const Uint8 * keyState = Game::Instance()->GetKeyState();

	if (keyState[SDL_SCANCODE_0])
	{
		gravityScale += 1;
		cout << gravityScale << endl;
	}
	if (keyState[SDL_SCANCODE_9])
	{
		gravityScale -= 1;
		cout << gravityScale << endl;
	}

	float forceStrength = 50000;

	if (keyState[SDL_SCANCODE_U])
	{
		myObject->rigidBody->applyCentralForce(btVector3(1, 1, -forceStrength));
	}
	if (keyState[SDL_SCANCODE_J])
	{
		myObject->rigidBody->applyCentralForce(btVector3(1, 1, forceStrength));
	}
	if (keyState[SDL_SCANCODE_H])
	{
		myObject->rigidBody->applyCentralForce(btVector3(-forceStrength, 1, 1));
	}
	if (keyState[SDL_SCANCODE_K])
	{
		myObject->rigidBody->applyCentralForce(btVector3(forceStrength, 1, 1));
	}

	if (keyState[SDL_SCANCODE_Q])
	{
		leftHinge->enableAngularMotor(true, 100, 100);
		rightHinge->enableAngularMotor(true, -100, 100);
	}
	if (keyState[SDL_SCANCODE_E])
	{
		leftHinge->enableAngularMotor(true, -100, 100);
		rightHinge->enableAngularMotor(true, 100, 100);
	}

	if (keyState[SDL_SCANCODE_O])
	{
		enableBlackhole = false;
	}
	if (keyState[SDL_SCANCODE_P])
	{
		enableBlackhole = true;
	}

	if (keyState[SDL_SCANCODE_N])
	{
		cout << "Game paused" << endl;
	}

		list<shared_ptr<PhysicsController>>::iterator it = items.begin();
		while (it != items.end())
		{
			auto item = (*it++);
			auto pos = item->transform->position - myObject->transform->position;

			if (enableBlackhole)
			{
				auto Xapply = -pos.x*gravityScale;
				auto Yapply = -pos.y*gravityScale;
				auto Zapply = -pos.z*gravityScale;

				auto Xapply2 = -myObject->transform->position.x*gravityScale;
				auto Yapply2 = -myObject->transform->position.y*gravityScale;
				auto Zapply2 = -myObject->transform->position.z*gravityScale;

				item->rigidBody->applyForce(btVector3(Xapply, Yapply, Zapply), btVector3(0, 0, 0));
			}
			item->rigidBody->applyForce(btVector3(0, -1000, 0), btVector3(0, 0, 0));
		}

	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> Assignment::CreateMyObject(glm::vec3 position)
{
	float scale = 5;
	float secondary_rad = scale / 1.5;

	shared_ptr<PhysicsController> body = physicsFactory->CreateBox(scale*1.5, scale*1.5, scale*1.5, position, glm::quat());
	body->rigidBody->setMassProps(600, btVector3());

	float shoulder_rad = glm::abs((glm::sqrt(2 * (scale * scale)) - scale) / 10);

	glm::vec3 left_wing_hinge = position + glm::vec3(0, ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))));
	glm::vec3 right_wing_hinge = position + glm::vec3(0, ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), -((scale + shoulder_rad) * (1 / glm::sqrt(2.0))));

	shared_ptr<PhysicsController> left_wing = physicsFactory-> CreateBox(scale * 2, scale / 20, scale, left_wing_hinge  + glm::vec3( 0, 0, (scale + shoulder_rad)), glm::quat());
	shared_ptr<PhysicsController> right_wing = physicsFactory->CreateBox(scale * 2, scale / 20, scale, right_wing_hinge + glm::vec3(0, 0, -(scale + shoulder_rad)), glm::quat());

	btHingeConstraint * body_l_wing = new btHingeConstraint(*body->rigidBody, *left_wing->rigidBody,  GLToBtVector(glm::vec3( ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), 0)), GLToBtVector(glm::vec3(-(scale + shoulder_rad), 0, 0)), btVector3(0, 0, 1), btVector3(0, 0, 1));
	btHingeConstraint * body_r_wing = new btHingeConstraint(*body->rigidBody, *right_wing->rigidBody, GLToBtVector(glm::vec3(-((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), ((scale + shoulder_rad) * (1 / glm::sqrt(2.0))), 0)), GLToBtVector(glm::vec3( (scale + shoulder_rad), 0, 0)), btVector3(0, 0, 1), btVector3(0, 0, 1));

	Game::dynamicsWorld->addConstraint(body_l_wing);
	Game::dynamicsWorld->addConstraint(body_r_wing);

	float hip_rad = ((scale + (scale / 2) - (secondary_rad / 2)) - (scale * (glm::sqrt(3.0) / 2))) / 2;
	shared_ptr<PhysicsController> left_leg =  physicsFactory->CreateCylinder((scale / 4), scale * 2, position + glm::vec3( (scale / 2), -(scale + (scale / 2.5)), 0), glm::quat());
	shared_ptr<PhysicsController> right_leg = physicsFactory->CreateCylinder((scale / 4), scale * 2, position + glm::vec3(-(scale / 2), -(scale + (scale / 2.5)), 0), glm::quat());
	shared_ptr<PhysicsController> middle_leg = physicsFactory->CreateCylinder((scale / 4), scale * 2, position + glm::vec3((scale / 2), -(scale + (scale / 2.5)), 0), glm::quat());

	btTransform t3, t4;
	t3.setIdentity();
	t4.setIdentity();
	t3.setOrigin(btVector3((scale / 2), -((scale * (glm::sqrt(3.0) / 2)) + hip_rad), 0));
	t4.setOrigin(btVector3(0, (secondary_rad / 2) + hip_rad, 0));
	btFixedConstraint * body_l_leg = new btFixedConstraint(*body->rigidBody, *left_leg->rigidBody, t3, t4);
	dynamicsWorld->addConstraint(body_l_leg);

	btTransform t5, t6;
	t5.setIdentity();
	t6.setIdentity();
	t5.setOrigin(btVector3(-(scale / 2), -((scale * (glm::sqrt(3.0) / 2)) + hip_rad), 0));
	t6.setOrigin(btVector3(0, (secondary_rad / 2) + hip_rad, 0));
	btFixedConstraint * body_r_leg = new btFixedConstraint(*body->rigidBody, *right_leg->rigidBody, t5, t6);
	dynamicsWorld->addConstraint(body_r_leg);

	btTransform t7, t8;
	t7.setIdentity();
	t8.setIdentity();
	t7.setOrigin(btVector3(0, -((scale * (glm::sqrt(3.0) / 2)) + hip_rad), 0));
	t8.setOrigin(btVector3(0, (secondary_rad / 2) + hip_rad, 0));
	btFixedConstraint * body_m_leg = new btFixedConstraint(*body->rigidBody, *middle_leg->rigidBody, t7, t8);
	dynamicsWorld->addConstraint(body_m_leg);

	leftHinge = body_l_wing;
	rightHinge = body_r_wing;

	items.push_back(body);
	items.push_back(left_wing);
	items.push_back(right_wing);
	items.push_back(left_leg);
	items.push_back(right_leg);
	items.push_back(middle_leg);

	return body;
}

