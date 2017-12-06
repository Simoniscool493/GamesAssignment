#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Assignment :
		public Game
	{
	private:

	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall();
		shared_ptr<PhysicsController> bird;
		shared_ptr<PhysicsController> CreateMyObject(glm::vec3 position);
	};
}