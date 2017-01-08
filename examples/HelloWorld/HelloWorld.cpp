#include <iostream>

#include <btBulletDynamicsCommon.h>

#include "../../../kklautodiff/src/kklautodiff.hpp"

int main (void)
{

        btBroadphaseInterface* broadphase = new btDbvtBroadphase();

        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

        btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

        kklautodiff::Tape<float> tape(1000000);
        kklautodiff::RealNumber<float> g = -10;
        kklautodiff::RealNumber<float> initial_height = 50;
        initial_height.attach_to_tape(tape);
        g.attach_to_tape(tape);
        dynamicsWorld->setGravity(btVector3(0.0f, g, 0.0f));


        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

        btCollisionShape* fallShape = new btSphereShape(1);


        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(groundRigidBody);


        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, initial_height, 0)));
        btScalar mass = 1;
        btVector3 fallInertia(0, 0, 0);
        fallShape->calculateLocalInertia(mass, fallInertia);
        btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
        btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
        dynamicsWorld->addRigidBody(fallRigidBody);


        for (int i = 0; i < 300; i++) {
                dynamicsWorld->stepSimulation(1 / 60.f, 10);

                btTransform trans;
                fallRigidBody->getMotionState()->getWorldTransform(trans);

                trans.getOrigin().getY().diff();
                std::cout << "sphere height: " << (float) trans.getOrigin().getY() << "\t" << (float) g.get_diff() << std::endl;
        }

        dynamicsWorld->removeRigidBody(fallRigidBody);
        delete fallRigidBody->getMotionState();
        delete fallRigidBody;

        dynamicsWorld->removeRigidBody(groundRigidBody);
        delete groundRigidBody->getMotionState();
        delete groundRigidBody;


        delete fallShape;

        delete groundShape;


        delete dynamicsWorld;
        delete solver;
        delete collisionConfiguration;
        delete dispatcher;
        delete broadphase;

        return 0;
}