#include <iostream>
#include "reactphysics3d.h"

float degToRad(float degrees){
    return degrees * 3.14159265 / 180;
}

void exercise1(){
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/60.0f;

    float inclineDegrees = 27.0f;     
    float crateWeight = 25.0f;
    float targetAcceleration = 0.3f;
    float forceOnCrate = 103.841f;
    float friction = 0.475f;
    float epsilon = 0.01f;

    rp3d::Quaternion inclineOrientation(rp3d::Vector3(0, 0, -degToRad(inclineDegrees)));

    //Create Crate Object 
    rp3d::Vector3 cratePosition(0.0, 0.60, 0.0);
    rp3d::BoxShape crateBoxShape(rp3d::Vector3(0.5, 0.5, 0.5));
    rp3d::Transform crateTransform(cratePosition, inclineOrientation);
    rp3d::RigidBody* crate;
    crate = world.createRigidBody(crateTransform);
    crate->setType(reactphysics3d::BodyType::DYNAMIC);
    crate->addCollisionShape(&crateBoxShape, rp3d::Transform::identity(), crateWeight);

    //Create incline Object
    rp3d::Vector3 inclinePosition(0.0, -1.0, 0.0);
    rp3d::BoxShape inclineBoxShape(rp3d::Vector3(30.0, 1.0, 30.0));
    rp3d::Transform inclineTransform(inclinePosition, inclineOrientation);
    rp3d::RigidBody* incline;
    incline = world.createRigidBody(inclineTransform);
    incline->setType(reactphysics3d::BodyType::STATIC);
    incline->addCollisionShape(&inclineBoxShape, rp3d::Transform::identity(), 1);

    //Set Material of incline
    rp3d::Material& inclineMaterial = incline->getMaterial();
    inclineMaterial.setBounciness(0.0);
    inclineMaterial.setFrictionCoefficient(friction);

    //Set Material of crate
    rp3d::Material& crateMaterial = crate->getMaterial();
    crateMaterial.setBounciness(0.0);
    crateMaterial.setFrictionCoefficient(friction);

    reactphysics3d::Vector3 inclineDirection(inclineOrientation * reactphysics3d::Vector3(1.0, 0.0, 0.0));
    inclineDirection.normalize();
    crate->applyForceToCenterOfMass(inclineDirection * forceOnCrate);

    float elapsedTime = 0.0;
    float currentAcceleration = 0.0;

    while (true){

        world.update(timeStep);
        elapsedTime += timeStep;

        rp3d::Vector3 currentVel = crate->getLinearVelocity();
        currentAcceleration = currentVel.length() / elapsedTime;

        if(currentAcceleration >= targetAcceleration - epsilon && currentAcceleration <= targetAcceleration + epsilon){
            std::cout << "Acceleration is: " << currentAcceleration << std::endl;
            std::cout << "Calculated force 103.841N and friction 0.475 seem to be correct since acceleration matches given one" << std::endl;
            break;
        }

    }


}

//No friction but with mass
void exercise2(){
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    
    const float timeStep = 1.0/60.0f;

    float inclineDegrees = 22.0f;     
    float crateWeight = 7.0f;
    float friction = 0.0f;
    float targetDistance = 12.0f;
    
    rp3d::Quaternion inclineOrientation(rp3d::Vector3(0, 0, -degToRad(inclineDegrees)));

    //Create crate Object
    rp3d::Vector3 cratePosition(0.0, 0.60, 0.0);
    rp3d::BoxShape crateBoxShape(rp3d::Vector3(0.5, 0.5, 0.5));
    rp3d::Transform crateTransform(cratePosition, inclineOrientation);
    rp3d::RigidBody* crate;
    crate = world.createRigidBody(crateTransform);
    crate->setType(reactphysics3d::BodyType::DYNAMIC);
    crate->addCollisionShape(&crateBoxShape, rp3d::Transform::identity(), crateWeight);

    //Create incline Object
    rp3d::Vector3 inclinePosition(0.0, -1.0, 0.0);
    rp3d::BoxShape inclineBoxShape(rp3d::Vector3(30.0, 1.0, 30.0));
    rp3d::Transform inclineTransform(inclinePosition, inclineOrientation);
    rp3d::RigidBody* incline;
    incline = world.createRigidBody(inclineTransform);
    incline->setType(reactphysics3d::BodyType::STATIC);
    incline->addCollisionShape(&inclineBoxShape, rp3d::Transform::identity(), 1);

    //Set Material of incline
    rp3d::Material& inclineMaterial = incline->getMaterial();
    inclineMaterial.setBounciness(0.0);
    inclineMaterial.setFrictionCoefficient(friction);

    //Set Material of crate
    rp3d::Material& crateMaterial = crate->getMaterial();
    crateMaterial.setBounciness(0.0);
    crateMaterial.setFrictionCoefficient(friction);

    float elapsedTime = 0.0;

    while (true){
        rp3d::Vector3 prevVel = crate->getLinearVelocity();
        
        world.update(timeStep);
        elapsedTime += timeStep;
        
        rp3d::Vector3 newVel = crate->getLinearVelocity();
        rp3d::Vector3 acceleration = (newVel - prevVel)/timeStep;
        
        rp3d::Vector3 cratePos = crate->getTransform().getPosition();
        //Here this works rather well, but it doesn't in exercise 4
        //float deltaX = cratePos.x - cratePosition.x;
        //float deltaY = cratePos.y - cratePosition.y;
        //if ( (deltaX * deltaX + deltaY * deltaY) >= targetDistance * targetDistance){
        if((cratePos - cratePosition).length() >= targetDistance){
            std::cout << "Acceleration: " << acceleration.length() << "m/s²"<< std::endl;
            std::cout << "Speed: " << crate->getLinearVelocity().length() << "m/s" << std::endl;
            break;
        }

    }

}

void exercise3(){

    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));
    //rp3d::DynamicsWorld world(gravity);

    const float timeStep = 1.0/200.0f;

    float inclineDegrees = 22.0f;     
    float crateWeight = 1.0f;
    float friction = 0.0f;
    float initSpeed = 4.5f;

    rp3d::Quaternion inclineOrientation(rp3d::Vector3(0, 0, -degToRad(inclineDegrees)));

    //Create crate Object
    rp3d::Vector3 cratePosition(0.0, 0.60, 0.0);
    rp3d::BoxShape crateBoxShape(rp3d::Vector3(0.5, 0.5, 0.5));
    rp3d::Transform crateTransform(cratePosition, inclineOrientation);
    rp3d::RigidBody* crate;
    crate = world.createRigidBody(crateTransform);
    crate->setType(reactphysics3d::BodyType::DYNAMIC);
    crate->addCollisionShape(&crateBoxShape, rp3d::Transform::identity(), crateWeight);

    //Create incline Object
    rp3d::Vector3 inclinePosition(0.0, -1.0, 0.0);
    rp3d::BoxShape inclineBoxShape(rp3d::Vector3(30.0, 1.0, 30.0));
    rp3d::Transform inclineTransform(inclinePosition, inclineOrientation);
    rp3d::RigidBody* incline;
    incline = world.createRigidBody(inclineTransform);
    incline->setType(reactphysics3d::BodyType::STATIC);
    incline->addCollisionShape(&inclineBoxShape, rp3d::Transform::identity(), 1);

    //Set Material of incline
    rp3d::Material& inclineMaterial = incline->getMaterial();
    inclineMaterial.setBounciness(0.0);
    inclineMaterial.setFrictionCoefficient(friction);

    //Set Material of crate
    rp3d::Material& crateMaterial = crate->getMaterial();
    crateMaterial.setBounciness(0.0);
    crateMaterial.setFrictionCoefficient(friction);
    
    //Set the velocity to go up the incline
    reactphysics3d::Vector3 inclineDirection(inclineOrientation * reactphysics3d::Vector3(1.0, 0.0, 0.0));
    inclineDirection *= -1.0;
    reactphysics3d::Vector3 vel = inclineDirection * initSpeed;
    crate->setLinearVelocity(vel);

    float elapsedTime = 0.0;
    reactphysics3d::Vector3 prevCratePos = cratePosition;
    bool downhill = false;

    while (true){
        
        world.update(timeStep);
        elapsedTime += timeStep;
        
        rp3d::Vector3 currentCratePos = crate->getTransform().getPosition();

        if(!downhill && (currentCratePos - cratePosition).length() < (prevCratePos - cratePosition).length()){
            std::cout << "Distance from start to standstill: " << (prevCratePos-cratePosition).length() << "m" << std::endl;
            downhill = true;
        }
        if(downhill && (currentCratePos.x > cratePosition.x)){
            std::cout << "Time until again at initial pos: " << elapsedTime << "s" << std::endl;
            break;
        }
        prevCratePos = currentCratePos;
    }

}

void exercise4(){
    
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/120.0f;

    float inclineDegrees = 25.0f;     
    float crateWeight = 1.0f;
    float friction = 0.19f;
    float targetDistance = 8.5f;

    rp3d::Quaternion inclineOrientation(rp3d::Vector3(0, 0, -degToRad(inclineDegrees)));

    //Create crate Object
    rp3d::Vector3 cratePosition(0.0, 0.60, 0.0);
    rp3d::BoxShape crateBoxShape(rp3d::Vector3(0.5, 0.5, 0.5));
    rp3d::Transform crateTransform(cratePosition, inclineOrientation);
    rp3d::RigidBody* crate;
    crate = world.createRigidBody(crateTransform);
    crate->setType(reactphysics3d::BodyType::DYNAMIC);
    crate->addCollisionShape(&crateBoxShape, rp3d::Transform::identity(), crateWeight);

    //Create incline Object
    rp3d::Vector3 inclinePosition(0.0, -1.0, 0.0);
    rp3d::BoxShape inclineBoxShape(rp3d::Vector3(30.0, 1.0, 30.0));
    rp3d::Transform inclineTransform(inclinePosition, inclineOrientation);
    rp3d::RigidBody* incline;
    incline = world.createRigidBody(inclineTransform);
    incline->setType(reactphysics3d::BodyType::STATIC);
    incline->addCollisionShape(&inclineBoxShape, rp3d::Transform::identity(), 1);

    //Set Material of incline
    rp3d::Material& inclineMaterial = incline->getMaterial();
    inclineMaterial.setBounciness(0.0);
    inclineMaterial.setFrictionCoefficient(friction);

    //Set Material of crate
    rp3d::Material& crateMaterial = crate->getMaterial();
    crateMaterial.setBounciness(0.0);
    crateMaterial.setFrictionCoefficient(friction);

    float elapsedTime = 0.0;

    while (true){
        rp3d::Vector3 prevVel = crate->getLinearVelocity();
        world.update(timeStep);
        elapsedTime += timeStep;
        rp3d::Vector3 newVel = crate->getLinearVelocity();
        rp3d::Vector3 acceleration = (newVel - prevVel)/timeStep;
        rp3d::Vector3 cratePos = crate->getTransform().getPosition();
        //This condition should be just as accurate but it doesn't seem that way
        //float deltaX = cratePos.x - cratePosition.x;
        //float deltaY = cratePos.y - cratePosition.y;
        //if ( (deltaX * deltaX + deltaY * deltaY) >= 8.15 * 8.15){
        if((cratePos - cratePosition).length() >= targetDistance){
            std::cout << "Acceleration: " << crate->getLinearVelocity().length()/elapsedTime << "m/s²" << std::endl;
            std::cout << "Speed: " << crate->getLinearVelocity().length() << "m/s" << std::endl;
            break;
        }

    }

}

void exercise5(){
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/120.0f;

    float inclineDegrees = 25.0f;     
    float crateWeight = 1.0f;
    float friction = 0.12f;
    float initSpeed = 3.0f;

    rp3d::Quaternion inclineOrientation(rp3d::Vector3(0, 0, -degToRad(inclineDegrees)));

    //Create crate Object
    rp3d::Vector3 cratePosition(0.0, 0.60, 0.0);
    rp3d::BoxShape crateBoxShape(rp3d::Vector3(0.5, 0.5, 0.5));
    rp3d::Transform crateTransform(cratePosition, inclineOrientation);
    rp3d::RigidBody* crate;
    crate = world.createRigidBody(crateTransform);
    crate->setType(reactphysics3d::BodyType::DYNAMIC);
    crate->addCollisionShape(&crateBoxShape, rp3d::Transform::identity(), crateWeight);

    //Create incline Object
    rp3d::Vector3 inclinePosition(0.0, -1.0, 0.0);
    rp3d::BoxShape inclineBoxShape(rp3d::Vector3(30.0, 1.0, 30.0));
    rp3d::Transform inclineTransform(inclinePosition, inclineOrientation);
    rp3d::RigidBody* incline;
    incline = world.createRigidBody(inclineTransform);
    incline->setType(reactphysics3d::BodyType::STATIC);
    incline->addCollisionShape(&inclineBoxShape, rp3d::Transform::identity(), 1);

    //Set Material of incline
    rp3d::Material& inclineMaterial = incline->getMaterial();
    inclineMaterial.setBounciness(0.0);
    inclineMaterial.setFrictionCoefficient(friction);

    //Set Material of crate
    rp3d::Material& crateMaterial = crate->getMaterial();
    crateMaterial.setBounciness(0.0);
    crateMaterial.setFrictionCoefficient(friction);

    //Set the velocity to go up the incline
    reactphysics3d::Vector3 inclineDirection(inclineOrientation * reactphysics3d::Vector3(1.0, 0.0, 0.0));
    inclineDirection *= -1.0;
    reactphysics3d::Vector3 vel = inclineDirection * initSpeed;
    crate->setLinearVelocity(vel);
    

    float elapsedTime = 0.0;
    reactphysics3d::Vector3 prevCratePos = cratePosition;
    bool downhill = false;

    while (true){
        rp3d::Vector3 prevVel = crate->getLinearVelocity();

        world.update(timeStep);
        elapsedTime += timeStep;
        
        rp3d::Vector3 currentCratePos = crate->getTransform().getPosition();

        if(!downhill && (currentCratePos - cratePosition).length() < (prevCratePos - cratePosition).length()){
            std::cout << "Distance from start to standstill: " << (prevCratePos-cratePosition).length() << "m" << std::endl;
            downhill = true;
        }
        if(downhill && (currentCratePos.x > cratePosition.x)){
            std::cout << "Time until again at initial pos: " << elapsedTime << "s" << std::endl;
            break;
        }
        prevCratePos = currentCratePos;

    }

    
}

int main(){

    std::cout << "First Exercise: Find friction and friction force, but the 'easy' way" << std::endl;
    exercise1();       
    std::cout << std::endl << "Second Exercise: Crate with no friction" << std::endl;
    exercise2();
    std::cout << std::endl << "Third Exercise: Crate goes up and down" << std::endl;
    exercise3();
    std::cout << std::endl << "Fourth Exercise: Another crate with no friction" << std::endl;
    exercise4();
    std::cout << std::endl << "Fifth Exercise: Another crate goes up and down" << std::endl;
    exercise5();

    std::cout << std::endl <<  "END" << std::endl;
    return 0;
}
