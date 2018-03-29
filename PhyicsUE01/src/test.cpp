#include <iostream>
#include "reactphysics3d.h"
#include <chrono>
#include <ctime>

void exercise1(){
    //Setup
    rp3d::DynamicsWorld world(rp3d::Vector3 (0.0, 0.0, 0.0));

    const float timeStep = 1.0 / 60.0;

    //add the car
    rp3d::Vector3 position(0.0, 0.0, 0.0);
    rp3d::Quaternion orient = rp3d::Quaternion::identity();
    rp3d::Transform transform(position, orient);

    rp3d::RigidBody* car;
    car = world.createRigidBody(transform);

    //add the train
    rp3d::Vector3 position1(1300.0, 0.0, 0.0);
    rp3d::Transform transform1(position1, orient);
    rp3d::RigidBody* train;
    train = world.createRigidBody(transform1);

    car->enableGravity(false);
    train->enableGravity(false);
    //95 km/h = 26.3889 m/s
    car->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));
    //75 km/h = 20.8333 m/s
    train->setLinearVelocity(rp3d::Vector3(20.8333, 0.0, 0.0));

    float timeSpent = 0.0;
    while (true) {
        //  Update  the  Dynamics  world  with a constant  time  step
        world.update(timeStep);
        timeSpent += timeStep;
        rp3d::Transform carTransform = car->getTransform();
        rp3d::Vector3 carPos = carTransform.getPosition();

        rp3d::Transform trainTransform = train->getTransform();
        rp3d::Vector3 trainPos = trainTransform.getPosition();

        if (carPos.x >= trainPos.x){
            std::cout << "Same Direction:" << std::endl;
            std::cout << "Car Position: " << carPos.x << ", Train Position: " << trainPos.x << " | Time: " << timeSpent << std::endl;
            break;
        }
    }

    //second exercise
    world.destroyRigidBody(car);
    world.destroyRigidBody(train);
    car = world.createRigidBody(transform);
    train = world.createRigidBody(transform1);
    
    car->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));

    train->setLinearVelocity(rp3d::Vector3(-20.833, 0.0, 0.0));

    timeSpent = 0.0;
    while (true){
        world.update(timeStep);
        timeSpent += timeStep;
        
        rp3d::Transform carTransform = car->getTransform();
        rp3d::Vector3 carPos = carTransform.getPosition();

        rp3d::Transform trainTransform = train->getTransform();
        rp3d::Vector3 trainPos = trainTransform.getPosition();

        if (carPos.x >= trainPos.x){
            std::cout << "Opposite Directions:" << std::endl;
            std::cout << "Car Position: " << carPos.x << ", Train Position: " << trainPos.x <<  " | Time: " << timeSpent << std::endl;
            break;
        }
    

    }



}

void exercise2(){
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/60.0;
    const float epsilon = 0.001;

    //add the water droplet
    rp3d::Vector3 position(0.0, 1.8 , 0.0);
    rp3d::Transform transform(position, rp3d::Quaternion::identity());
    
    rp3d::RigidBody* droplet;
    droplet = world.createRigidBody(transform);
    rp3d::Vector3 initialSpeed (0.0, 500.01, 0.0);
    droplet->applyForceToCenterOfMass(initialSpeed);
    droplet->enableGravity(true);
    droplet->setMass(1.0);
    //droplet->setLinearVelocity(initialSpeed);
    

    float timeSpent = 0.0;
    int i = 0;
    while (true){
        i++;
        while (timeSpent < 2.5){
            world.update(timeStep);
            //prints info that shouldn't be like that
            //std::cout << droplet->getTransform().getPosition().y << std::endl;
            timeSpent += timeStep;
        }
        rp3d::Transform dropTransform = droplet->getTransform();
        rp3d::Vector3 dropPos = dropTransform.getPosition();
        if (dropPos.y - epsilon <= 0.0 && dropPos.y + epsilon >= 0.0){
            std::cout << "finished: " << timeSpent << std::endl;
            break;
        }
        else {
            std::cout << "not yet finished: " << initialSpeed.y << " Pos : " << droplet->getTransform().getPosition().y << std::endl;
            timeSpent = 0.0;
            //droplet->setLinearVelocity(droplet->getLinearVelocity() + rp3d::Vector3(0.0, 0.01, 0.0));
            initialSpeed += rp3d::Vector3(0.0, 0.10, 0.0);
            world.destroyRigidBody(droplet);
            droplet = world.createRigidBody(transform);
            droplet->applyForceToCenterOfMass(initialSpeed);
            droplet->setTransform(transform);
        }
        
    }

}
void exercise3(){
    

}

int main(){

    exercise1();       
    //exercise2();
    //exercise3();
        
    std::cout << "END" << std::endl;
    return 0;
}
