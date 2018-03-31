#include <iostream>
#include "reactphysics3d.h"

//car catching up with a train going in the same and then also the opposite direction
void exercise1(){
    //Setup
    rp3d::DynamicsWorld world(rp3d::Vector3 (0.0, -9.81, 0.0));

    const float timeStep = 1.0 / 60.0;

    //add the car
    rp3d::Vector3 positionCar(0.0, 0.0, 0.0);
    rp3d::Quaternion orientCar = rp3d::Quaternion::identity();
    rp3d::Transform transformCar(positionCar, orientCar);

    rp3d::RigidBody* car;
    car = world.createRigidBody(transformCar);

    //add the train
    rp3d::Vector3 positionTrain(1300.0, 0.0, 0.0);
    rp3d::Transform transformTrain(positionTrain, orientCar);
    rp3d::RigidBody* train;
    train = world.createRigidBody(transformTrain);

    car->enableGravity(false);
    train->enableGravity(false);
    //95 km/h = 26.3889 m/s
    car->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));
    //75 km/h = 20.8333 m/s
    train->setLinearVelocity(rp3d::Vector3(20.8333, 0.0, 0.0));

    float timeSpent = 0.0;
    while (true) {

        world.update(timeStep);
        timeSpent += timeStep;
        rp3d::Transform carTransform = car->getTransform();
        rp3d::Vector3 carPos = carTransform.getPosition();

        rp3d::Transform trainTransform = train->getTransform();
        rp3d::Vector3 trainPos = trainTransform.getPosition();
        
        if (carPos.x >= trainPos.x){
            std::cout << "Same Direction:" << std::endl;
            std::cout << "Car Position: " << carPos.x << "m, Train Position: " << trainPos.x << "m | Time: " << timeSpent << std::endl;
            break;
        }
    }

    //second part of the exercise
    world.destroyRigidBody(car);
    world.destroyRigidBody(train);
    car = world.createRigidBody(transformCar);
    train = world.createRigidBody(transformTrain);
    
    //set the velocity of one of them to be negative
    car->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));
    train->setLinearVelocity(rp3d::Vector3(-20.833, 0.0, 0.0));
    car->enableGravity(false);
    train->enableGravity(false);

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
            std::cout << "Car Position: " << carPos.x << "m, Train Position: " << trainPos.x <<  "m | Time: " << timeSpent << std::endl;
            break;
        }
    

    }

}


//finding the velocity of a water droplet shot straight up into the air
void exercise2(){
    rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/200.0;
    const float epsilon = 0.01;

    //add the water droplet
    rp3d::Vector3 position(0.0, 1.8 , 0.0);
    rp3d::Transform transform(position, rp3d::Quaternion::identity());
    
    rp3d::RigidBody* droplet;
    droplet = world.createRigidBody(transform);
    rp3d::Vector3 initialSpeed (0.0, 0.01, 0.0);
    droplet->enableGravity(true);
    droplet->setMass(1.0);
    droplet->setLinearVelocity(initialSpeed);
    

    float timeSpent = 0.0;
    while (true){
        while (timeSpent < 2.5){
            world.update(timeStep);
            timeSpent += timeStep;
        }
        rp3d::Transform dropTransform = droplet->getTransform();
        rp3d::Vector3 dropPos = dropTransform.getPosition();
        if (dropPos.y - epsilon <= 0.0 && dropPos.y + epsilon >= 0.0){
            std::cout << "Using a time step of: " << timeStep << "s , Initial Velocity: " << initialSpeed.y << " m/s" << std::endl;
            break;
        }
        else {
            timeSpent = 0.0;
            initialSpeed += rp3d::Vector3(0.0, 0.001, 0.0);
            //not sure why but only seems to work when I destroy the rigidBodies
            world.destroyRigidBody(droplet);
            droplet = world.createRigidBody(transform);
            droplet->setLinearVelocity(initialSpeed);
            //droplet->applyForceToCenterOfMass(initialSpeed);
            droplet->setTransform(transform);
        }
        
    }

}

//finding out how far a package travels after being thrown out of a plane
void exercise3(){
     rp3d::DynamicsWorld world(rp3d::Vector3(0.0, -9.81, 0.0));

    const float timeStep = 1.0/200.0;
    const float epsilon = 0.1;

    //add the package
    rp3d::Vector3 position(0.0, 235.0 , 0.0);
    rp3d::Transform transform(position, rp3d::Quaternion::identity());
    
    rp3d::RigidBody* package;
    package = world.createRigidBody(transform);
    rp3d::Vector3 initialSpeed (69.4, 0.0, 0.0);
    package->enableGravity(true);
    package->setMass(1.0);
    package->setLinearVelocity(initialSpeed);
    
    //Now let the package fall with the plane's speed and see where it lands
    float timeSpent = 0.0;
    while (true){
        world.update(timeStep);
        timeSpent += timeStep;
        rp3d::Transform packTransform = package->getTransform();
        rp3d::Vector3 packPos = packTransform.getPosition();
        if (packPos.y - epsilon <= 0.0 && packPos.y + epsilon >= 0.0){
            std::cout << "Package travelled: " << packPos.x << "m" << std::endl;
            break;
        }
                
    }


}

int main(){

    std::cout << "First Exercise: Car vs. Train, The Reckoning" << std::endl;
    exercise1();       
    std::cout << std::endl << "Second Exercise: Why are the water droplets not hitting the hose if I'm shooting straight up?" << std::endl;
    exercise2();
    std::cout << std::endl << "Third Exercise: Training to throw things exactly onto civilians' heads, without any repercussions." << std::endl;
    exercise3();
        
    std::cout << std::endl <<  "END" << std::endl;
    return 0;
}
