#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <SFML/Graphics.hpp>
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
float Hysteresis(float x, bool rightSide, bool goingRight);

float TIME_STEP = 1.0f/240.0f;
int WINDOW_WIDTH = 1280;
int WINDOW_HEIGHT = 720;
float PEND_RADIUS = 50;
float FIX_RADIUS = 30;
float BLOCK_WIDTH = 200;
float BLOCK_HEIGHT = 100;

float hysteresisMult = 5.0;
float magneticStrength = 10;

int main(){


    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Hysteresis");

    //initialize simulation
    //Gravity goes in the other direction since I use the coordinate system of SFML
    rp3d::DynamicsWorld world(rp3d::Vector3 (0.0, +0.01, 0.0));
    // 0,0 is in the upper left corner
    //add the block
    rp3d::Vector3 positionBlock(WINDOW_WIDTH/2, 600.0, 0.0);
    rp3d::Quaternion orientBlock = rp3d::Quaternion::identity();
    rp3d::Transform transformBlock(positionBlock, orientBlock);

    rp3d::RigidBody* phBlock;
    phBlock = world.createRigidBody(transformBlock);

    //add the pendulum
    rp3d::Vector3 positionPend(WINDOW_WIDTH/4-100, 150.0, 0.0);
    rp3d::Quaternion orientPend = rp3d::Quaternion::identity();
    rp3d::Transform transformPend(positionPend, orientPend);
    rp3d::RigidBody* phPendulum;
    phPendulum = world.createRigidBody(transformPend);

    phPendulum->enableGravity(true);
    phPendulum->setInertiaTensorLocal(rp3d::Matrix3x3::identity());
    phBlock->enableGravity(false);
    //95 km/h = 26.3889 m/s
    //block->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));
    //75 km/h = 20.8333 m/s
    //pendulum->setLinearVelocity(rp3d::Vector3(20.8333, 0.0, 0.0));
    
    //visualization setup 
    sf::RectangleShape sfBlock(sf::Vector2f(BLOCK_WIDTH, BLOCK_HEIGHT));
    sfBlock.setPosition(positionBlock.x - BLOCK_WIDTH/2, positionBlock.y - BLOCK_HEIGHT/2);

    sf::CircleShape sfPendulum(PEND_RADIUS);
    //sfPendulum.setPosition(positionPend.x - PEND_RADIUS, positionPend.y - PEND_RADIUS);
    sfPendulum.setPosition(positionPend.x, positionPend.y);
    sfPendulum.setOrigin(PEND_RADIUS, PEND_RADIUS);

    sf::CircleShape sfAnchor(FIX_RADIUS);
    rp3d::Vector3 phAnchorPos(WINDOW_WIDTH/2, 50.0, 0.0);
    //sfPendFix.setPosition(phPendFixPos.x - FIX_RADIUS, phPendFixPos.y - FIX_RADIUS);
    sfAnchor.setPosition(phAnchorPos.x, phAnchorPos.y);
    sfAnchor.setOrigin(FIX_RADIUS, FIX_RADIUS);
    
    rp3d::RigidBody * phAnchor;
    rp3d::Transform anchorTransform(phAnchorPos, orientBlock);
    phAnchor = world.createRigidBody(anchorTransform);
    phAnchor->setType(rp3d::BodyType::STATIC);
    rp3d::HingeJointInfo hingeInfo (phAnchor, phPendulum, phAnchorPos, rp3d::Vector3(0,0,1));
    rp3d::HingeJoint * hingeJoint;
    hingeJoint = static_cast<rp3d::HingeJoint*>(world.createJoint(hingeInfo));
    //hingeJoint.enableLimit(true);
    //hingeJoint.enableMotor(true);

    std::cout << sfPendulum.getOrigin().x << ", " << sfPendulum.getOrigin().y << std::endl;// << ", " << simPendulumPos.z << std::endl;

    //initialize the magnetism vector parallel (so with strongest attraction)
    //The force will be an extra var
    rp3d::Vector3 magnetismVector = phAnchorPos - positionPend;
    magnetismVector.normalize();
    //magnetismVector /= 100;
    rp3d::Vector3 mostLeftMagnetismVector = phAnchorPos - positionPend;
    mostLeftMagnetismVector.normalize();
    rp3d::Vector3 mostRightMagnetismVector (-mostLeftMagnetismVector.x , mostLeftMagnetismVector.y, 0);
    
    //use this to determine if the magnet is charging or discharging
    rp3d::Vector3 rightVector (1,0,0);
    rp3d::Vector3 lastPosition = positionPend;

    rp3d::Vector3 forceApplyTest(0.5, 2, 0);
    float forceConstTest = 2.0;

    while (window.isOpen())
    {
        sf::Event event;
        while(window.pollEvent(event))
        {
            if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Escape 
                        || event.key.code == sf::Keyboard::Q)){
                window.close();
            }
            if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Return 
                        || event.key.code == sf::Keyboard::Space)){
                
            }

        }


        phPendulum->applyTorque(forceApplyTest * forceConstTest);

        //-------Simulation--------------
        world.update(TIME_STEP);


        rp3d::Transform simBlockTransform = phBlock->getTransform();
        rp3d::Vector3 simBlockPos = simBlockTransform.getPosition();

        rp3d::Transform simPendulumTransform = phPendulum->getTransform();
        rp3d::Vector3 simPendulumPos = simPendulumTransform.getPosition();

        bool goingRight = lastPosition.x < simPendulumPos.x;
        lastPosition = simPendulumPos;

        rp3d::Vector3 pendMagnetismDir = (phAnchorPos - simPendulumPos);
        pendMagnetismDir.normalize();
        rp3d::Vector3 dirLinePoint = simPendulumPos + pendMagnetismDir * 100;
        rp3d::Vector3 lowestPendulumPoint = phAnchorPos + rp3d::Vector3(0,1,0) * (phAnchorPos - positionPend).length();
        float distanceToLow = (simPendulumPos - lowestPendulumPoint).length();
        float maxDistanceToLow = (positionPend - lowestPendulumPoint).length();
        //Our function needs parameters in the range of [0,4] so we scale it
        float transformedDistance = (4-0)*(distanceToLow - 0)/maxDistanceToLow;
        //if the signbit is set, then the pendulum is on the right side of the center
        bool rightSide = std::signbit(rightVector.dot(pendMagnetismDir));

        //this makes intuitive sense and it should be approx correct:
        //use the hysteresis to get a vector that's lagging behind
        float hystValue = Hysteresis(transformedDistance, rightSide, goingRight);
        if (!rightSide)
            magnetismVector = ((1.0f - hystValue) * mostLeftMagnetismVector + hystValue * pendMagnetismDir);
        else 
            magnetismVector = ((1.0f - hystValue) * mostRightMagnetismVector + hystValue * pendMagnetismDir);

        rp3d::Vector3 magneticLinePoint = positionBlock + magnetismVector * 100;

        

        sf::Vertex pendString[] ={
            sf::Vertex(sf::Vector2f(phAnchorPos.x, phAnchorPos.y)),
            sf::Vertex(sf::Vector2f(simPendulumPos.x, simPendulumPos.y))
        };
        sf::Vertex pendMagnetismDirLine[] = {
            sf::Vertex(sf::Vector2f (simPendulumPos.x, simPendulumPos.y) ),
            sf::Vertex(sf::Vector2f (dirLinePoint.x, dirLinePoint.y  ) )
        };
        pendMagnetismDirLine[0].color = sf::Color::Red;
        pendMagnetismDirLine[1].color = sf::Color::Red;
        sf::Vertex blockMagnetismDirLine[] = {
            sf::Vertex(sf::Vector2f (positionBlock.x, positionBlock.y) ),
            sf::Vertex(sf::Vector2f (magneticLinePoint.x, magneticLinePoint.y  ) )
        };
        blockMagnetismDirLine[0].color = sf::Color::Red;
        blockMagnetismDirLine[1].color = sf::Color::Red;

        //std::cout << (phPendFixPos - simPendulumPos).length() << std::endl;
        //std::cout << (simPendulumPos - lowestPendulumPoint).length() << std::endl;
        //std::cout << rightSide << std::endl;
        //std::cout << hystValue << std::endl;
        
        sfPendulum.setPosition(simPendulumPos.x, simPendulumPos.y);

        window.clear();
        
        window.draw(sfBlock);
        window.draw(sfPendulum);
        window.draw(sfAnchor);
        
        window.draw(pendString, 2, sf::Lines);
        window.draw(pendMagnetismDirLine, 2, sf::Lines);
        window.draw(blockMagnetismDirLine, 2, sf::Lines);

        window.display();

    }

    //std::cout << "First Exercise: Car vs. Train, The Reckoning" << std::endl;
    //exercise1();       
        
    //std::cout << std::endl <<  "END" << std::endl;
    return 0;
}

//with this modified sigmoid function, the range of my x-values is [0,4]
//Assume x is the distance between current and lowestPoint
float Hysteresis(float x, bool rightSide, bool goingRight){
    float result;
    if (rightSide && goingRight){
        //discharging

        //TODO depending on what x might be I may have to invert it
        //if x is the distance, this is not the case because then I just need to
        //decide whether to add or subtract x from the resulting force.
        //But I probably still have to do something about it.... have to think about it
        //
        x = 4 - x;
        result = 1/(1+std::exp(-3*x + 5));
    } else if (rightSide && !goingRight) {
        //Charging
        //x is the biggest when the pendulum is far away
        //therefore I need to invert x when charging, which should be obvious when looking
        //at the beginning of the simulation
        x = 4 - x;
        result = 1/(1+std::exp(-3*x + 7));
    } else if (!rightSide && goingRight){
        //Charging
        //std::cout << "test" << std::endl;
        x = 4 - x;
        result = 1/(1+std::exp(-3*x + 7)); 
    } else if (!rightSide && !goingRight){
        //Discharging
        //std::cout << "test" << std::endl;
        x = 4 - x;
        result = 1/(1+std::exp(-3*x + 5));
    }
    return std::max(0.0f, result);

}
