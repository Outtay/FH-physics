#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include "reactphysics3d.h"

float Hysteresis(float x, bool rightSide, bool goingRight, int index);
float transformDistance(float currentDistance, float maxDistance, int index); 

float TIME_STEP = 1.0f/240.0f;
int WINDOW_WIDTH = 1280;
int WINDOW_HEIGHT = 720;
float PEND_RADIUS = 50;
float FIX_RADIUS = 30;
float BLOCK_WIDTH = 200;
float BLOCK_HEIGHT = 100;

float hysteresisMult = 5.0;
//I'm having the problem, that when the attraction force increases the speed towards the middle,
//due to the higher speed, there is less of a decrease in speed (which makes the pendulum go faster)
//This might be the correct physical behaviour, which is why I need to lessen the strength, for it to not go overboard
//For having an interesting almost more intuitively looking pendulum, the strength should be negative.
float magneticStrength = 1.0;

int main(){


    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Hysteresis");

    //initialize simulation
    //Gravity goes in the other direction since I use the coordinate system of SFML
    rp3d::DynamicsWorld world(rp3d::Vector3 (0.0, +2.81, 0.0));
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
    phBlock->enableGravity(true);
    
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

    rp3d::Vector3 forceApplyTest(-2, 2, 0);
    rp3d::Vector3 torqueApplyTest(0,0,1);

    
    float forceConst = 0.8;
    float torqueConst = 5.0;


    bool goingRight = true;
    int currentHysteresisIndex = 0;
    
    rp3d::Vector3 lowestPendulumPoint = phAnchorPos + rp3d::Vector3(0,1,0) * (phAnchorPos - positionPend).length();
    float maxDistanceToLow = (positionPend - lowestPendulumPoint).length();
    float updatedMaxDistance = maxDistanceToLow;

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


        //phPendulum->applyForceToCenterOfMass(forceApplyTest * forceConstTest);
        
        //-------Simulation--------------
        world.update(TIME_STEP);


        rp3d::Transform simBlockTransform = phBlock->getTransform();
        rp3d::Vector3 simBlockPos = simBlockTransform.getPosition();

        rp3d::Transform simPendulumTransform = phPendulum->getTransform();
        rp3d::Vector3 simPendulumPos = simPendulumTransform.getPosition();


        rp3d::Vector3 pendMagnetismDir = (phAnchorPos - simPendulumPos);
        pendMagnetismDir.normalize();
        rp3d::Vector3 dirLinePoint = simPendulumPos + pendMagnetismDir * 100;
        float distanceToLow = (simPendulumPos - lowestPendulumPoint).length();
        
        //if the signbit is set, then the pendulum is on the right side of the center
        bool rightSide = std::signbit(rightVector.dot(pendMagnetismDir));
        
        bool newGoingRight = lastPosition.x < simPendulumPos.x;
        if (newGoingRight != goingRight){ 
            if (rightSide){
                //mostRightMagnetismVector = 0.5 * (phAnchorPos - positionPend) + 0.5 * (phAnchorPos - simPendulumPos);
                //mostRightMagnetismVector = 0.5 * mostRightMagnetismVector + 0.5 * (phAnchorPos - simPendulumPos);
                mostRightMagnetismVector = magnetismVector;
                //mostRightMagnetismVector = 0.9 * magnetismVector + 0.1 * (phAnchorPos - simPendulumPos);
                mostRightMagnetismVector.normalize();
                mostLeftMagnetismVector.x = -mostRightMagnetismVector.x;
                mostLeftMagnetismVector.y = mostRightMagnetismVector.y;
            } else {
                //mostLeftMagnetismVector = 0.5 * (phAnchorPos - flippedPosPend) + 0.5 *(phAnchorPos - simPendulumPos);
                //mostLeftMagnetismVector = 0.5 * mostLeftMagnetismVector + 0.5 *(phAnchorPos - simPendulumPos);
                //mostLeftMagnetismVector = 0.9 * magnetismVector + 0.1 * (phAnchorPos - simPendulumPos);
                mostLeftMagnetismVector = magnetismVector;
                mostLeftMagnetismVector.normalize();
                mostRightMagnetismVector.x = -mostLeftMagnetismVector.x;
                mostRightMagnetismVector.y = mostLeftMagnetismVector.y;
            }
            updatedMaxDistance = (simPendulumPos - lowestPendulumPoint).length();
        }
        goingRight = newGoingRight;
        //std::cout << goingRight << std::endl;
        lastPosition = simPendulumPos;

        if(maxDistanceToLow * 0.8 < updatedMaxDistance){
            currentHysteresisIndex = 0;
        } else if (maxDistanceToLow * 0.6 < updatedMaxDistance){
            currentHysteresisIndex = 1;
        } else if (maxDistanceToLow  * 0.4 < updatedMaxDistance){
            currentHysteresisIndex = 2;
        } else if (maxDistanceToLow  * 0.2 < updatedMaxDistance){
            currentHysteresisIndex = 3;
        } else {
            currentHysteresisIndex = 4;
        }


        float transformedDistanceLag = transformDistance(distanceToLow, updatedMaxDistance, 0);
        
        //this makes intuitive sense and it should be approx correct:
        //use the hysteresis to get a vector that's lagging behind
        //float hystValueLag = Hysteresis(transformedDistanceLag, rightSide, goingRight, 0);
        float hystValueLag = Hysteresis(transformedDistanceLag, rightSide, goingRight, 0);
        
        //I also use the hysteresis for the force, for that I use different hysteresis curves based on the last maxDistance
        float transformedDistanceForce = transformDistance(distanceToLow, updatedMaxDistance, currentHysteresisIndex);
        float hystValueForce = Hysteresis(transformedDistanceForce, rightSide, goingRight, currentHysteresisIndex);

        //simulate a radius by reducing a constant amount, this makes sure that we're very close to 0 when at the lowest pendulum point 
        //float simulatedRadius = (lowestPendulumPoint - positionBlock).length() - 0.2f;
        //float distToBlockInvSquare = 1/(std::pow((simPendulumPos - positionBlock).length() - simulatedRadius , 2));
        float transformedDistanceToLow = transformDistance((simPendulumPos - lowestPendulumPoint).length(), maxDistanceToLow, 5);
        float distToBlockInvSquare = 1/(std::pow(transformedDistanceToLow + 0.1, 2));

        rp3d::Vector3 torqueForce(rp3d::Vector3(0,0, hystValueForce) * torqueConst * distToBlockInvSquare);
        if (!rightSide){
            magnetismVector = ((1.0f - hystValueLag) * mostLeftMagnetismVector + hystValueLag * pendMagnetismDir);
            phPendulum->applyTorque(torqueForce);
        }
        else {
            magnetismVector = ((1.0f - hystValueLag) * mostRightMagnetismVector + hystValueLag * pendMagnetismDir);
            torqueForce *= -1;
            phPendulum->applyTorque(torqueForce);
        }
        magnetismVector.normalize();


        rp3d::Vector3 attractionForce (rp3d::Vector3(-magnetismVector.x, -magnetismVector.y, 0) * distToBlockInvSquare * forceConst * hystValueForce);
        phPendulum->applyForceToCenterOfMass(attractionForce);
            
        //std::cout << hystValueForce << std::endl;
        //std::cout << distToBlockInvSquare << std::endl;
        //std::cout << "hystValue: " << hystValue << ", " << "DistToBlock: " << distToBlockInvSquare << std::endl;
        //std::cout << attractionForce.x << ", " << attractionForce.y << ", " << attractionForce.z << " : attraction" << std::endl; 
        //std::cout << torqueForce.x << ", " << torqueForce.y << ", " << torqueForce.z << " : torque" << std::endl; 
        //std::cout << magnetismVector.x << ", " << magnetismVector.y << ", " << magnetismVector.z << " : magnetism" << std::endl; 
        //std::cout << hystValue << std::endl;
        //std::cout << magnetismVector.length() << std::endl;
        //std::cout << distanceToLow << std::endl;

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
        //rp3d::Vector3 test = phPendulum->getAngularVelocity();
        //rp3d::Vector3 test (rp3d::Vector3(0,0, hystValueLag) * torqueConst);
        //std::cout << test.x << ", " << test.y << ", " << test.z << std::endl; 

        
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

//TODO change the range to be variable based on the last max distance... probably do this in the function as 
//it depends on the function that is used.
//0 to 4
//0.4 - 3.6
//0.9 - 3.1
//1.6 - 2.3
float transformDistance(float currentDistance, float maxDistance, int index){
    
    float upper = 0.0;
    float lower = 0.0;
    switch(index){
        case 1:
            lower = 0.4;
            upper = 3.6;
            break;
        case 2:
            lower = 0.9;
            upper = 3.1;
            break;
        case 3:
            lower = 1.6;
            upper = 2.3;
            break;
        case 4:
            return 0.0;
            break;
        case 0:
            lower = 0.0;
            upper = 4.0;
            break;
        case 5:
            //only used for distance transformation due to square distance magnetic strength decrease
            lower = 0.0;
            upper = 2.0;
        default:
            break;
    }
    float result = (upper-lower)*(currentDistance - lower)/maxDistance;

    return result;

}
//how do you implement that there's a new 0? With linear interpolation you would still never really 
//lag behind later as even 0.1 would get you somewhere around 0.5 which isn't really that great.

//with this modified sigmoid function, the range of my x-values is [0,4]
//Assume x is the distance between current and lowestPoint
float Hysteresis(float x, bool rightSide, bool goingRight, int index){


    //http://fooplot.com/#W3sidHlwZSI6MCwiZXEiOiIoMS8oMStlXigtMyp4KzUpKSkiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjAsImVxIjoiKDEvKDErZV4oLTMqeCs3KSkpIiwiY29sb3IiOiIjMDAwMDAwIn0seyJ0eXBlIjowLCJlcSI6IigxLygxK2VeKC0zKih4LzAuOC0wLjU1KSs1KSkpKjAuOCswLjEiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjAsImVxIjoiKDEvKDErZV4oLTMqKHgvMC44LTAuNDUpKzcpKSkqMC44KzAuMSIsImNvbG9yIjoiIzAwMDAwMCJ9LHsidHlwZSI6MCwiZXEiOiIoMS8oMStlXigtMyooeC8wLjYtMS41KSs1KSkpKjAuNiswLjIiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjAsImVxIjoiKDEvKDErZV4oLTMqKHgvMC42LTEuMSkrNykpKSowLjYrMC4yIiwiY29sb3IiOiIjMDAwMDAwIn0seyJ0eXBlIjowLCJlcSI6IigxLygxK2VeKC0zKih4LzAuMi04KSs1KSkpKjAuMiswLjQiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjAsImVxIjoiKDEvKDErZV4oLTMqKHgvMC4yLTcuNykrNykpKSowLjIrMC40IiwiY29sb3IiOiIjMDAwMDAwIn0seyJ0eXBlIjoxMDAwLCJ3aW5kb3ciOlsiMC40NTE5OTIzODE3MzQ5MTYxIiwiMy44NTk4NjQzODE3MzQ5MDc2IiwiLTAuNTgxOTIyNzg3MDMzMDg1NCIsIjEuNTE1MjI5MjEyOTY2OTExNCJdLCJncmlkIjpbIiIsIjAuMSJdfV0-
    
    float x_modifierDiv = 1.0;
    float afterFunctionModifierFactor = 1.0;
    float x_modifierSumCharge = 0.0;
    float x_modifierSumDischarge = 0.0;
    float afterFunctionModifierSum = 0.0;
    switch(index){
        case 1:
            x_modifierDiv = 0.8;
            x_modifierSumDischarge = -0.55;
            x_modifierSumCharge = -0.45;
            afterFunctionModifierFactor = 0.8;
            afterFunctionModifierSum = 0.1;
            break;
        case 2:
            x_modifierDiv = 0.6;
            x_modifierSumDischarge = -1.5;
            x_modifierSumCharge = -1.1;
            afterFunctionModifierFactor = 0.6;
            afterFunctionModifierSum = 0.2;
            break;
        case 3:
            x_modifierDiv = 0.2;
            x_modifierSumDischarge = -8.0;
            x_modifierSumCharge = -7.7;
            afterFunctionModifierFactor = 0.2;
            afterFunctionModifierSum = 0.4;
            break;
        case 4: 
            return 0.5;
        case 0:
            break;
        default:
            break;
    }


    float result;

    //original form of the functions:
    //1/(1+std::exp(-3*x + 5))
    //1/(1+std::exp(-3*x + 7))

    if (rightSide && goingRight){
        //discharging
        x = 4 - x;
        
        result = (1/(1+std::exp(-3*(x / x_modifierDiv - x_modifierSumDischarge) + 5))) * afterFunctionModifierFactor + afterFunctionModifierSum;
    } else if (rightSide && !goingRight) {
        //Charging
        //x is the biggest when the pendulum is far away
        //therefore I need to invert x when charging, which should be obvious when looking
        //at the beginning of the simulation
        x = 4 - x;
        result = (1/(1+std::exp(-3*(x / x_modifierDiv - x_modifierSumDischarge) + 7))) * afterFunctionModifierFactor + afterFunctionModifierSum;
    } else if (!rightSide && goingRight){
        //Charging
        //std::cout << "test" << std::endl;
        x = 4 - x;
        result = (1/(1+std::exp(-3*(x / x_modifierDiv - x_modifierSumDischarge) + 7))) * afterFunctionModifierFactor + afterFunctionModifierSum;
    } else if (!rightSide && !goingRight){
        //Discharging
        //std::cout << "test" << std::endl;
        x = 4 - x;
        result = (1/(1+std::exp(-3*(x / x_modifierDiv - x_modifierSumDischarge) + 5))) * afterFunctionModifierFactor + afterFunctionModifierSum;
    }
    return std::max(0.0f, result);

}
