#include <iostream>
#include <string>
#include <vector>
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

float TIME_STEP = 1.0f/240.0f;
int WINDOW_WIDTH = 1280;
int WINDOW_HEIGHT = 720;
float PEND_RADIUS = 50;
float FIX_RADIUS = 30;
float BLOCK_WIDTH = 200;
float BLOCK_HEIGHT = 100;

int main(){


    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Hysteresis");

    //initialize simulation
    //Gravity goes in the other direction since I use the coordinate system of SFML
    rp3d::DynamicsWorld world(rp3d::Vector3 (0.0, +9.81, 0.0));
    // 0,0 is in the upper left corner
    //add the block
    rp3d::Vector3 positionBlock(WINDOW_WIDTH/2, 600.0, 0.0);
    rp3d::Quaternion orientBlock = rp3d::Quaternion::identity();
    rp3d::Transform transformBlock(positionBlock, orientBlock);

    rp3d::RigidBody* phBlock;
    phBlock = world.createRigidBody(transformBlock);

    //add the pendulum
    rp3d::Vector3 positionPend(WINDOW_WIDTH/4, 100.0, 0.0);
    rp3d::Transform transformPend(positionPend, orientBlock);
    rp3d::RigidBody* phPendulum;
    phPendulum = world.createRigidBody(transformPend);

    phPendulum->enableGravity(true);
    phBlock->enableGravity(false);
    //95 km/h = 26.3889 m/s
    //block->setLinearVelocity(rp3d::Vector3(26.3889, 0.0, 0.0));
    //75 km/h = 20.8333 m/s
    //pendulum->setLinearVelocity(rp3d::Vector3(20.8333, 0.0, 0.0));
    
    //visualization setup 
    sf::RectangleShape sfBlock(sf::Vector2f(BLOCK_WIDTH, BLOCK_HEIGHT));
    sfBlock.setPosition(positionBlock.x - BLOCK_WIDTH/2, positionBlock.y - BLOCK_HEIGHT/2);

    sf::CircleShape sfPendulum(PEND_RADIUS);
    sfPendulum.setPosition(positionPend.x - PEND_RADIUS, positionPend.y - PEND_RADIUS);

    sf::CircleShape sfPendFix(FIX_RADIUS);
    rp3d::Vector3 phPendFixPos(WINDOW_WIDTH/2, 50.0, 0.0);
    sfPendFix.setPosition(phPendFixPos.x - FIX_RADIUS, phPendFixPos.y - FIX_RADIUS);
    


    while (window.isOpen())
    {
        sf::Event event;
        while(window.pollEvent(event))
        {
            if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Escape 
                        || event.key.code == sf::Keyboard::Q)){
                window.close();
            }

        }


        //-------Simulation--------------
        world.update(TIME_STEP);

        rp3d::Transform simBlockTransform = phBlock->getTransform();
        rp3d::Vector3 simBlockPos = simBlockTransform.getPosition();

        rp3d::Transform simPendulumTransform = phPendulum->getTransform();
        rp3d::Vector3 simPendulumPos = simPendulumTransform.getPosition();


        sf::Vertex line[] ={
            sf::Vertex(sf::Vector2f(phPendFixPos.x, phPendFixPos.y)),
            sf::Vertex(sf::Vector2f(simPendulumPos.x + PEND_RADIUS, simPendulumPos.y + PEND_RADIUS))
        };

        sfPendulum.setPosition(simPendulumPos.x, simPendulumPos.y);

        window.clear();
        
        window.draw(line, 2, sf::Lines);
        window.draw(sfBlock);
        window.draw(sfPendulum);
        window.draw(sfPendFix);


        window.display();

    }

    //std::cout << "First Exercise: Car vs. Train, The Reckoning" << std::endl;
    //exercise1();       
        
    //std::cout << std::endl <<  "END" << std::endl;
    return 0;
}
