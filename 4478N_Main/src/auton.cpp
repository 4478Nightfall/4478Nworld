#include "distanceOdom.h"
#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "pros/adi.h"
#include "sensor.h"
#include "moveFunctions.h"
#include "autonSelector.h"
#include "opticalAlign.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>
using namespace pros;
using namespace lemlib;

// ---------- auton routines ----------

void EndControlZone(){
    hood.set_value(HIGH);
    
   chassis.setPose(-24,47,270); 
    delay(2000);
    chassis.moveToPose(-64,14,180,3000,{.lead=0.5},false);
    handleMLMech();
    drivePID(100,100,600);
    handleMLMech();
    drivePID(60,100,3000);
    drivePID(-100,30,2000);

    
}
void parkSkills(){
    hood.set_value(HIGH);
    handleDescore();
    chassis.setPose(0,0,0);
    spinIntake(1);
    drivePID(30.5,100,1300);
       handleMLMech();

    chassis.turnToHeading(270,1000,{},false);
    drivePID(30,30,1700);
    drivePID(-1,2,300);
    resetOdom(false,false,false,true);
    delay(100);
    chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(1500);
    chassis.turnToHeading(230,800);
    chassis.moveToPose(-22,61,270,1000,{.forwards = false, .lead=0.3},false);
    handleMLMech();
    chassis.moveToPose(41,59,270,2000,{.forwards = false, .lead=0.1},false);
    chassis.turnToHeading(180,1000,{},false);
    drivePID(5,100,600);
    chassis.turnToHeading(90,1000,{},false);
    resetOdom(true,false,true,false);
    delay(100);
    chassis.moveToPose(24,47.5,90,2000,{.forwards = false,.lead=0.1,.minSpeed=60},false);
    
    handleHood();
    scoreLongFar(1);
    delay(2000);
    chassis.setPose(25,48,chassis.getPose().theta);  
              turnToHeadingSmart(90,1000);
              handleMLMech();
              handleHood();
              chassis.moveToPose(62,47.5,90,2000,{.lead = 0.1, .maxSpeed = 70},false);
                drivePID(20,35,1000);
                    drivePID(-1,2,300);

                resetOdom(false,false,true,false);
                delay(200);
                chassis.setPose(62,chassis.getPose().y,chassis.getPose().theta);
                delay(1500);
                chassis.moveToPose(26,47.5,90,2000,{.forwards = false,.lead=0.1,.minSpeed=100},false);
                handleHood();
                scoreLongFar(200);
                delay(2000);
                drivePID(10,100,500);
                chassis.turnToHeading(180,700);
                drivePID(10,100,500);
                resetOdom(false,true,true,false);
                handleHood();
                chassis.moveToPose(46,-50,180,3000,{.lead=0.01},false);
                chassis.turnToHeading(90,700);
                drivePID(30,40,1500);
                    drivePID(-1,2,300);

                resetOdom(false,false,false,true);
                delay(100);
                chassis.setPose(63,chassis.getPose().y,chassis.getPose().theta);
                delay(1500);
                chassis.turnToHeading(50,800);
    chassis.moveToPose(22,-63,90,1000,{.forwards = false, .lead=0.3},false);
    handleMLMech();
    chassis.moveToPose(-41,-59,90,2000,{.forwards = false, .lead=0.1},false);
     chassis.turnToHeading(0,1000,{},false);
    drivePID(5,100,600);
    chassis.turnToHeading(270,1000,{},false);
    resetOdom(true,false,true,false);
    delay(100);
    chassis.moveToPose(-24,-47.5,270,2000,{.forwards = false,.lead=0.1,.minSpeed=60},false);
    handleHood();
    scoreLongClose(1);
    delay(2000);
    chassis.setPose(-25,-48,chassis.getPose().theta);  
              turnToHeadingSmart(270,600);
              handleMLMech();
              chassis.moveToPose(-62,-47.5,270,2000,{.lead = 0.1, .maxSpeed = 70},false);
                handleHood();
                drivePID(20,35,1000);
                    drivePID(-1,2,300);

                resetOdom(false,false,true,false);
                delay(200);
                chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
                delay(1500);
                chassis.moveToPose(-26,-47.5,270,2000,{.forwards = false,.lead=0.1,.minSpeed=100},false);
                handleHood();
                scoreLongClose(200);
                delay(2000);
                chassis.setPose(-24,-47,chassis.getPose().theta);
                handleMLMech();
    chassis.moveToPose(-64,-10,0,3000,{.lead=0.5},false);
    handleMLMech();
    drivePID(30,100,2000);
handleMLMech();

    

}

void workingSkills(){
    startDatalogging();
    hood.set_value(HIGH);
    midGoal.set_value(LOW);
chassis.setPose(0,0,180);
handleDescore();
drivePID(30.5,100,1000);
handleMLMech();
chassis.turnToHeading(270,1000);
spinIntake(1);
drivePID(30,30,1700); //first loader
delay(1200); 
     resetOdom(false,false,true,false);
    delay(100);
    chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(100);
        chassis.turnToHeading(300,800);
    chassis.moveToPose(-22,-61,270,1000,{.forwards = false, .lead=0.3},false);
    handleMLMech();
    chassis.moveToPose(41,-56,270,2000,{.forwards = false, .lead=0.1},false);
    resetOdom(false,true,false,true);
    delay(100);
    chassis.moveToPose(24,-47.5,90,2000,{.forwards = false,.lead=0.1},false);
    
spinIntake(-1);
delay(150);
    spinIntake(1);
scoreLongFar(2000);
delay(1300);
       handleMLMech(); 

 turnToHeadingSmart(90,600);
              handleMLMech();
              handleHood();
                drivePID(30,100,550);
                drivePID(20,35,1000);
                resetOdom(false,false,false,true);
                delay(200);
                chassis.setPose(62,chassis.getPose().y,chassis.getPose().theta);
                delay(800);
                chassis.moveToPose(26,-47.5,90,2000,{.forwards = false,.lead=0.1},false);

handleMLMech();
handleHood();
spinIntake(-1);
delay(150);
spinIntake(1);
scoreLongFar(2000);
chassis.setPose(26,-48,chassis.getPose().theta); //seting position while scoring 3rd loader
delay(1300);
drivePID(10,100,500);
                chassis.turnToHeading(0,700);
                drivePID(10,100,500);
                resetOdom(false,true,false,true);
                chassis.moveToPose(46,48,0,3000,{.lead=0.01},false);
                chassis.turnToHeading(90,700);
                drivePID(30,40,1500);
                resetOdom(false,false,true,false);
                delay(100);
                
chassis.setPose(63,chassis.getPose().y,chassis.getPose().theta);
                delay(1000);
                chassis.turnToHeading(35,800);
    chassis.moveToPose(22,63,90,1000,{.forwards = false, .lead=0.3},false);
    handleMLMech();
    chassis.moveToPose(-41,56,90,2000,{.forwards = false, .lead=0.1},false);
    resetOdom(false,true,false,true);
    delay(100);
    chassis.moveToPose(-24,47.5,270,2000,{.forwards = false,.lead=0.1},false);
    handleHood();
    scoreLongClose(1);
    delay(2000);
    chassis.setPose(-25,48,chassis.getPose().theta);  
              chassis.turnToHeading(270,600);
              handleMLMech();
                drivePID(30,100,550);
                drivePID(20,35,1000);
                resetOdom(false,false,false,true);
                delay(200);
                chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
                delay(800);
                chassis.moveToPose(-26,47.5,270,2000,{.forwards = false,.lead=0.1},false);
                handleHood();
                scoreLongFar(200);
                delay(2000);
                chassis.setPose(-24,47,chassis.getPose().theta);
    chassis.moveToPose(-58,10,180,3000,{.lead=0.4},false);
    drivePID(50,100,2000);
    handleMLMech();
} 





void skillsMiddleBalls(){
    startDatalogging();
    hood.set_value(HIGH);
chassis.setPose(0,0,180);
handleDescore();
drivePID(29,100,800);
handleMLMech();
chassis.turnToHeading(270,1000);
spinIntake(1);
drivePID(25,35,1700); //first loader
delay(900);
    handleMLMech();
    chassis.setPose(-63,-46.3,chassis.getPose().theta); 
       delay(250);

    turnToHeadingSmart(-75,800,{.minSpeed = 60,.earlyExitRange = 10}); 
    chassis.moveToPose(-20, -60, -90, 1500, {.forwards = false,.lead=0.5,.minSpeed = 100,.earlyExitRange = 15},false); //control point
    chassis.moveToPose(30, -60, -90, 2000,{.forwards = false,.lead=0.03,.minSpeed = 100}, false); //going to long goal
    chassis.turnToHeading(0,700);
    drivePID(13,100,800);
    chassis.turnToHeading(90,500);
    alignToLongGoalOpposite(); //first score
    handleHood();
    spinIntake(-1);
    delay(100);
  scoreLongFar(2000);
    
       handleMLMech();
       spinIntake(1);
       turnToHeadingSmart(90,600);
    drivePID(20,100,1200);
handleHood();
turnToHeadingSmart(90,600);
    drivePID(25,35,1500); //2nd matchload
    delay(900);

    
        drivePID(-20,100,700);
alignToLongGoalOpposite(); //2nd score
handleMLMech();
handleHood();
spinIntake(-1);
delay(200);
scoreLongFar(2000);
chassis.setPose(26,-48,chassis.getPose().theta); //seting position while scoring 3rd loader
delay(200);
chassis.turnToPoint(22,30,1000,{},false);
spinIntake(1);
handleHood();
chassis.setPose(26,-38,chassis.getPose().theta);
delay(200);
chassis.moveToPose(23,24.5,0,2700,{},true);
delay(1200);
handleMLMech();
delay(800);

chassis.turnToPoint(7,9,1000,{},false);
handleMLMech();
drivePID(18,100,1250);
spinIntake(-0.6);
delay(1500);
turnToHeadingSmart(222,800);

drivePID(-51,100,2200);
spinIntake(1);
handleMLMech();

chassis.turnToHeading(90, 800);
drivePID(39,35,2000);
delay(900);


    chassis.setPose(57,46,chassis.getPose().theta); //setting positon for movement to second long.
    delay(200);
    turnToHeadingSmart(115,800,{.minSpeed = 60,.earlyExitRange = 10});
    chassis.moveToPose(20,60,90,1500,{.forwards = false,.lead=0.4,.minSpeed = 100,.earlyExitRange = 25},false);
        handleMLMech();

    chassis.moveToPose(-33,65,90,3000,{.forwards = false,.lead=0.02,.minSpeed = 100},false);
    chassis.turnToHeading(180,700);
    drivePID(13,100,1300);
    chassis.turnToHeading(270,800);
    alignToLongGoal(); //3rd score
    handleHood();
    spinIntake(-1);
    delay(200);
    scoreLongClose(2000);
      handleMLMech();
      turnToHeadingSmart(270,800);
      spinIntake(1);
    drivePID(20,100,1200); 
       handleHood();

    turnToHeadingSmart(270,1000);

    drivePID(30,35,2000);    
  
    delay(900);

    drivePID(-17,100,800);
    alignToLongGoal(); //4th score
    handleMLMech();
    handleHood();
    spinIntake(-1);
    delay(100);
    scoreLongClose(2000);

    chassis.setPose(-26,48,chassis.getPose().theta); //seting position while scoring 3rd loader
delay(200);
chassis.turnToPoint(-22,-30,1000,{},false);
spinIntake(1);
handleHood();
chassis.setPose(-28,38,chassis.getPose().theta);
delay(200);
chassis.moveToPose(-23,-30,180,2500,{},true);
delay(1100);
handleMLMech();
delay(700);
chassis.moveToPose(-26,17 ,180,2400,{.forwards=false},false);
chassis.moveToPose(-5,6,-45,2000,{.forwards=false},false);
handleMLMech();
scoreMiddle();
delay(1600);
drivePID(10,100,800);
turnToHeadingSmart(270,1000,{},false);
drivePID(45,100,2000);
chassis.turnToHeading(200,1000);  
drivePID(6,100,500);
      handleMLMech();
spinIntake(1);
handleHood();
    drivePID(40,100);

    handleMLMech();
    drivePID(-20,100,1200);
    stopDatalogging();
}



void skillsOppositePark(){
    startDatalogging();
    hood.set_value(HIGH);
// chassis.setPose(0,0,180);
// handleDescore();
// drivePID(27.5,100,900);
// handleMLMech();
// chassis.turnToHeading(270,1000);
// spinIntake(1);
// drivePID(17,35,1700); //first loader
// delay(900);

//     drivePID(-5,100,800);
//     handleMLMech();
//     chassis.setPose(-58,-46.3,chassis.getPose().theta); 
//        delay(250);

//     turnToHeadingSmart(-75,800,{.minSpeed = 60,.earlyExitRange = 10}); 
//     chassis.moveToPose(-30, -60, -85, 1500, {.forwards = false,.lead=0.3,.minSpeed = 60,.earlyExitRange = 10}); //control point
//     chassis.moveToPose(45, -60, -90, 2500,{.forwards = false,.lead=0.03}, false); //going to long goal
//     chassis.turnToHeading(0,1000);
//     drivePID(7,100,1300);

//     chassis.turnToHeading(90,1000);
//     alignToLongGoalOpposite(); //first score
//     handleHood();
//     spinIntake(-1);
//     delay(100);
//     spinIntake(1);
//     delay(2000);
    
//        handleMLMech();
//     drivePID(20,100,1500);
// handleHood();
// turnToHeadingSmart(90,1000);
//     drivePID(25,35,1500); //2nd matchload
//     delay(900);

    
//         drivePID(-10,100,800);
// alignToLongGoalOpposite(); //2nd score
// handleMLMech();
handleHood();
// spinIntake(-1);
// delay(200);
spinIntake(1);
// delay(2000);
chassis.setPose(26,-48,90); //seting position while scoring 3rd loader
delay(1000);
drivePID(15,100,1000);
chassis.turnToHeading(0,1000);
handleHood();
drivePID(60,100,2000);
chassis.turnToHeading(90,1000);
drivePID(20,100,1500);


chassis.turnToHeading(-85,800);
drivePID(35,100,1600);
chassis.turnToHeading(225,1000);
drivePID(19,100,800);
spinIntake(-0.5);
delay(2000);
chassis.moveToPose(45,49,0,3000,{.forwards=false, .lead=0.1},false); //going to 3rd loader
handleMLMech();

// chassis.turnToHeading(90,800);
// drivePID(22,35,2000); //3rd loader
// delay(900);


//     drivePID(-5,100,800);
//     handleMLMech();
//     chassis.setPose(52,46,chassis.getPose().theta); //setting positon for movement to second long.
//     delay(500);
//     turnToHeadingSmart(115,500,{.minSpeed = 60,.earlyExitRange = 10});
//     chassis.moveToPose(20,58,95,2000,{.forwards = false,.lead=0.3,.minSpeed = 60,.earlyExitRange = 10});
//     chassis.moveToPose(-47,60,90,3000,{.forwards = false,.lead=0.03},false);
//     chassis.turnToHeading(180,1000);
//     drivePID(6,100,1300);
//     chassis.turnToHeading(270,800);
//     alignToLongGoal(); //3rd score
//     handleHood();
//     spinIntake(-1);
//     delay(200);
//     spinIntake(1);
//     delay(2000);
//       handleMLMech();
//       turnToHeadingSmart(270,800);
//     drivePID(20,100,1500); 
//        handleHood();

//     turnToHeadingSmart(260,1000);

//     drivePID(30,35,2000);    
  
//     delay(900);

//     drivePID(-10,100,800);
//     alignToLongGoal(); //4th score
//     handleMLMech();
//     handleHood();
//     spinIntake(-1);
//     delay(100);
//     spinIntake(1);
//     delay(2000);

//     drivePID(20,100,1000); 
//    handleHood();

//     spinIntake(-1);
//     chassis.turnToHeading(200,2000);
//     drivePID(27,100,2500); // line up at park
//     chassis.turnToHeading(180,1000);
//     handleMLMech();
//     drivePID(40,100,2500); 
//     handleMLMech();
//     drivePID(-20,100,1200);
//     stopDatalogging();
}

void PIDTesting()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
    chassis.turnToHeading(45, 2000);
    turnToHeadingSmart(25, 2000);
    turnToHeadingSmart(15,2000);
 


}

// Forward declaration for datalogging functions
void sensorTesting()
{
    // Start datalogging

    startDatalogging();
    chassis.setPose(0, 0, 0);
    // Move forward 24 inches
drivePID(24,100,2000);    // Turn 90 degrees to the right
    chassis.turnToHeading(90, 2000);
    // Stop datalogging
    stopDatalogging();
}

void testAlignToLongGoal()
{
    alignToLongGoal();
    handleHood();
    spinIntake(1);  

}



void soloAWPTap(){
    startDatalogging();
    hood.set_value(HIGH);
        midGoal.set_value(HIGH);

    handleDescore();
chassis.setPose(-48,0,0);
spinIntake(1);
drivePID(10,100,250);
resetOdom(false, true, true, false);
chassis.moveToPose(-52,-49,0,1800,{.forwards = false,.lead=0.01},false);
handleMLMech();
turnToHeadingSmart(270,700);
drivePID(32,35,1500); //first loader
resetOdom(false,false,true,false);

delay(300);
chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
delay(100);
chassis.moveToPose(-25,-48,270,1000,{.forwards = false,.lead=0.05,.minSpeed=100},false);
        handleHood();
      scoreLongClose(800);
       handleMLMech();
       chassis.setPose(-25,-48,chassis.getPose().theta);
       delay(100);

chassis.turnToPoint(-25,24,800,{},false);   
    handleHood();
       chassis.setPose(-25,-38,chassis.getPose().theta);
        spinIntake(1);
       chassis.moveToPose(-23,28,0,2700,{.lead=0.1},true);
       delay(1000);  
                   handleMLMech();
                   delay(100);  
                                    chassis.cancelAllMotions();

        chassis.moveToPose(-10,4 ,-45,1300,{.forwards = false,.lead=0.3,.minSpeed=40},false);
              spinIntake(-1);
              delay(100);
       
              scoreMiddle();  
              delay(750);
              spinIntake(1);
              chassis.moveToPose(-70,43,268,1500,{.lead=0.5,.minSpeed=60},false);
                            chassis.moveToPose(-70,43,268,600,{.lead=0.5,.minSpeed=40},false);

drivePID(10,35,400);
               resetOdom(false,false,false,true);

delay(100);
chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
delay(100);
chassis.moveToPose(-25,48,270,1000,{.forwards = false,.lead=0.05,.minSpeed=100},false);
              spinIntake(-1);
              delay(100);
              spinIntake(1);
                              handleHood();
               handleMLMech();
scoreLongClose(800);
              
stopDatalogging();
       
}

void soloAWPCounter(){
    startDatalogging();
    hood.set_value(HIGH);
    midGoal.set_value(HIGH);
    handleDescore();
chassis.setPose(-48,0,0);
spinIntake(1);
drivePID(10,100,250);
resetOdom(false, true, true, false);
chassis.moveToPose(-48,-47,0,1500,{.forwards = false,.lead=0.0},false);
handleMLMech();
turnToHeadingSmart(270,700);
drivePID(32,30,1400); //first loader
delay(150);
resetOdom(false,false,true,false);
delay(150);
chassis.moveToPose(-26,-47.5,270,900,{.forwards = false,.lead=0.05,.minSpeed=100},false);
        handleHood();
      scoreLongClose(700);
       handleMLMech();
       chassis.setPose(-25,-48,chassis.getPose().theta);

chassis.turnToPoint(-25,20,1000,{},false);   
    handleHood();
       chassis.setPose(-25,-38,chassis.getPose().theta);
        spinIntake(1);
       chassis.moveToPose(-23,20,0,1000,{.lead=0.1,.minSpeed=100},false);
       handleMLMech();
       chassis.turnToHeading(174,600);
       drivePID(-15,100,1000);
chassis.swingToHeading(275,DriveSide::LEFT,800,{.minSpeed=60},true);
        handleHood();
        delay(100);
              spinIntake(-1);
              delay(100);
        
              spinIntake(1);
              scoreLongClose(700);
              delay(200);
              chassis.setPose(-25,48,chassis.getPose().theta);  
              handleHood();         
chassis.moveToPose(-65,45,270,500,{.lead=0.01,.minSpeed=100},false);
chassis.turnToHeading(270,600);
drivePID(20,35,1000);
                delay(100);
                
                resetOdom(false,false,false,true);
                delay(100);
                chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
                
           chassis.moveToPose(-11,10,315,2000,{.forwards = false,.lead=0.3,.minSpeed=60},false);
            spinIntake(-1);
              delay(150);
                scoreMiddle();

            
              
              
stopDatalogging();
       
}


void Left4_3LongLast(){
    // Initialize robot position and systems
hood.set_value(HIGH);
    chassis.setPose(0,0,75);  // Set initial position facing slightly left
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore

    // First ball collection and scoring sequence
    drivePID(34 ,100,1350);  // Drive forward to collect balls
    handleMLMech();
    drivePID(-8.5,100,900);
    chassis.turnToHeading(316,1000);  // Turn towards goal
    drivePID(-15,100,1300);
    spinIntake(-1);  // Reverse intake to score
    delay(200); 
    scoreMiddle();  // Score in middle goal
    delay(700);
    stopIntake();

    // Transition to long goal scoring position
    turnToHeadingSmart(315, 500); 

    drivePID(52,100,1750);  // Drive to long goal area
    turnToHeadingSmart(270,600);  // Face the long goal

    spinIntake(1);  // Start collecting balls
    drivePID(23,35,1700);  // Fine adjustment
    drivePID(-10,100,800);
    alignToLongGoal();
    turnToHeadingSmart(270,700);
    drivePID(-20,100,900);
// chassis.moveToPose(-28, -48, 270, 1400,{.forwards = false,.lead=0.05, .maxSpeed = 70},false);
       handleMLMech();
       handleHood();
spinIntake(-1);
delay(100);
       spinIntake(1);  

      chassis.setPose(-25, 48, chassis.getPose().theta);

       delay(600); 
       


    // Final descoring sequence
    drivePID(12,100,1000);  // Position for descoring 
    handleHood();

   handleDescore();  // Perform second descore

    chassis.turnToHeading(230, 500);  // Turn to first descore target
    drivePID(-9.5,100,1000);
    chassis.moveToPose(0, 50, 270, 1500,{.forwards = false,.lead=0.05},false);
    chassis.turnToHeading(220,1000,{.minSpeed = 100},false);
    setPose();
    pros::Task holdPushTask(holdPush);
    
}


void Left4_3LongFirst(){
    hood.set_value(HIGH);
    resetOdom(true,false,true,false);
    handleDescore();
    spinIntake(1);
    drivePID(30,100,900);
     handleMLMech();
     chassis.turnToHeading(270,1000);
    drivePID(40,35,1500);
    resetOdom(false,false,false,true);
    delay(200);
    chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(100);
    chassis.moveToPose(-24,47.5,270,1000,{.forwards = false,.lead=0.01,.minSpeed=100},false);
    handleHood();
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(800);
    chassis.setPose(-25,48,chassis.getPose().theta);
    handleMLMech();
    delay(100);
chassis.turnToPoint(-21,-27,1000,{.earlyExitRange = 5},false);
chassis.moveToPose(-23,26,180,1100,{.lead=0.1,.minSpeed=60,.earlyExitRange=5},true);
delay(400);
handleMLMech();
    handleHood();
   chassis.turnToHeading(315,600,{},false);
    chassis.moveToPose(-6,7,315,1200,{.forwards = false,.lead=0.01,.minSpeed = 60},false);
    spinIntake(-1);
    delay(200);
    scoreMiddle();
    delay(1400);
    handleDescore();
    handleMLMech();
    chassis.turnToHeading(315,800);
    chassis.moveToPose(-25,29,315,900,{.lead=0.1,.minSpeed=60,.earlyExitRange=15},false);
    chassis.swingToHeading(40,DriveSide::RIGHT,125,{},false);
    chassis.moveToPose(-5,40,100,1500,{.lead=0.05,.minSpeed=90},false);
    chassis.turnToHeading(120,900);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    left_motors.brake();
    right_motors.brake();

}

void Right4_3LongLast(){
    hood.set_value(HIGH);
    chassis.setPose(0,0,105);  // Set initial position facing slightly left
    spinIntake(1);  // Start intake

    // First ball collection and scoring sequence
    drivePIDAsync(31 ,100,800);  // Drive forward to collect balls
    delay(300);
    handleMLMech();
    delay(500);
    drivePID(-5.5,100,700);    // Back up slightly
    handleMLMech();
    turnToHeadingSmart(40,800);  // Turn towards goal
    drivePID(11.5,80,1300);  // Drive to scoring position
spinIntake(-0.7); 
   handleDescore();  // Perform first descore

    delay(1000);
    spinIntake(1);  // Start collecting balls 
    drivePID(-45,100,1000);

    chassis.turnToHeading(270,800);
    handleMLMech();
    drivePID(18,45,2000);  // Approach goal
    delay(500);
    drivePID(-10,100,800);
    alignToLongGoal();
       handleMLMech();
       handleHood();

       spinIntake(1);  

       delay(500);
                   chassis.setPose(-25, -48, chassis.getPose().theta);

       delay(300); 
       


    // Final descoring sequence
    drivePID(12,100,1000);  // Position for descoring 
    handleHood();

   handleDescore();  // Perform second descore

    chassis.turnToHeading(225, 500);  // Turn to first descore target
    drivePID(-10,100,1000);
    chassis.moveToPose(-25, -44, 270, 1500,{.forwards = false,.lead=0.05},false);
    chassis.turnToHeading(220,1000,{.minSpeed = 100},false);
         
}

void Right4_3LongFirst(){
hood.set_value(HIGH);
chassis.setPose(0,0,180);  // Set initial position facing slightly left
    resetOdom(true,false,false,true);
    handleDescore();
    spinIntake(1);
     drivePID(29.5,100,900);
     handleMLMech();
     chassis.turnToHeading(270,1000);
    drivePID(30,30,1800);
    resetOdom(false,false,true,false);
delay(100);
    chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(100);
    chassis.moveToPose(-24,-47,270,900,{.forwards = false,.lead=0.01,.minSpeed=100},false);
    handleHood();
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(800);
    chassis.setPose(-25,-48,chassis.getPose().theta);
    handleMLMech();
    delay(100);
chassis.turnToPoint(-21,27,1000,{.earlyExitRange = 5},false);
    handleHood();

chassis.moveToPose(-25,-24,0,1300,{.lead=0.01,.minSpeed=60},true);
delay(200);
handleMLMech();
delay(150);
   chassis.turnToHeading(45,600,{},false);
   handleMLMech();
    chassis.moveToPose(-10,-13.5,45,1200,{.forwards = true,.lead=0.05},false);
    
    spinIntake(-0.8);
    delay(1500);
    handleDescore();
 
    chassis.turnToHeading(45,800);
    drivePID(-33,100,1300);
    chassis.turnToHeading(274,700,{},false);
    drivePID(-24,100,1100);
    chassis.turnToHeading(250,900);
     left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    left_motors.brake();
    right_motors.brake();
}

void left7BallHold(){
    hood.set_value(HIGH);
    chassis.setPose(-46,8,90);  // Set initial position facing slightly left
    resetOdom(false,true,true,false);
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-24,22,60,1000,{.lead=0.4,.minSpeed = 127},true);
    delay(500);
    handleMLMech();
    delay(200);
    chassis.turnToHeading(-55,500,{.earlyExitRange = 5},false);
    chassis.moveToPose(-63,44,270,11400,{.lead=0.2,.minSpeed = 70},false);
    chassis.turnToHeading(272,400);  // Turn towards goal
    drivePID(30,30,1200);
    delay(100);    
        resetOdom(false,false,false,true);
chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(100);

chassis.moveToPose(-25,47,270,1000,{.forwards = false,.lead=0.01,.minSpeed = 70},false);

    handleMLMech();
    handleHood();
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(1800); 
    delay(400);
    drivePID(10,100,300);
    handleHood();
    drivePID(-20,100,1000);

}

void left7BallFast(){
    hood.set_value(HIGH);
    chassis.setPose(-46,8,90);  // Set initial position facing slightly left
    resetOdom(false,true,true,false);
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-24,22,60,1000,{.lead=0.4,.minSpeed = 127},true);
    delay(500);
    handleMLMech();
    delay(200);
    chassis.turnToHeading(-70,500,{.earlyExitRange = 5},false);
    chassis.moveToPose(-63,41,270,1300,{.lead=0.2,.minSpeed = 70},false);
    chassis.turnToHeading(272,400);  // Turn towards goal
    drivePID(30,30,1400);
    delay(100);    
        resetOdom(false,false,false,true);
chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(100);

chassis.moveToPose(-25,47,270,1000,{.forwards = false,.lead=0.01,.minSpeed = 70},false);

    handleMLMech();
    handleHood();
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(1800); 
    delay(400);
    wing();
    
}

void right7BallHold(){
hood.set_value(HIGH);
    chassis.setPose(-46,-8,90);  // Set initial position facing slightly left
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-24,-22,120,1200,{.lead=0.4,.minSpeed = 127},true);
    delay(550);
    handleMLMech();
    delay(200);
    chassis.turnToHeading(-105,600,{.earlyExitRange = 5},false);
    chassis.moveToPose(-62,-48.5,270,1500,{.lead=0.3,.minSpeed = 50,.earlyExitRange = 10},false);
    turnToHeadingSmart(268,300);  // Turn towards goal
    drivePID(20,35,1300);
    turnToHeadingSmart(271,700);
    backIntoLongGoalML();  
      spinIntake(-1);
    delay(100);
    spinIntake(1); 
       handleMLMech();
    handleHood();
    scoreLongClose(1);
delay(1000);

    drivePID(10,100,1000);
    handleHood();
    drivePID(-10,100,800);

}
void right7BallWing(){
 hood.set_value(HIGH);
    chassis.setPose(0,0,90);  // Set initial position facing slightly left
    resetOdom(false,true,false,true);
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-24,-25,100,1000,{.lead=0.4,.minSpeed = 127},true);
    delay(500);
    handleMLMech();
    chassis.turnToHeading(-110,300,{.minSpeed = 127,.earlyExitRange = 5},false);
    chassis.moveToPose(-70,-50,270,1300,{.lead=0.3,.minSpeed = 70,.earlyExitRange = 10},false);
    chassis.turnToHeading(270,400);  // Turn towards goal
drivePID(20,30,1000);
    resetOdom(false,false,true,false);
    delay(200);
    chassis.setPose(-62,chassis.getPose().y,chassis.getPose().theta);
    delay(300);
    chassis.moveToPose(-25,-46,270,900,{.forwards = false,.lead=0.1,.minSpeed = 100},false);
      spinIntake(-1);
    delay(100);
    spinIntake(1); 
       handleMLMech();
    handleHood();
    scoreLongClose(1);


    wing();

}

void left4Wing(){
hood.set_value(HIGH);
    chassis.setPose(-46,8,90);  // Set initial position facing slightly left
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-28,22,60,1100,{.lead=0.4,.minSpeed = 127},true);
    delay(550);
    handleMLMech();
    delay(200);
    chassis.turnToHeading(153,600,{.earlyExitRange = 5},false);
    drivePID(-30,100,600);
    chassis.swingToHeading(280,DriveSide::LEFT,900,{.minSpeed = 90},true);
    delay(400);
    handleHood();
    handleMLMech();
    scoreLongClose(800);
    delay(300);
wing();
}

void right4Wing(){
hood.set_value(HIGH);
    chassis.setPose(-46,-8,90);  // Set initial position facing slightly left
    spinIntake(1);  // Start intake
    handleDescore();  // Perform first descore
    chassis.moveToPose(-24,-22,120,1200,{.lead=0.4,.minSpeed = 127},true);
    delay(550);
    handleMLMech();
    delay(200);
    chassis.turnToHeading(18,800,{},false);
    drivePID(-30,100,600);
    chassis.swingToHeading(265,DriveSide::RIGHT,900,{},true);
    delay(300);
    handleHood();
    handleMLMech();
    scoreLongClose(800);
    delay(300);
wing();
}

void fakeRight(){
    hood.set_value(HIGH);
    chassis.setPose(0,0,180);  // Set initial position facing slightly left
    handleDescore();
    spinIntake(1);
     drivePID(31.5,100,1000);
     handleMLMech();
     chassis.turnToHeading(270,1000);
    drivePID(29,40,1400);
        drivePID(-1,2,500);

    delay(150);
    turnToHeadingSmart(270,600,{},false);
    backIntoLongGoalML();
    handleHood();
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(800);
    wing();
}

void fakeLeft(){
    hood.set_value(HIGH);
    chassis.setPose(0,0,0);  // Set initial position facing slightly left
    handleDescore();
    spinIntake(1);
     drivePID(31,100,1000);
     handleMLMech();
     chassis.turnToHeading(270,1000);
    drivePID(23,40,1400);
    drivePID(-1,2,300);
    delay(350);
    turnToHeadingSmart(271,1500,{},false);
    backIntoLongGoalML();
    handleHood();
    handleMLMech(); 
    spinIntake(-1);
    delay(100);
    spinIntake(1);
    scoreLongClose(800);
    wing();
}