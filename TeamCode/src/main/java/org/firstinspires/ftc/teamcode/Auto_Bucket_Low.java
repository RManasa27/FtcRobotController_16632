/*
Copyright 2024 FIRST Tech Challenge Team DEMO04

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 *
 */
@Autonomous(name="Scoop Low", group="Training")
public class Auto_Bucket_Low extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad();   
    private ElapsedTime runtime = new ElapsedTime();
        

    @Override
    public void runOpMode() {
        
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Close
        robot.leftHand.setPosition(0.49);   // leftHandPos = Range.clip(leftHandPos, 0.2, 0.48);
        robot.rightHand.setPosition(0.75);
        robot.wristHand.setPosition(0.72);
        robot.rightArmMotorDeg(0, 0, 0);
        ;//rightHandPos = nge.clip(rightHandPos, 0.52, 0.8);
        
        // Hold Sample
        robot.leftHand.setPosition(0.15);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAYY
        
        // Autonomous Code - Call Methods Here
        waitForStart();
        driveToBuckets();
        putInBucketLow();
        
        telemetry.addData("Status:", "Auto Finished");
        telemetry.update();
        
        // Wait until end of op Mode
        while(opModeIsActive()) {
            telemetry.addData("Status:", "Auto Holding Position");
            telemetry.update();
        }
        
    }

    public void driveToBuckets() {
        robot.leftArmMotorDeg(1, 1050, 0);
        robot.move(25, 0.5);
        robot.turnLeft(90, 0.5);
        robot.move(24, 0.5);
        robot.turnLeft(35,0.5);
    }

    public void putInBucketLow() {
       robot.leftArmMotorDeg(0.8, 1300, 4);
       delay(2);
       robot.rightArmMotorDeg(0.7, -340, 2);
       robot.leftHand.setPosition(0.5);
       robot.wristHand.setPosition(0.592);
       robot.leftHand.setPosition(0.36);
       robot.rightHand.setPosition(.69);
       delay(1);
       robot.rightArmMotorDeg(0.7, 310, 2);
    }

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }
    
    // Define Methods Here
    
}
