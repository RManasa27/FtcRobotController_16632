//package org.firstinspires.ftc.teamcode;
//test
//public class Auto_Specimen_Double {
//}
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
 */
@Autonomous(name="Auto Disassemble", group="Training")
public class Auto_Specimen_Double extends LinearOpMode {

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
        robot.rightHand.setPosition(.37);
        // Hold Sample


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        hangSpecimenOne();
        //pickupSpecimen();
        //hangSpecimen();
        //pickupSpecimen();
        //hangSpecimen();
        //pickupSpecimen();
        //hangSpecimen();
        //pickupSpecimen();
        //hangSpecimen();

        // Autonomous Code - Call Methods Here


        telemetry.addData("Status:", "Auto Finished");
        telemetry.update();

        // Wait until end of op Mode
        while(opModeIsActive()) {
            telemetry.addData("Status:", "Auto Holding Position");
            telemetry.update();
        }

    }
    public void pickupSpecimen() {

        robot.rightArmMotorDeg(1, -200, 0);
        robot.move(10,10);
        robot.turnLeft(160, 5);
        robot.move(3,10);
        robot.rightArmMotorDeg(1, -100, 0);

    }

    public void hangSpecimenOne() { // Update Test again
        //SPECIMIN 1
        robot.leftArmMotorDeg(1, 1050, 0);
        delay(1);
        robot.move(34, 0.5);
        delay(1);
        robot.rightArmMotorDeg(1, -115, 0);
        delay(0.5);
        robot.move(-8,0.5);
        delay(1);
        robot.rightHand.setPosition(.51);
        robot.rightArmMotorDeg(1, 115, 0);
        //MOVE TO OBS ZONE
        delay(1);
        robot.move(-4,0.5);
        delay(0.5);
        robot.turnRight(150, 0.7);
        delay(1);
        robot.leftArmMotorDeg(1, -1365, 0);
        robot.strafe(64,0.5);
        //PICKUP SPECIMIN 2
        delay(2);
        robot.move(10,0.1);
        delay(2);
        robot.rightHand.setPosition(.37);
        //MOVE TO CHAMBERS
        delay(1);
        robot.leftArmMotorDeg(1, 500, 0);
        robot.move(-10, 0.5);
        robot.turnRight(160, 0.5);
        robot.leftArmMotorDeg(1, 800, 0);
        delay(1);
        robot.strafe(62, 0.5);
        //CHAMBER APPROACH AND SCORE
        delay(1);
        robot.move(20, 0.7);
        robot.rightArmMotorDeg(1, -125, 0);
        delay(0.2);
        robot.move(-8, 0.5);
        robot.rightHand.setPosition(.37);
        robot.rightArmMotorDeg(1, 115, 0);
        robot.move(-6, 0.8);
        robot.rightHand.setPosition(.59);


        //FINAL MOVE TO OBS ZONE








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
