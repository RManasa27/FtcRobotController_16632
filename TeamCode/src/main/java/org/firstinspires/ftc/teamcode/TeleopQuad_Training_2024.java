/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Chicken", group="Training")
//@Disabled
public class    TeleopQuad_Training_2024 extends OpMode {

    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad();

   // double wristposition = 0.83;
   // double leftHandPos = 0.22;
    double rightHandPos = 0.37;


    int sliderPos = 0;
    int armPos = 0;

    int rightClimbPos = 0;
    int leftClimbPos = 0;

    double SPEED_CONTROL = 1.0;
    double ARM_SPEED = 0.8;
    double SLIDER_SPEED = 0.8;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Set Servo
        //robot.leftHand.setPosition(0.48);
        //robot.rightHand.setPosition(0.52);

        // Send telemetry message to signify robot waiting;

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();

        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderPos = robot.rightArm.getCurrentPosition();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
        double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
        double rightX = gamepad1.right_stick_x * SPEED_CONTROL;

        robot.driveStrafer(leftX, leftY, rightX);

        if (gamepad2.dpad_up) {
            SPEED_CONTROL = 1;
        }
        if (gamepad2.dpad_right) {
            SPEED_CONTROL = 0.5;
        }
        if (gamepad2.dpad_down) {
            SPEED_CONTROL = 0.25;
        }
        if (gamepad1.dpad_up) {
            SPEED_CONTROL = 1;
        }
        if (gamepad1.dpad_right) {
            SPEED_CONTROL = 0.5;
        }
        if (gamepad1.dpad_down) {
            SPEED_CONTROL = 0.25;
        }
        // Shoulder
        double deltaArmPos = gamepad1.left_trigger - gamepad1.right_trigger;

        if (Math.abs(deltaArmPos) > 0.1) {
            robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setPower(deltaArmPos);
        }
        else {
            armPos = robot.leftArm.getCurrentPosition();
            robot.leftArm.setTargetPosition(armPos);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(ARM_SPEED);
        }

        // Slider
        if (gamepad1.y && robot.rightArm.getCurrentPosition() > -3000) {
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(-0.8);
            sliderPos = robot.rightArm.getCurrentPosition();
        }
        else if (gamepad1.a) {
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(0.8);
            sliderPos = robot.rightArm.getCurrentPosition();
        }
        else {
            //sliderPos = robot.rightArm.getCurrentPosition();
            robot.rightArm.setTargetPosition(sliderPos);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setPower(SLIDER_SPEED);
        }

        //ClimbArms
        double deltaRightClimb = gamepad2.left_trigger - gamepad2.right_trigger;

        if (Math.abs(deltaRightClimb) > 0.1) {
            robot.rightClimbArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightClimbArm.setPower(deltaRightClimb);
            robot.leftClimbArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftClimbArm.setPower(-deltaRightClimb);
        }
        else {
            rightClimbPos = robot.rightClimbArm.getCurrentPosition();
            robot.rightClimbArm.setTargetPosition(rightClimbPos);
            robot.rightClimbArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightClimbArm.setPower(ARM_SPEED);

            leftClimbPos = robot.leftClimbArm.getCurrentPosition();
            robot.leftClimbArm.setTargetPosition(leftClimbPos);
            robot.leftClimbArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftClimbArm.setPower(ARM_SPEED);
        }





        //close grasper grad
        if (gamepad1.right_bumper) {
            rightHandPos-= 0.03;
        }

        if (gamepad2.b) {
            rightHandPos-= 0.03;
        }


        //open grasper grad
        if (gamepad1.left_bumper) {
            rightHandPos+= 0.03;
        }

        if (gamepad2.x) {
            rightHandPos+=0.03;
        }



            rightHandPos = Range.clip(rightHandPos, 0.37, 0.73);

        if (sliderPos > 0) {
            sliderPos = 0;
        }

        if (sliderPos < -3050) {
            sliderPos = -3050;
        }

        telemetry.addData("Say", "Robot Ready");    //
        telemetry.addData("Right hand:", rightHandPos);
        telemetry.addData("SliderPos", sliderPos);
        telemetry.addData("Speed:", SPEED_CONTROL);
        telemetry.addData("Right Shoulder:", rightClimbPos);
        telemetry.addData("Left Shoulder:", leftClimbPos);
        telemetry.addData("Mid Arm:", armPos);

       // robot.leftHand.setPosition(leftHandPos);
        robot.rightHand.setPosition(rightHandPos);






    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
