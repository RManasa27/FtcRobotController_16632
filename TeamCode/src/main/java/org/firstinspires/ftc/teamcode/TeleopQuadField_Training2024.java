package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name= "SpRiNklEs", group = "Training")
public class TeleopQuadField_Training2024 extends LinearOpMode {
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad();

    double rightHandPos = 0.37;


    int sliderPos = 0;
    int armPos = 0;

    int rightClimbPos = 0;
    int leftClimbPos = 0;

    double SPEED_CONTROL = 1.0;
    double ARM_SPEED = 0.8;
    double SLIDER_SPEED = 0.8;

    boolean isDown = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        robot.init(hardwareMap);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();

        //robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderPos = robot.rightArm.getCurrentPosition();




        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        // Starting Position for Arm

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double dy = gamepad1.left_stick_y;
            double dx = -gamepad1.left_stick_x;

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                imu.resetYaw();
            }

            if (gamepad2.left_bumper && gamepad2.right_bumper) {
                imu.resetYaw();
            }
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;


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
            if (gamepad1.dpad_left) {
                sliderPos = 0;
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

            // Softstop Slider
            if (robot.leftArm.getCurrentPosition() < 3500) { // Down
                isDown = true;
                if (robot.rightArm.getCurrentPosition() < -2000) {
                    sliderPos = -2000;
                }
            }
            else {
                isDown = false;
            }

            // Slider
            if (gamepad1.y) {
                if (isDown && robot.rightArm.getCurrentPosition() > -2000) {
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightArm.setPower(-0.8);
                    sliderPos = robot.rightArm.getCurrentPosition();
                }
                else if (!isDown) {
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightArm.setPower(-0.8);
                    sliderPos = robot.rightArm.getCurrentPosition();
                }
                else {
                    robot.rightArm.setPower(0);
                }
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

            double leftPowerY = gamepad1.left_stick_y;      //find the value of y axis on the left joystick;
            double leftPowerX = -gamepad1.left_stick_x;      //find the value of x axis on the left joystick;
            double rightPowerX = gamepad1.right_stick_x;     //find the value of x axis on the right joystick;

            double angleR = Math.atan2(leftPowerY, leftPowerX)-(Math.PI/2); //Calculating angle of which the joystick is commanded to in radians
            double angleD = Math.toDegrees(angleR); //Calculating angle of which the joystick is commanded to in degrees
            double speed = Math.sqrt((leftPowerY * leftPowerY) + (leftPowerX * leftPowerX)); //Calculating the magnitude of the joystick

            double a = Math.pow(dx,2);
            double b = Math.pow(dy,2);
            double c = a + b;
            double mag = Math.sqrt(c);
            double theta = Math.atan2(dy,dx);
            //theta += Math.PI;
            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            double worldTheta = theta - botHeading;

            double FR_BL = mag * (Math.sin(worldTheta - Math.PI/4));
            double FL_BR = mag * (Math.sin(worldTheta +  Math.PI/4));

            //double FR_BL = mag * (Math.sin(theta - Math.PI/4));
            //double FL_BR = mag * (Math.sin(theta + Math.PI/4));

            telemetry.addData("IMU", Math.toDegrees(botHeading));
            telemetry.addData("Input", Math.toDegrees(theta));
            telemetry.update();

            double divide = 1.25;

            double frontLeftPower = FL_BR - rx;
            double backLeftPower = FR_BL - rx;
            double frontRightPower = FR_BL + rx;
            double backRightPower = FL_BR + rx;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            robot.leftFront.setPower(frontLeftPower * SPEED_CONTROL);
            robot.leftRear.setPower(backLeftPower * SPEED_CONTROL);
            robot.rightFront.setPower(frontRightPower * SPEED_CONTROL);
            robot.rightRear.setPower(backRightPower * SPEED_CONTROL);

            telemetry.addData("Right hand:", rightHandPos);
            telemetry.addData("SliderPos", sliderPos);
            telemetry.addData("Speed:", SPEED_CONTROL);
            telemetry.addData("Right Shoulder:", rightClimbPos);
            telemetry.addData("Left Shoulder:", leftClimbPos);
            telemetry.addData("Mid Arm:", robot.leftArm.getCurrentPosition());

            robot.rightHand.setPosition(rightHandPos);
            robot.rightArm.setTargetPosition(sliderPos);



        }


    }




}


