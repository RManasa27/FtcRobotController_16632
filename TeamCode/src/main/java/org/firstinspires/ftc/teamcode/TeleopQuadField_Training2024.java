package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name= "Overkill", group = "Training")
public class TeleopQuadField_Training2024 extends LinearOpMode {
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad();


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

        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




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

            if (gamepad1.options) {
                imu.resetYaw();
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

            telemetry.addData("botHeading", Math.toDegrees(botHeading));
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

            robot.leftFront.setPower(frontLeftPower);
            robot.leftRear.setPower(backLeftPower);
            robot.rightFront.setPower(frontRightPower);
            robot.rightRear.setPower(backRightPower);
        }


    }




}


