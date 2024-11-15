package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Loading...", group="Training")
//@Disabled
public class Auto_Test_2024 extends LinearOpMode {

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


        robot.rightHand.setPosition(0.37);
        robot.rightArmMotorDeg(0, 0, 0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Perform the steps of Autonomous One Step at a Time.
        // MARIST: Add Code here!
        
        // Sample Code - forward 12 inches, backward 12 inches
        robot.leftArmMotorDeg(0.8,100, 5);
        robot.strafeInches(-45 , 0.5);
        
        
        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
    }
    // Sample Delay Code
    // t is in seconds
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }

    // REACH: Add Functions Here

    
    
}


