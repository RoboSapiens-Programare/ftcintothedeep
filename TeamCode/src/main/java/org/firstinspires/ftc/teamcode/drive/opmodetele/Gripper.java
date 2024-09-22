package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.subsystems.Crane;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class Gripper extends LinearOpMode {
    private Crane crane = null;
    int direction = 1; //daca e true e in fata daca e false e in spate
    double servoPosSlides = 0.5;
    double servoPosGrippy = 0;
    // EXPONENTIAL THROTTLE
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * 3 * abs(x);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        crane = new Crane(hardwareMap);

        // INIT CODE
        telemetry.addData(">", "Initialized");
        telemetry.update();



        waitForStart();
        if (isStopRequested()) return;

            // TELEOP CODE

        while (opModeIsActive()) {

            // GAMEPAD 2

            if(gamepad1.right_trigger > 0.1){
                crane.gripperDirection = 1;
                crane.setGripper(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0.1){
                crane.gripperDirection = -1;
                crane.setGripper(gamepad1.left_trigger);
            }
            else crane.setGripper(0);

            if (gamepad2.left_bumper) {
                crane.slidesDirection = -1;
                crane.setSlides(1);
            } else if (gamepad2.right_bumper) {
                crane.slidesDirection = 1;
                crane.setSlides(1);
            } else {
                crane.setSlides(0);
            }
                telemetry.addData("GRIPPER POWER LEFT: ", gamepad1.left_trigger);
                telemetry.addData("GRIPPER POWER RIGHT: ", gamepad1.right_trigger);

            telemetry.update();
            }

        }

    }



