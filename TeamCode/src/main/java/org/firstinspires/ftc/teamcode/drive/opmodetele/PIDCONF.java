//package org.firstinspires.ftc.teamcode.drive.opmodetele;
//import static java.lang.Math.abs;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@Config
//@TeleOp
//public class PIDCONF extends OpMode {
//    private PIDController controller;
//
//    public static double p = 0, i = 0, d = 0;
//    double IntegralSum = 0;
//    public static double f = 0;
//    public static double delay = 0.004;
//
//    public static int target = 0;
//    public static int offset = 50;
//
//    private final double ticks_in_degree = 537.7 / 360;
//
//    private DcMotorEx motorCrane1, motorCrane2;
//
//    ElapsedTime timer = new ElapsedTime();
//    double lastError = 0;
//    public double calculateThrottle(float x) {
//        int sign = -1;
//        if (x > 0) sign = 1;
//        return sign * 3 * abs(x);
//    }
//
//    @Override
//    public void init(){
//        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//
//        motorCrane1 = hardwareMap.get(DcMotorEx.class, "motorCrane1");
//        motorCrane2 = hardwareMap.get(DcMotorEx.class, "motorCrane2");
//        motorCrane1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorCrane2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorCrane1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorCrane2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorCrane1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorCrane2.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//
//    public void loop(){
//        controller.setPID(p, i, d);
//        int armPos = motorCrane1.getCurrentPosition();
//        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians((armPos - offset) / ticks_in_degree)) * f;
//
//        double power = PIDControl(target, armPos) + ff;
//
//        if(gamepad1.right_trigger > 0.1) {
//            target += (int) calculateThrottle(gamepad1.right_trigger);
//        }
//        if(gamepad1.left_trigger > 0.1){
//            target -= (int) calculateThrottle(gamepad1.left_trigger);
//        }
//        motorCrane1.setPower(power);
//        motorCrane2.setPower(power);
//
//
//        telemetry.addData("pos ", armPos);
//        telemetry.addData("target ", target);
//        telemetry.addData("Power ", power);
//        telemetry.addData("FF: ", ff);
//        telemetry.addData("error: ", lastError);
//        telemetry.update();
//    }
//
//    public double PIDControl(double target, double armPos){
//        double error = target - armPos;
//        IntegralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * p) + (derivative * d) + (IntegralSum * i);
//        return output;
//
//    }
//}
