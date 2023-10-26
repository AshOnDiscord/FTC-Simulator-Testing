package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders. This op mode will work with
 * either the MecBot or the XDriveBot robot configuration.
 */
@TeleOp(name = "TestOdom", group = "OdomBot")
public class TestOdom extends LinearOpMode {

    EncBot bot = new EncBot();
    Pose2d pose;
    PID xPID = new PID(0.35 * 0.6, 0.35 * 1.2 ,0.35 * 0.075, 0);
    PID yPID = new PID(0.35 * 0.6, 0.35 * 1.2 ,0.35 * 0.075, 0);
    PID hPID = new PID(5.5 * 0.6, 5.5 * 1.2 ,5.5 * 0.075, 0, true);
//    PID xPID = new PID(0.35 * 0.2, 0.35 * 0.4 ,0.35 * 0.066, 0);
//    PID yPID = new PID(0.35 * 0.2, 0.35 * 0.4 ,0.35 * 0.066, 0);
//    PID hPID = new PID(5.5 * 0.2, 5.5 * 0.4 ,5.5 * 0.066, 0, true);

    ElapsedTime timer;

    public void runOpMode(){
        bot.init(hardwareMap);
        bot.resetOdometry(0, 0, 0);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("time", this.time);
            telemetry.update();
        }

        Pose2d[] path = new Pose2d[] {
                new Pose2d(0, 24 * 2, Math.toRadians(90)),
                new Pose2d(12 * 2 , 0, Math.toRadians(-90)),
                new Pose2d(-12 * 2 , -24 * 2, Math.toRadians(0)),
                new Pose2d(0 , 0, Math.toRadians(0)),
        };
        int currentWaypoint = 0;

        timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()){
            pose = bot.updateOdometry();
            if (goTo(path[currentWaypoint])) {
                currentWaypoint++;
                timer.reset();
                if (currentWaypoint >= path.length) return;
            }
            telemetry.addData("time", this.time);
            telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }

    public boolean goTo(Pose2d target) {
        double xTolerance = 1;
        double yTolerance = 1;
        double hTolerance = 2;

        double accelMax = Math.min(timer.milliseconds() / 750, 1);
//        double accelMax = 1;
        System.out.println(accelMax + " " + timer.seconds());

        telemetry.addData("max", accelMax);

        double x = xPID.getError(target.getX(), pose.getX(), accelMax);
        double y = yPID.getError(target.getY(), pose.getY(), accelMax);
        double rx = hPID.getError(target.getHeading(), pose.getHeading(), accelMax);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        fieldCentricMove(x, y, rx, accelMax);

        return Math.abs(target.getX() - pose.getX()) < xTolerance && Math.abs(target.getY() - pose.getY()) < yTolerance && Math.abs(Math.toDegrees(target.getHeading() - pose.getHeading())) < hTolerance;
    }

    public void fieldCentricMove(Pose2d dir, double maxPower) {
        fieldCentricMove(dir.getX(), dir.getY(), dir.getHeading(), maxPower);
    }
    public void fieldCentricMove(double x, double y, double rx, double maxPower) {

        double botHeading = -pose.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1) * 1 / maxPower;
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        bot.motors.frontLeft.setPower(frontLeftPower);
        bot.motors.backLeft.setPower(backLeftPower);
        bot.motors.frontRight.setPower(frontRightPower);
        bot.motors.backRight.setPower(backRightPower);
    }
}
