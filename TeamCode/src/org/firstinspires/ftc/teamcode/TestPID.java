package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.Consumer;
import java.util.function.Function;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders. This op mode will work with
 * either the MecBot or the XDriveBot robot configuration.
 */
@TeleOp(name = "TestPID", group = "OdomBot")
public class TestPID extends LinearOpMode {

    EncBot bot = new EncBot();
    double[] pose;
    BNO055IMU imu;
    Rotation rotation;

    public void runOpMode() {
        bot.init(hardwareMap);
        bot.resetOdometry(0, 0, 0);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        rotation = new Rotation(imu, new DcMotor[]{bot.motors[0], bot.motors[1]}, new DcMotor[]{bot.motors[2], bot.motors[3]});

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("time", this.time);
            telemetry.update();
        }

//        while (opModeIsActive()){
//            pose = bot.updateOdometry();
//            telemetry.addData("time", this.time);
//            telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.1f", pose[0], pose[1],
//                    Math.toDegrees(pose[2]));
//            telemetry.addData("Back Left", "T = %d  V = %.0f", bot.motors[0].getCurrentPosition(), bot.motors[0].getVelocity());
//            telemetry.addData("Front Left", "T = %d  V = %.0f", bot.motors[1].getCurrentPosition(), bot.motors[1].getVelocity());
//            telemetry.addData("Front Right", "T = %d  V = %.0f", bot.motors[2].getCurrentPosition(), bot.motors[2].getVelocity());
//            telemetry.addData("Back Right", "T = %d  V = %.0f", bot.motors[3].getCurrentPosition(), bot.motors[3].getVelocity());
//            telemetry.addData("Right Enc", "T = %d  V = %.0f", bot.encoders[0].getCurrentPosition(), bot.encoders[0].getVelocity());
//            telemetry.addData("Left Enc", "T = %d  V = %.0f", bot.encoders[1].getCurrentPosition(), bot.encoders[1].getVelocity());
//            telemetry.addData("X Enc", "T = %d  V = %.0f", bot.encoders[2].getCurrentPosition(), bot.encoders[2].getVelocity());
//            telemetry.update();
//            double px = gamepad1.left_stick_x;
//            double py = -gamepad1.left_stick_y;
//            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
//            bot.setDrivePower(px, py, pa);
//        }
        GenericPID.PID(90.0, (target) -> rotation.getError(target), (power) -> rotation.setPower(power));
    }
}

class Rotation {
    BNO055IMU imu;
    DcMotor[] left;
    DcMotor[] right;

    public Rotation(BNO055IMU imu) {
        this.imu = imu;
    }

    public Rotation(BNO055IMU imu, DcMotor[] left, DcMotor[] right) {
        this.imu = imu;
        this.left = left;
        this.right = right;
    }

    public double getAngle() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getError(double target) {
        return getError(this.getAngle(), target);
    }

    public static double getError(double start, double target) {
        double error = ((target - start + 540) % 360) - 180;
        if (Math.abs(error) > 180) {
            if (error > 0) {
                error -= 360;
            } else {
                error += 360;
            }
        }
        return error;
    }

    public void setPower(double power) {
        if (left.length < 1 || right.length < 1) {
            System.out.println("Error, missing motors");
            return;
        }
        for (DcMotor motor : left) {
            motor.setPower(power);
        }
        for (DcMotor motor : right) {
            motor.setPower(-power);
        }
    }
}
class GenericPID {
    public static void PID(double target, Function<Double, Double> getError, Consumer<Double> correctError) {
        float tolerance = 2;
//        long delay = 13;
        float attempts = 160;

        // snappier
        // float kp = 0.02;
        // float ki = 0.00000;
        // float kd = 0.05;

        // smoother but a bit slow ending
        double kp = 0.05;
        double ki = 0.00000;
        double kd = 0.2;

        float i = 0;
        double lastError = 0;

        while (Math.abs(getError.apply(target)) > tolerance) {
            double error = getError.apply(target);
//            i += (error * delay);
            i += error;
            double d = (error - lastError);
            double power = kp * error + ki * i + kd * d;
            if (power > 0) {
                power = Math.min(1, Math.max(0.01, power));
            } else {
                power = Math.max(-1, Math.min(-0.01, power));
            }
            correctError.accept(power);
            lastError = error;
            if (power > 0 && power < 1) {
                System.out.println(power);
            }
            if (power < 0 && power > -1) {
                System.out.println(power);
            }
//            try {
//                LinearOpMode.class.wait(delay);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//                return;
//            }
            attempts--;
            if (attempts <= 0) {
                System.out.println(power);
                System.out.println(error);
                break;
            }
            for (int j = 0; j < 10; j++) {
                if (Math.abs(getError.apply(target)) < tolerance) {
//                    try {
//                        LinearOpMode.class.wait(10);
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                        return;
//                    }
                } else {
                    break;
                }
            }
        }
        correctError.accept(0.0);
        System.out.println("Done!");
    }
}