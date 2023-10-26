package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import virtual_robot.util.AngleUtils;

/**
 * Utility class that represents a robot with mecanum drive wheels and three "dead-wheel" encoders.
 */
public class EncBot {

    public final double WHEEL_DIAMETER = 4;
    public final double INTER_WHEEL_WIDTH = 16;
    public final double INTER_WHEEL_LENGTH = 14;
    public final double TICKS_PER_DRIVE_ROTATION = 1120;
    public final double TICKS_PER_ENCODER_ROTATION = 1120;
    public final double ENCODER_WHEEL_DIAMETER = 2;
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0;
    private final double ENCODER_WIDTH = 12.0;

    public final Motors motors = new Motors();

    public static class Motors {
        DcMotorEx backLeft;
        DcMotorEx frontLeft;
        DcMotorEx frontRight;
        DcMotorEx backRight;

        public Motors() {
            backLeft = null;
            frontLeft = null;
            frontRight = null;
            backRight = null;
        }

        public void init(HardwareMap hwMap) {
            backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
            frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
            frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
            backRight = hwMap.get(DcMotorEx.class, "back_right_motor");
        }
    }
    public final DcMotorEx[] encoders = new DcMotorEx[3]; //right, left, X

    public int[] prevTicks = new int[3];

    public double[] pose = new double[3];

    public void init(HardwareMap hwMap){
        motors.init(hwMap);
        motors.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        String[] encoderNames = new String[]{"enc_right", "enc_left", "enc_x"};
        for (int i=0; i<3; i++) encoders[i] = hwMap.get(DcMotorEx.class, encoderNames[i]);
    }

    public void resetOdometry(double x, double y, double headingRadians){
        pose[0] = x;
        pose[1] = y;
        pose[2] = headingRadians;
        for (int i=0; i<3; i++) prevTicks[i] = encoders[i].getCurrentPosition();
    }

    public Pose2d updateOdometry() {
        double[] pose = internalUpdate();
        return new Pose2d(-pose[1], pose[0], -pose[2]);
    }

    private double[] internalUpdate(){
        int[] ticks = new int[3];
        for (int i=0; i<3; i++) ticks[i] = encoders[i].getCurrentPosition();
        int newRightTicks = ticks[0] - prevTicks[0];
        int newLeftTicks = ticks[1] - prevTicks[1];
        int newXTicks = ticks[2] - prevTicks[2];
        prevTicks = ticks;
        double rightDist = newRightTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double leftDist = -newLeftTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double dyR = 0.5 * (rightDist + leftDist);
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
        double dxR = -newXTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double avgHeadingRadians = pose[2] + headingChangeRadians / 2.0;
        double cos = Math.cos(avgHeadingRadians);
        double sin = Math.sin(avgHeadingRadians);
        pose[0] += dxR*sin + dyR*cos;
        pose[1] += -dxR*cos + dyR*sin;
        pose[2] = AngleUtils.normalizeRadians(pose[2] + headingChangeRadians);
        return pose;
    }

    public double[] getPose(){
        return pose;
    }

}
