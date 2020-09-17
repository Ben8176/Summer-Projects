package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.TimerUtil;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.util.MathUtil.TAU;

public class TestDrivetrain {

    MathUtil myMath = new MathUtil();
    TimerUtil myTimer = new TimerUtil();

    Telemetry telemetry;
    ExpansionHubMotor m0, m1, m2, m3;

    //misc. constants
    private final double TRACK_WIDTH = 301; //wheelbasediameter in mm
    private final double WHEEL_DIAMETER = 64; //mm
    private final double MAX_WHEEL_RPM  = 550;
    private double MAX_ROBOT_RPM  = 60;
    private double MAX_MODULE_RPM = 50;
    private final double MAX_MOTOR_RPM  = 1150;
    private final double TICKS_PER_REV = 145.6;

    //Gear ratio constants
    private final double INTERMEDIATE_TO_MOTOR = 16/8;
    private final double MODULE_TO_INTERMEDIATE = 52/13;
    private final double WHEEL_TO_MODULE = 0.25;
    private final double COMMON_GEAR_RATIO = MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;

    private final double TICKS_PER_MODULE_REV =
            TICKS_PER_REV * INTERMEDIATE_TO_MOTOR * MODULE_TO_INTERMEDIATE;

    // PID constants
    private double kRobot = .5;
    private double kModule = 0.12;
    private double kD = 0.7;

    double leftSign = 1;
    double rightSign = 1;
    int leftInvert = 1, rightInvert = 1;
    double prevLeftAlpha = 0;
    double prevRightAlpha = 0;
    double prevTime = 0;

    public TestDrivetrain(ExpansionHubMotor m0, ExpansionHubMotor m1, ExpansionHubMotor m2, ExpansionHubMotor m3, Telemetry telemetry) {
        this.m0 = m0;
        this.m1 = m1;
        this.m2 = m2;
        this.m3 = m3;
        this.telemetry = telemetry;
    }

    public void updateSwerve(Vector2d rotVec, Vector2d transVec, double heading, RevBulkData cdata, RevBulkData edata) {

        double rotSign;
        double robotRPM;

        double rotLeftTheta;
        double rotRightTheta;

        double currentTime = myTimer.getMillis();

        double m0Encoder =  cdata.getMotorCurrentPosition(0);
        double m1Encoder = -cdata.getMotorCurrentPosition(1);
        double m2Encoder =  edata.getMotorCurrentPosition(0);
        double m3Encoder = -edata.getMotorCurrentPosition(1);

        double rotPower = rotVec.magnitude();
        double rotTheta = Math.toDegrees(rotVec.getAngle());

        double transPower = Range.clip(transVec.magnitude(), 0.0, 1.0);
        double transTheta = Math.toDegrees(transVec.getAngle());

        //current robot heading
        double rotPhi = heading;
        // difference in desired robot heading and current robot heading
        double rotAlpha = myMath.wrapTo180(rotTheta - rotPhi);

        //
        if (rotAlpha < 0) { rotSign = -1; }
        else if (rotAlpha != 0) { rotSign = 1; }// Needed to avoid divide by 0 error
        else { rotSign = 0; }

        // Left and right module headings as estimated by motor encoders
        double leftModulePhi =
                myMath.wrapTo180(-360 * ((m0Encoder + m1Encoder) / 2) / TICKS_PER_MODULE_REV);
        double rightModulePhi =
                myMath.wrapTo180(-360 * ((m2Encoder + m3Encoder) / 2) / TICKS_PER_MODULE_REV);

        //set the robot RPM to either MAX RPM or some P loop with angle input
        robotRPM = Math.min(kRobot * Math.abs(rotAlpha), MAX_ROBOT_RPM);

        //get module direction in order to turn robot

        if (rotSign != 0) {
            rotLeftTheta = 90 + (90 * rotSign);
            rotRightTheta = 90 - (90 * rotSign);
        }
        else {
            rotLeftTheta = leftModulePhi;
            rotRightTheta = rightModulePhi;
        }

        double rotWheelRPM = robotRPM * (TRACK_WIDTH / WHEEL_DIAMETER);

        double transLeftTheta = myMath.wrapTo180(transTheta - rotPhi);
        double transRightTheta = myMath.wrapTo180(transTheta - rotPhi);
        //think of something better to get full powers
        double transWheelRPM = transPower * (MAX_WHEEL_RPM - Math.abs(rotWheelRPM));

        //x y component of left module
        double leftX = rotWheelRPM * myMath.cosDeg(rotLeftTheta) + transWheelRPM * myMath.cosDeg(transLeftTheta);
        double leftY = rotWheelRPM * myMath.sinDeg(rotLeftTheta) + transWheelRPM * myMath.sinDeg(transLeftTheta);
        //x y component of right module
        double rightX = rotWheelRPM * myMath.cosDeg(rotRightTheta) + transWheelRPM * myMath.cosDeg(transRightTheta);
        double rightY = rotWheelRPM * myMath.sinDeg(rotRightTheta) + transWheelRPM * myMath.sinDeg(transRightTheta);

        //get final module headings and RPM from components
        double leftModuleTheta  = Math.toDegrees(Math.atan2(leftY, leftX));
        double leftWheelRPM     = myMath.magnitude(leftX, leftY);
        double rightModuleTheta = Math.toDegrees(Math.atan2(rightY, rightX));
        double rightWheelRPM    = myMath.magnitude(rightX, rightY);

        double leftAlpha  = myMath.wrapTo180(leftModuleTheta  - leftModulePhi );
        double rightAlpha = myMath.wrapTo180(rightModuleTheta - rightModulePhi);

        //check which quadrant leftalpha is in
        if (leftAlpha > 90) { //quadrant 2
            leftSign = -1;
            leftInvert = -1;
            leftAlpha = myMath.wrapTo180(leftAlpha + 180);
        }
        else if (leftAlpha < -90) { //quadrant 3
            leftSign = 1;
            leftInvert = -1;
            leftAlpha = myMath.wrapTo180(leftAlpha + 180);
        }
        else if (leftAlpha > 0) { //quadrant 1
            leftSign = 1;
            leftInvert = 1;
            leftAlpha = myMath.wrapTo180(leftAlpha);
        }
        else if (leftAlpha != 0) { //quadrant 4
            leftSign = -1;
            leftInvert = 1;
            leftAlpha = myMath.wrapTo180(leftAlpha);
        }
        else { leftSign = 0; }

        //check which quadrant rightAlpha is in
        if (rightAlpha > 90) { //quadrant 2
            rightSign = -1;
            rightInvert = -1;
            rightAlpha = myMath.wrapTo180(rightAlpha + 180);
        }
        else if (rightAlpha < -90) { //quadrant 3
            rightSign = 1;
            rightInvert = -1;
            rightAlpha = myMath.wrapTo180(rightAlpha + 180);
        }
        else if (rightAlpha > 0) { //quadrant 1
            rightSign = 1;
            rightInvert = 1;
            rightAlpha = myMath.wrapTo180(rightAlpha);
        }
        else if (rightAlpha != 0) { //quadrant 4
            rightSign = -1;
            rightInvert = 1;
            rightAlpha = myMath.wrapTo180(rightAlpha);
        }
        else { rightAlpha = 0; }

        /*
        Derivative control for module turning pid
         */
        double leftModuleD = kD * (leftAlpha - prevLeftAlpha) / (currentTime - prevTime);
        double rightModuleD = kD * (rightAlpha - prevRightAlpha) / (currentTime - prevTime);

        rightSign = Math.copySign(1, -rightAlpha);
        leftSign = Math.copySign(1, -leftAlpha);

        //get module RPMs based on angle and max module rpm
//        double leftRPM  = Math.min(kModule * Math.abs(myMath.wrapTo180(leftAlpha)),  MAX_MODULE_RPM);
        double leftRPM  = leftModuleD + kModule * Math.abs(myMath.wrapTo180(leftAlpha));
//        double rightRPM = Math.min(kModule * Math.abs(myMath.wrapTo180(rightAlpha)), MAX_MODULE_RPM);
        double rightRPM  = rightModuleD + kModule * Math.abs(myMath.wrapTo180(rightAlpha));

        //normalize RPMs to be below MAX_MOTOR_RPM
        double wheelModifier = 0.05;
//                1 - (Math.max(leftRPM, rightRPM) * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR) / (MAX_MOTOR_RPM);

        double safety = 1;
        /*
        Get final motor RPMs
        Put a negative sign in two calculations to make sure modules and wheels are turning coherently
         */
        double m0RPM =
                (-(wheelModifier * leftInvert * leftWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + leftSign * leftRPM * COMMON_GEAR_RATIO) * safety;
        double m1RPM =
                ( (wheelModifier * leftInvert * leftWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + leftSign * leftRPM * COMMON_GEAR_RATIO) * safety;
        double m2RPM =
                (-(wheelModifier * rightInvert * rightWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + rightSign * rightRPM * COMMON_GEAR_RATIO) * safety;
        double m3RPM =
                ( (wheelModifier * rightInvert * rightWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + rightSign * rightRPM * COMMON_GEAR_RATIO) * safety;

        //RPM to Radians Per Second conversion
        double RPM_to_RadPerSec = TAU / 60;

        //set motor velocities

        m0.setVelocity(m0RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
        m1.setVelocity(m1RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
        m2.setVelocity(m2RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
        m3.setVelocity(m3RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);

        telemetry.addData("wheelModifier", wheelModifier);
        telemetry.addData("m0 wheel RPM", -wheelModifier * leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR);
        telemetry.addData("m1 wheel RPM", wheelModifier * leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR);
        telemetry.addData("leftModuleRPM", leftRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR);
        telemetry.addData("leftRPM", leftRPM);
        telemetry.addData("leftTHETA", leftModuleTheta);
        telemetry.addData("leftPHI", leftModulePhi);
        telemetry.addData("leftwheelRPM no modifier:",leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR);
        telemetry.addData("module + wheel", leftRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR + leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR);

        prevLeftAlpha = leftAlpha;
        prevRightAlpha = rightAlpha;
        prevTime = currentTime;
    }


}
