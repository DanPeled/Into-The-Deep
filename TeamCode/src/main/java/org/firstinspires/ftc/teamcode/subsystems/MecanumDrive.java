package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;

public class MecanumDrive extends SubsystemBase {
    DcMotor fl, fr, bl, br;
    DistanceSensor distanceSensor;
    MecanumDriveKinematics kinematics;
    BNO055IMU imu;
    MecanumDriveWheelSpeeds wheelSpeeds;
    MecanumDriveOdometry odometry;
    int[] lastTick = {0, 0, 0, 0};
    double lastTime = 0;
    ElapsedTime time = new ElapsedTime();
    Pose2d pos;
    double correctedHeading;
    boolean isFieldOriented = true;
    final double ticksPerMeter = 1000;

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm) {
        fl = hm.dcMotor.get("fl_motor");
        fr = hm.dcMotor.get("fr_motor");
        br = hm.dcMotor.get("br_motor");
        bl = hm.dcMotor.get("bl_motor");
        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        imu = hm.get(BNO055IMU.class, "imu");

        Translation2d flLocation = new Translation2d(100, 164);
        Translation2d frLocation = new Translation2d(100, -164);
        Translation2d brLocation = new Translation2d(-100, -164);
        Translation2d blLocation = new Translation2d(-100, 164);
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, brLocation, blLocation);

        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()));
        time.reset();
    }

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm, Point start, Rotation2d startAngle) {
        fl = hm.dcMotor.get("fl_motor");
        fr = hm.dcMotor.get("fr_motor");
        br = hm.dcMotor.get("br_motor");
        bl = hm.dcMotor.get("bl_motor");
        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        imu = hm.get(BNO055IMU.class, "imu");

        Translation2d flLocation = new Translation2d(100, 164);
        Translation2d frLocation = new Translation2d(100, -164);
        Translation2d brLocation = new Translation2d(-100, -164);
        Translation2d blLocation = new Translation2d(-100, 164);
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, brLocation, blLocation);


        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(start.x, start.y, startAngle));
        time.reset();
    }

    @Override
    public void periodic() {
        int[] thisTick = {fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
        double currentTime = time.seconds();
        double deltaTime = currentTime - lastTime;
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        (thisTick[0] - lastTick[0]) / deltaTime * ticksPerMeter, (thisTick[1] - lastTick[1]) / deltaTime * ticksPerMeter,
                        (thisTick[2] - lastTick[2]) / deltaTime * ticksPerMeter, (thisTick[3] - lastTick[3]) / deltaTime * ticksPerMeter);

        Rotation2d gyroAngle = Rotation2d.fromDegrees(getHeading());

        pos = odometry.updateWithTime(currentTime, gyroAngle, wheelSpeeds);
        lastTime = currentTime;
        lastTick = thisTick;
    }

    public void drive(double x, double y, double rotation, double boost) {
        x *= boost;
        y *= boost;
        rotation *= boost;
        ChassisSpeeds speeds;
        if (isFieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(getAdjustedHeading()));
        } else {
            speeds = new ChassisSpeeds(x, y, rotation);
        }

// Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        double[] speedsArr = {wheelSpeeds.frontLeftMetersPerSecond,
        wheelSpeeds.frontRightMetersPerSecond,
        wheelSpeeds.rearLeftMetersPerSecond,
        wheelSpeeds.rearRightMetersPerSecond};
        speedsArr = modulateSpeeds(speedsArr);
        double frontLeft = speedsArr[0];
        double frontRight= speedsArr[1];
        double backLeft =  speedsArr[2];
        double backRight = speedsArr[3];
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle) % 360 + 180;
    }

    public void resetHeading() {
        correctedHeading = getHeading() + 180;
    }

    public void setHeading(double heading) {
        correctedHeading = heading + 180;
    }

    public double getAdjustedHeading() {
        return getHeading() + correctedHeading;
    }
    public Point getPosition(){
        return new Point(pos.getX(), pos.getY());
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.isFieldOriented = fieldOriented;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    public double[] modulateSpeeds(double[] speeds){
        double max = Math.max(Math.max(speeds[0],speeds[1]),Math.max(speeds[2],speeds[3]));
        if(Math.abs(max) > 1){
            for (int i = 0; i < 4; i++){
                speeds[i] /= max;
            }
        }
        return speeds;
    }


}
