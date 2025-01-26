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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.IMU_Integrator;
import org.opencv.core.Point;

public class MecanumDrive extends SubsystemBase {
    DcMotorEx fl, fr, bl, br;
    DistanceSensor distanceSensor;
    MecanumDriveKinematics kinematics, odometryKinematics;
    BNO055IMU imu;
    MecanumDriveWheelSpeeds wheelSpeeds;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    MecanumDriveOdometry odometry;

    ElapsedTime time = new ElapsedTime();
    Pose2d pos;
    double correctedHeading;
    boolean isFieldOriented = true;

    MultipleTelemetry telemetry;
    Point startingPosition;
    double forwardTicksPerMeter = 1781, strafeTicksPerMeter = 2032;
    double tickPerMeter = 1783;
    LinearOpMode opMode;
    double startAngle = 0;

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm, LinearOpMode opMode) {
        this.telemetry = telemetry;
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        this.opMode = opMode;
        imu = hm.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        startingPosition = new Point(0, 0);
        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, forwardTicksPerMeter, strafeTicksPerMeter, startingPosition, startAngle, fl, fr, bl, br, telemetry);
        imu.initialize(parameters);

        Translation2d flLocation = new Translation2d(100, 164);//164
        Translation2d frLocation = new Translation2d(100, -164);//-164
        Translation2d brLocation = new Translation2d(-100, -164);//-164
        Translation2d blLocation = new Translation2d(-100, 164);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);
//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(startingPosition.x,startingPosition.y,Rotation2d.fromDegrees(0)));
        time.reset();
        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);
    }

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm, Point start, double startAngle, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startingPosition = start;
        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        this.startAngle = startAngle;
//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);
        Translation2d flLocation = new Translation2d(164, 100);//164
        Translation2d frLocation = new Translation2d(164, -100);//-164
        Translation2d brLocation = new Translation2d(-164, -100);//-164
        Translation2d blLocation = new Translation2d(-164, 100);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);


        imu = hm.get(BNO055IMU.class, "imu");
        initIMU(imu, opMode);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(start.x, start.y, startAngle));
        time.reset();
    }


    private void initIMU(BNO055IMU imu, LinearOpMode robot) {
        this.imu = imu;

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        RobotLog.d("imu params init start");
        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, forwardTicksPerMeter, strafeTicksPerMeter, startingPosition, startAngle, fl, fr, bl, br, telemetry);
        RobotLog.d("imu init");
        imu.initialize(parameters);
        RobotLog.d("imu init finished");


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !robot.isStopRequested() && timer.seconds() < 5) {
            robot.sleep(50);
        }
        if (imu.isGyroCalibrated()) {
            robot.telemetry.addData("Gyro", "Done Calibrating");
            RobotLog.d("Gyro done init");

        } else {
            robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
        }

        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);

        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
        RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
    }


    public void drive(double x, double y, double rotation, double boost) {
        x *= boost * 3;
        y *= boost * 3;
        rotation *= boost * 2;
        ChassisSpeeds speeds;
        if (isFieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, 0, Rotation2d.fromDegrees(getAdjustedHeading()));
        } else {
            speeds = new ChassisSpeeds(-x, y, 0);
        }

// Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        double[] speedsArr = {
                wheelSpeeds.frontLeftMetersPerSecond - rotation,
                wheelSpeeds.frontRightMetersPerSecond - rotation,
                wheelSpeeds.rearLeftMetersPerSecond + rotation,
                wheelSpeeds.rearRightMetersPerSecond + rotation};
        speedsArr = modulateSpeeds(speedsArr);
        double frontLeft = speedsArr[0];
        double frontRight = speedsArr[1];
        double backLeft = speedsArr[2];
        double backRight = speedsArr[3];
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
//        telemetry.addData("x",x);
//        telemetry.addData("y",y);
        telemetry.addData("fl", fl.getCurrentPosition());
        telemetry.addData("fr", fr.getCurrentPosition());
        telemetry.addData("bl", bl.getCurrentPosition());
        telemetry.addData("br", br.getCurrentPosition());
        telemetry.addData("posx", imu.getPosition().x);
        telemetry.addData("posy", imu.getPosition().y);

        telemetry.update();
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

    public Point getPosition() {
        return new Point(imu.getPosition().x, imu.getPosition().y);
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.isFieldOriented = fieldOriented;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double[] modulateSpeeds(double[] speeds) {
        double max = Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.max(Math.abs(speeds[2]), Math.abs(speeds[3])));
        if (Math.abs(max) > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= max;
            }
        }
        return speeds;
    }


}
