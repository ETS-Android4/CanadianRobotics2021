package org.firstinspires.ftc.teamcode.foundation;
// adb connect 192.168.43.1:5555
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

public class MechanumHardware {

    static final double COUNTS_PER_MOTOR_REV = 5281.1 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = (1/13.7) * 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * -DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Telemetry tel = null;
    HardwareMap hwm;

    public double arm1Target = 0;
    public double arm2Target = 0;
    public ArrayList<Integer> targets = new ArrayList<>();
    int current_target = 0;

    public BNO055IMU imu;

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor arm1 = null;
    public DcMotor arm2 = null;
    public DcMotor spinMotor;
    public Servo claw;

    private boolean initialized = false;

    public MechanumHardware(HardwareMap _hwm) {
        this.hwm = _hwm;
    }

    public void init()   {
        if (initialized) return;
        frontLeftDrive = hwm.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwm.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hwm.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hwm.get(DcMotor.class, "backRightDrive");
        spinMotor = hwm.get(DcMotor.class, "spinMotor");
        claw = hwm.get(Servo.class, "claw");

        arm1 = hwm.get(DcMotor.class, "arm1");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setPower(0.5);
        arm1Target = arm1.getCurrentPosition();
        arm1.setTargetPosition((int)arm1Target);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targets.add(-170);
        targets.add(-116);
        targets.add(-65);
        targets.add(0);

        arm2 = hwm.get(DcMotor.class, "arm2");
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setPower(0.5);
        arm2Target = arm1.getCurrentPosition();
        arm2.setTargetPosition((int)arm2Target);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523;
        parameters.useExternalCrystal = true;
        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initialized = true;
    }

    public void lock_wheels() {
        setDrivePower(1);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition());
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition());
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition());
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition());
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void nextTarget() {
        current_target %= targets.size();
        moveArm1To(targets.get(current_target));
        current_target++;
    }

    public void setTarget(int t) {
        t %= targets.size();
        current_target = t;
        moveArm1To(targets.get(current_target));
    }

    public void openClaw() {
        claw.setPosition(0.5);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }

    public void setMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeftDrive.setZeroPowerBehavior(mode);
        frontRightDrive.setZeroPowerBehavior(mode);
        backLeftDrive.setZeroPowerBehavior(mode);
        backRightDrive.setZeroPowerBehavior(mode);
    }

    public void moveArm1To(int pos) {
        arm1Target = pos;
        arm1.setTargetPosition(pos);
    }

    public void moveArm1(double delta) {
        arm1Target += delta;
        arm1.setTargetPosition((int)arm1Target);
    }

    public void moveArm2(double delta) {
        arm2Target += delta;
        arm2.setTargetPosition((int)arm2Target);
    }

    public void setOmniPower(float drive, float turn, float strafe) {
        frontLeftDrive.setPower(drive + turn + strafe);
        frontRightDrive.setPower(drive - turn - strafe);
        backLeftDrive.setPower(drive + turn - strafe);
        backRightDrive.setPower(drive - turn + strafe);
    }

    public void setDrivePower(float power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void setTurnPower(float power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    public void setStrafePower(float power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    public void brake() {
        setDrivePower(0);
    }

    public void encoderDrive(float distance, float power, int timeout) {
        distance *= -1;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition(
                frontLeftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        frontRightDrive.setTargetPosition(
                frontRightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        backLeftDrive.setTargetPosition(
                backLeftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        backRightDrive.setTargetPosition(
                backRightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        setDrivePower(1);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (timer.time() <= timeout &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
            if (tel != null) {
                tel.addData("Current: ", frontLeftDrive.getCurrentPosition());
                tel.addData("Target: ", frontLeftDrive.getTargetPosition());
                tel.update();
            }
        }
        lock_wheels();
    }

    public void encoderStrafe(double distance, float power, int timeout) {
        distance *= -1;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);frontLeftDrive.setTargetPosition(
                frontLeftDrive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH)
        );
        frontRightDrive.setTargetPosition(
                frontRightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        backLeftDrive.setTargetPosition(
                backLeftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        backRightDrive.setTargetPosition(
                backRightDrive.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH)
        );

        setDrivePower(1);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (timer.time() <= timeout &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
            if (tel != null) {
                tel.addData("Current: ", frontLeftDrive.getCurrentPosition());
                tel.addData("Target: ", frontLeftDrive.getTargetPosition());
                tel.update();
            }
        }
        lock_wheels();
    }

    public void imuTurn(double target) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double error = getError(target);
        double TOLERANCE = 0.5;
        while (Math.abs(error) > TOLERANCE) {
            error = getError(target);
            setTurnPower((float) (error / -45.f));
        }
        brake();
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getError(double target) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = target - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
}
