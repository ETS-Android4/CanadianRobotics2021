package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.foundation.MechanumHardware;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@Autonomous(name="imu test", group="Autonomous")
public class IMU_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MechanumHardware hw = new MechanumHardware(hardwareMap);
        hw.init();
        hw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.setMsTransmissionInterval(1);
        float power = -0.5f;
        boolean lock = false;
        waitForStart();
        while (opModeIsActive()) {
            float rot = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (rot >= 90 && !lock) {
                hw.lock_wheels();
                lock = true;
            }
            hw.setTurnPower(power);
            telemetry.addData("angle: ", rot);
            telemetry.update();
        }
    }
}
