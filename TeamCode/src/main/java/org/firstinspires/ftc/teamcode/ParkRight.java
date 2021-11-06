package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ParkRight", group="Autonomous")
public class ParkRight extends LinearOpMode {
    MechanumHardware hwm;
    @Override
    public void runOpMode() throws InterruptedException {
        hwm = new MechanumHardware(hardwareMap);
        hwm.init();
        hwm.arm1.setPower(0);
        hwm.arm2.setPower(0);
        hwm.tel = telemetry;
        waitForStart();
        hwm.encoderDrive(20, 1, 4000);
        hwm.imuTurn(90);
        hwm.encoderDrive(70, 1, 5000);
    }
}
