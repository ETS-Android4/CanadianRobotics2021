package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.foundation.MechanumHardware;

@Autonomous(name="Red Side", group="Autonomous")
public class RedSide extends CameraAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init_camera();
        hwm = new MechanumHardware(hardwareMap);
        hwm.init();
        hwm.closeClaw();
        int rect = getRect();
        hwm.setTarget(rect);
        hwm.encoderDrive(36, 1, 10000);
        hwm.imuTurn(-90);
        hwm.encoderDrive(5, 1, 1000);
        hwm.openClaw();
        sleep(1000);
        hwm.encoderDrive(-30, 1, 5000);
        hwm.encoderStrafe(-34, 1, 5000);
        hwm.spinMotor.setPower(-1);
        sleep(5000);
        hwm.spinMotor.setPower(0);
        hwm.encoderStrafe(26, 1, 5000);
    }
}
