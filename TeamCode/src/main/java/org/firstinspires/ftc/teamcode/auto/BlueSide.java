package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.foundation.MechanumHardware;

@Autonomous(name="Blue Side", group="Autonomous")
public class BlueSide extends CameraAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init_camera();
        hwm = new MechanumHardware(hardwareMap);
        hwm.init();
        hwm.closeClaw();
        int rect = getRect();
        telemetry.addData("num: ", rect);
        telemetry.addData("%num: ", rect %= hwm.targets.size());
        telemetry.update();
        hwm.setTarget(rect);
        // Line up with target
        hwm.encoderStrafe(26, 1, 5000);
        // Approach target
        hwm.encoderDrive(21, 1, 5000);
        // Release package
        hwm.openClaw();
        sleep(1000);
        // Back away from target
        hwm.encoderDrive(-15, 1, 2000);
        // Approach turnstile
        hwm.encoderStrafe(-48, 1, 10000);
        // Operate turnstile
        hwm.spinMotor.setPower(1);
        sleep(5000);
        hwm.spinMotor.setPower(0);
        // Park
        hwm.encoderDrive(20, 1, 5000);
        hwm.encoderStrafe(-2, 1, 5000);
    }
}
