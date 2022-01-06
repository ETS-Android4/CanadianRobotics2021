package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.foundation.MechanumHardware;

import java.util.ArrayList;

@TeleOp(name="Driver code Logan", group="Linear Opmode")
public class DriveControlLogan extends OpMode {
    MechanumHardware hw;
    float speed_mod = 1.f;

    @Override
    public void init() {
        hw = new MechanumHardware(hardwareMap);
        hw.init();
        hw.openClaw();
        hw.arm1.setPower(0.5);
    }

    @Override
    public void loop() {
        // Wheel control
        hw.brake();
        if (gamepad1.left_trigger != 0) speed_mod = 0.5f;
        if (gamepad1.right_trigger != 0) speed_mod = 1.f;
        float drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive) * speed_mod;
        float turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn) * speed_mod;
        float strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe) * speed_mod;
        hw.setOmniPower(drive, turn, strafe);
        // Arm 1 (up and down)
        if (gamepad1.dpad_up) hw.moveArm1(-1);
        if (gamepad1.dpad_down) hw.moveArm1(1);

        // Arm 2 (left and right)
        hw.moveArm1(gamepad2.left_stick_y);
        hw.moveArm2(gamepad2.right_stick_x);
        if (gamepad2.dpad_up) hw.moveArm1(-1);
        if (gamepad2.dpad_down) hw.moveArm1(1);
        if (gamepad2.dpad_right) hw.moveArm2(1);
        if (gamepad2.dpad_left) hw.moveArm2(-1);

        if (gamepad2.a) hw.setTarget(3);
        if (gamepad2.x) hw.setTarget(2);
        if (gamepad2.y) hw.setTarget(1);
        if (gamepad2.b) hw.setTarget(0);
        // Claw
        if (gamepad2.left_trigger != 0) hw.openClaw();
        else if (gamepad2.right_trigger != 0) hw.closeClaw();
        // Spinner
        if (gamepad2.left_bumper) hw.spinMotor.setPower(-1);
        if (gamepad2.right_bumper) hw.spinMotor.setPower(1);
        if (gamepad2.left_bumper == gamepad2.right_bumper)
            hw.spinMotor.setPower(0);

        telemetry.addLine("arm " + hw.arm1Target);
        telemetry.update();
    }
}
