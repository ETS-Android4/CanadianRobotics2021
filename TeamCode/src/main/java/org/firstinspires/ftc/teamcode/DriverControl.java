package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.foundation.MechanumHardware;

@TeleOp(name="Driver code", group="Linear Opmode")
@Disabled
public class DriverControl extends OpMode {

    MechanumHardware hw;
    DcMotor spinMotor;
    Servo claw;

    boolean claw_open = true;
    boolean a_held = false;

    @Override
    public void init() {
        hw = new MechanumHardware(hardwareMap);
        hw.init();
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        claw = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop() {
        hw.brake();

        if (Math.abs(gamepad1.left_stick_y) >= Math.abs(gamepad1.left_stick_x)) {
            hw.setDrivePower(gamepad1.left_stick_y);
        }

        else if (gamepad1.left_stick_x != 0) {
            hw.setTurnPower(gamepad1.left_stick_x);
        }

        if (gamepad1.right_stick_x != 0) {
            hw.setStrafePower(gamepad1.right_stick_x);
        }

        if (gamepad1.dpad_up) hw.moveArm1(-1);
        else if (gamepad1.dpad_down) hw.moveArm1(1);

        if (gamepad1.left_trigger != 0) hw.moveArm2(-1);
        else if (gamepad1.right_trigger != 0) hw.moveArm2(1);

        if (gamepad1.a) {
            if (!a_held) {
                claw_open = !claw_open;
                if (claw_open) claw.setPosition(0);
                else claw.setPosition(1);
            }
            a_held = true;
        }
        else {
            a_held = false;
        }
        spinMotor.setPower(0);
        if (gamepad1.x) spinMotor.setPower(1);
        if (gamepad1.y) spinMotor.setPower(-1);

        telemetry.addLine("arm " + hw.arm2Target);
        telemetry.update();
    }
}
