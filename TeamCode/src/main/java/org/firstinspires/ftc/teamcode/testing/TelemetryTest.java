package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="telemetry", group="Autonomous")
public class TelemetryTest extends LinearOpMode {
    int test = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(1);
        Telemetry.Item item = telemetry.addData("+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+", "");
        waitForStart();
        while (opModeIsActive()) {
            //test += 1;
            //item.setValue(test);
            telemetry.update();
        }

    }
}
