package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class AutonBlueNear_IntoDeep extends LinearOpMode {


    AutoDriveTrain autoDriveTrain;



    @Override
    public void runOpMode() {
        double pwr =1;
        autoDriveTrain = new AutoDriveTrain(hardwareMap, telemetry);
        // encoder reading
        double a=12,b=4,c=5,d=6;
        sleep(1000);

        waitForStart(); // wait to press unpause button

        autoDriveTrain.moveBackward(24);
        autoDriveTrain.moveStop();
        sleep(5000);
        autoDriveTrain.moveLeft(12);
        autoDriveTrain.moveStop();
        sleep(5);
        autoDriveTrain.moveForward(24);
        autoDriveTrain.moveStop();
        sleep(5);
        autoDriveTrain.moveRight(12);
        autoDriveTrain.moveStop();
        sleep(5);



           // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
    }
}

