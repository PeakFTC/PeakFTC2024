package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutonBlueFar_peakFTC extends LinearOpMode {

    AutoDriveTrain autoDriveTrain;


    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */


         autoDriveTrain = new AutoDriveTrain(hardwareMap, telemetry);






//            @Override

//            @Override


        waitForStart();
        boolean i = true ;
        while (opModeIsActive()) {
                autoDriveTrain.moveForward();
                sleep(650);
                autoDriveTrain.moveRight();
                // until Color sensor detect the spike mark blue/red depend aligns
                sleep(600);
                autoDriveTrain.moveStop();
//                autoDriveTrain.placePixel();
                sleep(500);
//                autoDriveTrain.offPlacer();
                autoDriveTrain.moveBackward();
                sleep(50);
                autoDriveTrain.moveStop();;

                //drop the pixels
                // move to the parking



            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}