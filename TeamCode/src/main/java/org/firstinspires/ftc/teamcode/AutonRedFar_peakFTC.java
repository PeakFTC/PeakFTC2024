package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutonRedFar_peakFTC extends LinearOpMode {

    OpenCvWebcam webCam;
    imageTraingingRed pipeline;

    AutoColorSensor colorSensor ;
    AutoDriveTrain autoDriveTrain;

    imageTraingingRed.PropPosition propPosition ;



    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

         colorSensor = new AutoColorSensor(hardwareMap, telemetry);
         autoDriveTrain = new AutoDriveTrain(hardwareMap, telemetry);
         propPosition = imageTraingingRed.PropPosition.NONE;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new imageTraingingRed(telemetry);
        webCam.setPipeline(pipeline);
        sleep(1000);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        boolean i = true ;
        while (opModeIsActive()) {
            propPosition = pipeline.getAnalysis();

            telemetry.addData("Analysis",propPosition);
            telemetry.update();
            if(propPosition == imageTraingingRed.PropPosition.RIGHT)
            {
                autoDriveTrain.moveForward();
                sleep(650);
                autoDriveTrain.moveRight();
                // until Color sensor detect the spike mark blue/red depend aligns
                sleep(600);
                autoDriveTrain.moveStop();
//                autoDriveTrain.placePixel();
                sleep(1000);
                autoDriveTrain.moveBackward();
                sleep(500);
                autoDriveTrain.moveStop();
//                autoDriveTrain.offPlacer();
                sleep(30000);
                //drop the pixels
                // move to the parking
            } else if(propPosition == imageTraingingRed.PropPosition.LEFT){
                autoDriveTrain.moveForward();
                sleep(1400);
                autoDriveTrain.moveLeft();
                // until Color sensor detect the spike mark blue/red depend aligns
                sleep(1000);
                //autoDriveTrain.moveForward();
                //sleep(200);
                autoDriveTrain.moveStop();
                sleep(100);
//                autoDriveTrain.placePixel();
                sleep(500);
//                autoDriveTrain.offPlacer();
                autoDriveTrain.moveStop();
                //drop the pixels
                // move to the parking
                sleep(30000);
            }else if(propPosition == imageTraingingRed.PropPosition.CENTER){
                autoDriveTrain.moveForward();
                sleep(1300 );

                // Keep moving until Color sensor detect the spike mark blue/red depend aligns
                autoDriveTrain.moveStop();
//                autoDriveTrain.placePixel();
                sleep(1000);
//                autoDriveTrain.offPlacer();

                autoDriveTrain.moveBackward();
                sleep(500);
                autoDriveTrain.moveStop();
                // move to the parking

                sleep(30000);

            }else{
                // do nothing

            }


            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}