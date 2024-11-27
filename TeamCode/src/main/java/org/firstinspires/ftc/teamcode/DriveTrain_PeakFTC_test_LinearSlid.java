package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// into the deep code working with gobilda linear slide
@TeleOp
public class DriveTrain_PeakFTC_test_LinearSlid extends OpMode {

    // Declare motor variables
    private DcMotor TestSlide;// port number  rev expansion hub
    private DcMotor SlideHelper;// port number  control hub
    private ElapsedTime     runtime = new ElapsedTime();





    // Initialize hardware
    @Override
    public void init() {

        // Map motors to hardware configuration names
        TestSlide = hardwareMap.dcMotor.get("TestSlide");
        SlideHelper = hardwareMap.dcMotor.get("SlideHelper");


        // Set motor directions (adjust based on your robot)
        SlideHelper.setDirection(DcMotor.Direction.FORWARD);
        TestSlide.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes (RUN_USING_ENCODER for better control)
        TestSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TestSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlideHelper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideHelper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Starting at",  "%7d :%7d",
                SlideHelper.getCurrentPosition(),
                SlideHelper.getCurrentPosition());
        telemetry.update();


    }

    // Run the robot
    @Override
    public void loop() {

        double TestSlideVal = gamepad2.left_stick_y;
        double SlideHelperVal = gamepad2.right_stick_y;
        boolean armPwr = gamepad2.x;

        TestSlide.setPower(scaleInput(TestSlideVal));


        if(gamepad2.x)
            pickTheSample();
        else if(gamepad2.y) {
            dropTheSample();
        }

        telemetry.addData("Currently at", " at %7d",
                SlideHelper.getCurrentPosition());
        telemetry.update();

    }

    private void pickTheSample() {
        double currentPos = SlideHelper.getCurrentPosition();
        double pwr =0.4;
        double timeOutSec = 3;
        runtime.reset();
        while ( (currentPos < 1500) && (runtime.seconds()< timeOutSec) ) {
            currentPos = SlideHelper.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    SlideHelper.getCurrentPosition());
            telemetry.update();

            if(runtime.seconds()<1)
                pwr = 0.4;
            else if (runtime.seconds()<2)
                pwr = 0.3;
           else if (runtime.seconds()<3)
                pwr = 0.2;

            SlideHelper.setPower(pwr);
        }
        SlideHelper.setPower(-0.051);
        runtime.reset();
        while(runtime.seconds()<1 ){
            telemetry.addData("Currently at", " at %7d",
                    SlideHelper.getCurrentPosition());
            telemetry.update();
        }
        SlideHelper.setPower(0);
    }


    private void dropTheSample() {
        double currentPos = SlideHelper.getCurrentPosition();
        double pwr =-0.4;
        while (currentPos >100) {
            currentPos = SlideHelper.getCurrentPosition();
            telemetry.addData("Currently at", " at %7d",
                    SlideHelper.getCurrentPosition());
            telemetry.update();

            SlideHelper.setPower(pwr);
        }
        SlideHelper.setPower(0.15);
    }



    // Scale input values to keep them within the range of -1 to 1
    private double scaleInput(double input) {
        return Math.max(-.5, Math.min(.5, input));
    }

    private double arm1Power(double coordinates) {
        return Math.max(-.05, Math.min(.05, coordinates));
    }


    private double hangingPower(double coordinates) {
        return Math.max(-1, Math.min(1, coordinates));

    }

}