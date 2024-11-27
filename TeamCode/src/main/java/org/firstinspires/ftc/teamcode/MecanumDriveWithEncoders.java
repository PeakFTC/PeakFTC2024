package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Strafe Mecanum in Inches")

public class MecanumDriveWithEncoders extends LinearOpMode {



    // Declare motor objects

    private DcMotor frontLeft;

    private DcMotor frontRight;

    private DcMotor backLeft;

    private DcMotor backRight;



    // Constants

    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Wheel diameter in inches

    private static final int TICKS_PER_REVOLUTION = 1440; // Encoder ticks per motor revolution

    private static final double GEAR_RATIO = 1.0; // Adjust for any gear ratio

    private static final double STRAFE_FACTOR = 1.1; // Adjust for strafing inefficiency



    @Override

    public void runOpMode() {

        // Initialize motors

        frontLeft = hardwareMap.get(DcMotor.class, "Fleft");

        frontRight = hardwareMap.get(DcMotor.class, "Fright");

        backLeft = hardwareMap.get(DcMotor.class, "Rleft");

        backRight = hardwareMap.get(DcMotor.class, "Rright");



        // Reverse necessary motors

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);



        // Reset encoders

       // resetEncoders();



        // Wait for the game to start

        waitForStart();
        moveLeft();
        sleep(3000);
        moveRight();
        sleep(3000);
        stopMotors();

        // Strafe 12 inches to the right

//        strafeInches(12, 0.5);



        // Strafe 12 inches to the left

  //      strafeInches(-12, 0.5);

    }

    private void moveLeft() {

        double pwr = 0.5;
        // Fleft, Rright Forward
        frontLeft.setPower(pwr);
        backRight.setPower(pwr-0.1);
        frontRight.setPower(-pwr);
        backLeft.setPower(-pwr+0.1);
    }

    private void moveRight() {
        double pwr = 0.5;
        // Fleft, Rright reverse
        frontLeft.setPower(-pwr);
        backRight.setPower(-pwr+0.1);
        frontRight.setPower(pwr);
        backLeft.setPower(pwr-0.09);
    }


    /**

     * Strafes the robot a specified distance in inches.

     *

     * @param distanceInches Distance to strafe in inches (positive for right, negative for left)

     * @param power Motor power (0.0 to 1.0)

     */

    private void strafeInches(double distanceInches, double power) {

        int targetPosition = (int) ((distanceInches * TICKS_PER_REVOLUTION * GEAR_RATIO) /

                (Math.PI * WHEEL_DIAMETER_INCHES * STRAFE_FACTOR));





        // Set target positions for strafing

        frontLeft.setTargetPosition(targetPosition);

        frontRight.setTargetPosition(-targetPosition);

        backLeft.setTargetPosition(-targetPosition);

        backRight.setTargetPosition(targetPosition);



        // Set motors to RUN_TO_POSITION mode

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Set motor power

        frontLeft.setPower(power);

        frontRight.setPower(power);

        backLeft.setPower(power);

        backRight.setPower(power);



        // Wait for motors to reach target position

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() &&

                backLeft.isBusy() && backRight.isBusy())) {

            telemetry.addData("Target Position", targetPosition);

            telemetry.addData("Current Position", frontLeft.getCurrentPosition());

            telemetry.update();

        }



        // Stop motors

        stopMotors();



        // Reset mode to RUN_USING_ENCODER

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    /**

     * Stops all motors.

     */

    private void stopMotors() {

        frontLeft.setPower(0);

        frontRight.setPower(0);

        backLeft.setPower(0);

        backRight.setPower(0);

    }



    /**

     * Resets all motor encoders.

     */

    private void resetEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}