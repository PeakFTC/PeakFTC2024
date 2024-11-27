package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoDriveTrain{

    private DcMotor frontLeftMotor;// port number 0 rev expansion hub
    private DcMotor frontRightMotor;// port number 1 control hub
    private DcMotor rearLeftMotor;//port number 2expansion hub
    private DcMotor rearRightMotor;// port number 3 control hub
    private DcMotor hanging;// port number 1 expansion hub
    private DcMotor arm;// port number 2 expansion hub

    //Intake things
    private Servo claw;// EX port 4
    private Servo hand; // Ex port 5
    private Servo dropper;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public AutoDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initTrain();
    }

    public void initTrain() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Map motors to hardware configuration names
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeft");
        rearRightMotor = hardwareMap.dcMotor.get("rearRight");


        // Set motor directions (adjust based on your robot)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes (RUN_USING_ENCODER for better control)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)


    }




    public void moveRight(){
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = 1;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;
        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }
    public void moveLeft(){
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = -1;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }
    public void moveForward(){
        // Get gamepad inputs for driving
        double drive = -1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }

    public void moveBackward(){
        // Get gamepad inputs for driving
        double drive = 1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor
    }

    public void moveStop() {
        // Get gamepad inputs for driving
        double drive = 0;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        //set the power for all DC motor

    }
    private void setMovePower(double fLeftPower,double fRightPower, double rLeftPower,double rRightPower )   {
        // Set motor powers (scale to keep values between -1 and 1)
        frontLeftMotor.setPower(scaleInput(fLeftPower));
        frontRightMotor.setPower(scaleInput(fRightPower));
        rearLeftMotor.setPower(scaleInput(rLeftPower));
        rearRightMotor.setPower(scaleInput(rRightPower));
    }

    // Scale input values to keep them within the range of -1 to 1
    private double scaleInput(double input) {
        return Math.max(-1, Math.min(1, input));
    }

}

