package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Wheel diameter in inches

    private static final int TICKS_PER_REVOLUTION = 1440; // Encoder ticks per motor revolution

    private static final double GEAR_RATIO = 1.0; // Adjust for any gear ratio

    private static final double STRAFE_FACTOR = 1.1; // Adjust for strafing inefficiency
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REVOLUTION * GEAR_RATIO) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    private double distanceInchesFactor = 0;


    public AutoDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        distanceInchesFactor = (double) (( TICKS_PER_REVOLUTION * GEAR_RATIO) /

                (Math.PI * WHEEL_DIAMETER_INCHES * STRAFE_FACTOR));
        initTrain();
    }

    public void initTrain() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Map motors to hardware configuration names
        frontLeftMotor = hardwareMap.dcMotor.get("Fleft");
        frontRightMotor = hardwareMap.dcMotor.get("Fright");
        rearLeftMotor = hardwareMap.dcMotor.get("Rleft");
        rearRightMotor = hardwareMap.dcMotor.get("Rright");
        arm = hardwareMap.dcMotor.get("arm");
        hanging = hardwareMap.dcMotor.get("hanging");


        // Set motor directions (adjust based on your robot)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //reset the encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor modes (RUN_USING_ENCODER for better control)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hanging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)

        hand=hardwareMap.get(Servo.class,"hand");
        claw=hardwareMap.get(Servo.class,"claw");
        dropper = hardwareMap.get(Servo.class,"dropper");

        hand.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

    }

    public void moveRight(double inch){
       moveMotorInInchs(1,2, inch,5);
    }
    public void moveLeft(double inch){
        moveMotorInInchs(1,3,inch,5);
    }
    public void moveForward(double inch){

        moveMotorInInchs(1,0,inch,5);
       /*
        // Get gamepad inputs for driving
        int  targetPosition = (int) (COUNTS_PER_INCH * inch);
        int FleftPosition= frontLeftMotor.getCurrentPosition() + targetPosition;
        int FrightPosition = frontRightMotor.getCurrentPosition() + targetPosition;
        int RleftPosition = rearLeftMotor.getCurrentPosition() + targetPosition;
        int RrightPosition = rearRightMotor.getCurrentPosition()+ targetPosition;
        double drive = 1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        frontLeftMotor.setTargetPosition(FleftPosition);
        frontRightMotor.setTargetPosition(FrightPosition);
        rearLeftMotor.setTargetPosition(RleftPosition);
        rearRightMotor.setTargetPosition(RrightPosition);

        // Set motors to RUN_TO_POSITION mode

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        while ( frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&

                rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            telemetry.addData(" Target Position", targetPosition);
            telemetry.addData("F left Target Position", FleftPosition);
            telemetry.addData("F Left Current Position", frontLeftMotor.getCurrentPosition());
            telemetry.addData("F right Target Position", FrightPosition);
            telemetry.addData("F right Current Position", frontRightMotor.getCurrentPosition());
            telemetry.addData("Rear left Target Position", RleftPosition);
            telemetry.addData("Rear left Current Position", rearLeftMotor.getCurrentPosition());
            telemetry.addData("Rear right Target Position", RrightPosition);
            telemetry.addData("Rear right Current Position", rearRightMotor.getCurrentPosition());
            telemetry.update();

        }
        telemetry.addData(" Target Position", targetPosition);
        telemetry.addData("F left Target Position", FleftPosition);
        telemetry.addData("F Left Current Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("F right Target Position", FrightPosition);
        telemetry.addData("F right Current Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear left Target Position", RleftPosition);
        telemetry.addData("Rear left Current Position", rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear right Target Position", RrightPosition);
        telemetry.addData("Rear right Current Position", rearRightMotor.getCurrentPosition());
        telemetry.update();

        //rest encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        */
    }

    private void moveMotorInInchs(double pwr, int direction, double inches, double timeOut_Sec){
        int  targetPosition = (int) (COUNTS_PER_INCH * inches);
        int FleftPosition= 0;
        int FrightPosition = 0;
        int RleftPosition = 0;
        int RrightPosition = 0;
        double frontLeftPower =pwr;
        double frontRightPower=pwr;
        double rearLeftPower=pwr;
        double rearRightPower=pwr;

        if(direction == 0 ){ // move forward
            FleftPosition= frontLeftMotor.getCurrentPosition() + targetPosition;
            FrightPosition = frontRightMotor.getCurrentPosition() + targetPosition;
            RleftPosition = rearLeftMotor.getCurrentPosition() + targetPosition;
            RrightPosition = rearRightMotor.getCurrentPosition() + targetPosition;

        }else if(direction ==1 ){ // move reverse
            FleftPosition= frontLeftMotor.getCurrentPosition() - targetPosition;
            FrightPosition = frontRightMotor.getCurrentPosition() - targetPosition;
            RleftPosition = rearLeftMotor.getCurrentPosition() - targetPosition;
            RrightPosition = rearRightMotor.getCurrentPosition() - targetPosition;

        }else if(direction==2){ // move right
            FleftPosition = frontLeftMotor.getCurrentPosition() + targetPosition;
            FrightPosition = frontRightMotor.getCurrentPosition() - targetPosition;
            RleftPosition = rearLeftMotor.getCurrentPosition() - targetPosition;
            RrightPosition = rearRightMotor.getCurrentPosition() + targetPosition;

        }else if(direction ==3){ //more left
            FleftPosition = frontLeftMotor.getCurrentPosition() - targetPosition;
            FrightPosition = frontRightMotor.getCurrentPosition() + targetPosition;
            RleftPosition = rearLeftMotor.getCurrentPosition() + targetPosition;
            RrightPosition = rearRightMotor.getCurrentPosition() - targetPosition;

        }else{
            // do nothing
            frontLeftPower = frontRightPower=rearLeftPower= rearRightPower=0;
        }


        frontLeftMotor.setTargetPosition(FleftPosition);
        frontRightMotor.setTargetPosition(FrightPosition);
        rearLeftMotor.setTargetPosition(RleftPosition);
        rearRightMotor.setTargetPosition(RrightPosition);

        // Set motors to RUN_TO_POSITION mode

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        runtime.reset();
        while ( (runtime.seconds()<timeOut_Sec) && frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&
                rearLeftMotor.isBusy() && rearRightMotor.isBusy()){
            // do the wheel movment correction.
            double FleftPwrFactor =  frontLeftMotor.getCurrentPosition()  ;
            double RleftPwrFactor =  rearLeftMotor.getCurrentPosition();
            double FrightPwrFactor = frontRightMotor.getCurrentPosition();
            double RrightPwrFactor = rearRightMotor.getCurrentPosition();

            double factor = Math.min(Math.min(FleftPwrFactor,RleftPwrFactor), Math.min(FrightPwrFactor,RrightPwrFactor));

            double pwr1 = (factor-FleftPwrFactor)/factor*100;
            double pwr2 = (factor-RleftPwrFactor)/factor*100;
            double pwr3 = (factor-FrightPwrFactor)/factor*100;
            double pwr4 = (factor-RrightPwrFactor)/factor*100;

            // correction factor of the macenum sheel
            pwr1 = pwr+ pwr1*pwr;
            pwr2 = pwr+ pwr2*pwr;
            pwr3 = pwr+ pwr3*pwr;
            pwr4 = pwr+ pwr4*pwr;

            setMovePower(pwr1,pwr2,pwr3, pwr4);

        }

        telemetry.addData(" Target Position", targetPosition);
        telemetry.addData("F left Target Position", FleftPosition);
        telemetry.addData("F Left Current Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("F right Target Position", FrightPosition);
        telemetry.addData("F right Current Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear left Target Position", RleftPosition);
        telemetry.addData("Rear left Current Position", rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear right Target Position", RrightPosition);
        telemetry.addData("Rear right Current Position", rearRightMotor.getCurrentPosition());
        telemetry.update();
        //rest encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void moveBackward(double inch){
        moveMotorInInchs(1,1,inch,5);
       /*
        // Get gamepad inputs for driving
        int  targetPosition = (int) (COUNTS_PER_INCH * inch);
        int FleftPosition= frontLeftMotor.getCurrentPosition() - targetPosition;
        int FrightPosition = frontRightMotor.getCurrentPosition() - targetPosition;
        int RleftPosition = rearLeftMotor.getCurrentPosition() - targetPosition;
        int RrightPosition = rearRightMotor.getCurrentPosition() - targetPosition;
        double drive = -1;
        double strafe = 0;
        double rotate = 0;
        // Calculate motor powers based on Mecanum drive equations
        double frontLeftPower = drive - strafe - rotate;
        double frontRightPower = drive + strafe + rotate;
        double rearLeftPower = drive + strafe -rotate;
        double rearRightPower = drive - strafe + rotate;

        frontLeftMotor.setTargetPosition(FleftPosition);
        frontRightMotor.setTargetPosition(FrightPosition);
        rearLeftMotor.setTargetPosition(RleftPosition);
        rearRightMotor.setTargetPosition(RrightPosition);

        // Set motors to RUN_TO_POSITION mode

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMovePower(frontLeftPower,frontRightPower,rearLeftPower, rearRightPower);
        while ( frontLeftMotor.isBusy() && frontRightMotor.isBusy() &&

                rearLeftMotor.isBusy() && rearRightMotor.isBusy()) {
            telemetry.addData(" Target Position", targetPosition);
            telemetry.addData("F left Target Position", FleftPosition);
            telemetry.addData("F Left Current Position", frontLeftMotor.getCurrentPosition());
            telemetry.addData("F right Target Position", FrightPosition);
            telemetry.addData("F right Current Position", frontRightMotor.getCurrentPosition());
            telemetry.addData("Rear left Target Position", RleftPosition);
            telemetry.addData("Rear left Current Position", rearLeftMotor.getCurrentPosition());
            telemetry.addData("Rear right Target Position", RrightPosition);
            telemetry.addData("Rear right Current Position", rearRightMotor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData(" Target Position", targetPosition);
        telemetry.addData("F left Target Position", FleftPosition);
        telemetry.addData("F Left Current Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("F right Target Position", FrightPosition);
        telemetry.addData("F right Current Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear left Target Position", RleftPosition);
        telemetry.addData("Rear left Current Position", rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear right Target Position", RrightPosition);
        telemetry.addData("Rear right Current Position", rearRightMotor.getCurrentPosition());
        telemetry.update();
        //rest encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        */
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

