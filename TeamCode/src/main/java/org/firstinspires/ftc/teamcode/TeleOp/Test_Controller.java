package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TestController")
public class Test_Controller extends LinearOpMode {

    private Boolean CrSensLastDep = true;//ultima directie in care am mers(scos sau bagat)
    private Boolean GlSensLastDep = true;//ultima directie in care am mers(sus sau jos)
    double glPwr = 0.8;//putere glisiera
    double crPwr = 0.8;//putere cremaliera
    boolean crMg_OK;//daca senzorul de pe cremaliera simte unul dintre magneti
    boolean glMg_OK;//daca senzorul de pe glisiera simte unul dintre magneti
    boolean prevX = false, prevLeft = false, prevRight = false;
    double speed, robotAngle;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    public final double offTLD = 3.25;
    public final double offBleftDriveD = 2.25;
    public final double offTRD = 4;
    public final double offBrightDriveD = 6.5;

    double ok = 0;
    double power = 1;//1.41
    double FranaS, FranaD;
    boolean leftstickpress; //Outside of loop()
    double intakePower = 0;
    boolean intakeOk = true;
    int target = 0;
    int intakePowerR = 0;
    boolean intakeOkR = true;
    boolean changed = false;
    //DcMotor TleftDrive = null;
    //DcMotor TrightDrive = null;
    //DcMotor BleftDrive = null;
    //DcMotor BrightDrive = null;
    final double TICKS_PER_REV = 537.6 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    /**
     *
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");
        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        // set the digital channel to input.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        digChannel.setMode(DigitalChannel.Mode.INPUT);
        TleftDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)
            //WRealeaseLeftTeleOp();//A si B pentru WRelease
            //SingleShotTeleOp();

            //MultiShotTeleOp();

            ClampTeleOp();//X si Y pentru clamp


            CremalieraTeleOp();//sageti stanga dreapta


            GlisieraTeleOp();//Sageti sus si jos

            //IntakeTeleOp();//LeftStickBumper

            //SingleShotSlowTeleOp();
            //IntakeInversTeleOp();

            BetterIntakeTeleOp();

            JustLauncherTeleOp();

            JustShooterTeleOp();

            JustLauncherTeleOpSlow();

            ArmTeleOp();

            FingerTeleOp();

            GearBox();

            PowerShotAutomation();

            prevX = gamepad1.x;


            //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
            speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower((speed * Math.sin(robotAngle) - gamepad1.right_stick_x) * power);
            TrightDrive.setPower((speed * Math.cos(robotAngle) + gamepad1.right_stick_x) * power);
            BleftDrive.setPower((speed * Math.cos(robotAngle) - gamepad1.right_stick_x) * power);
            BrightDrive.setPower((speed * Math.sin(robotAngle) + gamepad1.right_stick_x) * power);

            //telemetry.update();


        }
    }
    private void PowerShotAutomation(){
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //deci te dai in colt,apesi butonul si face PRA PRA PRA
        if (gamepad2.start){
        normalstrafeDrive(0.6,-40,40);

        normalDrive(0.4,-3,3);
            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);


            //sleep(300);
            Launcher1.setPower(-0.8);
            Launcher2.setPower(-0.8);
            sleep(600);
            Shooter.setPosition(0.3);

            InTake.setPower(-1);
            sleep(700);
            InTake.setPower(0);

                  // SSBAutonom(0.8);
        normalDrive(0.4,-4.5,4.5);
            InTake.setPower(-1);
            sleep(700);
            InTake.setPower(0);
        normalDrive(0.4,-6.75,6.75);
            InTake.setPower(-1);
            sleep(700);
            InTake.setPower(0);
        sleep(1000);


            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }

    private void SSBAutonom(double power) {
        //double power = -0.78;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TleftDrive.setPower(0);
        TrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        //sleep(300);
        Launcher1.setPower(-power);
        Launcher2.setPower(-power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(200);
        InTake.setPower(-1);
        sleep(700);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }
    public void normalDrive(double speed, double leftInches, double rightInches){
        TleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        BleftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotor.class, "rightRear");
//stop and reset
        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// this offsets the two motors that are facing the opposite direction.
        TrightDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        TleftDrive.setTargetPosition(newLeftFrontTarget);
        TrightDrive.setTargetPosition(newRightFrontTarget);
        BrightDrive.setTargetPosition(newRightBackTarget);
        BleftDrive.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        TrightDrive.setPower(Math.abs(speed));
        TleftDrive.setPower(Math.abs(speed));
        BrightDrive.setPower(Math.abs(speed));
        BleftDrive.setPower(Math.abs(speed));
        while(TleftDrive.isBusy() && BleftDrive.isBusy() &&  TrightDrive.isBusy() &&  BrightDrive.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",TleftDrive.getCurrentPosition());
            telemetry.update();
            //initDiff=frontEncoder.getCurrentPosition()-leftEncoder.getCurrentPosition();
        }


        TrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BrightDrive.setPower(0);
        BleftDrive.setPower(0);
// this stops the run to position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void normalstrafeDrive(double speed, double leftInches, double rightInches){
        TleftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        BleftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotor.class, "rightRear");
//stop and reset
        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// this offsets the two motors that are facing the opposite direction.
        TrightDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BleftDrive.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        TleftDrive.setTargetPosition(-newLeftFrontTarget);
        TrightDrive.setTargetPosition(-newRightFrontTarget);
        BrightDrive.setTargetPosition(newRightBackTarget);
        BleftDrive.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        TrightDrive.setPower(Math.abs(speed));
        TleftDrive.setPower(Math.abs(speed));
        BrightDrive.setPower(Math.abs(speed));
        BleftDrive.setPower(Math.abs(speed));


        while(TleftDrive.isBusy() && BleftDrive.isBusy() &&  TrightDrive.isBusy() &&  BrightDrive.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",TleftDrive.getCurrentPosition());
            telemetry.update();
        }

        TrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BrightDrive.setPower(0);
        BleftDrive.setPower(0);
// this stops the run to position.
        TleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        TleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void GearBox()
    {

        if(gamepad1.a && !changed) {
            if(power == 0.5) power=1;
            else power=0.5;
            changed = true;
        } else if(!gamepad1.a) changed = false;

        telemetry.addData("power:",power);
        telemetry.update();
    }
    private void JustLauncherTeleOpSlow() {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power = -0.82;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.right_stick_button) {


            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
        } else if (!gamepad1.left_bumper) {
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }

    public void JustLauncherTeleOp() {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power = -0.975;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.left_bumper) {

            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
        } else if (!gamepad1.right_stick_button) {
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }

    public void JustShooterTeleOp() {
        /*double power = -1;
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        if (gamepad1.right_bumper){
            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            }


        double power = 1;

        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        boolean shooterOk = true;
        double shooterPos = 0;
        /*if(gamepad1.left_stick_button && !changed) {
            if(InTake.getPower() == 0) InTake.setPower(power);
            else InTake.setPower(0);
            changed = true;
        } else if(!gamepad1.left_stick_button) changed = false;
        boolean changed = false; //Outside of loop()
        if (gamepad1.right_bumper && !changed) {
            if (Shooter.getPosition() == 0) Shooter.setPosition(0.3);
            else Shooter.setPosition(0);
            changed = true;
        } else if (!gamepad1.right_bumper) changed = false;


        */

        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");

        if (gamepad1.left_bumper||gamepad1.right_stick_button)
        {Shooter.setPosition(0.3);}
        else Shooter.setPosition(0);
    }

    // run until the end of the match (driver presses STOP)
    public void WRealeaseLeftTeleOp() {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");


        if (gamepad1.a) {
            // move to 0 degrees.
            wRelease.setPosition(0.0);

        } else if (gamepad1.b) {
            // move to 90 degrees.
            wRelease.setPosition(0.3);
        }
        //telemetry.addData("Servo Position", wRelease.getPosition());
        //telemetry.addData("Status", "Running");
    }

    public void SingleShotTeleOp() {
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.back) {
            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            sleep(1500);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }

    public void SingleShotSlowTeleOp() {
        double power = -0.9;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.ps) {

            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            sleep(1000);
            Shooter.setPosition(0.3);
            sleep(300);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }

    public void MultiShotTeleOp() throws InterruptedException {
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad1.start) {
            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            Thread.sleep(1500);
            Shooter.setPosition(0.3);
            Thread.sleep(400);
            Shooter.setPosition(0);
            Thread.sleep(400);
            Shooter.setPosition(0.3);
            Thread.sleep(400);
            Shooter.setPosition(0);
            Thread.sleep(400);
            Shooter.setPosition(0.3);
            Thread.sleep(400);
            Shooter.setPosition(0);
            Thread.sleep(400);
            Launcher1.setPower(0);
            Launcher2.setPower(0);

        }


    }

    public void IntakeTeleOp() {
        double power = 1;

        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*if(gamepad1.left_stick_button && !changed) {
            if(InTake.getPower() == 0) InTake.setPower(power);
            else InTake.setPower(0);
            changed = true;
        } else if(!gamepad1.left_stick_button) changed = false;*/
        if (gamepad1.left_stick_button) {
            if (intakeOk == true) {
                if (intakePower == 1) intakePower = 0;
                else intakePower = 1;
                intakeOk = false;
            }
        } else intakeOk = true;

        InTake.setPower(intakePower);

    }

    public void IntakeInversTeleOp() {
        double power = 1;


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*if(gamepad1.left_stick_button && !changed) {
            if(InTake.getPower() == 0) InTake.setPower(power);
            else InTake.setPower(0);
            changed = true;
        } else if(!gamepad1.left_stick_button) changed = false;*/
        if (gamepad2.right_stick_button) {
            if (intakeOkR == true) {
                if (intakePowerR == -1) intakePowerR = 0;
                else intakePowerR = -1;
                intakeOkR = false;
            }
        } else intakeOkR = true;

        InTake.setPower(intakePowerR);

    }

    public void BetterIntakeTeleOp() {
        double power = -1;


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (gamepad1.right_bumper) {//1
            InTake.setPower(power);
            telemetry.addLine("caz 1 ");
        } else if (gamepad1.b)//-1
        {
            InTake.setPower(-power);
            telemetry.addLine("caz -1 ");
        } else if (!gamepad1.right_bumper && !gamepad1.b) {//0
            InTake.setPower(0);
            telemetry.addLine("caz 0 ");
        }
        telemetry.update();
    }

    public void ClampTeleOp() {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");


        if (gamepad2.b) {
            // move to 0 degrees.
            clamp.setPosition(0.0);

        } else if (gamepad2.a) {
            // move to 90 degrees.
            clamp.setPosition(0.3);
        }
        //telemetry.addData("Servo Position", clamp.getPosition());
        //telemetry.addData("Status", "Running");
    }


    public void CremalieraTeleOp() {

        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        digChannel.setMode(DigitalChannel.Mode.INPUT);
        crMg_OK = !digChannel.getState();
        if (CrSensLastDep) {
            if (crMg_OK) {
                if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(0);
                    //CrSensLastDep = false;
                } else if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                    //CrSensLastDep = false;
                }

            } else {
                if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    ////////////////////////CrSensLastDep = true;
                } else if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(0);
                    CrSensLastDep = false;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        } else {
            if (crMg_OK) {
                if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(0);
                } else if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    //CrSensLastDep = true;
                }
            }
            if (!crMg_OK) {
                if (gamepad1.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                } else if (gamepad1.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    CrSensLastDep = true;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        }


    }

    public void GlisieraTeleOp() {
        double glsPower = 0.8;

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (gamepad1.dpad_up) {
            if (target <= 2850) {
                glisiera.setPower(glsPower);
                target = target + 50;
            } else glisiera.setPower(0);
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            glisiera.setPower(0);
        }


        if (gamepad1.dpad_down) {
            if (glsMg.getState()) {
                glisiera.setPower(-glsPower);
                target = target - 100;
            } else {
                target = 0;
                glisiera.setPower(0);
            }
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            glisiera.setPower(0);
        }
        glisiera.setTargetPosition(target);


        glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);


/* public void GlisieraTeleOp() {


        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BrightDriveAKE);


        if (gamepad1.dpad_up) {
            if (target <= 2850) {
                glisiera.setPower(glsPower);
                target = target + 50;
                if (target <= 2000) glisiera.setPower(glsPower);
                else glisiera.setPower(glsPower * 0.5);
            } else glisiera.setPower(0);


        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            glisiera.setPower(0);
        }


        if (gamepad1.dpad_down) {
            if (glsMg.getState()) {
                glisiera.setPower(-glsPower);
                target = target - 100;
                    if (target <= 2000) glisiera.setPower(-glsPower);
                    else glisiera.setPower(-glsPower * 0.5);

            } else {
                target = 1;
                glisiera.setPower(0);
            }
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            glisiera.setPower(0);
        }
        glisiera.setTargetPosition(target);


        glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("pos gls", target);
        telemetry.addData("speed", glsPower);
        telemetry.update();


    }*/

    }

    public void ArmTeleOp() {
        CRServo Arm = hardwareMap.get(CRServo.class, "SR_ARM");


        if (gamepad1.back) {
            // move to 0 degrees.
            Arm.setPower(0.4);

        } else if (gamepad1.start) {
            // move to 90 degrees.
            Arm.setPower(-0.4);
        } else {
            Arm.setPower(0);
        }
        telemetry.addData("Servo Power", Arm.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void FingerTeleOp() {
        Servo finger = hardwareMap.get(Servo.class, "SR_FINGER");


        if (gamepad1.x) {
            // move to 0 degrees.
            finger.setPosition(0.6);

        } else if (gamepad1.y) {
            // move to 90 degrees.
            finger.setPosition(0.0);
        }
        telemetry.addData("Servo Position", finger.getPosition());
        telemetry.addData("Status", "Running");
        //telemetry.update();
    }
}


