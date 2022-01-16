package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "DebugTeleOp")
public class DebugTeleOp extends LinearOpMode {

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
    double wTarget = 0;
    double power = 1.5;//1.41
    double FranaS, FranaD;
    boolean leftstickpress; //Outside of loop()
    double intakePower = 0;
    boolean intakeOk = true;
    double target = 0.1;
    int intakePowerR = 0;
    boolean intakeOkR = true;
    boolean changed = false;
    boolean okClamp = false;
    double clampPos =0;
boolean lainceput=true;
    BNO055IMU imu;
    private Orientation angles;
    public DistanceSensor sensorRange;
    Rev2mDistanceSensor sensorDistance;
    //DcMotor TleftDrive = null;
    //DcMotor TrightDrive = null;
    //DcMotor BleftDrive = null;
    //DcMotor BrightDrive = null;
    final double TICKS_PER_REV = 537.6;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    double ogAngle;
    double ogDistance;
double n = 1;
boolean shoot=false;
double fire = 0.0;
double katyusha = 0.87;
double timer = 0;
String inALoop = "at the beginning";
    Servo Trigger;
    Servo Shooter;
    //Servo clamp;
    boolean artiljerija  = false;
    double LaunchPower=0;
    double ICBM = -0.90 ;
    NormalizedColorSensor colorUp,colorDown;
    NormalizedRGBA colorsUp,colorsDown;
    float gain = 95;
    double Fpower;
    double Tpower =0.35;
    boolean Fchanged;
    boolean Tchanged;
    double Spower ;
    //boolean Fchanged;
    boolean Schanged;
    Servo finger;
    DcMotor Arm;
    Servo flag;
    //VoltageSensor voltSensor;
    double voltage;
    double sPos1;
    double sPos2;
    double sPos3;
    long sSleep=700;
    Servo glisiera;
    double glsPos = 0;//Trigger GearBox Katyusha    Intake
    int channel = 0;
    double fullRange = 4.32; // The analog value when your pot is at full range
    double offset = 0; // Use this if you have a desired offset.
    //AnalogPotentiometer pot = new AnalogPotentiometer(channel, fullRange, offset);
    AnalogInput potentiometer;


    /**
     *
     */
    @Override
    public void runOpMode() throws InterruptedException {
        potentiometer = hardwareMap.analogInput.get("potentiometer");
        colorUp = hardwareMap.get(NormalizedColorSensor.class, "color_up");
        colorDown = hardwareMap.get(NormalizedColorSensor.class, "color_down");//color_sensor = hardwareMap.colorSensor.get("color_up");
        colorsUp = colorUp.getNormalizedColors();
        colorsDown = colorDown.getNormalizedColors();
        //voltSensor = hardwareMap.get(VoltageSensor.class,"ARM");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        Trigger = hardwareMap.get(Servo.class, "SR_TRIGGER");
        //Shooter.setDirection(Servo.Direction.REVERSE);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorDistance = (Rev2mDistanceSensor) sensorRange;

         Arm = hardwareMap.get(DcMotor.class, "ARM");
        TleftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        BleftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        finger = hardwareMap.get(Servo.class, "SR_FINGER");
        Servo flag = hardwareMap.get(Servo.class,"SR_FLAG");
         //clamp = hardwareMap.get(Servo.class, "SR_CLAMP");
        DriveThread TeleThread = new DriveThread();
        voltage = getBatteryVoltage();

        colorUp.setGain(gain);
        colorDown.setGain(gain);
        waitForStart();
        TeleThread.start();
        while (opModeIsActive()) {
            /*if (lainceput == true)
            {
                Trigger.setPosition(0.85);

                    sleep(100);
                lainceput=false;

            }*/
            voltage = getBatteryVoltage();
            colorsUp = colorUp.getNormalizedColors();
            colorsDown = colorDown.getNormalizedColors();
            // run until the end of the match (driver presses STOP)
            TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //WRealeaseLeftTeleOp();//A si B pentru WRelease
            //SingleShotTeleOp();
             inALoop = "in OpMode";
            //MultiShotTeleOp();
            //setNicePos();
            //ShooterAutomation();
            PowerShotAutomation();
            GearBox();
            HandBreak();
            Katyusha1();
            //Katyusha1();
            //JustLauncherTeleOpSlow();
            //JustShooterTeleOp();
            BetterIntakeTeleOp();
            //ClampTeleOp();
            //GlisieraTeleOp();
            //CremalieraTeleOp();
            //SingleShotTeleOp();
            TriggerTeleOp();
            ShooterTeleOp();
            //InterContinentalBallisticMissleRisingDaPeStrafe();
            ArmTeleOp();
            FingerTeleOp();
            //CremalieraTeleOp();//uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
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
            if (colorsUp.blue<0.85) {
                flag.setPosition(0.5);
            } else if (colorsDown.blue<0.85) {
                flag.setPosition(0.3);
            } else flag.setPosition(0.0);


        }
        TeleThread.interrupt();
    }

    public void SingleShotTeleOp()
    {
        Servo Trigger = hardwareMap.get(Servo.class, "SR_TRIGGER");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");

        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad1.left_bumper)
        {
            Trigger.setPosition(0.7);
            Launcher1.setPower(power);
            Launcher2.setPower(power);
            sleep(1000);
            Shooter.setPosition(0.3);
        }

    }
    public void TriggerTeleOp()
    {
        
        if (gamepad1.x && !Tchanged) {
            if (Tpower == 0.35) Tpower = 0.87;
            else Tpower = 0.35;
            Tchanged = true;
        } else if (!gamepad1.x) Tchanged = false;
        Trigger.setPosition(Tpower);
    }

    public void ShooterTeleOp()
    {
        //Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");

        if (gamepad1.y && !Schanged) {
            if (Spower == 0.0) Spower = 0.6;
            else Spower = 0.0;
            Schanged = true;
        } else if (!gamepad1.y) Schanged = false;
        Shooter.setPosition(Spower);
    }
    public void setNicePos() {
        if (gamepad2.left_bumper) {
            ogAngle = angles.firstAngle;
            ogDistance = sensorDistance.getDistance(DistanceUnit.CM);
            if (n==1) n=0;
        }
    }

    public void ShooterAutomation() {
        timer+=1;
        if (gamepad2.right_bumper) {



            if (n==0&&shoot==false&&timer>100) {
                normalGyroDrive(0.6, ogAngle, 2);
                normalDistanceDrive(0.6, ogDistance, 2);
                normalGyroDrive(0.5, ogAngle, 2);
                normalDistanceDrive(0.5, ogDistance, 1);
                timer = 0;
                shoot = true;
            }
            if (shoot==true&&timer<100)
            {
                //ne imaginam ca trage
                telemetry.addData("PRA PRA","PRA");
                telemetry.update();
                shoot = false;
            }
if (timer>100)
{shoot=false;}
        }

    }

    public void normalGyroDrive(double speed, double angle, double buffer) {
        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle < angle) {
            //this gets the absolute speed and converdets it into power for the motor.
            TrightDrive.setPower(-Math.abs(speed));
            TleftDrive.setPower(Math.abs(speed));
            BrightDrive.setPower(-Math.abs(speed));
            BleftDrive.setPower(Math.abs(speed));
        } else if (angles.firstAngle > angle) {
            //this gets the absolute speed and converdets it into power for the motor.
            TrightDrive.setPower(Math.abs(speed));
            TleftDrive.setPower(-Math.abs(speed));
            BrightDrive.setPower(Math.abs(speed));
            BleftDrive.setPower(-Math.abs(speed));
        }
        while (angles.firstAngle > angle + buffer || angles.firstAngle < angle - buffer && opModeIsActive()) {
            inALoop = "in normalGyro loop";
            if (gamepad2.back)
            {
                break;
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle < angle ) {
                //this gets the absolute speed and converdets it into power for the motor.
                TrightDrive.setPower(-Math.abs(speed));
                TleftDrive.setPower(Math.abs(speed));
                BrightDrive.setPower(-Math.abs(speed));
                BleftDrive.setPower(Math.abs(speed));
            } else if (angles.firstAngle > angle ) {
                //this gets the absolute speed and converdets it into power for the motor.
                TrightDrive.setPower(Math.abs(speed));
                TleftDrive.setPower(-Math.abs(speed));
                BrightDrive.setPower(Math.abs(speed));
                BleftDrive.setPower(-Math.abs(speed));
            }
        }
inALoop = "out of normal gyro loop";

        TrightDrive.setPower(0);
        TleftDrive.setPower(0);
        BrightDrive.setPower(0);
        BleftDrive.setPower(0);
// this stops the run to position.

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

    public void normalDistanceDrive(double speed, double distance, double buffer) {
        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double distance2 = sensorDistance.getDistance(DistanceUnit.CM);
        if (distance2 < distance) {
            //this gets the absolute speed and converdets it into power for the motor.
            TrightDrive.setPower(Math.abs(speed));
            TleftDrive.setPower(-Math.abs(speed));
            BrightDrive.setPower(-Math.abs(speed));
            BleftDrive.setPower(Math.abs(speed));
        } else if (distance2 > distance) {
            //this gets the absolute speed and converdets it into power for the motor.
            TrightDrive.setPower(-Math.abs(speed));
            TleftDrive.setPower(Math.abs(speed));
            BrightDrive.setPower(Math.abs(speed));
            BleftDrive.setPower(-Math.abs(speed));
        }
        while (distance2 > distance + buffer || distance2 < distance - buffer && opModeIsActive()) {
            inALoop="in DistanceDrive loop";
            if (gamepad2.back)
            {

                break;
            }
            if (Math.abs(distance2 - distance) < distance / 6) speed = speed / 2;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            distance2 = sensorDistance.getDistance(DistanceUnit.CM);
            if (distance2 < distance ) {
                //this gets the absolute speed and converdets it into power for the motor.
                TrightDrive.setPower(Math.abs(speed));
                TleftDrive.setPower(-Math.abs(speed));
                BrightDrive.setPower(-Math.abs(speed));
                BleftDrive.setPower(Math.abs(speed));
            } else if (distance2 > distance ) {
                //this gets the absolute speed and converdets it into power for the motor.
                TrightDrive.setPower(-Math.abs(speed));
                TleftDrive.setPower(Math.abs(speed));
                BrightDrive.setPower(Math.abs(speed));
                BleftDrive.setPower(-Math.abs(speed));
            }
        }
inALoop="Out of distance drive loop";

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
    public void InterContinentalBallisticMissleShooter() {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//normalGyroDrive(0.3,-2,0.6);
        //normalDrive(0.3, -2, 2);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1890);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1890);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(250);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,-6,0.6);
        //normalDrive(0.3, -5, 5);
        normalstrafeDrive(0.5,-20,20);
        sleep(100);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1907);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1905);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(250);
        //Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        //normalDrive(0.3, -5, 5);
        normalstrafeDrive(0.5,-20,20);
        sleep(250);
        Trigger.setPosition(0.87);
        Launcher1.setVelocity(-1915);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1915.);
        //sleep(1300);
        Shooter.setPosition(0.87);
        sleep(500);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }
    private void PowerShotAutomation() {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //deci te dai in colt,apesi butonul si face PRA PRA PRA
        if (gamepad1.start) {
            normalstrafeDrive(0.5, -45, 45);

            InterContinentalBallisticMissleShooter();
        }
    }

    private void SSBAutonom(double power) {
        //double power = -0.78;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public void normalDrive(double speed, double leftInches, double rightInches) {
        TleftDrive = hardwareMap.get(DcMotor.class, "FL");
        TrightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");
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
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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
        while (TleftDrive.isBusy() && BleftDrive.isBusy() && TrightDrive.isBusy() && BrightDrive.isBusy() && opModeIsActive()) {
            inALoop = "in normal drive loop";
            if (gamepad2.back)
            {
                break;
            }
            //initDiff=frontEncoder.getCurrentPosition()-leftEncoder.getCurrentPosition();
        }
inALoop = "out of normal drive loop";

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

    public void normalstrafeDrive(double speed, double leftInches, double rightInches) {
        TleftDrive = hardwareMap.get(DcMotor.class, "FL");
        TrightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");
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
        //TleftDrive.setDirection(DcMotor.Direction.REVERSE);
        //BleftDrive.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = TleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
        newRightFrontTarget = TrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newRightBackTarget = BrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_CM);
        newLeftBackTarget = BleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_CM);
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
int caca =1;

        while (TleftDrive.isBusy() && BleftDrive.isBusy() && TrightDrive.isBusy() && BrightDrive.isBusy() && opModeIsActive()) {
            inALoop = "in strafe drive loop";

            /*TrightDrive.setPower(Math.abs(speed));
            TleftDrive.setPower(Math.abs(speed));
            BrightDrive.setPower(Math.abs(speed));
            BleftDrive.setPower(Math.abs(speed));*/
            if (caca == 1)
            {
                TrightDrive.setPower(Math.abs(speed));
                TleftDrive.setPower(Math.abs(speed));
                BrightDrive.setPower(Math.abs(speed));
                BleftDrive.setPower(Math.abs(speed));
                caca = 0;
            }
        }
inALoop = "outside of strafe drive loop";
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

    public void GearBox() {

        if (1==1) {
            if (gamepad1.a && !changed) {
                if (power == 0.5) power = 1;
                else power = 0.5;
                changed = true;
            } else if (!gamepad1.a) changed = false;
        }
power = 1-gamepad1.right_trigger;
    }
    public void HandBreak() {

       /* if (gamepad1.a && !changed) {
            if (power == 0.5) power = 1;
            else power = 0.5;
            changed = true;
        } else if (!gamepad1.a) changed = false;
*/
        while (gamepad1.left_trigger>0.05) {
            if (gamepad1.left_trigger<0.05) break;


            TrightDrive.setPower(0);
            TleftDrive.setPower(0);
            BrightDrive.setPower(0);
            BleftDrive.setPower(0);
        }

    }
    private void JustLauncherTeleOpSlow() {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double LaunchPower = -0.82;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad1.right_stick_button) {


            TleftDrive.setPower(0);
            TrightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(0);
            Launcher1.setPower(LaunchPower);
            Launcher2.setPower(LaunchPower);
        } else if (!gamepad1.left_bumper) {
            Launcher1.setPower(0);
            Launcher2.setPower(0);
        }
    }
/*
    public void JustLauncherTeleOp() {
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad1.left_bumper && !artiljerija ) {
            if (LaunchPower == 0) LaunchPower = ICBM;
            else LaunchPower = 0;
            artiljerija  = true;
        } else if (!gamepad1.left_bumper) artiljerija  = false;

        TleftDrive.setPower(0);
        TrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        Launcher1.setPower(LaunchPower);
        Launcher2.setPower(LaunchPower);
    }
    */

    public void Katyusha1()
    {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ICBM = -0.76;
        if (gamepad1.left_bumper) {

            Trigger.setPosition(0.87);
            Launcher1.setVelocity(-2700);
            Launcher2.setVelocity(-2700);//2180
            sleep(1000);
            Shooter.setPosition(0.21);
            sleep(700);
            Shooter.setPosition(0.62);
            sleep(700);
            Shooter.setPosition(0.89);
            sleep(700);
            //Trigger.setPosition(0.87);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);
            //Trigger.setPosition(0.35);

        }
            if(gamepad1.right_bumper)
        {
            normalDrive(0.7, 153, 153);

            //InterContinentalBallisticMissleShooters();
            //normalGyroDrive(0.25, 0, 0.3);
            //normalDrive(0.45, 40, 40);
            normalstrafeDrive(0.6, -55, 55);
            normalDrive(0.3,0,1);
        }
    }
    public void InterContinentalBallisticMissle()
    {


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ICBM = -0.76;

            /*Trigger.setPosition(0.0);
            Launcher1.setVelocity(-2160);
            Launcher2.setVelocity(-2160);
            sleep(1550);
            Shooter.setPosition(0.1);
            sleep(250);
            Launcher1.setVelocity(-1990);
            Launcher2.setVelocity(-1990);
            sleep(500);
            Shooter.setPosition(0.5);
            sleep(250);
            Launcher1.setVelocity(-2100);
            Launcher2.setVelocity(-2100);
            sleep(500);
            Shooter.setPosition(0.88);
            sleep(1000);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);*/
            /*if (voltage >= 13.6 && voltage < 13.9) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-1930);
                Launcher2.setVelocity(-1930);
                sleep(750);
                Shooter.setPosition(0.77);
                sleep(250);
                Launcher1.setVelocity(-2100);
                Launcher2.setVelocity(-2100);
                sleep(750);
                Shooter.setPosition(0.82);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.5 && voltage < 13.6) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2180);
                Launcher2.setVelocity(-2180);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2010);
                Launcher2.setVelocity(-2010);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2120);
                Launcher2.setVelocity(-2120);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage >= 13.4 && voltage < 13.5) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }
            else
            if (voltage < 13.4) {
                Trigger.setPosition(0.0);
                Launcher1.setVelocity(-2185);
                Launcher2.setVelocity(-2185);
                sleep(1550);
                Shooter.setPosition(0.1);
                sleep(250);
                Launcher1.setVelocity(-2032);
                Launcher2.setVelocity(-2035);
                sleep(500);
                Shooter.setPosition(0.5);
                sleep(250);
                Launcher1.setVelocity(-2138);
                Launcher2.setVelocity(-2138);
                sleep(500);
                Shooter.setPosition(0.88);
                sleep(1000);
                Shooter.setPosition(0);
                Launcher1.setPower(0);
                Launcher2.setPower(0);
            }*/
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-2200);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-2200);
        sleep(1300);
        Shooter.setPosition(0.23);
        sleep(500);
        Shooter.setPosition(0.6);
        sleep(600);
        Shooter.setPosition(0.83);
        sleep(600);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);

    }
    public void InterContinentalBallisticMissleRising() {

/*
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
if (gamepad2.left_bumper) {
//normalGyroDrive(0.3,-2,0.6);
    //normalDrive(0.3, -2.2, 2.2);
    Trigger.setPosition(0.0);
    Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1880);
    sleep(1300);
    Shooter.setPosition(0.21);
    sleep(500);
    //Shooter.setPosition(0);

    normalDrive(0.3, 7.5, -7.5);
    sleep(250);
    Trigger.setPosition(0.0);
    Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1880);
    //sleep(1300);
    Shooter.setPosition(0.87);
    sleep(500);
    //Shooter.setPosition(0);


        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, -3, 3);
        sleep(100);
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1880);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(250);
        Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        Launcher1.setPower(0);
        Launcher2.setPower(0);


*/
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad2.left_bumper){


//normalGyroDrive(0.3,-2,0.6);
    normalDrive(0.3, -2, 2);
    sleep(100);
    Trigger.setPosition(0.35);
    Launcher1.setVelocity(-1897);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1897);
    sleep(1300);
    Shooter.setPosition(0.23);
    sleep(250);
    //Shooter.setPosition(0);

    //normalGyroDrive(0.3,-6,0.6);
            normalDrive(0.3, 8.3, -8.3);
    sleep(100);
    Trigger.setPosition(0.35);
    Launcher1.setVelocity(-1887);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1887);
    //sleep(1300);
    Shooter.setPosition(0.55);
    sleep(250);
    //Shooter.setPosition(0);

    //normalGyroDrive(0.3,3.2,0.35);
            normalDrive(0.3, -3.3, 3.3);
    sleep(250);
    Trigger.setPosition(0.35);
    Launcher1.setVelocity(-1900);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1900);
    //sleep(1300);
            sleep(100);
    Shooter.setPosition(0.87);
    sleep(500);
    Shooter.setPosition(0);
    Launcher1.setPower(0);
    Launcher2.setPower(0);

}
if (gamepad2.right_bumper){
    normalDrive(0.55, 155, 155);
}
    }
    public void InterContinentalBallisticMissleRisingDaPeStrafe() {

/*
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
if (gamepad2.left_bumper) {
//normalGyroDrive(0.3,-2,0.6);
    //normalDrive(0.3, -2.2, 2.2);
    Trigger.setPosition(0.0);
    Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1880);
    sleep(1300);
    Shooter.setPosition(0.21);
    sleep(500);
    //Shooter.setPosition(0);

    normalDrive(0.3, 7.5, -7.5);
    sleep(250);
    Trigger.setPosition(0.0);
    Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
    Launcher2.setVelocity(-1880);
    //sleep(1300);
    Shooter.setPosition(0.87);
    sleep(500);
    //Shooter.setPosition(0);


        //normalGyroDrive(0.3,-6,0.6);
        normalDrive(0.3, -3, 3);
        sleep(100);
        Trigger.setPosition(0.0);
        Launcher1.setVelocity(-1880);//2250/13.72V      2220/13.62V
        Launcher2.setVelocity(-1880);
        //sleep(1300);
        Shooter.setPosition(0.55);
        sleep(250);
        Shooter.setPosition(0);

        //normalGyroDrive(0.3,3.2,0.35);
        Launcher1.setPower(0);
        Launcher2.setPower(0);


*/
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        //Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shooter.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType =
                Launcher1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        Launcher1.setMotorType(motorConfigurationType);
        Launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType2 =
                Launcher2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        Launcher2.setMotorType(motorConfigurationType2);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad2.left_bumper){


//normalGyroDrive(0.3,-2,0.6);
            normalDrive(0.3, -2, 2);
            sleep(100);
            Trigger.setPosition(0.35);
            Launcher1.setVelocity(-1897);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(-1897);
            sleep(1300);
            Shooter.setPosition(0.23);
            sleep(250);
            //Shooter.setPosition(0);

            //normalGyroDrive(0.3,-6,0.6);
            telemetry.addData("we at","nsd");
            normalstrafeDrive(0.3, 10, -10);

            telemetry.addData("we at","ngd");
            normalGyroDrive(0.3,0,0.2);


            sleep(100);
            Trigger.setPosition(0.35);
            Launcher1.setVelocity(-1887);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(-1887);
            //sleep(1300);
            Shooter.setPosition(0.55);
            sleep(250);
            //Shooter.setPosition(0);

            //normalGyroDrive(0.3,3.2,0.35);
            telemetry.addData("we at","nsd");
            normalstrafeDrive(0.3, 10, -10);
            telemetry.addData("we at","ngd");
            normalGyroDrive(0.3,0,0.2);

            sleep(250);
            Trigger.setPosition(0.35);
            Launcher1.setVelocity(-1900);//2250/13.72V      2220/13.62V
            Launcher2.setVelocity(-1900);
            //sleep(1300);
            sleep(100);
            Shooter.setPosition(0.87);
            sleep(500);
            Shooter.setPosition(0);
            Launcher1.setPower(0);
            Launcher2.setPower(0);

        }
        if (gamepad2.right_bumper){
            normalDrive(0.55, 155, 155);
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

        if (gamepad1.left_bumper || gamepad1.right_stick_button) {
            Shooter.setPosition(0.3);
        } else Shooter.setPosition(0);
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
        double IntakePower = -1;


        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (gamepad1.right_bumper) {//1
            InTake.setPower(IntakePower);

        } else if (gamepad1.right_stick_button)//-1
        {
            InTake.setPower(-IntakePower);

        } else if (!gamepad1.right_bumper && !gamepad1.right_stick_button) {//0
            InTake.setPower(0);

        }

    }
/*
    public void ClampTeleOp() {




        if (gamepad2.a && !okClamp) {
            if (clampPos == 0.5) clampPos = 1;
            else clampPos = 0.5;
            okClamp = true;
        } else if (!gamepad2.a) okClamp = false;
        //telemetry.addData("Servo Position", clamp.getPosition());
        //telemetry.addData("Status", "Running");
        //clamp.setPosition(clampPos);
    }


    public void CremalieraTeleOp() {

        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel digChannel = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        digChannel.setMode(DigitalChannel.Mode.INPUT);
        crMg_OK = !digChannel.getState();
        if (CrSensLastDep) {
            if (crMg_OK) {
                if (gamepad2.dpad_left) {
                    cremaliera_Servo.setPower(0);
                    //CrSensLastDep = false;
                } else if (gamepad2.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                    //CrSensLastDep = false;
                }

            } else {
                if (gamepad2.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    ////////////////////////CrSensLastDep = true;
                } else if (gamepad2.dpad_right) {
                    cremaliera_Servo.setPower(0);
                    CrSensLastDep = false;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        } else {
            if (crMg_OK) {
                if (gamepad2.dpad_right) {
                    cremaliera_Servo.setPower(0);
                } else if (gamepad2.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    //CrSensLastDep = true;
                }
            }
            if (!crMg_OK) {
                if (gamepad2.dpad_right) {
                    cremaliera_Servo.setPower(-crPwr);
                } else if (gamepad2.dpad_left) {
                    cremaliera_Servo.setPower(crPwr);
                    CrSensLastDep = true;
                } else {
                    cremaliera_Servo.setPower(0);
                }
            }
        }


    }

    public void GlisieraTeleOp() {


        glisiera = hardwareMap.get(Servo.class, "GLS");
        //glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");
        //glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (gamepad2.dpad_up) {
            target = target   +  0.1;
        }


        if (gamepad2.dpad_down) {
            if (glsMg.getState()) {
if (target>0.1)
                target = target - 0.1;
            }
        }
if (opModeIsActive()) {
    glisiera.setPosition(target);
    glsPos =glisiera.getPosition();
}

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


    }

    }
*/
    public void ArmTeleOp() {

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*if (gamepad2.x) {
            // move to 0 degrees.
            Arm.setPower(0.2);

        } else if (gamepad2.y) {
            // move to 90 degrees.
            Arm.setPower(-0.2);
        } else {
            Arm.setPower(0);
        }
*/
double pot=potentiometer.getVoltage();





            if (gamepad1.dpad_left&&pot>0.4)
            {
                wTarget-=25;
            }
            if (gamepad1.dpad_right&&pot<1.8)
            {
                wTarget+=25;
            }
        Arm.setPower(0.75);
        Arm.setTargetPosition((int) wTarget);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void FingerTeleOp() {



        if (gamepad1.b && !Fchanged) {
            if (Fpower == 1) Fpower = 0;
            else Fpower = 1;
            Fchanged = true;
        } else if (!gamepad1.b) Fchanged = false;
finger.setPosition(Fpower);
        //telemetry.update();
    }
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public class DriveThread extends Thread {
        public DriveThread() {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {


            while (!isInterrupted()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double distance2 = sensorDistance.getDistance(DistanceUnit.CM);
                telemetry.addData("ogAngle", ogAngle);
                telemetry.addData("ogDistance", ogDistance);
                telemetry.addData("speed", power);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("distance", distance2);
                telemetry.addData("target", target);
                telemetry.addData("gls pos",glsPos);
                telemetry.addData("runMode", TleftDrive.getMode());
                telemetry.addData("power",TleftDrive.getPower());
                telemetry.addData("inALoop",inALoop);
                try {
                    telemetry.addData("Trigger pos", Trigger.getPosition());
                    telemetry.addData("Shooter pos", Shooter.getPosition());
                } catch (Exception e) {

                }
                telemetry.addData("ICBM=",ICBM);
                telemetry.addData("voltage", "%.1f volts", getBatteryVoltage());
                telemetry.addData("potentiometer",potentiometer.getVoltage());
                telemetry.update();


            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }
}


