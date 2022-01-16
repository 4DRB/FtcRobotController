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


@TeleOp(name = "Dacia logan sport gpl 1.2")
public class TeleOpV3 extends LinearOpMode {

public int stop=0;
public int stopPos;
    double speed;
    double robotAngle;
    boolean changed;
    // Declare OpMode members.
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    private DcMotor BeyBlade = null;
    private DcMotor kanon = null;
    private DcMotor turret = null;
    private Servo grip =null;
    double wTarget = 0;
    double power = 1;//1.41



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
    AnalogInput potentiometer;



    /**
     *
     */
    @Override
    public void runOpMode() throws InterruptedException {

        potentiometer = hardwareMap.analogInput.get("potentiometer");
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //potentiometer = hardwareMap.analogInput.get("potentiometer");
        //flag = hardwareMap.get(Servo.class,"SR_FLAG");
        //colorUp = hardwareMap.get(NormalizedColorSensor.class, "color_up");
        //colorDown = hardwareMap.get(NormalizedColorSensor.class, "color_down");//color_sensor = hardwareMap.colorSensor.get("color_up");
        //colorsUp = colorUp.getNormalizedColors();
        //colorsDown = colorDown.getNormalizedColors();
        //voltSensor = hardwareMap.get(VoltageSensor.class,"ARM");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //Shooter = hardwareMap.get(Servo.class, "SR_SHOOTER");
        //Trigger = hardwareMap.get(Servo.class, "SR_TRIGGER");
        //Shooter.setDirection(Servo.Direction.REVERSE);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        //sensorDistance = (Rev2mDistanceSensor) sensorRange;

         //Arm = hardwareMap.get(DcMotor.class, "ARM");
        TleftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        BleftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        BeyBlade = hardwareMap.get(DcMotorEx.class, "BeyBlade");
        kanon=hardwareMap.get(DcMotorEx.class, "kanon");

         grip = hardwareMap.get(Servo.class, "grip");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BeyBlade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        BeyBlade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kanon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveThread TeleThread = new DriveThread();

        //finger = hardwareMap.get(Servo.class, "SR_FINGER");
              //clamp = hardwareMap.get(Servo.class, "SR_CLAMP");
        //voltage = getBatteryVoltage();

        //colorUp.setGain(gain);
        //colorDown.setGain(gain);
        waitForStart();
        //TeleThread.run();
        while (opModeIsActive()) {

            // run until the end of the match (driver presses STOP)
            TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BeyBlade();
            //GearBox();
            //HandBreak();

            Kanon();
             Turret();
            Grip();
            //speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower(gamepad1.left_stick_y * power+gamepad1.right_stick_x*power);
            TrightDrive.setPower(gamepad1.left_stick_y * power-gamepad1.right_stick_x*power);
            BleftDrive.setPower(gamepad1.left_stick_y * power+gamepad1.right_stick_x*power);
            BrightDrive.setPower(gamepad1.left_stick_y * power-gamepad1.right_stick_x*power);

        }
    }
public void Grip() {
        if (gamepad2.a) {
            grip.setPosition(0);
        }else if (gamepad2.b)
        {
            grip.setPosition(0.8);
        }
    }
public void Turret(){

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
double power = 0;




turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    if (gamepad2.dpad_up)
    {
       turret.setPower(0.5);
    }else
    if (gamepad2.dpad_down )
    {
        turret.setPower(-0.5);
    }else
    {
turret.setPower(0);
     }



    telemetry.addData("curPos",turret.getCurrentPosition());
    telemetry.addData("target pos",turret.getTargetPosition());
    telemetry.addData("power",turret.getPower());
telemetry.update();


/*
    if (gamepad1.dpad_up)
    {stop=0;
    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0.25);
    }
    else
    if (gamepad1.dpad_down)
    {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop=0;
        turret.setPower(-0.25);
    }
    else
    {
        stopPos=turret.getCurrentPosition();
        stop=1;
        turret.setPower(0);
    }
    if (stop == 1)
    {
        turret.setTargetPosition(stopPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.3);
    }
    telemetry.addData("stop",stop);
    telemetry.addData("stopPos",stopPos);
    telemetry.update();*/
}
public void Kanon(){
    kanon=hardwareMap.get(DcMotorEx.class, "kanon");

    if (gamepad1.dpad_left)
        {
            kanon.setPower(0.25);
        }
        else
        if (gamepad1.dpad_right)
        {
            kanon.setPower(-0.25);
        }
        else
        {

            kanon.setPower(0);
        }
}
public void BeyBlade(){
        if (gamepad2.y)
        {
            BeyBlade.setPower(1);
        }
        else if (gamepad2.x)
        {
            BeyBlade.setPower(-1);
        }
        else {
            BeyBlade.setPower(0);
        }
}
    public void GearBox() {

        if (1==1) {
            if (gamepad1.a && !changed) {
                if (power == 0.5) power = 1;
                else power = 0.5;
                changed = true;
            } else if (!gamepad1.a) changed = false;
        }
    }
    /*
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

                /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double distance2 = sensorDistance.getDistance(DistanceUnit.CM);
                telemetry.addData("speed", power);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("distance", distance2);
                telemetry.addData("runMode", TleftDrive.getMode());
                telemetry.addData("power",TleftDrive.getPower());
*/
                telemetry.update();


            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }
}


