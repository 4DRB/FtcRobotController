package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "TeleTestV1")
public class TeleOpTestV1 extends LinearOpMode {

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
    private CRServo baws =null;
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
    int forkVal=0;



    /**
     *
     */
    @Override
    public void runOpMode() throws InterruptedException {


        //Drive Motors

        TleftDrive = hardwareMap.get(DcMotor.class, "FL");
        TrightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");


        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //Utility Motors

        BeyBlade = hardwareMap.get(DcMotor.class, "SPIN");

        BeyBlade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, "FORK");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Servos

        grip = hardwareMap.get(Servo.class,"SR_GATE");
        baws = hardwareMap.get(CRServo.class,"SR_BALLS");
        //colorDown.setGain(gain);
        waitForStart();
        //TeleThread.run();
        while (opModeIsActive()) {

            DriveStick();
            DriveDPad(0.7);
            Grip();
            BeyBlade();
            BetterIntakeTeleOp();
            Kanon();
            //Turret(); Obsolete
            //Baws();   inclus in betterIntake

        }
    }

    public void DriveStick(){

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        //finds the angle the robot is moving at
        //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        //finds the percent of power to each wheel and multiplies it by the speed
        TleftDrive.setPower((speed * Math.sin(robotAngle) - gamepad1.right_stick_x) * power);
        TrightDrive.setPower((speed * Math.cos(robotAngle) + gamepad1.right_stick_x) * power);
        BleftDrive.setPower((speed * Math.cos(robotAngle) - gamepad1.right_stick_x) * power);
        BrightDrive.setPower((speed * Math.sin(robotAngle) + gamepad1.right_stick_x) * power);

    }
    public void DriveDPad(double speed){

        TleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        //speed = -Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        //finds the angle the robot is moving at
        //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
        //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        //finds the percent of power to each wheel and multiplies it by the speed
        if (gamepad2.dpad_up)
        {
            TleftDrive.setPower(1*speed);
            TrightDrive.setPower(1*speed);
            BleftDrive.setPower(1*speed);
            BrightDrive.setPower(1*speed);
        }
        else if (gamepad2.dpad_down)
        {
            TleftDrive.setPower(-1*speed);
            TrightDrive.setPower(-1*speed);
            BleftDrive.setPower(-1*speed);
            BrightDrive.setPower(-1*speed);
        }

    }

    public void Grip() {
            if (gamepad1.a) {
                grip.setPosition(0);
            }else if (gamepad1.b)
            {
                grip.setPosition(0.5);
            }
        }
    public void Baws() {
        if (gamepad1.x) {
            baws.setPower(-0.5);
        }else if (gamepad1.y)
        {
            baws.setPower(0);
        }
    }
    public void Turret(){




        if (gamepad1.dpad_left)
        {
           forkVal=forkVal+10;
           sleep(10);
        }else
        if (gamepad1.dpad_right)
        {
            forkVal=forkVal-10;
            sleep(10);
        }else
        {
    //turret.setPower(0.001);
    //turret.setPower(-0.001);
         }
        turret.setTargetPosition(forkVal);
        turret.setPower(0.25);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("curPos",turret.getCurrentPosition());
        telemetry.addData("target pos",turret.getTargetPosition());
        telemetry.addData("power",turret.getPower());
    telemetry.update();
    }
    public void Kanon(){
        kanon = hardwareMap.get(DcMotorEx.class, "LIFT");
        int upk = 1, downk = 1;
        DigitalChannel mag_up = hardwareMap.get(DigitalChannel.class, "mag_up_limit");
        mag_up.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel mag_down = hardwareMap.get(DigitalChannel.class, "mag_down_limit");
        mag_down.setMode(DigitalChannel.Mode.INPUT);

        if (mag_up.getState() == false) {
            upk = 0;
        } else upk = 1;

        if (mag_down.getState() == false) {
            downk = 0;
        } else downk = 1;

        if (gamepad1.dpad_up) {
            kanon.setPower(-0.6 * upk);
        } else if (gamepad1.dpad_down) {
            kanon.setPower(0.6*downk);
        } else {
            kanon.setPower(0);
        }


        telemetry.addData("upk", upk);
        telemetry.update();
    }
    public void BeyBlade(){
            BeyBlade.setPower(gamepad1.right_trigger);
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
    public void BetterIntakeTeleOp() {
        double IntakePower = 1;

        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "COLLECT");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (gamepad1.right_bumper) {//1
            InTake.setPower(-IntakePower);
            baws.setPower(-0.5);
        } else if (gamepad1.right_stick_button)//-1
        {
            InTake.setPower(IntakePower);
            baws.setPower(0.5);
        } else if (!gamepad1.right_bumper && !gamepad1.right_stick_button) {//0
            InTake.setPower(0);
            baws.setPower(0);
        }

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


