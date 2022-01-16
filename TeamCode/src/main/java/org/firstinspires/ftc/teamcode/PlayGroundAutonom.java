/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;


import java.util.Arrays;
import java.util.*;

@Autonomous
public class PlayGroundAutonom extends LinearOpMode
{
    static double ringCount = 0;

    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    public double offTLD=3.25;
    public double offBLD=2.25;
    public double offTRD=4;
    public double offBRD=6.5;
    double  power   = 0.3;
    final double TICKS_PER_REV = 537.6 ;    // 1440 eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference 9.6
    double TICKS_PER_CM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    final double TICKS_PER_REV_ENC = 8192 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION_ENC = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_CM_ENC = 6;     // For figuring circumference 9.6
    //double TICKS_PER_CM_ENC = (TICKS_PER_REV_ENC * DRIVE_GEAR_REDUCTION_ENC) / (WHEEL_DIAMETER_CM_ENC * 3.1415)-1;

    final double TICKS_PER_100_CM = 40972;//40972//41341//43088
    final double TICKS_PER_CM_ENC = TICKS_PER_100_CM/100;

    int initDiff,lastDiff,diffDiff;
    Encoder leftEncoder, rightEncoder, frontEncoder;
    BNO055IMU imu;
    Orientation angles;
    double vE = 0;
    double vI = vE;
    double vA = vE - vI;

    double vT =0;
    double vTresh = vT*20/100;
    double vDI = 1;
    double vR= vT-vA;
    double vCmA;
    double vCmR;

    int once = 0;
    double gyroX;
    double initX;
    double radiansX;
    double realX;
    double posX;
    double cosX;
    int semn;
    double powerL;
    double powerR;
    double multiplierDecc=1;



    @Override
    public void runOpMode()
    {
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Shooter.setDirection(Servo.Direction.REVERSE);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);


        FL = hardwareMap.get(DcMotor.class, "leftFront");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        BL = hardwareMap.get(DcMotor.class, "leftRear");
        BR = hardwareMap.get(DcMotor.class, "rightRear");
//stop and reset
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// this offsets the two motors that are facing the opposite direction.
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveThread TeleThread = new DriveThread();
        OdoThread OdoThread = new OdoThread();
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");
        sleep(1000);
        InitValX=leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
        InitValY=frontEncoder.getCurrentPosition();

        waitForStart();


        while (1==1)
        {
            sleep(100);
        }

    }

    private void MSBAutonom() {
        double power = -0.94;;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(400);
        InTake.setPower(-1);
        sleep(2500);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }
    private void BetterShooterAutonomV2(double power,long sleep) {
        //double power = -0.78;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(-power);
        Launcher2.setPower(-power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(200);
        InTake.setPower(-1);
        sleep(sleep);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }
    private void SSBAutonom() {
        double power = -0.94;
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //sleep(300);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(400);
        Shooter.setPosition(0.3);
        sleep(700);
        InTake.setPower(-1);
        sleep(1300);
        InTake.setPower(0);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches){

        double encPower = 0.2;
// this creates the variables that will be calculated

        double OldXL = leftEncoder.getCurrentPosition();
        double OldXR = rightEncoder.getCurrentPosition();

        double PosXL = leftEncoder.getCurrentPosition();
        double PosXR = rightEncoder.getCurrentPosition();


        double semn;

        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;



        double WantedY = 0;
        double WantedXL = leftInches*TICKS_PER_CM_ENC;
        double WantedXR = rightInches*TICKS_PER_CM_ENC;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));
        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn",frontEncoder.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// resets all the data for the encoders.

        /*int XLSign = 1;
        int XRSign = 1;
        int LegitY = 0;
        double LegitXL = PosXL+WantedXL-leftEncoder.getCurrentPosition();
        double LegitXR = PosXR+WantedXR-rightEncoder.getCurrentPosition();
        boolean CloseEnough4Me=false;
        while(CloseEnough4Me==false)
        {
            LegitXL = PosXL+WantedXL-leftEncoder.getCurrentPosition();
            LegitXR = PosXR+WantedXR-rightEncoder.getCurrentPosition();

            telemetry.addData("Pos XL",PosXL);
            telemetry.addData("Wanted XL",WantedXL);
            telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
            telemetry.addData("Legit XL",LegitXL);
            telemetry.addData("Pos XR",PosXR);
            telemetry.addData("Wanted XR",WantedXR);
            telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
            telemetry.addData("Legit XR",LegitXR);
            telemetry.update();


            if (Math.abs(LegitXL)<CloseEnough&&Math.abs(LegitXR)<CloseEnough){
                CloseEnough4Me = true;
                break;
            }


            if (LegitXL<0) XLSign = -1;
            else XLSign = 1;
            if (Math.abs(LegitXL)>100)
            {
                FL.setPower(Math.abs(encSpeed)*XLSign);
                BL.setPower(Math.abs(encSpeed)*XLSign);
                telemetry.addData("Pos XL",PosXL);
                telemetry.addData("Wanted XL",WantedXL);
                telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
                telemetry.addData("Legit XL",LegitXL);
                telemetry.addData("Pos XR",PosXR);
                telemetry.addData("Wanted XR",WantedXR);
                telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
                telemetry.addData("Legit XR",LegitXR);
                telemetry.update();
            }

            if (LegitXR<0) XRSign = -1;
            else XRSign = 1;
            if (Math.abs(LegitXR)>100)
            {
                FR.setPower(Math.abs(encSpeed)*XRSign);
                BR.setPower(Math.abs(encSpeed)*XRSign);
                telemetry.addData("Pos XL",PosXL);
                telemetry.addData("Wanted XL",WantedXL);
                telemetry.addData("Curr XL",leftEncoder.getCurrentPosition());
                telemetry.addData("Legit XL",LegitXL);
                telemetry.addData("Pos XR",PosXR);
                telemetry.addData("Wanted XR",WantedXR);
                telemetry.addData("Curr XR",rightEncoder.getCurrentPosition());
                telemetry.addData("Legit XR",LegitXR);
                telemetry.update();
            }

        }
*/
        PosXL = leftEncoder.getCurrentPosition();
        PosXR = rightEncoder.getCurrentPosition();

        double x=(PosXL-OldXL)-(PosXR-OldXR);

        while (Math.abs(x)>50&&opModeIsActive()){
            PosXL = leftEncoder.getCurrentPosition();
            PosXR = rightEncoder.getCurrentPosition();
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("SE CORECTEAZA POG!1!!",1);
            telemetry.update();


            x=(PosXL-OldXL)-(PosXR-OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn*encPower);
            BL.setPower(-semn*encPower);
            if (Math.abs(x)<50)break;

        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void strafeDrive(double speed, double leftInches, double rightInches){

        double encPower = 0.2;
// this creates the variables that will be calculated

        double OldXL = leftEncoder.getCurrentPosition();
        double OldXR = rightEncoder.getCurrentPosition();

        double PosXL = leftEncoder.getCurrentPosition();
        double PosXR = rightEncoder.getCurrentPosition();

        double x=(PosXL-OldXL)-(PosXR-OldXR);

        double semn;
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        FL.setTargetPosition(-newLeftFrontTarget);
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));


        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PosXL = leftEncoder.getCurrentPosition();
        PosXR = rightEncoder.getCurrentPosition();
// resets all the data for the encoders.
        x=(PosXL-OldXL)-(PosXR-OldXR);

        while (Math.abs(x)>50&&opModeIsActive()){
            PosXL = leftEncoder.getCurrentPosition();
            PosXR = rightEncoder.getCurrentPosition();
            telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEn",frontEncoder.getCurrentPosition());
            telemetry.update();


            x=(PosXL-OldXL)-(PosXR-OldXR);

            semn = x / Math.abs(x);

            FL.setPower(-semn*encPower);
            BL.setPower(-semn*encPower);
            if (Math.abs(x)<50)break;

        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void normalDrive(double speed, double leftInches, double rightInches){

// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));
        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
            initDiff=frontEncoder.getCurrentPosition()-leftEncoder.getCurrentPosition();
        }


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void betterDrive(double speed, double leftInches, double rightInches){

        /**
         * variabile deccelerare
         */

        vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
        vI = vE;
        vA = vE - vI;

        vT = ((leftInches+rightInches) / 2 )* 434;
        vR = vT - vA;
        vTresh = vT*40/100;
        vDI = 1;
        once = 0;
        /**
         * variabile PID de buget
         */
        gyroX=angles.firstAngle;
        initX=gyroX;
        realX=gyroX-initX;
        radiansX=Math.toRadians(realX);
        cosX=Math.cos(radiansX);
        if (cosX>0)
        {
            semn=1;
        }
        else if(cosX<0)
        {
            semn=-1;
        }
        else
        {
            semn=0;
        }

        if (semn==1)
        {
            powerR=power*cosX;
            powerL=power;
        }
        else if(semn==-1)
        {
            powerL=power*cosX;
            powerR=power;
        }
        else {
            powerL=power;
            powerR=power;
        }
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));

        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {   vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
            vA = vE - vI;
            vR = vT - vA;
            if (vR<vTresh)
            {
                if(once<=10)
                {
                    vDI = vA;
                    once++;
                }
                speed = vR/vDI;
            }
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            telemetry.addData("vE",vE);
            telemetry.addData("vI",vI);
            telemetry.addData("vA",vA);
            telemetry.addData("vT",vT);
            telemetry.addData("vTresh",vTresh);
            telemetry.addData("vDI",vDI);
            telemetry.addData("vR",vR);
            telemetry.addData("speed",speed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angles", angles.firstAngle);
            telemetry.update();

        }


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void corectionDriveV2(double speed, double leftInches, double rightInches){

        /**
         * variabile deccelerare
         */
        //TODO pune taxa de 5% pe vT
        vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
        vI = vE;
        vA = vE - vI;

        vT = ((leftInches+rightInches) / 2 )* TICKS_PER_CM_ENC;
        vR = vT - vA;
        vTresh = vT*15/100;
        vDI = 1;
        once = 0;
        /**
         * variabile PID de buget
         */
        gyroX=angles.firstAngle;
        initX=gyroX;
        realX=gyroX-initX;
        posX=Math.abs(realX);
        if (realX>0)
        {
            powerR=power*posX;
            powerL=power;
        }
        else if(realX<0)
        {
            powerL=power*posX;
            powerR=power;
        }
        else
        {
            powerL=power;
            powerR=power;
        }


// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        /*FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);
*/
        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));

        while(vA<=(TICKS_PER_100_CM-TICKS_PER_CM_ENC*2) && opModeIsActive())
        {   angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gyroX=angles.firstAngle;
            realX=gyroX-initX;
            posX=Math.abs(realX);
            if (realX>0)
            {
                powerR=power+0.01*posX;
                powerL=power;
            }
            else if(realX<0)
            {
                powerL=power+0.01*posX;
                powerR=power;
            }
            else
            {
                powerL=power;
                powerR=power;
            }
            vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
            vA = vE - vI;
            vR = vT - vA;
            vCmA = vA/TICKS_PER_CM_ENC;
            vCmR = vR/TICKS_PER_CM_ENC;

            if (vR<vTresh)
            {
                if(once<=10)
                {
                    vDI = vA;
                    once++;
                }
                if (multiplierDecc>=0.1) {
                    multiplierDecc = vR / vDI;
                }
            }
            FR.setPower(powerR*2*multiplierDecc);
            FL.setPower(powerL*2*multiplierDecc);
            BR.setPower(powerR*2*multiplierDecc);
            BL.setPower(powerL*2*multiplierDecc);




            telemetry.addData("vDI",vDI);
            telemetry.addData("vR",vR);
            telemetry.addData("multi",multiplierDecc);
            telemetry.addData("vCmR",vCmR);
            telemetry.addData("vCmA",vCmA);
            telemetry.addData("powerR",multiplierDecc);
            telemetry.addData("powerL",multiplierDecc);
            telemetry.update();

        }


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void corectionDriveV1(double speed, double leftInches, double rightInches){

        /**
         * variabile deccelerare
         */

        vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
        vI = vE;
        vA = vE - vI;

        vT = ((leftInches+rightInches) / 2 )* 434;
        vR = vT - vA;
        vTresh = vT*40/100;
        vDI = 1;
        once = 0;
        /**
         * variabile PID de buget
         */
        gyroX=angles.firstAngle;
        initX=gyroX;
        realX=gyroX-initX;
        radiansX=Math.toRadians(realX);
        cosX=Math.cos(radiansX);
        if (cosX>0)
        {
            semn=1;
        }
        else if(cosX<0)
        {
            semn=-1;
        }
        else
        {
            semn=0;
        }

        if (semn==1)
        {
            powerR=power*cosX;
            powerL=power;
        }
        else if(semn==-1)
        {
            powerL=power*cosX;
            powerR=power;
        }
        else {
            powerL=power;
            powerR=power;
        }
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;

// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converdets it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));

        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {   angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gyroX=angles.firstAngle;
            realX=gyroX-initX;
            radiansX=Math.toRadians(realX);
            cosX=Math.cos(radiansX);
            if (cosX>0)
            {
                semn=1;
            }
            else if(cosX<0)
            {
                semn=-1;
            }
            else
            {
                semn=0;
            }

            if (semn==1)
            {
                powerR=power*cosX;
                powerL=power;
            }
            else if(semn==-1)
            {
                powerL=power*cosX;
                powerR=power;
            }
            else {
                powerL=power;
                powerR=power;
            }
            vE = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
            vA = vE - vI;
            vR = vT - vA;
            if (vR<vTresh)
            {
                if(once<=10)
                {
                    vDI = vA;
                    once++;
                }
                multiplierDecc = vR/vDI;
            }
            FR.setPower(powerR*2*multiplierDecc);
            FL.setPower(powerL*2*multiplierDecc);
            BR.setPower(powerR*2*multiplierDecc);
            BL.setPower(powerL*2*multiplierDecc);

            telemetry.addData("angles", angles.firstAngle);
            telemetry.addData("powerR",powerR);
            telemetry.addData("powerL",powerL);
            telemetry.addData("realX",realX);
            telemetry.addData("cosX",cosX);
            telemetry.addData("multiplier",multiplierDecc);
            telemetry.addData("vE",vE);
            telemetry.addData("vI",vI);
            telemetry.addData("vA",vA);
            telemetry.addData("vT",vT);
            telemetry.addData("vTresh",vTresh);
            telemetry.addData("vDI",vDI);
            telemetry.addData("vR",vR);
            telemetry.update();

        }


        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






    }
    public void normalstrafeDrive(double speed, double leftInches, double rightInches){
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_CM);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_CM);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        FL.setTargetPosition(-newLeftFrontTarget);
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));


        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
        }

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void SingleShotAutonom(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1700);
        Shooter.setPosition(0.3);
        sleep(500);
        Shooter.setPosition(0);
        sleep(100);
        Launcher1.setPower(0);
        Launcher2.setPower(0);
    }

    /*public void MultiShotAutonom(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1800);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        sleep(100);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        sleep(250);
        Shooter.setPosition(0.3);
        sleep(600);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }*/
    public void MultiShottestAutonom(){
        double power = -1;
        DcMotorEx Launcher1 = hardwareMap.get(DcMotorEx.class,"rightEncoder");
        DcMotorEx Launcher2 = hardwareMap.get(DcMotorEx.class,"frontEncoder");
        Servo Shooter = hardwareMap.get(Servo.class,"SR_SHOOTER");
        Launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        Launcher1.setPower(power);
        Launcher2.setPower(power);
        sleep(1700);
        Shooter.setPosition(0.3);
        sleep(100);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(100);
        Shooter.setPosition(0);
        sleep(300);
        Shooter.setPosition(0.3);
        sleep(300);
        Shooter.setPosition(0);
        Launcher1.setPower(0);
        Launcher2.setPower(0);



    }
    public void IntakeAutonom(double power){
        DcMotorEx InTake = hardwareMap.get(DcMotorEx.class,"leftEncoder");
        InTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InTake.setPower(power);
    }
    public void ClampAutonom(double position) {
        Servo clamp = hardwareMap.get(Servo.class, "SR_CLAMP");


        clamp.setPosition(position);
        sleep(500);
        //telemetry.addData("Servo Position", clamp.getPosition());
        //telemetry.addData("Status", "Running");
    }
    public void CremalieraAutonom(double viteza) {      //     - scoate , + baga
        CRServo cremaliera_Servo = hardwareMap.get(CRServo.class, "SR_CRM");
        DigitalChannel mag_crm = hardwareMap.get(DigitalChannel.class, "Mag_CRM");
        mag_crm.setMode(DigitalChannel.Mode.INPUT);

        cremaliera_Servo.setPower(viteza);

        sleep(500);

        while (mag_crm.getState())
        {
            cremaliera_Servo.setPower(viteza);
            if (!mag_crm.getState())
            {
                cremaliera_Servo.setPower(0);
                break;
            }
        }
        if (!mag_crm.getState())
        {
            cremaliera_Servo.setPower(0);
        }



    }
    public void GlisieraAutonom(long sleep,double speed) {

        DcMotor glisiera = hardwareMap.get(DcMotorEx.class, "GLS");
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DigitalChannel glsMg = hardwareMap.get(DigitalChannel.class, "Mag_GLS");


        glisiera.setPower(speed);
        sleep(sleep);
        glisiera.setPower(0);
    }


    public void move(double x, double y, double rot, double cmdistance){
        //do some sort of distance conversion
        double distance;
        distance = cmdistance * TICKS_PER_CM;
        if(FL.getCurrentPosition() < FL.getCurrentPosition() + distance && FR.getCurrentPosition() < FR.getCurrentPosition() + distance
                && BR.getCurrentPosition() < BR.getCurrentPosition() + distance && BL.getCurrentPosition() < BL.getCurrentPosition() + distance)
        {
            //drive at a power
            setPower(x,y,rot);
        }
        else{
            setPower(0.0,0.0,0.0);
        }
        while(FL.isBusy() && BL.isBusy() &&  FR.isBusy() &&  BR.isBusy() && opModeIsActive())
        {
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("Pos:",FL.getCurrentPosition());
            telemetry.update();
        }
        setPower(0.0,0.0,0.0);
    }

    public void setPower(double x, double y, double rot){
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};

        Arrays.sort(motorPowers);

        if(motorPowers[3] != 0){
            frontLeftMotorPower /= motorPowers[3];
            frontRightMotorPower /= motorPowers[3];
            backRightMotorPower /= motorPowers[3];
            backLeftMotorPower /= motorPowers[3];
        }
    }












    public void WRealeaseLeftAutonom(double position) {
        Servo wRelease = hardwareMap.get(Servo.class, "Sr_WReleaseLeft");

        wRelease.setPosition(position);

        for(int i =1;i<=2000;i++)
        {wRelease.setPosition(position);}

        telemetry.addData("Servo Position", wRelease.getPosition());
        telemetry.update();
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
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("powerR",powerR);
                telemetry.addData("powerL",powerL);
                telemetry.addData("realX",realX);
                telemetry.addData("cosX",cosX);
                telemetry.addData("multiplier",multiplierDecc);
                telemetry.addData("vE",vE);
                telemetry.addData("vI",vI);
                telemetry.addData("vA",vA);
                telemetry.addData("vT",vT);
                telemetry.addData("vTresh",vTresh);
                telemetry.addData("vDI",vDI);
                telemetry.addData("vR",vR);
                telemetry.update();
//if(!opModeIsActive()){break;}

            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }
    double InitValX;
    double InitValY;
    double EncValX;
    double EncValY;
    double OdoValX;
    double OdoValY;
    double CmValX;
    double CmValY;

    public class OdoThread extends Thread {
        public OdoThread() {
            this.setName("OdoThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {


            while (!isInterrupted()) {
                /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                EncValX = leftEncoder.getCurrentPosition()/2 + rightEncoder.getCurrentPosition() / 2;
                EncValY = frontEncoder.getCurrentPosition();
                OdoValX = EncValX-InitValX;
                OdoValY = EncValY-InitValY;
                CmValX=OdoValX/TICKS_PER_CM_ENC;
                CmValY=OdoValY/TICKS_PER_CM_ENC;
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("OdoValX",OdoValX);
                telemetry.addData("EncValX",EncValX);
                telemetry.addData("CmValX",CmValX);
                telemetry.addData("OdoValY",OdoValY);
                telemetry.addData("CmValY",CmValY);
                telemetry.update();
//if(!opModeIsActive()){break;}
*/
            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }


}