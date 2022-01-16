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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous
public class AutonomHackathonRosuParcare extends LinearOpMode
{
    static double ringCount = 0;

    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    private DcMotor BeyBlade = null;
    public double offTLD=3.25;
    public double offBLD=2.25;
    public double offTRD=4;
    public double offBRD=6.5;
    double  power   = 0.3;

    final double TICKS_PER_CM_TETRIX=45.83;

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


        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BeyBlade = hardwareMap.get(DcMotorEx.class, "BeyBlade");

//stop and reset
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BeyBlade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// this offsets the two motors that are facing the opposite direction.
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BeyBlade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();


        forward(900,1);
        spin(1000,-1);
        forward(2900,1);
        //spin(1000,1);
        /*BeyBlade(6000,-1);
        spin(750,-1);
        forward(5000,1);
        //sleep(10000);
        //forward(300,300,1);
*/
    }

    public void forward(double seconds ,double speed ){



        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated

        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
        while(opModeIsActive()&&seconds>0)
        {
            sleep(100);
            seconds = seconds - 100;
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("seconds",seconds);
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
    public void spin(double seconds ,double speed ){



        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// this creates the variables that will be calculated

        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(speed);
        BL.setPower(-speed);
        while(opModeIsActive()&&seconds>0)
        {
            sleep(100);
            seconds = seconds - 100;
            telemetry.addData("Status:", "Moving to pos");
            telemetry.addData("seconds",seconds);
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
    public void BeyBlade(int sleep,double power){
        BeyBlade.setPower(power);
        sleep(sleep);
    }




}