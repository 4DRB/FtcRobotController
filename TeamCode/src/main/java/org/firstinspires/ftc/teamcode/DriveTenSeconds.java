package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Array;
import java.util.Arrays;

@Autonomous(name = "DriveTenSeconds")
@Disabled
public class DriveTenSeconds extends OpMode {
    //member data
    //declaring left motor
    DcMotor FL = null;
    //declaring right motor
    DcMotor FR = null;

    DcMotor BR = null;

    DcMotor BL = null;



    //variable for current robot power
    double robotPower = .5;

    //creating a new elapsed time timer
    ElapsedTime timer = new ElapsedTime();

    //variable for current time
    double currentTime;


    @Override
    public void init() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        currentTime = 0.0;
    }

    @Override
    public void start() {
        timer.startTime();
    }

    @Override
    public void loop() {
        move(.2,0,.7,10);

    }
    public void move(double x, double y, double rot, double distance){
            //do some sort of distance conversion
            if(FL.getCurrentPosition() < FL.getCurrentPosition() + distance && FR.getCurrentPosition() < FR.getCurrentPosition() + distance
            && BR.getCurrentPosition() < BR.getCurrentPosition() + distance && BL.getCurrentPosition() < BL.getCurrentPosition() + distance)
            {
                //drive at a power
                setPower(x,y,rot);
            }
            else{
                setPower(0.0,0.0,0.0);
            }
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
}