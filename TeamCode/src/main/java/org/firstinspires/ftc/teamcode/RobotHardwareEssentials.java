package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardwareEssentials {
    //constants
    public static final Double TRIGGER_TRESHOLD = 0.5;

    //drive base motors
    public DcMotor motorLeftFront;
    public DcMotor motorLeftBack;
    public DcMotor motorRightFront;
    public DcMotor motorRightBack;

    //odometers
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    private HardwareMap hardwareMap;

    public RobotHardwareEssentials(HardwareMap aHardwareMap) {
        hardwareMap = aHardwareMap;

        //conf drive motors
        motorLeftFront=hardwareMap.dcMotor.get("motorLeftFront");
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftBack=hardwareMap.dcMotor.get("motorLeftBack");
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRightFront=hardwareMap.dcMotor.get("motorRightFront");
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRightBack=hardwareMap.dcMotor.get("motorRightBack");
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // shadow motors with odo enc
        encoderLeft=motorLeftBack;
        encoderRight=motorRightBack;
        encoderAux = motorRightFront;

        stop();
        resetDriveEncoders();
    }

    public void resetDriveEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }

    //robot geo constants:
    final static double L = 38.4; //distance between encoder 1 and 2 in cm
    final static double B = 17.0; //distance between encoder 1 and 3
    final static double R = 3.0;  //wheel radius in cm
    final static double N = 8192; //enc tick per rev, REV through bore encoder
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    //keep track of odo encoders between updates:
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;

    /**************************************************************************
     * Odometry
     *
     * Notes:
     * n1,n2,n3 are encoder values for the left right and back(aux) omni-wheels
     * dn1,dn2,dn3 are the diff of encoder values between two reads
     * dx ,dy ,dtheta describe the robot movement between two reads (in robot coords)
     * X, Y, Theta are the coords on the field and the heading of the robot
     *
     *************************************************************************/

    //XyhVector is a tuple (x,y,h)

    public XyhVector START_POS = new XyhVector(213,102,Math.toRadians(-173));
    public XyhVector pos = new XyhVector(START_POS);

    public void odometry() {
        oldRightPosition=currentRightPosition;
        oldLeftPosition=currentLeftPosition;
        oldAuxPosition=currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = -encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldAuxPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        double dtheta = cm_per_tick * (dn2-dn1) / L;
        double dx = cm_per_tick * (dn1+dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L);

        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;


    }

}
