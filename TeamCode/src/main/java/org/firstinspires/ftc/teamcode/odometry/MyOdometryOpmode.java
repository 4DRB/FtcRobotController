package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final static double L = 38.4; //distance between encoder 1 and 2 in cm
    final static double B = 17.0; //distance between encoder 1 and 3
    final static double R = 3.0;  //wheel radius in cm
    final static double N = 8192; //enc tick per rev, REV through bore encoder
    final double COUNTS_PER_INCH = N / (2.0 * Math.PI * R) ;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rf", rbName = "rb", lfName = "lf", lbName = "lb";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        //verticalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        globalPositionUpdate.reverseLeftEncoder();
        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseNormalEncoder();

        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
        telemetry.update();
        goToPositionStraight(0*COUNTS_PER_INCH,100*COUNTS_PER_INCH,0.5,0,0.5*COUNTS_PER_INCH);


        //Stop the thread
        globalPositionUpdate.stop();

    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        double ogOrientation = globalPositionUpdate.returnOrientation();
        double speed;
        while (opModeIsActive() && distance > allowableDistanceError)
        {


            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            //double pivotCorrection = 0;
   /*         speed = -Math.hypot(robotMovementXComponent, robotMovementYComponent);
            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            double robotAngle = Math.atan2(robotMovementYComponent, robotMovementXComponent) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            left_front.setPower(speed * Math.sin(robotAngle) - pivotCorrection*pivotK);
            right_front.setPower(speed * Math.cos(robotAngle) + pivotCorrection*pivotK);
            left_back.setPower(speed * Math.cos(robotAngle) - pivotCorrection*pivotK);
            right_back.setPower(speed * Math.sin(robotAngle) + pivotCorrection*pivotK);
*/
            double frontLeftMotorPower = -robotMovementYComponent - robotMovementXComponent ;
            double frontRightMotorPower = -robotMovementYComponent + robotMovementXComponent;
            double backLeftMotorPower = -robotMovementYComponent + robotMovementXComponent ;
            double backRightMotorPower = -robotMovementYComponent - robotMovementXComponent;

            left_front.setPower(frontLeftMotorPower);
            right_front.setPower(frontRightMotorPower);
            left_back.setPower(backLeftMotorPower);
            right_back.setPower(backRightMotorPower);

            telemetry.addData("xcomp", robotMovementXComponent);
            telemetry.addData("ycomp",robotMovementYComponent);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("verticalRight", verticalRight.getCurrentPosition());
            telemetry.addData("verticalLeft", verticalLeft.getCurrentPosition());
            telemetry.addData("horizontal", horizontal.getCurrentPosition());

            telemetry.addData("distance", distance);
            telemetry.addData("AllowableDistanceErr", allowableDistanceError);
            telemetry.update();
        }

        double pivotCorrection=1;

        while (opModeIsActive() && Math.abs(desiredRobotOrientation - globalPositionUpdate.returnOrientation() ) >=3){


            if (pivotCorrection>=0.25){
                pivotCorrection = 0.0125*(desiredRobotOrientation - globalPositionUpdate.returnOrientation());
            }

            //double pivotCorrection = 0;
   /*         speed = -Math.hypot(robotMovementXComponent, robotMovementYComponent);
            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            double robotAngle = Math.atan2(robotMovementYComponent, robotMovementXComponent) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            left_front.setPower(speed * Math.sin(robotAngle) - pivotCorrection*pivotK);
            right_front.setPower(speed * Math.cos(robotAngle) + pivotCorrection*pivotK);
            left_back.setPower(speed * Math.cos(robotAngle) - pivotCorrection*pivotK);
            right_back.setPower(speed * Math.sin(robotAngle) + pivotCorrection*pivotK);
*/
            double frontLeftMotorPower = - pivotCorrection;
            double frontRightMotorPower =  pivotCorrection;
            double backLeftMotorPower = - pivotCorrection;
            double backRightMotorPower =  pivotCorrection;

            left_front.setPower(frontLeftMotorPower);
            right_front.setPower(frontRightMotorPower);
            left_back.setPower(backLeftMotorPower);
            right_back.setPower(backRightMotorPower);


            telemetry.addLine("IN Orientation CORRECTION");
            telemetry.addData("pivotCorr", pivotCorrection);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("verticalRight", verticalRight.getCurrentPosition());
            telemetry.addData("verticalLeft", verticalLeft.getCurrentPosition());
            telemetry.addData("horizontal", horizontal.getCurrentPosition());

            telemetry.addData("distance", distance);
            telemetry.addData("AllowableDistanceErr", allowableDistanceError);
            telemetry.update();
        }
    }
    public void goToPositionStraight(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        double ogDistance=distance;
        double distanceK=1;
        double pivotCorrection=1;
        while (opModeIsActive() && distance > allowableDistanceError || Math.abs(desiredRobotOrientation - globalPositionUpdate.returnOrientation() ) >=0.5)
        {


            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);


                pivotCorrection = 0.0125*(desiredRobotOrientation - globalPositionUpdate.returnOrientation());

            if (distance/ogDistance<=0.2)
            {
                robotPower=0.1;
            }
            double frontLeftMotorPower = -robotMovementYComponent - robotMovementXComponent- pivotCorrection;;
            double frontRightMotorPower = -robotMovementYComponent + robotMovementXComponent+pivotCorrection;;
            double backLeftMotorPower = -robotMovementYComponent + robotMovementXComponent- pivotCorrection;;
            double backRightMotorPower = -robotMovementYComponent - robotMovementXComponent+pivotCorrection;;

            left_front.setPower(frontLeftMotorPower*distanceK);
            right_front.setPower(frontRightMotorPower*distanceK);
            left_back.setPower(backLeftMotorPower*distanceK);
            right_back.setPower(backRightMotorPower*distanceK);

            telemetry.addData("xcomp", robotMovementXComponent);
            telemetry.addData("ycomp",robotMovementYComponent);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("verticalRight", verticalRight.getCurrentPosition());
            telemetry.addData("verticalLeft", verticalLeft.getCurrentPosition());
            telemetry.addData("horizontal", horizontal.getCurrentPosition());

            telemetry.addData("distance", distance);
            telemetry.addData("AllowableDistanceErr", allowableDistanceError);
            telemetry.update();
        }



        while (opModeIsActive() && Math.abs(desiredRobotOrientation - globalPositionUpdate.returnOrientation() ) >=3){


            if (pivotCorrection>=0.25){
                pivotCorrection = 0.0125*(desiredRobotOrientation - globalPositionUpdate.returnOrientation());
            }

            //double pivotCorrection = 0;
   /*         speed = -Math.hypot(robotMovementXComponent, robotMovementYComponent);
            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            double robotAngle = Math.atan2(robotMovementYComponent, robotMovementXComponent) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            left_front.setPower(speed * Math.sin(robotAngle) - pivotCorrection*pivotK);
            right_front.setPower(speed * Math.cos(robotAngle) + pivotCorrection*pivotK);
            left_back.setPower(speed * Math.cos(robotAngle) - pivotCorrection*pivotK);
            right_back.setPower(speed * Math.sin(robotAngle) + pivotCorrection*pivotK);
*/
            double frontLeftMotorPower = - pivotCorrection;
            double frontRightMotorPower =  pivotCorrection;
            double backLeftMotorPower = - pivotCorrection;
            double backRightMotorPower =  pivotCorrection;

            left_front.setPower(frontLeftMotorPower);
            right_front.setPower(frontRightMotorPower);
            left_back.setPower(backLeftMotorPower);
            right_back.setPower(backRightMotorPower);


            telemetry.addLine("IN Orientation CORRECTION");
            telemetry.addData("pivotCorr", pivotCorrection);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("verticalRight", verticalRight.getCurrentPosition());
            telemetry.addData("verticalLeft", verticalLeft.getCurrentPosition());
            telemetry.addData("horizontal", horizontal.getCurrentPosition());

            telemetry.addData("distance", distance);
            telemetry.addData("AllowableDistanceErr", allowableDistanceError);
            telemetry.update();
        }
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        //verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
