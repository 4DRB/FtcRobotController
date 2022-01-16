package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpStick", group="Linear Opmode")
@Disabled
public class  TeleOpStick extends LinearOpMode {

    boolean prevX = false, prevLeft = false, prevRight = false;
    double speed, robotAngle;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    public final double offTLD=3.25;
    public final double offBLD=2.25;
    public final double offTRD=4;
    public final double offBRD=6.5;

    double ok=0;
    double  power   = 0.3;
    double FranaS,FranaD;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        TleftDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        TrightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        BleftDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        BrightDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        TrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(ok==0)
            {TleftDrive.setPower(0);
                BleftDrive.setPower(0);
                TrightDrive.setPower(0);
                BrightDrive.setPower(0);
                ok=1;
            }
            telemetry.update();
            //ColectorE.setPower(0.7);
            //ColectorV.setPower(0.7);
            while (gamepad1.dpad_up) {
                TleftDrive.setPower(-power + power*-offTLD/100);
                BleftDrive.setPower(-power + power*-offBLD/100);
                TrightDrive.setPower(-power + power*-offTRD/100);
                BrightDrive.setPower(-power + power*-offBRD/100);

                CurbaFata(power, FranaD, FranaS);

                if (gamepad1.start)
                    power = 0.7;
                if (!gamepad1.dpad_up) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }
            telemetry.update();
            while (gamepad1.dpad_down) {
                TleftDrive.setPower(power + power*(offTLD-0.5)/100);
                BleftDrive.setPower(power + power*(offBLD-0.5)/100);
                TrightDrive.setPower(power + power*(offTRD+1)/100);
                BrightDrive.setPower(power + power*(offBRD+0.6)/100);

                CurbaSpate(power, FranaD, FranaS);

                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.dpad_down) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_left) {
                TleftDrive.setPower(power + power*offTLD/100);
                BleftDrive.setPower(-power + power* -offBLD/100);
                TrightDrive.setPower(-power + power* -offTRD/100);
                BrightDrive.setPower(power + power* offBRD/100);
                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.start)
                    power = 0.3;
                if (!gamepad1.dpad_left) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }
            telemetry.update();

            while (gamepad1.dpad_right) {
                TleftDrive.setPower(-power + power*-offTLD/100);
                BleftDrive.setPower(power + power*offBLD/100);
                TrightDrive.setPower(power + power*offTRD/100);
                BrightDrive.setPower(-power + power*-offBRD/100);
                if (gamepad1.start)
                    power = 0.3;
                if (!gamepad1.start)
                    power = 0.3;
                if (!gamepad1.dpad_right) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 0.3;
                    break;
                }
            }

            while (gamepad1.y) {
                TleftDrive.setPower(0.4);
                BrightDrive.setPower(0.4);
                if (!(gamepad1.y)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.a) {
                BleftDrive.setPower(0.4);
                TrightDrive.setPower(0.4);
                if (!(gamepad1.a)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            telemetry.update();
            while (gamepad1.x) {
                TleftDrive.setPower(-0.4);
                BrightDrive.setPower(-0.4);
                if (!(gamepad1.x)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.b) {
                BleftDrive.setPower(-0.4);
                TrightDrive.setPower(-0.4);
                if (!(gamepad1.b)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.left_bumper) {
                TleftDrive.setPower(0.4);
                BleftDrive.setPower(0.4);
                TrightDrive.setPower(-0.4);
                BrightDrive.setPower(-0.4);
                if (!gamepad1.left_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.right_bumper) {
                TleftDrive.setPower(-0.4);
                BleftDrive.setPower(-0.4);
                TrightDrive.setPower(0.4);
                BrightDrive.setPower(0.4);
                if (!gamepad1.right_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            prevX = gamepad1.x;


            //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
            speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //finds the angle the robot is moving at
            //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //finds the percent of power to each wheel and multiplies it by the speed
            TleftDrive.setPower(speed * Math.sin(robotAngle) - gamepad1.right_stick_x + power*-offTLD/100);
            TrightDrive.setPower(speed * Math.cos(robotAngle) + gamepad1.right_stick_x + power*offTLD/100);
            BleftDrive.setPower(speed * Math.cos(robotAngle) - gamepad1.right_stick_x + power*-offTLD/100);
            BrightDrive.setPower(speed * Math.sin(robotAngle) + gamepad1.right_stick_x + power*offTLD/100);
            telemetry.update();

        }
    }

    public void CurbaFata(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(-(power - FranaS));
            BleftDrive.setPower(-(power - FranaS));
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(-(power-FranaD));
            BrightDrive.setPower(-(power-FranaD));
        }

    }
    public void CurbaSpate(double power,double FranaD,double FranaS){
        FranaD=gamepad1.right_trigger;
        FranaS=gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(power - FranaS);
            BleftDrive.setPower(power - FranaS);
        }
        if(gamepad1.right_trigger!=0){
            TrightDrive.setPower(power-FranaD);
            BrightDrive.setPower(power-FranaD);
        }

    }
}