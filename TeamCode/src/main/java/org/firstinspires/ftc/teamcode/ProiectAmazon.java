package org.firstinspires.ftc.teamcode;

import android.content.Context;

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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.OutputStreamWriter;


@TeleOp(name = "ProiectAmazon")
public class ProiectAmazon extends LinearOpMode {

    Encoder leftEncoder, rightEncoder, frontEncoder;

    /**
     *
     */
    @Override
    public void runOpMode() throws InterruptedException {
        DriveThread TeleThread = new DriveThread();

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));


        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);


        String filename = "coords.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);

String toAmazon=null;
        waitForStart();
        //TeleThread.start();
        while (opModeIsActive()) {
            int fE= frontEncoder.getCurrentPosition();
            int lE= leftEncoder.getCurrentPosition();
            int rE= rightEncoder.getCurrentPosition();
            toAmazon =  toAmazon+Integer.toString(fE)+" "+Integer.toString(lE)+" "+Integer.toString(rE)+"\n";




telemetry.addData("file",ReadWriteFile.readFile(file));
            telemetry.addData("amazon",toAmazon);

            telemetry.update();
        }
        ReadWriteFile.writeFile(file, toAmazon);
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
                Context context = hardwareMap.appContext;
                /*try {
                    wait(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }*/
                String filename = "coords.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                String toAmazon = frontEncoder.getCurrentPosition()+" "+leftEncoder.getCurrentPosition()+" "+rightEncoder.getCurrentPosition();
                try {
                    OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(filename, Context.MODE_APPEND));

                    // write each configuration parameter as a string on its own line
                    outputStreamWriter.write(toAmazon+"\n");


                    outputStreamWriter.close();
                }
                catch (IOException e) {
                    telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
                }

                telemetry.addData("leftEN",leftEncoder.getCurrentPosition());
                telemetry.addData("rightEN",rightEncoder.getCurrentPosition());
                telemetry.addData("frontEn",frontEncoder.getCurrentPosition());
                telemetry.update();


            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.


            //Logging.log("end of thread %s", this.getName());
        }
    }
}


