package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
public class Intake {
    DcMotor intakeL = null;
    DcMotor intakeR = null;
    HardwareMap hw = null;
    public Intake(HardwareMap hw){
        this.hw = hw;
        intakeL = hw.get(DcMotor.class, "IntakeL");
        intakeR = hw.get(DcMotor.class,"IntakeR");
    }

    public void run(double power){
        intakeR.setPower(power);
        intakeL.setPower(power);
    }







}
