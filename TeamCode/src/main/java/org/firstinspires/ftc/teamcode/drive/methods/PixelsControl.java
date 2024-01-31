package org.firstinspires.ftc.teamcode.drive.methods;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class PixelsControl {
    public DcMotor motor_suction;
    public DcMotor motor_tele;
    public Servo servo_flip;
    public Servo servo_hook;
    public Servo servo_plane;

    public void runOpMode() throws InterruptedException {

    }
    public void initHW(OpMode op) {

        motor_suction = op.hardwareMap.get(DcMotor.class, "motor_capture");
        motor_tele = op.hardwareMap.get(DcMotor.class, "motor_tele");
        servo_flip = op.hardwareMap.get(Servo.class, "servo_flip");
        servo_hook = op.hardwareMap.get(Servo.class, "servo_hook");
        servo_plane = op.hardwareMap.get(Servo.class, "servo_plane");

        motor_suction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_tele.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_suction.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_tele.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_suction.setPower(0);
        motor_tele.setPower(0);
        servo_flip.setPosition(0);
        servo_hook.setPosition(0.8);
        servo_plane.setPosition(0.2);
    }

    public void setSuction(double power) {
        motor_suction.setPower(power);
    }

    public void setTele(double power) {
        motor_tele.setPower(power);
    }

    public void setFlip(double angle) {
        servo_flip.setPosition(angle);
    }

    public void setHook(double angle) {
        servo_hook.setPosition(angle);
    }

    public void setPlane(double angle) {
        servo_plane.setPosition(angle);
    }
}