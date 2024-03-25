package org.firstinspires.ftc.teamcode.drive.methods;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
public class PixelsControl {
    public DcMotor motor_suction;
    public DcMotor motor_tele;
    public DcMotor motor_flip;
    public DcMotor encLeft;
    public Servo servo_hook;
    public Servo servo_hook_2;
    public Servo servo_plane;

    public Servo servo_hang;
    public TouchSensor touch;

    public void runOpMode() throws InterruptedException {

    }
    public void initHW(OpMode op) {

        motor_suction = op.hardwareMap.get(DcMotor.class, "motor_suction");
        motor_tele = op.hardwareMap.get(DcMotor.class, "motor_tele");
        servo_hook = op.hardwareMap.get(Servo.class, "servo_hook");
        servo_hook_2 = op.hardwareMap.get(Servo.class, "servo_hook_2");
        servo_plane = op.hardwareMap.get(Servo.class, "servo_plane");
        servo_hang = op.hardwareMap.get(Servo.class, "servo_hang");
        motor_flip = op.hardwareMap.get(DcMotor.class, "motor_flip");
        encLeft = op.hardwareMap.get(DcMotor.class, "encLeft");

        touch = op.hardwareMap.get(TouchSensor.class, "touch");

        motor_suction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_tele.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_suction.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_tele.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_suction.setPower(0);
        motor_tele.setPower(0);
        servo_hook_2.setPosition(0.3);
        servo_hook.setPosition(0.795);
        servo_plane.setPosition(0.25);
        motor_flip.setPower(-0.003);
        servo_hang.setPosition(1);
        encLeft.setPower(0);
    }

    public void setSuction(double power) {
        motor_suction.setPower(power);
    }

    public void setTele(double power) {
        motor_tele.setPower(power);
    }

    public void setFlip(double power) {
        motor_flip.setPower(power);
    }

    public void setHook(double angle) { servo_hook.setPosition(angle); }

    public void sethang(double angle) { servo_hang.setPosition(angle); }

    public void setHook_2(double angle) {
        servo_hook_2.setPosition(angle);
    }

    public void setPlane(double angle) {
        servo_plane.setPosition(angle);
    }


    public void encLeft(int i) {
        encLeft.setPower(i);
    }
}