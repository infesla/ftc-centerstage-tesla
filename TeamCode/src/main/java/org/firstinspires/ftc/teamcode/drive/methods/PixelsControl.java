package org.firstinspires.ftc.teamcode.drive.methods;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
public class PixelsControl {
    public DcMotor motor_suction;
    public DcMotor motor_tele;
    public DcMotor encLeft;

    public Servo servo_flip;
    public Servo servo_hook;
    public Servo servo_hook_2;
    public Servo servo_plane;

    public Servo servo_hang;
    public TouchSensor touch;

    public RevColorSensorV3 color;

    public DcMotor led_flip;


    public void runOpMode() throws InterruptedException {

    }
    public void initHW(OpMode op) {

        motor_suction = op.hardwareMap.get(DcMotor.class, "motor_suction");
        motor_tele = op.hardwareMap.get(DcMotor.class, "motor_tele");
        servo_hook = op.hardwareMap.get(Servo.class, "servo_hook");
        servo_hook_2 = op.hardwareMap.get(Servo.class, "servo_hook_2");
        servo_plane = op.hardwareMap.get(Servo.class, "servo_plane");
        servo_hang = op.hardwareMap.get(Servo.class, "servo_hang");
        servo_flip = op.hardwareMap.get(Servo.class, "servo_flip");
        encLeft = op.hardwareMap.get(DcMotor.class, "encLeft");
        led_flip = op.hardwareMap.get(DcMotor.class, "led_flip");

        touch = op.hardwareMap.get(TouchSensor.class, "touch");
        color = op.hardwareMap.get(RevColorSensorV3.class, "color");
        color.initialize();

        motor_suction.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_tele.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_suction.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_tele.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_suction.setPower(0);
        motor_tele.setPower(0);
        servo_flip.setPosition(0);
        servo_hook.setPosition(0.3);
        servo_hook_2.setPosition(0.3);
        servo_plane.setPosition(0.46);
        servo_hang.setPosition(1);
        encLeft.setPower(0);
    }

    public void setSuction(double power) {
        motor_suction.setPower(power);
    }

    public void setTele(double power) {
        motor_tele.setPower(power);
    }

    public void setFlip(double angle) { servo_flip.setPosition(angle); }

    public void setHook(double angle) { servo_hook.setPosition(angle); }

    public void setHang(double angle) { servo_hang.setPosition(angle); }

    public void setHook_2(double angle) {
        servo_hook_2.setPosition(angle);
    }

    public void setPlane(double angle) {
        servo_plane.setPosition(angle);
    }

    public void encLeft(double power) { encLeft.setPower(power); }

    public void setLed_flip(double power) {
        led_flip.setPower(power);
    }

    public double getDistance(){ return color.getDistance(DistanceUnit.MM); }

}