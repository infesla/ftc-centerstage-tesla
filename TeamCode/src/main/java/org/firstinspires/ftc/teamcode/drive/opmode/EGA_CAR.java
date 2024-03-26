package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.methods.PixelsControl;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp(group = "drive")
public class EGA_CAR extends LinearOpMode {
    public PixelsControl PixelsControl = new PixelsControl();
    public ElapsedTime runtime = new ElapsedTime();

    private double flip_last_moment_switch = 0.0, flip_moment_diff_switch = 0.0, color_moment_diff = 0.0;
    private double hook_last_moment_switch = 0.0, hook_moment_diff_switch = 0.0, color_moment_last = 0.0;
    private double a;

    private boolean is_servo_flip_opened = false;
    private boolean is_servo_hook_opened = false;
    private boolean is_servo_plane_opened = false;
    private boolean is_servo_hang_opened = false;

    /*
    class FlipMotorThread extends Thread {
        @Override
        public void run() {
            if (is_motor_flip_opened) {

                while (PixelsControl.motor_flip.getCurrentPosition() > 5) {
                    PixelsControl.motor_flip.setPower(-0.5);
                    telemetry.addData("num", 1);
                    telemetry.addData("Motor_flip", PixelsControl.motor_flip.getCurrentPosition());
                    telemetry.update();
                }

                PixelsControl.motor_flip.setPower(-0.002);
                is_motor_flip_opened = false;

            } else {

                runtime_flip.reset();

                while ((runtime_flip.milliseconds() - 700) < 0) {
                    PixelsControl.motor_flip.setPower(0.3);
                    telemetry.addData("num", 2);
                    telemetry.addData("Motor_flip", PixelsControl.motor_flip.getCurrentPosition());
                    telemetry.update();
                }

                PixelsControl.motor_flip.setPower(0);
                is_motor_flip_opened = true;

            }
        }
    }
    */

    @Override
    public void runOpMode() throws InterruptedException {

        PixelsControl.initHW(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //FlipMotorThread flipMotorThread = new FlipMotorThread();

        waitForStart();

        while (!isStopRequested()) {

            a = gamepad1.left_trigger > 0.15 ? 1 : 0.65;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * a,
                            gamepad1.left_stick_x * a,
                            gamepad1.right_stick_x * a
                    )
            );

            drive.update();

            //Hook
            //if (gamepad2.y == true && hook_moment_diff_switch > 200) {
            //if (is_servo_hook_opened == false) {
            // PixelsControl.setHook(0.2);
            // is_servo_hook_opened = true;
            // } else {
            //  PixelsControl.setHook(0.795);
            //    is_servo_hook_opened = false;
            // }
            // hook_last_moment_switch = runtime.milliseconds();
            //  }

            //First gamepad

            //Hang
            if (gamepad1.right_trigger > 0.1) {
                PixelsControl.setHang(0.5);
            } else {
                PixelsControl.setHang(1);
            }

            //Airplane
            if (gamepad1.back) {
                if (is_servo_plane_opened == false) {
                    PixelsControl.setPlane(0.19);
                    is_servo_plane_opened = true;
                }
            }

            //Pull up
            if (gamepad1.a) {
                PixelsControl.encLeft(1);
            } else if (gamepad1.b) {
                PixelsControl.encLeft(-1);
            } else {
                PixelsControl.encLeft(0);
            }

            //Second gamepad

            //Suction
            if (gamepad2.right_bumper) {
                PixelsControl.setSuction(1.0);
            } else if (gamepad2.left_bumper && PixelsControl.touch.isPressed()) {
                PixelsControl.setSuction(-1.0);
            } else {
                PixelsControl.setSuction(0);
            }

            //Tele
            if (gamepad2.left_stick_y > 0.15 && !PixelsControl.touch.isPressed()) {
                PixelsControl.setTele(-gamepad2.left_stick_y);
            } else if (gamepad2.left_stick_y < -0.15) {
                PixelsControl.setTele(-gamepad2.left_stick_y);
            } else {
                PixelsControl.setTele(0);
            }

            //Hook
            if (gamepad2.right_trigger > 0.1) {
                PixelsControl.setHook(0.3);
            } else {
                PixelsControl.setHook(0.795);
            }


            //Hook 2
            hook_moment_diff_switch = runtime.milliseconds() - hook_last_moment_switch;

            if (gamepad2.x && hook_moment_diff_switch > 1500) {
                if (is_servo_hook_opened) {
                    PixelsControl.setHook_2(0.3);
                    is_servo_hook_opened = false;
                } else {
                    PixelsControl.setHook_2(0);
                    is_servo_hook_opened = true;
                }
                hook_last_moment_switch = runtime.milliseconds();
            }

            //Flip
            flip_moment_diff_switch = runtime.milliseconds() - flip_last_moment_switch;

            if (gamepad2.y && flip_moment_diff_switch > 500) {
                if (!is_servo_flip_opened) {
                    PixelsControl.setFlip(0.03);
                    is_servo_flip_opened = true;
                } else {
                    PixelsControl.setFlip(1);
                    is_servo_flip_opened = false;
                }
                flip_last_moment_switch = runtime.milliseconds();
            }


            //Color
            color_moment_diff = runtime.milliseconds() - color_moment_last;

            if (color_moment_diff > 2000) {

                if (PixelsControl.getDistance() < 70) {
                    PixelsControl.setHook_2(0);
                    is_servo_hook_opened = true;
                } else {
                    PixelsControl.setHook_2(0.3);
                    is_servo_hook_opened = false;
                }

                color_moment_last = runtime.milliseconds();

            }

            //LED
            PixelsControl.setLed_flip(is_servo_hook_opened ? 0.5 : 0);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("color", PixelsControl.getDistance());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}