package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class telop_odom extends OpMode {

    boolean stop = false;
    public double sm = 1;


    public void init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        drive.motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        drive.motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        drive.motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void start(){
        Chassis.start();

    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();



            }
        }
    });
    public void stop(){stop = true;}

    @Override
    public void loop() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        telemetry.update();
    }


}
