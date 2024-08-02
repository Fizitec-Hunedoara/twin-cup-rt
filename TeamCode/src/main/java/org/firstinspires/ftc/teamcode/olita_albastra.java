package org.firstinspires.ftc.teamcode;



//import static org.firstinspires.ftc.teamcode.Var_BlueDa.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.*;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.ode.events.Action;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

@Autonomous
public class olita_albastra extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    public OpenCvCamera webcam;
    public boolean ceva,altceva=false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    public boolean faza1 = false,faza2 = false,faza0 = false,faza = false;
    public TrajectorySequence ts1, ts2, ts3, ts4, stanga, dreapta;
    boolean ok, ok2;
    @Override
    public void runOpMode() throws InterruptedException {

        p.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.783464, -58.5, Math.toRadians(270));
        drive.setPoseEstimate(startPose);



        CV_detectionType = DetectionTypes.DAY_red;


        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStopRequested() && !isStarted()) {
            try {
                p.init_servo();
                rectx = pipeline.getRect().width;
                recty = pipeline.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                x = pipeline.getRect().x + pipeline.getRect().width / 2.0;
                telemetry.addData("x:", pipeline.getRect().x + pipeline.getRect().width / 2);
                if (x > 470) {
                    varrez = "Stanga";
                } else if (x > 250 && x < 470) {
                    varrez = "Mijloc";
                } else if (x < 150) {
                    varrez = "Dreapta";
                } else {
                    varrez = "Dreapta";
                }
                telemetry.addData("caz:", varrez);
                if (Objects.equals(varrez, "Mijloc")) {
                    ts1 = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(12, -35, Math.toRadians(270)))
                            ///.addDisplacementMarker(5, this::erectie)
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectie();
                            })
                            .lineToSplineHeading(new Pose2d(47, -31.5, Math.toRadians(180)))
                            // .waitSeconds(0.3)
                            .addDisplacementMarker( this::disfunctieerectila)

                            .addDisplacementMarker(this::servo)
                            .build();
                }
                if (Objects.equals(varrez, "Stanga")) {
                    stanga = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(325) ))
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectie();
                            })
                            .lineToSplineHeading(new Pose2d(47, -29, Math.toRadians(180) ))
                            .addDisplacementMarker( this::disfunctieerectila)
                            .addDisplacementMarker(this::servo)
                            .build();

                }
                if (Objects.equals(varrez, "Dreapta")) {
                    dreapta = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(22, -39, Math.toRadians(270)))
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectie();
                            })
                            .lineToSplineHeading(new Pose2d(45, -39, Math.toRadians(180) ))
                            .addDisplacementMarker( this::disfunctieerectila)
                            .addDisplacementMarker(this::servo)
                            .build();

                }


            } catch (Exception E) {
                varrez = "Stanga";
                telemetry.addData("Webcam error:", "please restart");
                telemetry.update();
            }
            telemetry.update();
        }
        PiD.start();

//            drive.followTrajectorySequence(ts1);
//          //  drive.getPoseEstimate();
//            drive.followTrajectorySequence(ts2);
        if (Objects.equals(varrez, "Mijloc")) {
            drive.followTrajectorySequence(ts1);
        }
        if (Objects.equals(varrez, "Stanga")) {
            drive.followTrajectorySequence(dreapta);
        }
        if (Objects.equals(varrez, "Dreapta")) {
            drive.followTrajectorySequence(stanga);
        }

        ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .waitSeconds(0.3)
                .splineTo(new Vector2d(0, -55), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, -55, Math.toRadians(175)))
                .addDisplacementMarker( () -> {
                    p.sugere1();
                })
                .splineTo(new Vector2d(-64, -31), Math.toRadians(170))
                //.waitSeconds(0.5)
                .addDisplacementMarker(this::senzor)

                .build();
        drive.followTrajectorySequence(ts2);
        p.inchis();
        p.setSugatorPower(0);
        ts3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)

                .splineTo(new Vector2d(-17, -55), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(10, -55, Math.toRadians(185)))
                .addDisplacementMarker( this::erectie)
                .splineTo(new Vector2d(50, -39), Math.toRadians(0))
                .build();
        drive.followTrajectorySequence(ts3);
        ts4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .waitSeconds(0.2)
                .addDisplacementMarker( () -> {
                    deschis();
                    disfunctieerectila();
                    servo();
                } )
                .splineTo(new Vector2d(10, -62), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, -59, Math.toRadians(175)))
                .addDisplacementMarker( () -> {
                    p.sugere1();
                })
                .splineTo(new Vector2d(-64, -31), Math.toRadians(170))
                .waitSeconds(1)
                .addDisplacementMarker(this::senzor)
                .build();
        drive.followTrajectorySequence(ts4);
















    }

    private final Thread PiD = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!isStopRequested()){
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);
                if(altceva){
                    ceva = true;
                }
                else{
                    if(ceva){
                        ceva = false;
                        pid.setSetpoint(p.slider1.getCurrentPosition());
                    }
                    if (p.taci_dreapta.isPressed() || p.taci_stanga.isPressed()){
                        p.slider1.setPower(0);
                        p.slider2.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(p.slider1.getCurrentPosition());
                        p.slider1.setPower(pidResult);
                        p.slider2.setPower(pidResult);
                    }
                }
                telemetry.addData("color1", p.color1.blue());
                telemetry.addData("color2", p.color2.blue());
                telemetry.update();
            }
        }
    });

    public synchronized void erectie(){
        p.kdf(300);
        altceva = true;
        p.ansamblul_leleseana(-200, 5000,15);
        p.kdf(50);
        p.sculare();
        p.kdf(50);
        altceva = false;
        p.kdf(50);

    }
    public synchronized void deschis(){
        p.deschis();
    }
    public synchronized void disfunctieerectila(){

        //  p.deschis();
        altceva = true;
        p.slider1.setVelocity(5000);
        p.slider2.setVelocity(5000);
        while (p.taci_dreapta.isPressed() || !p.taci_stanga.isPressed()) {
        }
        p.slider1.setVelocity(0);
        p.slider2.setVelocity(0);
        altceva = false;
        ceva=true;

    }

    public synchronized void servo(){
        ///p.intake.setPosition();
        p.kdf(200);
        p.deschis();
        p.kdf(200);
        p.pleostire();
        p.kdf(150);
        p.kdf(150);
        p.brat_stanga.setPosition(0.92);
        p.brat_dreapta.setPosition(0.92);

    }

    public synchronized void senzor(){

        if (p.color1.blue() > 1000) {
            ok = true;
        } else ok = false;
        if (ok){
            p.gheara_dreapta.setPosition(0.28);
        }

        if (p.color2.blue() > 1000) {
            ok2 = true;
        } else ok2 = false;
        if (ok2){
            p.gheara_stanga.setPosition(0.47);
        }

        if (ok && ok2) {
            p.setSugatorPower(0);
            p.kdf(100);
            p.setSugatorPower(-0.7);
        }

    }






}

