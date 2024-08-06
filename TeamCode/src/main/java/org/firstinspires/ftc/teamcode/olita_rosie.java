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
public class olita_rosie extends LinearOpMode {
    double rectx, recty, hperw, x, pidResult;
    String varrez;
    public OpenCvCamera webcam;
    public boolean ceva,altceva=false;
    public PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public SubPrograme p = new SubPrograme(this);
    public boolean faza1 = false,faza2 = false,faza0 = false,faza = false;
    public TrajectorySequence ts1, ts2, ts3, ts4, stanga, dreapta, ts5;
    boolean ok, ok2, bine = false, isCollecting = false;
    private boolean can_lift_intake = false, started_right = false, started_left = false;
    private long time_final_left = 0, time_final_right = 0, time_final = 0;
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
                    varrez = "Dreapta";
                } else if (x > 250 && x < 470) {
                    varrez = "Mijloc";
                } else if (x < 150) {
                    varrez = "Stanga";
                } else {
                    varrez = "Dreapta";
                }
                telemetry.addData("caz:", varrez);
                if (Objects.equals(varrez, "Mijloc")) {
                    ts1 = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(8, -32, Math.toRadians(270)))
                            ///.addDisplacementMarker(5, this::erectie)
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectienoua();
                            })
                            .lineToLinearHeading(new Pose2d(44.2, -34.7, Math.toRadians(180)))
                            .waitSeconds(0.3)
                            .addDisplacementMarker( this::disfunctieerectila)

                            .addDisplacementMarker(this::servo)
                            .build();
                }
                if (Objects.equals(varrez, "Stanga")) {
                    stanga = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(6.3, -33.5, Math.toRadians(325) ))
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectienoua();
                            })
                            .lineToLinearHeading(new Pose2d(43.8, -28.4, Math.toRadians(180) ))
                            .waitSeconds(0.2)
                            .addDisplacementMarker( this::disfunctieerectila)
                            .addDisplacementMarker(this::servo)
                            .build();

                }
                if (Objects.equals(varrez, "Dreapta")) {
                    dreapta = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(22, -39, Math.toRadians(270)))
                            .addDisplacementMarker( () -> {
                                p.intake.setPosition(0.53);
                                erectienoua();
                            })
                            .lineToLinearHeading(new Pose2d(43.7, -39.7, Math.toRadians(180) ))
                            .waitSeconds(0.2)
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
        Automatizare.start();
//            drive.followTrajectorySequence(ts1);
//          //  drive.getPoseEstimate();
//            drive.followTrajectorySequence(ts2);
        if (Objects.equals(varrez, "Mijloc")) {
            drive.followTrajectorySequence(ts1);
        }
        if (Objects.equals(varrez, "Dreapta")) {
            drive.followTrajectorySequence(dreapta);
        }
        if (Objects.equals(varrez, "Stanga")) {
            drive.followTrajectorySequence(stanga);
        }

        ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .waitSeconds(0.2)
                .splineTo(new Vector2d(0, -52.9), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-20, -52.9, Math.toRadians(175)))
                .addDisplacementMarker(() -> new Thread(() -> {
                   p.sugere1();
                   p.kdf(1500);
                   p.sugerepixel2();
                   isCollecting = true;
                   p.kdf(1500);
                   p.scuipare();
                }).start())
                .splineTo(new Vector2d(-64.5, -32), Math.toRadians(168))
                //.waitSeconds(0.5)
                .build();
        if(!isStopRequested()) {
            drive.followTrajectorySequence(ts2);
        }
//        if (bine) {

            ts3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    /*.addDisplacementMarker(() ->{
                        p.intake.setPosition(0.668);
                    })
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() ->{
                        p.intake.setPosition(0.6);

                    })
                    .waitSeconds(0.5)*/

                    .splineTo(new Vector2d(-25, -54), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(10, -54, Math.toRadians(185)))
                    .addDisplacementMarker(() -> {
                        erectie();
                        isCollecting = false;
                        p.sugator.setPower(0);
                    })
                    .splineTo(new Vector2d(49.9, -36), Math.toRadians(0))
                    .build();
            if(!isStopRequested()) {
                drive.followTrajectorySequence(ts3);
            }
            ts4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(false)
                    .waitSeconds(0.15)
                    .addDisplacementMarker(() -> {
                        p.deschis();
                        disfunctieerectila();
                        servo();
                    })
                    .splineTo(new Vector2d(10, -53.4), Math.toRadians(180))
                    .lineToSplineHeading(new Pose2d(-20, -53.6, Math.toRadians(175)))
                    .addDisplacementMarker(() -> {
                        p.sugere2();
                        isCollecting = true;
                        ok = false;
                        ok2 = false;
                        p.kdf(300);
                    })
                    .splineTo(new Vector2d(-62.95, -30.2), Math.toRadians(170))
                    .waitSeconds(0.28)
                    .addDisplacementMarker(this::senzor)
                    .build();
            if(!isStopRequested()) {
                drive.followTrajectorySequence(ts4);
            }
            ts5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .addDisplacementMarker(()->p.scuipare())
                    .splineTo(new Vector2d(-25, -54), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(11, -54, Math.toRadians(185)))
                    .addDisplacementMarker(() -> {
                        erectiets5();
                        isCollecting = false;
                        p.sugator.setPower(0);
                    })
                    .splineTo(new Vector2d(48.7, -36.9), Math.toRadians(0))
                    .build();
            if(!isStopRequested()) {
                drive.followTrajectorySequence(ts5);
                p.kdf(300);
                disfunctieerectila();
                servo();
                p.kdf(20000);
            }
        //}
















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
                telemetry.addData("bine", bine);
                telemetry.update();
            }
        }
    });
    private final Thread Automatizare = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!isStopRequested()) {
                if (p.color2.blue() > 900 || p.color2.red() > 900) {
                    ok2 = true;
                } else ok2 = false;
                if (p.color1.blue() > 900 || p.color1.red() > 900) {
                    ok = true;
                } else ok = false;
                if(ok && p.sugator.getPower() != 0 && !started_left && isCollecting){
                    started_left = true;
                    telemetry.addData("in loop","ok");
                    telemetry.update();
                    long lastTime = System.currentTimeMillis();
                    while(lastTime + 500 > System.currentTimeMillis()){
                        telemetry.addData("time:",System.currentTimeMillis());
                        telemetry.update();
                    }
                    p.gheara_dreapta.setPosition(0.33);
                    time_final_left = System.currentTimeMillis();
                    started_left = false;
                }
                if(ok2 && p.sugator.getPower() != 0 && !started_right && isCollecting) {
                    started_right = true;
                    telemetry.addData("in loop", "ok");
                    telemetry.update();
                    long lastTime = System.currentTimeMillis();
                    while (lastTime + 500 > System.currentTimeMillis()) {
                        telemetry.addData("time:", System.currentTimeMillis());
                        telemetry.update();
                    }
                    p.gheara_stanga.setPosition(0.54);
                    time_final_right = System.currentTimeMillis();
                    started_right = false;
                }
                if(time_final_right != 0 && time_final_left != 0 && !can_lift_intake && isCollecting){
                    can_lift_intake = true;
                    time_final = Math.max(time_final_left,time_final_right);
                }
                if(can_lift_intake && isCollecting){
                    can_lift_intake = false;
                    time_final_left = 0;
                    time_final_right = 0;
                    long lastTime = System.currentTimeMillis();
                    while(lastTime + 750 > System.currentTimeMillis());
                    p.sugator.setPower(0);
                    p.intake.setPosition(0.73);
                }
            }
        }
    });
    public synchronized void erectie(){
        p.kdf(300);
        altceva = true;
        p.ansamblul_leleseana(-200, 5000,15);
        altceva = false;
        p.kdf(50);
        p.sculare();
        p.kdf(50);
        p.kdf(50);

    }
    public synchronized void erectiets5(){
        p.kdf(300);
        altceva = true;
        p.ansamblul_leleseana(-255, 5000,15);
        altceva = false;
        p.kdf(50);
        p.sculare();
        p.kdf(50);
        p.kdf(50);

    }
    public synchronized void erectienoua(){
        p.kdf(300);
        altceva = true;
        p.ansamblul_leleseana(-150, 5000,15);
        p.kdf(70);
        altceva = false;
        p.kdf(100);
        p.sculare();
        p.kdf(50);


    }
    public synchronized void disfunctieerectila( ){
        p.deschis();
        p.kdf(200);
        altceva = true;
        p.slider1.setVelocity(5000) ;
        p.slider2.setVelocity(5000) ;
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
        p.kdf(300);
        p.brat_stanga.setPosition(0.92);
        p.brat_dreapta.setPosition(0.92);
    }

    public synchronized void senzor(){

        if (p.color1.blue() > 900          ) {
            ok = true;
        } else ok = false;
        if (ok){
            p.gheara_dreapta.setPosition(0.28);
        }

        if (p.color2.blue() > 900) {
            ok2 = true;
        } else ok2 = false;
        if (ok2){
            p.gheara_stanga.setPosition(0.47);
        }
        do {
            p.setSugatorPower(0.7);
        } while (!ok && !ok2 && !bine);

        if (ok && ok2) {
            p.setSugatorPower(0);
            p.kdf(200);
            p.setSugatorPower(-0.7);
            p.kdf(100);
            bine = true;
        }



    }








}


