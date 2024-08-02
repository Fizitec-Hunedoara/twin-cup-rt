package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class SubPrograme{
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DcMotorEx motorBL, motorBR, motorFL, motorFR;
    public DcMotorEx slider1,slider2,sugator;;
    public Servo intake, gheara_dreapta, gheara_stanga, rotitor, incheietura, brat_dreapta, brat_stanga;
    public TouchSensor taci_dreapta,taci_stanga;
    public ColorSensor color1, color2;

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean sasiuInited;
    public boolean ceva = false;
    public double timp = 0;

    private boolean isStopRequested = false;
    LinearOpMode opMode;
    public SubPrograme(LinearOpMode opMode){
        this.opMode = opMode;
    }
    public void init(HardwareMap hard){
        this.init(hard, null, false);
    }

    public void init(HardwareMap hard, Telemetry telemetry, boolean shouldInitSasiu) {
        this.hardwareMap = hard;
        this.telemetry = telemetry;

        if (shouldInitSasiu) {
            initSasiu(hard);
        }
        sasiuInited = shouldInitSasiu;

        taci_dreapta = (TouchSensor) hardwareMap.get("tacidreapta");
        taci_stanga = (TouchSensor) hardwareMap.get("tacistanga");
        intake = hardwareMap.servo.get("articulatie");
        gheara_dreapta = hardwareMap.servo.get("dreapta");
        gheara_stanga = hardwareMap.servo.get("stanga");
        rotitor = hardwareMap.servo.get("rotire");
        incheietura = hardwareMap.servo.get("incheietura");
        brat_dreapta = hardwareMap.servo.get("bratdreapta");
        brat_stanga = hardwareMap.servo.get("bratstanga");
        color1 = (ColorSensor)  hardwareMap.get("color1");
        color2 = (ColorSensor)  hardwareMap.get("color2");
        sugator = hardwareMap.get(DcMotorEx.class, "sugator");
        slider1 = hardwareMap.get(DcMotorEx.class, "cipri");
        slider2 = hardwareMap.get(DcMotorEx.class, "achim");


        slider1.setDirection(DcMotorEx.Direction.REVERSE);

        sugator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sugator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sugator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void stop(){
        this.isStopRequested = true;
    }

    public void initSasiu(HardwareMap hard) {
        motorBL = hard.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "motorFR"); // Motor Back-Left

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sasiuInited = true;
    }

    public synchronized void POWER(double df1, double ss1, double sf1, double ds1) {
        if (sasiuInited) {
            motorFR.setPower(df1);
            motorBL.setPower(ss1);
            motorFL.setPower(sf1);
            motorBR.setPower(ds1);
        }
        else {
            throw new NullPointerException("Bro sasiul nu e initializat");
        }
    }
    public synchronized void erectie(){
        kdf(300);
        ansamblul_leleseana(-200, 5000,15);
        kdf(50);
        sculare();
        kdf(100);
    }
    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (this.opMode.opModeIsActive() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                kdf(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                kdf(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            kdf(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            kdf(20);
        }
    }

    public void initTaguriAprilie() {
        initAprilTag();
        setManualExposure(6, 250);
    }

    public void detectieTaguriAprilie(int DESIRED_TAG_ID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                if (detection.id == DESIRED_TAG_ID) {
                    telemetry.addData("April tag detection corners:", detection.corners);
                    telemetry.update();
                    break;
                }
            }
            else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }

    public void miscare(double pow) {
        motorBL.setPower(pow);
        motorFL.setPower(pow);
        motorBR.setPower(pow);
        motorFR.setPower(pow);

    }








    public synchronized void ansamblul_leleseana(int poz1,int vel,double tolerance){

        if (poz1 > slider1.getCurrentPosition()){
            while (slider1.getCurrentPosition() < poz1 && this.opMode.opModeIsActive()){

                slider1.setVelocity(vel);
                slider2.setVelocity(vel);
            }

        }
        else {
            while (slider1.getCurrentPosition()>poz1 + tolerance && this.opMode.opModeIsActive()){
                slider1.setVelocity(-vel);
                slider2.setVelocity(-vel);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        slider1.setVelocity(0);
        slider2.setVelocity(0);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ceva = true;
    }

    public synchronized void kobra_kai(int poz1,int vel,double tolerance){

        if (poz1 > slider1.getCurrentPosition()){
            while (slider1.getCurrentPosition() < poz1 &&  this.opMode.opModeIsActive()){

                slider1.setVelocity(vel);
                slider2.setVelocity(vel);
            }

        }
        else {
            while (slider1.getCurrentPosition()>poz1 + tolerance && this.opMode.opModeIsActive()){
                slider1.setVelocity(-vel);
                slider2.setVelocity(-vel);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        slider1.setVelocity(0);
        slider2.setVelocity(0);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ceva = true;
    }
    public synchronized void kobra_kai_cu_ces(int poz1,int vel,double tolerance){

        if (poz1 > slider1.getCurrentPosition()){
            while (slider1.getCurrentPosition() < poz1 && this.opMode.opModeIsActive() ){
                slider1.setVelocity(vel);
                slider2.setVelocity(vel);
            }

        }
        else {
            while (slider1.getCurrentPosition()>poz1 + tolerance && this.opMode.opModeIsActive()){
                slider1.setVelocity(-vel);
                slider2.setVelocity(-vel);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        slider1.setVelocity(0);
        slider2.setVelocity(0);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ceva = true;
    }
    public synchronized void Burdu(int poz, double pow, DcMotorEx motor){
        while (motor.getCurrentPosition() < poz && this.opMode.opModeIsActive()){
            motor.setPower(pow);
        }
        motor.setPower(0);
//            while(sculantul.getCurrentPosition() <= poz){
//                pidResult = pid.performPID(sculantul.getCurrentPosition());
//                sculantul.setPower(pidResult);
//            }


    }



    public double getMotorBLPower() {
        return motorBL.getPower();
    }

    public double getMotorFLPower() {
        return motorFL.getPower();
    }

    public double getMotorBRPower() {
        return motorBR.getPower();
    }

    public double getMotorFRPower() {
        return motorFR.getPower();
    }

    public double getslider1Power() {
        return slider1.getPower();
    }

    public double getsugatorPower() {
        return sugator.getPower();
    }





    public boolean getTaciStanga() {
        return taci_stanga.isPressed();
    }

    public boolean getTaciDreapta() {
        return taci_dreapta.isPressed();
    }

    public DcMotorEx getslider1() {
        return slider1;
    }

    public void setIsStopRequested(boolean value){
        isStopRequested = value;
    }

    public synchronized void setSugatorPower(double pow) {
        sugator.setPower(pow);
    }

    public void sculare(){
        brat_dreapta.setPosition(0.39);
        brat_stanga.setPosition(0.39);
        incheietura.setPosition(0.854);
        rotitor.setPosition(0.305);
    }

    public void pleostire(){
        incheietura.setPosition(0.417);
        rotitor.setPosition(0.305);
        gheara_dreapta.setPosition(1);
        gheara_stanga.setPosition(1);
    }

    public void deschis(){
        gheara_dreapta.setPosition(1);
        gheara_stanga.setPosition(1);
    }
    public void inchis(){
        gheara_stanga.setPosition(0.47);
        gheara_dreapta.setPosition(0.28);
    }

    public synchronized void sugere1(){
        intake.setPosition(0.65);
        sugator.setPower(0.7);
    }
    public synchronized void sugere2(){
        intake.setPosition(0.54);
        sugator.setPower(0.7);
    }
    public synchronized void sugerepixel2(){
        intake.setPosition(0.62);
        sugator.setPower(0.7);
    }
    public synchronized void scuipare(){
        intake.setPosition(0.73);
        sugator.setPower(-0.7);
    }
    public void init_servo(){
        incheietura.setPosition(0.417);
        rotitor.setPosition(0.305);
        brat_stanga.setPosition(0.92);
        brat_dreapta.setPosition(0.92);
        gheara_stanga.setPosition(0.47);
        gheara_dreapta.setPosition(0.28);
        intake.setPosition(0.95);
    }





    public synchronized void agatare(){
        //scula.setPosition();
    }
    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if (motor.getCurrentPosition() < poz) {
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while ( this.opMode.opModeIsActive()
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
    }




    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() &&  this.opMode.opModeIsActive()){

        }
    }
}