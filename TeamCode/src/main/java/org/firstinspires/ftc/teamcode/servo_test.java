package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo_test extends OpMode {
    Servo gheara_dreapta, gheara_stanga;
    Servo stanga, dreapta, intake, incheietura, rotitor, brat_stanga, brat_dreapta;
    DcMotorEx sugator;
    double poz = 0.366, poz2 = 0.5;

    @Override
    public void init() {
        gheara_dreapta = hardwareMap.servo.get("dreapta");
        gheara_stanga = hardwareMap.servo.get("stanga");
        stanga = hardwareMap.servo.get("bratstanga");
        dreapta = hardwareMap.servo.get("bratdreapta");
        intake = hardwareMap.servo.get("articulatie");
        // servo2 = hardwareMap.servo.get("relu");
        incheietura = hardwareMap.servo.get("incheietura");
        rotitor = hardwareMap.servo.get("rotire");
        brat_dreapta = hardwareMap.servo.get("bratdreapta");
        brat_stanga = hardwareMap.servo.get("bratstanga");


        sugator = hardwareMap.get(DcMotorEx.class, "sugator");

        sugator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void loop() {
//        intake.setPosition(poz);
//        if (gamepad2.dpad_left){
//            poz+=0.001;
//        }
//        if (gamepad2.dpad_right){
//            poz-=0.001;
//        }
//        if (gamepad2.dpad_up) {
//            sugator.setPower(-0.7);
//        }
//        if (gamepad2.dpad_down){
//            sugator.setPower(0.7);
//        }
//        if (gamepad2.dpad_left){
//            poz-=0.001;
//        }
//        if (gamepad2.dpad_right){
//            poz+=0.001;
//        }
//        incheietura.setPosition(poz);

//        if (gamepad2.dpad_left){
//            poz-=0.001;
//        }
//        if (gamepad2.dpad_right){
//            poz+=0.001;
//        }
//        gheara_dreapta.setPosition(poz);

        if (gamepad2.dpad_up){
            poz-=0.001;
        }
        if (gamepad2.dpad_down){
            poz+=0.001;
        }
        intake.setPosition(poz);


//        brat_dreapta.setPosition(0.92);
//        brat_stanga.setPosition(0.92);




//        if (gamepad2.x) {
//            stanga.setPosition(0.432);
//        }
//        if (gamepad2.b){
//            dreapta.setPosition(0.432);
//        }
//        if (gamepad1.a){
//            servo_fudul.setPosition(0.5);
//        }
//        if (gamepad1.b){
//            servo_fudul.setPosition(1);
//        }
//        if (gamepad1.right_trigger > 0 && poz < 1.0){
//            while(poz < 0.972){
//                poz+= 0.03;
//                try {
//                    Thread.sleep(45);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//                servo_fudul.setPosition(poz);
//            };
//
//        }
//        if (gamepad1.left_trigger > 0 && poz > 0){
//            poz -= 0.0001;
//            servo_fudul.setPosition(poz);
//
//        }
//        if (gamepad1.dpad_right && poz2 < 1.0){
//            poz2 += 0.0001;
//            servo2.setPosition(poz2);
//
//        }
//        if (gamepad1.dpad_left  && poz2 > 0){
//            poz2 -= 0.0001;
//            servo2.setPosition(poz2);
//
//        }
//
//        if (gamepad1.a){
//            servo_fudul.setPosition(0.52);//deschis
//        }
//        if (gamepad1.b){
//            servo_fudul.setPosition(0.605);//inchis
//        }
////
////        if (gamepad1.x){
////            servo_fudul.setPosition(0.603);//1 pixel
////        }
//        if (gamepad1.a) {
//            poz -= 0.01;
//        }
//        if (gamepad1.y) {
//            poz += 0.01;
//        }
//        servo_fudul.setPosition(poz);
//
//        if (gamepad1.dpad_right) {
//            poz2 -= 0.01;
//        }
//        if (gamepad1.dpad_left) {
//            poz2 += 0.01;
//        }
//        servo2.setPosition(poz2);

//        if (gamepad1.dpad_up){
//            poz+=0.001;
//        }
//        if (gamepad1.dpad_down){
//            poz-=0.001;
//        }
//        gheara_dreapta.setPosition(poz);
//
//        if (gamepad1.y){
//            poz2+=0.001;
//        }
//        if (gamepad1.a){
//            poz2-=0.001;
//        }
//        gheara_stanga.setPosition(poz2);

       // telemetry.addData("stanga", poz2);
        telemetry.addData("brat ", poz); //0.33  //0.668, 0.633
        telemetry.addData("rotire", poz2); //0.54  //
        telemetry.update();
    }
}
