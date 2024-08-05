/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*@TeleOp face ca programul sa apara in configuratia driver hub-ului/telefonului cu aplicatia de driver station, in partea de TeleOp. */
@TeleOp
/*Linia asta de cod incepe cu numele programului(FoundationTeleOp) si la sfarsitul liniei este tipul de program:
    OpMode = TeleOp
    LinearOpMode = Autonom
  Linia de cod va da eroare dupa ce o scrii, doar apasa pe cod, apasa pe beculetul rosu si apoi apasa pe implement methods, asta va importa functiile de init si loop.
  Functia de init se declanseaza numai o data, dar cea de loop se repeta incontinuu si este locul unde se pun functiile care misca robotul in general, sau face telemetrie in cazul asta.
 */
public class MikiOAreMica extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorBR,motorBL,motorFL,motorFR, sugator;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    public DcMotorEx slider1, slider2;
    public TouchSensor taci_dreapta;
    public TouchSensor taci_stanga;
    public Servo intake, gheara_stanga, gheara_dreapta, rotitor, incheietura, brat_dreapta, brat_stanga;
    public ColorSensor color1, color2;
   // public DistanceSensor distanta1, distanta2;
    boolean stop = false;
    public long cacat;
    long time_final_left = 0, time_final_right = 0, time_final = 0;

    public boolean ceva = false, started_left = false, started_right = false, can_lift_intake = false;
    public double corection, error, pidResult;
    boolean encodersResetting = false;

    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    ElapsedTime supt = new ElapsedTime();
    ElapsedTime supt2 = new ElapsedTime();
    boolean roni = false, mure = false, andrei= false, jupanu = false, maia = false;
    boolean ok, ok2;


    public double poz_artic = 0.5; //0.15

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);


    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        cacat =   System.currentTimeMillis();
       // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

       // drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        slider1 = hardwareMap.get(DcMotorEx.class, "cipri");
        slider2 = hardwareMap.get(DcMotorEx.class, "achim");
        sugator = hardwareMap.get(DcMotorEx.class, "sugator");

        taci_dreapta = (TouchSensor) hardwareMap.get("tacidreapta");
        taci_stanga = (TouchSensor) hardwareMap.get("tacistanga");
        intake = hardwareMap.servo.get("articulatie");
        gheara_dreapta = hardwareMap.servo.get("dreapta");
        gheara_stanga = hardwareMap.servo.get("stanga");
        rotitor = hardwareMap.servo.get("rotire");
        incheietura = hardwareMap.servo.get("incheietura");
        brat_dreapta = hardwareMap.servo.get("bratdreapta");
        brat_stanga = hardwareMap.servo.get("bratstanga");
        //distanta1 = (DistanceSensor)  hardwareMap.get("distanta1");





        color1 = (ColorSensor)  hardwareMap.get("color1");
        color2 = (ColorSensor)  hardwareMap.get("color2");



// Motor Back-Left

        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt folosite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        slider1.setDirection(DcMotorEx.Direction.REVERSE);


        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sugator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        /*Liniile astea de cod fac ca encoderele(masoara cat a mers motorul, dar nu este foarte precis, este necesar un cablu ca sa accesezi encoder-ul) sa se opreaca si sa se reseteze la valoarea initiala*/
//        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sugator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        /*Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sugator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        gheara_stanga.setPosition(1);
        gheara_dreapta.setPosition(1);
//        brat_stanga.setPosition(1);
//        brat_dreapta.setPosition(1);
        incheietura.setPosition(0.417);
        rotitor.setPosition(0.305);
        brat_stanga.setPosition(0.92);
        brat_dreapta.setPosition(0.92);




    }
    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start(){
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        Systems.start();

    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
//                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x ;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = (y + x + rx)/1.17;
                pmotorBL = (y - x + rx)/1.17;
                pmotorBR = (y + x - rx)/1.17;
                pmotorFR = (y - x - rx)/1.17;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
                //SLOW-MOTION
                if (gamepad1.left_trigger>0) {
                    sm = 1;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_trigger>0) {
                        sm = 2;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    } else {
                        sm = 1.27;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    }
                }
//                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x,
//                                -gamepad1.right_stick_x
//                        )
//                );
//
//                drive.update();


            }
        }
    });

    private final Thread Systems = new Thread(new Runnable() {

        @Override
        public void run(){
            pid.enable();

            while (!stop){
                pid.setPID(Config.pstatic, Config.istatic, Config.dstatic);

                if (gamepad2.left_stick_y != 0.0) {
                    slider1.setPower(gamepad2.left_stick_y);
                    slider2.setPower(gamepad2.left_stick_y);
                    ceva = true;
                } else {
                    if (ceva) {
                        ceva = false;
                        pid.setSetpoint(slider1.getCurrentPosition());
                    }
                    if(taci_dreapta.isPressed() || taci_stanga.isPressed()){
                        slider1.setPower(0);
                        slider2.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(slider1.getCurrentPosition());
                        slider1.setPower(pidResult);
                        slider2.setPower(pidResult);
                    }
                }
                if (gamepad2.b) {
                    rotitor.setPosition(0.305);
                    incheietura.setPosition(0.417);
                    timer2.reset();
                    mure = true;

                    timer3.reset();
                    andrei = true;

                }
                if (timer3.milliseconds() >= 150 && andrei){
                    andrei = false;
                    slider1.setVelocity(5000);
                    slider2.setVelocity(5000);
                    while (!taci_stanga.isPressed() || taci_dreapta.isPressed()) {

                    }
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
                    brat_stanga.setPosition(0.92);
                    brat_dreapta.setPosition(0.92);

                    ceva = true;


                }

                if (timer2.milliseconds() >=150 && mure){
                    mure = false;
                    brat_stanga.setPosition(0.92);
                    brat_dreapta.setPosition(0.92);                }

                if (gamepad2.x){
                    ansamblul_leleseana(-200, 5000, 15);
                    sugator.setPower(0);
                    brat_dreapta.setPosition(0.39);
                    brat_stanga.setPosition(0.39);
                    incheietura.setPosition(0.854);


                }

                if (gamepad2.dpad_down){
                    sugator.setPower(0.8);
                    intake.setPosition(0.53);
                }
                if (gamepad2.dpad_up) {
                    sugator.setPower(-0.7);
                    intake.setPosition(0.73);
                }

//                if (gamepad2.y){
//                    sugator.setPower(-0.7);
//                }

                if (gamepad2.right_bumper){
                   rotitor.setPosition(0.149);
                }
                if (gamepad2.left_bumper){
                  rotitor.setPosition(0.485);
                }
                if (gamepad2.touchpad){
                    rotitor.setPosition(0.305);
                }
                if (gamepad2.dpad_left) {
                    sugator.setPower(0);
                }





                if (gamepad2.a){
                    gheara_stanga.setPosition(0.47);
                    gheara_dreapta.setPosition(0.28);
                    sugator.setPower(0);
                    intake.setPosition(0.53);


                }
                if (gamepad2.y){
                    gheara_dreapta.setPosition(1);
                    gheara_stanga.setPosition(1);
                    time_final_left = 0;
                    time_final_right = 0;
                }
//                if (gamepad2.left_trigger>0){
//                    poz_artic+=0.001;
//                    incheietura.setPosition(poz_artic);
//                }
//                if (gamepad2.right_trigger>0){
//                    poz_artic-=0.001;
//                    incheietura.setPosition(poz_artic);
//                }
                if (gamepad2.left_trigger >0) {
                    incheietura.setPosition(0.98);
                }


//
//
//                if (gamepad2.b){
//                    poz_artic+=0.001;
//
//                }
//                if (gamepad2.x){
//                    poz_artic-=0.001;
//
//                }
               // gheara_dreapta.setPosition(poz_artic);

//                if (gamepad2.y){
//                    brat_dreapta.setPosition(0.5);
//                    brat_stanga.setPosition(0.5);
//                    incheietura.setPosition(0.987);
//
//                }

                if (color1.blue() > 900 || color1.red() > 900 || color1.green() > 900) {
                    ok = true;
                } else ok = false;

                if (ok){
                    supt.reset();

                    //gheara_dreapta.setPosition(0.28);

                }
                if (supt.milliseconds() > 2000 && roni) {
                    gheara_dreapta.setPosition(0.28);

                }

                if (color2.blue() > 900 || color2.red() > 900 || color2.green() > 900) {
                    ok2 = true;
                } else ok2 = false;

                if (ok2){
                    //gheara_stanga.setPosition(0.47);
                    supt2.reset();


                }
                if (supt2.milliseconds() > 2000 && jupanu){
                    gheara_stanga.setPosition(0.47);

                }

//                long cacat = System.currentTimeMillis();
//                while (lastTime + t > System.currentTimeMillis() )
//                if (ok && ok2 && sugator.getPower() != 0){
//                    sugator.setPower(0);
//                    intake.setPosition(0.73);
//                }
                if(ok && sugator.getPower() != 0 && !started_left){
                    started_left = true;
                    long lastTime = System.currentTimeMillis();
                    while(lastTime + 500 > System.currentTimeMillis()){
                    }
                    gheara_dreapta.setPosition(0.33);
                    time_final_left = System.currentTimeMillis();
                    started_left = false;
                }
                if(ok2 && sugator.getPower() != 0 && !started_right) {
                    started_right = true;
                    long lastTime = System.currentTimeMillis();
                    while (lastTime + 500 > System.currentTimeMillis()) {
                    }
                    gheara_stanga.setPosition(0.54);
                    time_final_right = System.currentTimeMillis();
                    started_right = false;
                }
                if(time_final_right != 0 && time_final_left != 0 && !can_lift_intake){
                    can_lift_intake = true;
                    time_final = Math.max(time_final_left,time_final_right);
                }
                if(can_lift_intake){
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                    can_lift_intake = false;
                    time_final_left = 0;
                    time_final_right = 0;
                    long lastTime = System.currentTimeMillis();
                    while(lastTime + 750 > System.currentTimeMillis());
                    sugator.setPower(0);
                    intake.setPosition(0.73);
                }
            }
        }
    });



    public  void ansamblul_leleseana(int poz1,int vel,double tolerance){

        if (poz1 > slider1.getCurrentPosition()){
            while (slider1.getCurrentPosition() < poz1){
                slider1.setVelocity(vel);
                slider2.setVelocity(vel);
            }

        }
        else {
            while (slider1.getCurrentPosition()>poz1 + tolerance ){
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



    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/

    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//        telemetry.addData("error", pid.getError());
//        telemetry.addData("p", pid.getP());
//        telemetry.addData("d", pid.getD());
//        telemetry.addData("i", pid.getI());
        telemetry.addData("taci stanga", taci_stanga.isPressed());
        telemetry.addData("taci dreapta", taci_dreapta.isPressed());
        telemetry.addData("poz servo", poz_artic);
       // telemetry.addData("distanta 1", distanta1.getDistance(DistanceUnit.MM));
      //  telemetry.addData("distanta 2", distanta2.getDistance(DistanceUnit.MM));

        telemetry.addData("albastru2", color1.blue());
        telemetry.addData("albastru2", color2.blue());
        telemetry.addData("ok", ok);
        telemetry.addData("ok2", ok2);
        telemetry.addData("rosu1", color1.red());
        telemetry.addData("rosu2", color2.red());
        telemetry.addData("verde1", color1.green());
        telemetry.addData("verde2", color2.green());
        telemetry.addData("alfa1", color1.alpha());
        telemetry.addData("alfa2", color2.alpha());
        //telemetry.addData("brat", brat_dreapta.getPosition());


        /*Aceasta functie face ca telemetria sa trimita date cat timp ruleaza programul*/
        telemetry.update();
    }


    /*Functia asta face ca toate motoarele a ruleze cu o anumita putere;
    Functiile sunt linii de cod comprimate in una singura, ceea ce este foarte fain daca vrei sa faci o secventa de linii de cod de mai multe ori. De asemenea, cand apelezi o functie, trebuie sa scrii si parametrii ei, daca exista.*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}