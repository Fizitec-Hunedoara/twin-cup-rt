package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp

public class luca_pid extends OpMode {
    public DcMotorEx slider1, slider2;
    public boolean stop;
    public boolean ceva;
    public double pidResult;

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);


    @Override
    public void init() {
        slider1 = hardwareMap.get(DcMotorEx.class, "motorD");
        slider2 = hardwareMap.get(DcMotorEx.class, "motorS");

        slider1.setDirection(DcMotorEx.Direction.REVERSE);
        slider1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void start(){
        System.start();

    }
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while (!stop) {
                pid.setPID(Config.pretard, Config.iretard, Config.dretard);

                if (gamepad2.left_stick_y != 0.0) {
                    slider1.setPower(gamepad2.left_stick_y);
                    slider2.setPower(gamepad2.left_stick_y);
                    ceva = true;
                } else {
                    if (ceva) {
                        ceva = false;
                        pid.setSetpoint(slider1.getCurrentPosition());
                    } else {
                        pidResult = pid.performPID(slider1.getCurrentPosition());
                        slider1.setPower(pidResult);
                        slider2.setPower(pidResult);
                    }
                }


            }

        }
    });
    public void stop(){stop = true;}
    @Override
    public void loop(){}

}
