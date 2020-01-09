package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="New TeleOp", group="BionicBot")
public class RevampedTeleOp extends LinearOpMode {


HardwareBionicbot robot = new HardwareBionicbot();

public void runOpMode() {
    robot.init(hardwareMap);
    waitForStart();
    while (opModeIsActive()) {

        //Movement
            //Drive Forward and Back
        if(gamepad1.right_trigger > 0)
        {
            robot.leftDrive.setPower(gamepad1.right_trigger);
            robot.rightDrive.setPower(gamepad1.right_trigger);
            robot.leftBack.setPower(gamepad1.right_trigger);
            robot.rightBack.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger > 0)
        {
            robot.leftDrive.setPower(-gamepad1.left_trigger);
            robot.rightDrive.setPower(-gamepad1.left_trigger);
            robot.leftBack.setPower(-gamepad1.left_trigger);
            robot.rightBack.setPower(-gamepad1.left_trigger);
        }
        else
        {
            StopDriving();
        }
            //Turn Right and Left
        if(gamepad1.left_stick_x > 0)
        {
            robot.leftDrive.setPower(gamepad1.left_stick_x);
            robot.rightDrive.setPower(-gamepad1.left_stick_x);
            robot.leftBack.setPower(gamepad1.left_stick_x);
            robot.rightBack.setPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.left_stick_x < 0)
        {
            robot.leftDrive.setPower(gamepad1.left_stick_x);
            robot.rightDrive.setPower(-gamepad1.left_stick_x);
            robot.leftBack.setPower(gamepad1.left_stick_x);
            robot.rightBack.setPower(-gamepad1.left_stick_x);
        }
        else
        {
            StopDriving();
        }
            //Strafing Left and Right
        if(gamepad1.dpad_left)
        {
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(.5);
            robot.leftBack.setPower(.5);
            robot.rightBack.setPower(-.5);
        }
        else if(gamepad1.dpad_right)
        {
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(-.5);
            robot.leftBack.setPower(-.5);
            robot.rightBack.setPower(.5);
        }
        else
        {
            StopDriving();
        }


    }

}
    public void StopDriving()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
}