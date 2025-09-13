package org.firstinspires.ftc.teamcode.Holonomic;

import com.qualcomm.robotcore.hardware.DcMotorEx;
public class MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double forward, double strafe, double turn) {
        frontLeft.setPower(forward + strafe + turn);
        frontRight.setPower(forward - strafe - turn);
        backLeft.setPower(forward - strafe + turn);
        backRight.setPower(forward + strafe - turn);
    }

    public void stop() {
        setPower(0, 0, 0);
    }

    public void setRunMode(DcMotorEx.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }
}
