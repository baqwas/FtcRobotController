/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
