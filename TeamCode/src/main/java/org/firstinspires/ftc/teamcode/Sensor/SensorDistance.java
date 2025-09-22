/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * {@link SensorDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * Measurement Range: 5cm - 200cm
 * Measurement Resolution: 1mm
 * Field of View: 25°
 * Laser Type: 940nm (IR) Class 1
 * Sensor Type: I2C
 * I2C Address: 0x52
 * Voltage Range: 3.3V - 5.0V
 * Max. Operating Current: 40mA
 * @see <a href="https://docs.revrobotics.com/2m-distance-sensor/specifications">REV Robotics Web Page</a>
 * @see <a href="https://www.st.com/en/embedded-software/stsw-img005.html">V53LOX API</a>
 * Black - GND
 * Red   - VDC, 3.3V
 * White - SDA, n
 * Blue  - SCL, n+1
 * Distance sensor cannot be configured on the same bus as Color Sensor
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Test")
@Disabled

public class SensorDistance extends LinearOpMode {

    private void telemetryData(DistanceSensor ds, Rev2mDistanceSensor tof) {
        // generic DistanceSensor methods
        telemetry.addData("deviceName", ds.getDeviceName() );
        telemetry.addData("range", String.format(Locale.US,"%.01f mm", ds.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format(Locale.US,"%.01f cm", ds.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format(Locale.US,"%.01f m", ds.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format(Locale.US,"%.01f in", ds.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods
        telemetry.addData("ID", String.format("%x", tof.getModelID()));
        telemetry.addData("did time out", Boolean.toString(tof.didTimeoutOccur()));

        telemetry.update();
        sleep(3000L);
    }

    @Override
    public void runOpMode() {
        // REV 2M Distance Sensor
        DistanceSensor sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensorDistanceRight");
        DistanceSensor sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensorDistanceLeft");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTOF1 = (Rev2mDistanceSensor) sensorRange1;
        Rev2mDistanceSensor sensorTOF2 = (Rev2mDistanceSensor) sensorRange2;

        telemetry.addLine("Right is 1, Left is 2");
        telemetry.addData("Start", "Press PLAY");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            telemetry.addLine("Right distance reading");
            telemetryData(sensorRange1, sensorTOF1);
            telemetry.addLine("Left distance readings");
            telemetryData(sensorRange2, sensorTOF2);
        }
    }

}