/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.Driving;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Example Drive Forward and Back", group = "DriveExample")
@Disabled

public class ExampleDriveByGyro extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;

    static final double     countsPerMotorRev       = 1440;
    static final double     wheelDiameterInches     = 4;
    static final double     driveGearReduction      = 1;
    static final double     countsPerInch           = countsPerMotorRev*driveGearReduction/(wheelDiameterInches*3.14);

    int targetPosition = 0;

    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("right motor");
        leftMotor = hardwareMap.dcMotor.get("left motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setTargetPosition(targetPosition);
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status:", "Robot is Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("1 Right Motor Pos", rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Pos", leftMotor.getCurrentPosition());
    }


    @Override
    public void start() {
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {
        if (!rightMotor.isBusy() && !leftMotor.isBusy()) {
            if (targetPosition == 0) {
                targetPosition = (int) (24 * countsPerInch);
                rightMotor.setTargetPosition(targetPosition);
                leftMotor.setTargetPosition(targetPosition);

            }
            else {
                targetPosition = 0;
                rightMotor.setTargetPosition(targetPosition);
                leftMotor.setTargetPosition(targetPosition);
            }
        }


        rightMotor.setPower(.8);
        leftMotor.setPower(.8);

        // send the info back to driver station using telemetry function.
        telemetry.addData("1 Right Motor Power", rightMotor.getCurrentPosition());
        telemetry.addData("2 Left Motor Power", leftMotor.getCurrentPosition());
        telemetry.addData("3 Target Position", targetPosition);
    }


    @Override
    public void stop() {
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}