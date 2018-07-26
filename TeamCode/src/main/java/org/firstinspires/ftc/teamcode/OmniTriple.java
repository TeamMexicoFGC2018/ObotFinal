/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Herman", group="Teleop")
//@Disabled
public class OmniTriple extends OpMode {

    HardwareObot robot       = new HardwareObot(); // use the class created to define a Pushbot's hardware

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }
    
    public static double controlP(double pAct, double des) {
      double dif = Math.abs(des - pAct);
      if (dif > 0.3) {
        if (des > pAct) {
          pAct += 0.1;
        } else if (des < pAct) {
          pAct -= 0.1;
        }
      }  else {
        pAct = des;
      }
      return Range.clip(pAct, -1, +1);
    }
    
    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    double tiempo = 0;
    double leftPower = 0;
    double rightPower = 0;
    double centrePower = 0;
    double leftVel = 0;
    double rightVel = 0;
    double centreVel = 0;
    
    @Override
    public void loop() {
        double tiempoActual = runtime.milliseconds();

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        leftDeseado    = Range.clip(drive + turn, -1.0, 1.0);
        rightDeseado   = Range.clip(drive - turn, -1.0, 1.0);
        centreDeseado = gamepad1.right_stick_x;
        
        //Acceleration control
        if (tiempoActual >= tiempo + 40) {
          leftVel = controlP(leftVel,leftDeseado);
          rightVel = controlP(rightVel,rightDeseado);
          centreVel = controlP(centreVel, centreDeseado);
          tiempo = tiempoActual;
        }

        // Control power of wheels.
        if (gamepad1.right_trigger > 0) {
          leftPower = leftVel * 0.75;
          rightPower = rightVel * 0.75;
          centrePower = centreVel * 0.75;
        } else if(gamepad1.left_trigger > 0){
          leftPower = leftVel * 0.25 + leftVel * 0.5 * (1 - gamepad1.left_trigger);
          rightPower = rightVel * 0.25 + rightVel * 0.5 * (1 - gamepad1.left_trigger);
          centrePower = centreVel * 0.25 + centreVel * 0.5 * (1 - gamepad1.left_trigger);
        } else {
          leftPower = leftVel;
          rightPower = rightVel;
          centrePower = centreVel;
        }

        // Send calculated power to wheels
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
        robot.centreDrive.setPower(centrePower);

        if(gamepad1.a) {
            robot.recogedorLeft.setPower(1);
            robot.recogedorRight.setPower(1);
            robot.llantaL.setPower(1);
            robot.llantaR.setPower(1);
        } else if (gamepad1.b){
            robot.recogedorLeft.setPower(-1);
            robot.recogedorRight.setPower(-1);
            robot.llantaR.setPower(-1);
            robot.llantaL.setPower(-1);
        } else {
            robot.recogedorLeft.setPower(0);
            robot.recogedorRight.setPower(0);
            robot.llantaL.setPower(0);
            robot.llantaR.setPower(0);
        }


        if (gamepad1.y){
            robot.puerta.setPosition(1);
        } else if (gamepad1.x){
            robot.puerta.setPosition(0);
        }

        if (gamepad1.dpad_up) {
            robot.elevador.setPower(1);

        } else if (gamepad1.dpad_down){
            robot.elevador.setPower(-1);

        } else {
            robot.elevador.setPower(0);
        }

        //La parte cool del morro este, Santi
        if (gamepad2.a){
            robot.grip.setPosition(0);
        } else if (gamepad2.b) {
            robot.grip.setPosition(1);
        }

        if (gamepad2.dpad_up){
            robot.lift.setPower(1);
        } else if (gamepad2.dpad_down) {
            robot.lift.setPower(0);
        } else {
           robot.lift.setPower(0);
        }

        if (gamepad2.x){
            robot.eolico.setPower(1);
        } else  if (gamepad2.y) {
            robot.eolico.setPower(-1);
        } else {
            robot.eolico.setPower(0);
        }
        // bye bye Santi.

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}
