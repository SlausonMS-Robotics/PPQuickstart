package org.firstinspires.ftc.teamcode.robot;


public class states {
    motors robotMotors = new motors();





        public void setRobotState(int state, servos Servos, motors robotMotors) {


            switch (state) {
                case 0: //intake off, transfer hold
                    robotMotors.setTransferState(0);
                    robotMotors.setIntakeState(0);
                    Servos.setIntakeServos(true);
                    break;
                case 1: //intake on, transfer slow, initial intake
                    robotMotors.setTransferState(2);
                    robotMotors.setIntakeState(1);
                    Servos.setIntakeServos(true);
                    break;
                case 2: //intake on, transfer on, shooting
                    robotMotors.setTransferState(1);
                    robotMotors.setIntakeState(1);
                    Servos.setIntakeServos(true);
                    break;
                case 3: //intake on, transfer hold, ball near flywheel
                    robotMotors.setTransferState(0);
                    robotMotors.setIntakeState(1);
                    Servos.setIntakeServos(true);
                    break;
                case 4: //reversed intake
                    robotMotors.setTransferState(0);
                    robotMotors.setIntakeState(-1);
                    Servos.setIntakeServos(true);
                    break;


            }
        }
}
