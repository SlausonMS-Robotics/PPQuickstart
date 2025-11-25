package org.firstinspires.ftc.teamcode.robot;


public class states {
    motors robotMotors = new motors();
    servos servos = new servos();




        public void setRobotState(int state) {


            switch (state) {
                case 0: //intake off, transfer off
                    robotMotors.setTransferState(0);
                    robotMotors.setIntakeState(0);
                    servos.setIntakeServos(false);
                    break;
                case 1: //intake on, transfer backwards
                    robotMotors.setTransferState(2);
                    robotMotors.setIntakeState(1);
                    servos.setIntakeServos(true);
                    break;
                case 2: //intake on, transfer on, shooting
                    robotMotors.setTransferState(1);
                    robotMotors.setIntakeState(1);
                    servos.setIntakeServos(true);
                    break;
            }
        }
}
