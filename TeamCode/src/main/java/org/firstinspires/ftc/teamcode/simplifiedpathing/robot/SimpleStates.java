package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

/**
 * Handles the high-level states of the robot mechanisms.
 */
public class SimpleStates {
    private final SimplifiedRobot robot;

    public enum State {
        OFF,
        INTAKE_ON,
        SHOOTING,
        HOLD_BALL,
        SLOW_INTAKE
    }

    public SimpleStates(SimplifiedRobot robot) {
        this.robot = robot;
    }

    public void setState(State state) {
        switch (state) {
            case OFF:
                robot.motors.intakeMotor.setPower(0);
                robot.motors.transferMotor.setPower(0);
                robot.servos.setIntakeServos(false);
                break;
            case INTAKE_ON:
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(0.25);
                robot.servos.setIntakeServos(true);
                break;
            case SHOOTING:
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(1.0);
                robot.servos.setIntakeServos(true);
                break;
            case HOLD_BALL:
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(0);
                robot.servos.setIntakeServos(true);
                break;
            case SLOW_INTAKE:
                robot.motors.intakeMotor.setPower(0.15);
                robot.motors.transferMotor.setPower(0);
                robot.servos.setIntakeServos(true);
                break;
        }
    }
}
