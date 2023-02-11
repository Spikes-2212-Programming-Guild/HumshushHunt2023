package frc.robot;

public class RobotMap {

    public interface CAN {

        int DRIVETRAIN_LEFT_SPARKMAX_MASTER = 1;
        int DRIVETRAIN_LEFT_SPARKMAX_SLAVE = 2;
        int DRIVETRAIN_RIGHT_SPARKMAX_MASTER = 3;
        int DRIVETRAIN_RIGHT_SPARKMAX_SLAVE = 4;

        int ARM_FIRST_JOINT_SPARKMAX_MASTER = 5;
        int ARM_FIRST_JOINT_SPARKMAX_SLAVE = 6;
        int ARM_SECOND_JOINT_SPARKMAX_MASTER = 7;
    }

    public interface DIO {

        int ARM_FIRST_JOINT_ABSOLUTE_ENCODER = 1;
        int ARM_SECOND_JOINT_ABSOLUTE_ENCODER = 2;
    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

        int GRIPPER_SOLENOID_FORWARD = 0;
        int GRIPPER_SOLENOID_REVERSE = 1;
    }
}
