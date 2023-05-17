package frc.robot.commands.athena;

public class Group2 extends ProgramBase {

    @Override
    public void writeProgram() {
    moveForwardInSeconds(1);
    moveArm();
    moveForwardInSeconds(0.5);
    releaseCube();
    moveBackwardInSeconds(1.5);
    }
}
