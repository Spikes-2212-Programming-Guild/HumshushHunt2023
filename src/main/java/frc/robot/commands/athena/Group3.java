package frc.robot.commands.athena;

public class Group3 extends ProgramBase {

    @Override
    public void writeProgram() {
    moveForwardInSeconds(7);
    moveArm();
    releaseCube();
    }
}
