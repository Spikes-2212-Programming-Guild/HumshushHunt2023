package frc.robot.commands.athena;

public class Group4 extends ProgramBase {

    @Override
    public void writeProgram() {
        moveArm();
        moveForwardInSeconds(3.5);
        releaseCube();
    }
}
