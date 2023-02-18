package frc.robot.services;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Gripper;

public class LedsService {

    public enum Mode {

        EMPTY_GRIPPER(254, 0, 0), ALLIGNED_TO_GAME_PIECE(204, 0, 254), HAS_GAME_PIECE(0, 0, 254),
        HAS_GAME_PIECE_AND_ALLIGNED(0, 254, 0);

        public final int red;
        public final int green;
        public final int blue;

        Mode(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static final int NUMBER_OF_LEDS = 60;

    private static LedsService instance;

    private final Gripper gripper;

    private final VisionService vision;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public static LedsService getInstance() {
        if (instance == null) {
            instance = new LedsService(new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS), VisionService.getInstance(), Gripper.getInstance());
        }
        return instance;
    }

    private LedsService(AddressableLED led, AddressableLEDBuffer ledBuffer, VisionService vision, Gripper gripper) {
        this.led = led;
        this.ledBuffer = ledBuffer;
        this.vision = vision;
        this.gripper = gripper;
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void periodic() {
        Mode mode;
        if (gripper.sensorHasGamePiece()) {
            if (vision.limelightCentered()) {
                mode = Mode.HAS_GAME_PIECE_AND_ALLIGNED;
            } else {
                mode = Mode.HAS_GAME_PIECE;
            }
        } else {
            if (vision.photonVisionCentered()) {
                mode = Mode.ALLIGNED_TO_GAME_PIECE;
            } else {
                mode = Mode.EMPTY_GRIPPER;
            }
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, mode.red, mode.green, mode.blue);
        }
    }

    public void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }
}
