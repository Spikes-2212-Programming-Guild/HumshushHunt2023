package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

public class Leds extends DashboardedSubsystem {

    public static final int NUMBER_OF_LEDS = 60;

    public static Leds instance;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private Leds(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        super("led");
        this.led = new AddressableLED(RobotMap.PWM.LED_PORT);
        this.ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
    }

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds("led", new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS));
        }
        return instance;
    }

    private

    @Override
    public void configureDashboard() {

    }
}
