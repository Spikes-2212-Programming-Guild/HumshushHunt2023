package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

public class Leds extends DashboardedSubsystem {

    public static final int LED_NUMBER = 0;

    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    public Leds(Namespace namespace) {
        super(namespace);
        this.led = new AddressableLED(RobotMap.PWM.LED_PORT);
        this.ledBuffer = new AddressableLEDBuffer(LED_NUMBER);
        led.setData(ledBuffer);
        led.start();
        led.setLength(LED_NUMBER);
    }

    @Override
    public void configureDashboard() {

    }
}
