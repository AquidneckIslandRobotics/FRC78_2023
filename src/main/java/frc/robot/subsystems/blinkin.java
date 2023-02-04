package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class blinkin extends SubsystemBase {

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private static Spark m_blinkin = null;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solid_orange();
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void rainbow() {
    set(-0.99);
  }

  public void solid_orange() {
    set(0.65);
  }
  public void allianceColor() {
      ((blinkin) RobotContainer.m_blinkin).set(-0.01);
  }
}