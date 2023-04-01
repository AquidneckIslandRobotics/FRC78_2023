package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RevBlinkin{
  public static BlinkinLEDMode ledMode = BlinkinLEDMode.BLUE;
  private static Spark m_blinkin = new Spark(9);

public static enum BlinkinLEDMode{
  YELLOW(0.69),
  PURPLE(0.91),
  RED(0.61),
  BLUE(0.87),
  WHITE(0.93),
  SHOT1(0.13), //Shot1 correlates to color pattern 1 
  STROBE1(0.15), //Strobe1 correlates to color pattern 1
  SHOT2(0.33), //Shot2 correlates to color pattern 2
  STROBE2(0.35); //Strobe2 correlates to color pattern 2

  private double value;
  private BlinkinLEDMode(double val){
    this.value = val;
  }
  public double getValue() {
    return value;
  }
}
  
// public static void ledMode(BlinkinLEDMode ledMode){
//   this.ledMode = ledMode;
// }
  
public static void set(double BlinkinLEDMode) {
      m_blinkin.set(BlinkinLEDMode);
    // new PrintCommand("Setting LEDs to " + val);
    // SmartDashboard.putNumber("LedValue", val);  
  }
}


