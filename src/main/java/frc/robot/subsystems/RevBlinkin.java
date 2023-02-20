package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class RevBlinkin extends SubsystemBase {
  private static Spark m_blinkin = null;
  public RevBlinkin() {
    m_blinkin = new Spark(9);
    //solid_orange();
  }

  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);}
  }
 // public void solid_orange() {
 // set(0.65);
 // }
}