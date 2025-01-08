package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  // RPM
  public enum VelocityTarget {
    IDLE(0, 0),
    SHOOT(7500, 7500),
    SLOW(1200, 1200);
    private int topVelocity, bottomVelocity;

    private VelocityTarget(int topVelocity, int bottomVelocity) {
      this.topVelocity = topVelocity;
      this.bottomVelocity = bottomVelocity;
    }
  }

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  @AutoLogOutput(key = "Mechanism/Flywheels/Target")
  private VelocityTarget velocityTarget = VelocityTarget.IDLE;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mechanism/Flywheels", inputs);

    io.runVelocity(velocityTarget.topVelocity, velocityTarget.bottomVelocity);

    Logger.recordOutput("Mechanism/Flywheels/TopTargetVelocity", velocityTarget.topVelocity);
    Logger.recordOutput("Mechanism/Flywheels/BottomTargetVelocity", velocityTarget.bottomVelocity);
  }

  public void setVelocityTarget(VelocityTarget target) {
    velocityTarget = target;
  }
}
