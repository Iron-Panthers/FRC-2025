package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; ++i) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; ++i) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera " + i, inputs[i]);
    }

    for (int i = 0; i < io.length; ++i) {
      var observations = inputs[i].observations;
      for (var observation : observations) {
        if (observation.ambiguity() > AMBIGUITY_CUTOFF || observation.ambiguity() == -1) continue;
      }
    }
  }
}
