package frc.robot.subsystems.rgb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rgb.RGBConstants.Colors;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage.MessagePriority;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage.RGBPattern;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class RGB extends SubsystemBase {
  public static enum RGBMessages {
    CRITICAL_NETWORK_FAILURE(
        new RGBMessage(
            Colors.ORANGE, RGBPattern.STROBE, MessagePriority.A_CRITICAL_NETWORK_FAILURE, true)),
    MISSING_CAN_DEVICE(
        new RGBMessage(Colors.RED, RGBPattern.FIRE, MessagePriority.B_MISSING_CAN_DEVICE, true)),
    CORAL_DETECTED(
        new RGBMessage(Colors.GREEN, RGBPattern.STROBE, MessagePriority.C_CORAL_DETECTED, true)),
    READY_TO_INTAKE(
        new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.D_READY_TO_INTAKE, true)),
    L1(new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.E_L1, true)),
    L2(new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.F_L2, true)),
    L3(new RGBMessage(Colors.TEAL, RGBPattern.STROBE, MessagePriority.G_L3, true)),
    L4(new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.H_L4, true)),
    CLIMB(new RGBMessage(Colors.GREEN, RGBPattern.SOLID, MessagePriority.I_CLIMB, true)),
    DEFAULT(new RGBMessage(Colors.WHITE, RGBPattern.RAINBOW, MessagePriority.J_DEFAULT, false));

    RGBMessage rgbMessage;

    private RGBMessages(RGBMessage rgbMessage) {
      this.rgbMessage = rgbMessage;
    }

    public void setIsExpired(boolean isExpired) {
      rgbMessage.setIsExpired(isExpired);
    }
  }

  private final RGBIO rgbIO;
  private RGBIOInputsAutoLogged inputs = new RGBIOInputsAutoLogged();
  private Optional<RGBMessage> currentMessage = Optional.empty();

  public RGB(RGBIO rgbIO) {
    this.rgbIO = rgbIO;
  }

  @Override
  public void periodic() {
    currentMessage = Optional.empty();
    int total = 0;
    for (RGBMessages message : RGBMessages.values()) {
      if (!message.rgbMessage.getIsExpired()
          && (currentMessage.isPresent()
              ? currentMessage.get().getPriority().compareTo(message.rgbMessage.getPriority()) > 0
              : true)) {
        currentMessage = Optional.of(message.rgbMessage);
        total++;
      }
    }
    if (currentMessage.isPresent()) {
      rgbIO.displayMessage(currentMessage.get());
    } else {
      rgbIO.displayMessage(RGBMessages.DEFAULT.rgbMessage);
    }
    rgbIO.updateInputs(inputs);
    Logger.processInputs("RGB", inputs);
    Logger.recordOutput("RGB/Total messages not expired", total);
    Logger.recordOutput(
        "RGB/Message",
        currentMessage.isPresent() ? currentMessage.get().getPriority().name() : "None");
  }

  public Command startMessageCommand(RGBMessages message) {
    return new InstantCommand(() -> message.setIsExpired(false));
  }

  public Command endMessageCommand(RGBMessages message) {
    return new InstantCommand(() -> message.setIsExpired(true));
  }
}
