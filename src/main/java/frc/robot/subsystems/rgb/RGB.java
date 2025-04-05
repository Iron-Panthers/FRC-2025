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
    CLIMB(new RGBMessage(Colors.PURPLE, RGBPattern.STROBE, MessagePriority.C_CLIMB, true)),
    READY_TO_INTAKE(
        new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.D_READY_TO_INTAKE, true)),
    L2(new RGBMessage(Colors.ORANGE, RGBPattern.STROBE, MessagePriority.E_L2, true)),
    L3(new RGBMessage(Colors.YELLOW, RGBPattern.STROBE, MessagePriority.F_L3, true)),
    L4(new RGBMessage(Colors.BLUE, RGBPattern.STROBE, MessagePriority.G_L4, true)),
    L1(new RGBMessage(Colors.RED, RGBPattern.STROBE, MessagePriority.H_L1, true)),
    CORAL_DETECTED(
        new RGBMessage(Colors.GREEN, RGBPattern.PULSE, MessagePriority.I_CORAL_DETECTED, true)),
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
      if (RGBMessages.CORAL_DETECTED.rgbMessage.getIsExpired()
          // if one of the level messages
          && (currentMessage.get().equals(RGBMessages.L1.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L2.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L3.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L4.rgbMessage))) {
        currentMessage = Optional.of(RGBMessages.DEFAULT.rgbMessage);
      }
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

  public Command clearLevelCommands() {
    return new InstantCommand(
        () -> {
          RGBMessages.L1.setIsExpired(true);
          RGBMessages.L2.setIsExpired(true);
          RGBMessages.L3.setIsExpired(true);
          RGBMessages.L4.setIsExpired(true);
        });
  }
}
