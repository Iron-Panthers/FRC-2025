package frc.robot.subsystems.rgb;

import frc.robot.Constants;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage.RGBColor;

public class RGBConstants {
  public static final RGBConfig RGB_CONFIGS =
      switch (Constants.getRobotType()) {
        case COMP -> new RGBConfig(34, 220);
        case ALPHA -> new RGBConfig(0, 0);
        case PROG -> new RGBConfig(0, 0);
        case SIM -> new RGBConfig(0, 0);
      };

  public static final class Colors {
    public static final RGBColor YELLOW = new RGBColor(255, 107, 0);
    public static final RGBColor PURPLE = new RGBColor(127, 0, 127);
    public static final RGBColor RED = new RGBColor(255, 0, 0);
    public static final RGBColor ORANGE = new RGBColor(255, 35, 0);
    public static final RGBColor BLUE = new RGBColor(0, 0, 255);
    public static final RGBColor PINK = new RGBColor(250, 35, 100);
    public static final RGBColor MINT = new RGBColor(55, 255, 50);
    public static final RGBColor GREEN = new RGBColor(0, 255, 0);
    public static final RGBColor TEAL = new RGBColor(0, 255, 255);
    public static final RGBColor WHITE = new RGBColor(255, 255, 255);
  }

  public record RGBConfig(int id, int numLEDs) {}

  public static class RGBMessage {
    public enum RGBPattern {
      STROBE,
      SOLID,
      RAINBOW,
      FIRE,
      BOUNCE,
      PULSE
    }

    public enum MessagePriority {
      A_CRITICAL_NETWORK_FAILURE,
      B_MISSING_CAN_DEVICE,
      C_CLIMB,
      D_READY_TO_INTAKE,
      E_L2,
      F_L3,
      G_L4,
      H_L1,
      I_CORAL_DETECTED,
      J_DEFAULT
    }

    public static class RGBColor {
      public final int r;
      public final int g;
      public final int b;

      public RGBColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
      }
    }

    private RGBColor color;
    private RGBPattern pattern;
    private MessagePriority priority;
    private boolean isExpired;

    public RGBMessage(
        RGBColor color, RGBPattern pattern, MessagePriority priority, boolean isExpired) {
      this.color = color;
      this.pattern = pattern;
      this.priority = priority;
      this.isExpired = isExpired;
    }

    public void setIsExpired(boolean isExpired) {
      this.isExpired = isExpired;
    }

    public RGBColor getColor() {
      return color;
    }

    public RGBPattern getPattern() {
      return pattern;
    }

    public MessagePriority getPriority() {
      return priority;
    }

    public boolean getIsExpired() {
      return isExpired;
    }
  }
}
