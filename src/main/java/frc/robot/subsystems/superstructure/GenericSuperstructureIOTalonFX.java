package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class GenericSuperstructureIOTalonFX implements GenericSuperstructureIO {
  protected final TalonFX talon;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  // zeroing stuff
  private final double zeroingVolts;
  private final double zeroingOffset;
  private final double zeroingVoltageThreshold;

  private final double positionTargetEpsilon;

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0);

  /**
   * Constructs a new GenericSuperstructureIOTalonFX.
   *
   * @param id The ID of the TalonFX motor controller.
   * @param inverted Whether the motor is inverted.
   * @param supplyCurrentLimit The supply current limit for the motor.
   * @param canCoderID The optional ID of the CANcoder.
   * @param reduction The reduction ratio for the mechanism.
   * @param upperLimit The upper limit for the motor position.
   * @param lowerLimit The lower limit for the motor position.
   * @param upperVoltLimit The upper voltage limit for the motor.
   * @param lowerVoltLimit The lower voltage limit for the motor.
   * @param zeroingVolts The voltage to apply during zeroing.
   * @param zeroingOffset The offset to set after zeroing.
   * @param positionTargetEpsilon The allowable error in position target.
   */
  public GenericSuperstructureIOTalonFX(
      int id,
      boolean inverted,
      double supplyCurrentLimit,
      Optional<Integer> canCoderID,
      double reduction,
      Optional<Double> upperLimit,
      Optional<Double> lowerLimit,
      double upperVoltLimit,
      double lowerVoltLimit,
      double zeroingVolts,
      double zeroingOffset,
      double zeroingVoltageThreshold,
      double positionTargetEpsilon) {
    talon = new TalonFX(id);

    // set the zeroing values such that when the robot zeros it will apply the zeroing volts and
    // when it reaches a resistance from part of the mechanism, it sets the position to the zeroing
    // Offset
    this.zeroingVolts = zeroingVolts;
    this.zeroingOffset = zeroingOffset;
    this.zeroingVoltageThreshold = zeroingVoltageThreshold;

    this.positionTargetEpsilon = positionTargetEpsilon;

    // VOLTAGE, LIMITS AND RATIO CONFIG
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    if (upperLimit.isPresent()) { // only set the upper limit if we have one
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
      config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(upperLimit.get());
    }
    if (lowerLimit.isPresent()) { // only set the lower limit if we have one
      config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
      config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(lowerLimit.get());
    }

    config.Voltage.withPeakForwardVoltage(upperVoltLimit);
    config.Voltage.withPeakReverseVoltage(lowerVoltLimit);
    config.Feedback.withSensorToMechanismRatio(reduction);

    // CANCODER CONFIG
    if (canCoderID.isPresent()) {
      CANcoder canCoder = new CANcoder(canCoderID.get());
      canCoder
          .getConfigurator()
          .apply(
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                          .withMagnetOffset(0)));

      canCoder.getConfigurator().setPosition(0);
      config.Feedback.withRemoteCANcoder(canCoder);
      config.Feedback.withSensorToMechanismRatio(reduction);
    }
    talon.getConfigurator().apply(config);
    setOffset();
    talon.setNeutralMode(NeutralModeValue.Brake);

    // STATUS SIGNALS
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
    positionRotations = talon.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp)
            .isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(zeroingVolts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(zeroingOffset);
  }

  @Override
  public double getPositionTargetEpsilon() {
    return positionTargetEpsilon;
  }

  @Override
  public double getZeroingVoltageThreshold() {
    return zeroingVoltageThreshold;
  }

  @Override
  public void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      GravityTypeValue gravityTypeValue) {
    Slot0Configs gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;
    gainsConfig.kG = kG;
    gainsConfig.GravityType = gravityTypeValue;

    talon.getConfigurator().apply(gainsConfig);
  }
}
