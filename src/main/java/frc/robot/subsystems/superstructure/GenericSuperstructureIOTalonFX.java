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

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0);

  public GenericSuperstructureIOTalonFX(
      int id,
      boolean inverted,
      double supplyCurrentLimit,
      Optional<Integer> canCoderID,
      double reduction,
      double upperLimit,
      double upperVoltLimit,
      double lowerVoltLimit) {
    talon = new TalonFX(id);

    // VOLTAGE, LIMITS AND RATIO CONFIG
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(upperLimit);
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

  /** */
  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(-1));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(0);
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
