package frc.team670.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.team670.libs.logging.MustangNotifier;
import frc.team670.libs.motors.MotorUtils;
import frc.team670.libs.utils.QuadConsumer;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Automatically tunes a motor's PID values. based on
 * https://tlk-energy.de/blog-en/practical-pid-tuning-guide
 *
 * <p>P: Wait until maxVal - startValue >~ 0.21 * (maxVal - minVal) I: Wait until settling then
 * check Math.abs(max - setpoint) = Math.abs(min - setpoint) move in the direction of the larger
 * error 3-5 checks before picking best one if unable to find optimal value D: When max decreases
 * bellow the setpoint
 */
public class AutoPID<MotorType> {

  private MotorType motor;
  private double[] tunedValues = new double[3]; // P, I, D

  private int stage = 0; // 0=P tuning, 1=I tuning, 2=D tuning, 3=done
  private boolean started = false;

  private double targetPosition;
  private double startPosition;

  private boolean returning = false;

  private Consumer<AutoPID<MotorType>> onFinishCallback;
  private boolean called = false;

  private BiConsumer<MotorType, Double> setTarget;
  private Function<MotorType, Double> getPosition;
  private QuadConsumer<MotorType, Double, Double, Double> setPID;

  private double pStep;
  private double iStep;
  private double dStep;

  private AutoPID(
      MotorType motor,
      BiConsumer<MotorType, Double> setTarget,
      Function<MotorType, Double> getPosition,
      QuadConsumer<MotorType, Double, Double, Double> setPID) {
    this.motor = motor;
    this.setTarget = setTarget;
    this.getPosition = getPosition;
    this.setPID = setPID;
  }

  private double getRotations() {
    return getPosition.apply(motor);
  }

  private void setPID(double p, double i, double d) {
    setPID.accept(motor, p, i, d);
  }

  private void setTarget(double target) {
    setTarget.accept(motor, target);
  }

  public static AutoPID<TestMotor> tuneTest(int motorId) {
    TestMotor motor = new TestMotor(motorId);
    return new AutoPID<TestMotor>(
        motor,
        (m, s) -> m.setTarget(s, 0),
        (m) -> m.getRotations(),
        (m, p, i, d) -> m.setPID(p, i, d));
  }

  public static AutoPID<TalonFX> tuneTalonFX(int motorId, TalonFXConfiguration config) {
    TalonFX motor = MotorUtils.createTalonFX(motorId, config);
    return new AutoPID<TalonFX>(
        motor,
        (m, t) -> {
          System.out.println(t);
          m.setControl(new MotionMagicVoltage(0).withPosition(t));
        },
        (m) -> {
          return m.getPosition().getValueAsDouble();
        },
        (m, p, i, d) -> {
          Slot0Configs slot_config = new Slot0Configs();
          m.getConfigurator().refresh(slot_config);
          slot_config.kP = p;
          slot_config.kI = i;
          slot_config.kD = d;
          m.getConfigurator().apply(slot_config);
        });
  }

  public void start(
      double initialTarget, double initalP, double pStep, double iStep, double dStep) {
    this.pStep = pStep;
    this.iStep = iStep;
    this.dStep = dStep;
    this.targetPosition = initialTarget;
    this.startPosition = getRotations();
    this.maxVal = -9e67;
    this.minVal = 9e67;
    setPID(initalP, 0, 0);
    tunedValues[0] = initalP;
  }

  public void onFinish(Consumer<AutoPID<MotorType>> callback) {
    this.onFinishCallback = callback;
  }

  private Runnable then;

  private void requestNextTest() {
    returning = true;
    maxVal = -9e67;
    minVal = 9e67;
    setTarget(startPosition);

    then =
        () -> {
          setPID(tunedValues[0], tunedValues[1], tunedValues[2]);
          setTarget(targetPosition);
        };
  }

  private double maxVal = -9e67;
  private double minVal = 9e67;
  private double lastPosition = startPosition;
  private MustangDebouncer stabilityDebouncer = new MustangDebouncer(2.5);
  private double stabillitySlopeThreashold = 0.001; // tune threshold
  private double allowedIError = 0.05;
  private boolean stable = false;
  private boolean increasing = false;
  private double prevDelta = 0;
  private boolean oscillating = false;
  private int oCount = 0;

  private void waitTillStable(Consumer<Double> tunningCommand, Runnable unstable) {
    double currentPos = getRotations();
    if (currentPos > maxVal) {
      maxVal = currentPos;
    }

    increasing = currentPos > maxVal;

    if (currentPos < minVal
        && maxVal > currentPos
        && !increasing
        && currentPos > targetPosition * 0.55) {
      minVal = currentPos;
    }

    MustangNotifier.log("AutoPID/max", maxVal);
    MustangNotifier.log("AutoPID/min", minVal);

    double delta = currentPos - lastPosition;

    // // --- OSCILLATION DETECTION ---
    boolean slopeFlip = (delta * prevDelta) < 0; // sign change
    boolean amplitudeLarge = lastPosition - currentPos > 0.5; // amplitude threshold

    if (slopeFlip && amplitudeLarge) {
      oCount++;
    }

    if (oCount >= 4) {
      oscillating = true;
      oCount = 0;
    }

    prevDelta = delta;
    lastPosition = currentPos;

    MustangNotifier.log("AutoPID/oCount", oCount);

    if (oscillating) {

      unstable.run();
      oscillating = false;
      stable = false;
      stabilityDebouncer.calculate(false);
      requestNextTest();

      return;
    }

    MustangNotifier.log("AutoPID/stable", stable);

    if (stable) {
      tunningCommand.accept(currentPos);
      requestNextTest();
      stable = false;
      stabilityDebouncer.calculate(false);
      MustangNotifier.log("AutoPID/waitingforStable", false);
    } else {
      MustangNotifier.log("AutoPID/lastPos", lastPosition);
      stable =
          stabilityDebouncer.calculate(
              Math.abs(currentPos - lastPosition) < stabillitySlopeThreashold);
      MustangNotifier.log("AutoPID/waitingforStable", true);
    }
  }

  private void tuneP(double currentPos) {
    double travelDistance = Math.max(0, maxVal + startPosition);
    double curveDistance = maxVal - minVal;
    boolean withinLowerThreashold = curveDistance > 0.20 * travelDistance;
    boolean withinUpperThreashold = curveDistance < 0.23 * travelDistance;
    if (withinLowerThreashold && withinUpperThreashold || targetPosition - maxVal < 0.01) {
      System.out.println("P tuning complete. New P: " + tunedValues[0]);
      stage++;
    } else {
      if (curveDistance < 0.20 * travelDistance) {
        tunedValues[0] += pStep;
      } else if (curveDistance > 0.23 * travelDistance) {
        if (tunedValues[0] <= pStep) {
          pStep = tunedValues[0] / 2;
          tunedValues[0] += pStep;
          return;
        }
        tunedValues[0] -= pStep;
      }
    }
  }

  private void tuneI(double currentPos) {
    double topArea = Math.abs(maxVal - targetPosition);
    double bottomArea = Math.abs(minVal - targetPosition);
    if (Math.abs(topArea - bottomArea) < allowedIError) {
      System.out.println("I tuning complete. New I: " + tunedValues[1]);
      stage++;
    } else if (topArea < bottomArea) {
      tunedValues[1] += iStep;
    } else if (topArea > bottomArea) {

      tunedValues[1] -= iStep;
    }
  }

  private void tuneD(double currentPos) {
    if (maxVal < targetPosition) {
      System.out.println("D tuning complete. New D: " + tunedValues[2]);
      stage++;
    } else {
      tunedValues[2] += dStep;
    }
  }

  public void loop() {
    if (stage > 3) return;

    MustangNotifier.log("AutoPID/currentPos", getRotations());
    MustangNotifier.log("AutoPID/values", tunedValues);
    MustangNotifier.log("AutoPID/stage", stage);

    if (!started) {
      setTarget(targetPosition);
      started = true;
    }

    if (returning) {
      if (Math.abs(getRotations() - startPosition) < 0.9) {
        returning = false;

        then.run();
      } else {
        MustangNotifier.log("AutoPID/target", startPosition);
      }
      return;
    } else {
      MustangNotifier.log("AutoPID/target", targetPosition);
    }

    if (stage == 0) {
      waitTillStable(
          this::tuneP,
          () -> {
            tunedValues[0] -= pStep; // decrease P if unstable
          });
    } else if (stage == 1) {
      waitTillStable(
          this::tuneI,
          () -> {
            tunedValues[1] -= iStep; // decrease I if unstable
          });
    } else if (stage == 2) {
      waitTillStable(
          this::tuneD,
          () -> {
            tunedValues[2] -= dStep; // decrease D if unstable
          });
    }

    if (stage >= 3 && !called) {
      called = true;
      if (onFinishCallback != null) onFinishCallback.accept(this);
    }
  }

  public boolean isFinished() {
    return stage >= 3;
  }

  public double[] getTunedValues() {
    return tunedValues;
  }

  public MotorType getMotor() {
    return motor;
  }
}
