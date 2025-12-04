package frc.team670.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.team670.libs.motors.MustangMotor;
import java.util.function.BiConsumer;

/** Practical PID tuner for MustangMotor based on TLK PID guide. */
public class AutoPID {

  private MustangMotor<?> motor;
  private double[] tunedValues = new double[3]; // P, I, D

  private int stage = 0; // 0=P tuning, 1=I tuning, 2=D tuning, 3=done
  private boolean started = false;

  private double targetPosition;

  private double Ku; // ultimate gain
  private double Pu; // oscillation period
  private double oscillationStartTime;
  private int oscillationCount;
  private double previousErrorSign;

  private BiConsumer<AutoPID, Integer> onFinishCallback;
  private boolean called = false;
  private int motorId;

  private AutoPID(MustangMotor<?> motor) {
    this.motor = motor;
  }

  public static AutoPID tuneTest(int motorId) {
    AutoPID tuner = new AutoPID(new TestMotor(motorId));
    tuner.motorId = motorId;
    return tuner;
  }

  public void start(double initialTarget) {
    targetPosition = initialTarget;
    motor.setTarget(targetPosition, 0.0);

    started = true;
  }

  public void onFinish(BiConsumer<AutoPID, Integer> callback) {
    this.onFinishCallback = callback;
  }

  public void loop() {
    if (!started || stage > 3) return;

    double now = Timer.getFPGATimestamp();

    double position = motor.getRotations();
    double error = targetPosition - position;

    if (stage == 0) {
      double KpStep = 0.05; // increment P slowly
      tunedValues[0] += KpStep;

      motor.setPID(tunedValues[0], 0.0, 0.0);
      motor.setTarget(targetPosition, 0.0);

      double errorSign = Math.signum(error);

      if (errorSign != previousErrorSign && Math.abs(error) > 0.01) {
        oscillationCount++;
        if (oscillationCount == 1) {
          oscillationStartTime = now;
        } else if (oscillationCount == 3) {
          Pu = (now - oscillationStartTime) / 2.0;
          Ku = tunedValues[0];
          stage++;
        }
      }
      previousErrorSign = errorSign;
    } else if (stage == 1) {
      tunedValues[1] = 0.45 * Ku / Pu;
      stage++;
    } else if (stage == 2) {
      tunedValues[2] = Ku * Pu / 8.0;
      stage++;
    }

    motor.setPID(tunedValues[0], tunedValues[1], tunedValues[2]);
    motor.setTarget(targetPosition, 0.0);

    if (stage >= 3 && !called) {
      called = true;
      if (onFinishCallback != null) onFinishCallback.accept(this, motorId);
    }
  }

  public boolean isFinished() {
    return stage >= 3;
  }

  public double[] getTunedValues() {
    return tunedValues;
  }
}
