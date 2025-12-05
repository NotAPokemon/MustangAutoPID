package frc.team670.robot;

import frc.team670.libs.health.Health;
import frc.team670.libs.motors.MustangMotor;

public class TestMotor extends MustangMotor<Integer> {

  private double position = 0.0;
  private double velocity = 0.0;

  private double target = 0.0;

  private double p = 0.0, i = 0.0, d = 0.0;
  private double integral = 0.0;
  private double lastError = 0.0;

  private long lastTime = System.nanoTime();

  public TestMotor(int motor) {
    super(motor);
    RobotContainer.add(this);
  }

  @Override
  public void setTarget(double rotations, double feedforward) {
    this.target = rotations;
  }

  @Override
  public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  // ---- Simple realistic constants ----
  private static final double MAX_ACCEL = 50.0; // rot/sec²
  private static final double MAX_VELOCITY = 80.0; // rot/sec
  private static final double FRICTION = 0.05;

  public void updateSimulation() {

    long now = System.nanoTime();
    double dt = (now - lastTime) / 1e9;
    lastTime = now;
    if (dt <= 0 || dt > 0.1) return;

    // ----- PID Control -----
    double error = target - position;
    integral += error * dt;
    double derivative = (error - lastError) / dt;
    lastError = error;

    double control = p * error + i * integral + d * derivative;

    // Clamp the controller output to realistic physical limits
    control = Math.max(-1.0, Math.min(1.0, control));

    // ----- Simple motor physics -----
    // convert control into acceleration
    double accel = control * MAX_ACCEL;

    // apply friction
    if (Math.abs(velocity) > 0.01) {
      accel -= Math.signum(velocity) * FRICTION;
    }

    // integrate acceleration → velocity
    velocity += accel * dt;

    // clamp to realistic motor speed
    velocity = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, velocity));

    // integrate position
    position += velocity * dt;
  }

  @Override
  public double getRotations() {
    return position;
  }

  @Override
  public double getVelocity() {
    return velocity;
  }

  @Override
  public void setSpeed(double speed) {}

  @Override
  public void setEncoderPosition(double rotations) {}

  @Override
  public void toggleIdleMode() {}

  @Override
  public double getSpeed() {
    return 0;
  }

  @Override
  public double getCurrent() {
    return 0;
  }

  @Override
  public Health checkHealth() {
    return Health.GREEN;
  }

  @Override
  public void setFF(double s, double v, double a, double g) {}
}
