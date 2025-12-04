package frc.team670.robot;

import frc.team670.libs.health.Health;
import frc.team670.libs.motors.MustangMotor;

@SuppressWarnings("unused")
public class TestMotor extends MustangMotor<Integer> {

  private double position = 0.0; // rotations
  private double velocity = 0.0; // rotations/sec
  private double lastPosition = 0.0;
  private double lastVelocity = 0.0;
  private double p = 0.0, i = 0.0, d = 0.0;
  private double s = 0.0, v = 0.0, a = 0.0, g = 0.0;
  private double target = 0.0;
  private double feedforward = 0.0;

  private long lastTime = System.nanoTime();

  public TestMotor(int motor) {
    super(motor);
    RobotContainer.add(this);
  }

  @Override
  public void setTarget(double rotations, double feedforward) {
    this.target = rotations;
    this.feedforward = feedforward;
  }

  @Override
  public void setSpeed(double speed) {
    this.velocity = speed;
  }

  @Override
  public void setEncoderPosition(double rotations) {
    this.position = rotations;
  }

  @Override
  public void toggleIdleMode() {}

  @Override
  public double getRotations() {
    return position;
  }

  @Override
  public double getVelocity() {
    return velocity;
  }

  @Override
  public double getSpeed() {
    return velocity;
  }

  @Override
  public double getCurrent() {
    return 0.0;
  }

  @Override
  public Health checkHealth() {
    return Health.GREEN;
  }

  @Override
  public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
  }

  @Override
  public void setFF(double s, double v, double a, double g) {
    this.s = s;
    this.v = v;
    this.a = a;
    this.g = g;
  }

  private double lastError = 0.0;

  public void updateSimulation() {
    long now = System.nanoTime();
    double dt = (now - lastTime) / 1e9;
    lastTime = now;

    double error = target - position;
    double integral = error * dt;
    double derivative = (error - lastError) / dt;
    lastError = error;

    double pidOutput = p * error + i * integral + d * derivative;
    double ffOutput =
        s * Math.signum(velocity) + v * velocity + a * (velocity - lastVelocity) / dt + g;

    double totalOutput = pidOutput + ffOutput;

    double mass = 1.0; // system inertia
    double damping = 0.1; // friction/resistance
    double acceleration = totalOutput / mass - damping * velocity;

    velocity += acceleration * dt;
    position += velocity * dt + 0.1;

    lastVelocity = velocity;
  }
}
