package frc.team670.robot;

public class MustangDebouncer {

  private double current;
  private double time;

  public MustangDebouncer(double time) {
    current = System.nanoTime();
    this.time = time;
  }

  public boolean calculate(boolean value) {
    if (!value) {
      current = System.nanoTime();
      return false;
    }

    return (System.nanoTime() - current) / 1e9 > time;
  }
}
