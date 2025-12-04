// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot;

import frc.team670.libs.logging.MustangNotifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RobotContainer {

  private static List<TestMotor> motors = new ArrayList<>();
  private List<AutoPID> tuners = new ArrayList<>();

  AutoPID tuner2;

  public RobotContainer() {
    AutoPID tuner = AutoPID.tuneTest(25);
    tuner.start(250);
    tuner.onFinish(this::handleFinish);
    tuners.add(tuner);
    tuner2 = AutoPID.tuneTest(25);
    tuner2.start(250);
    tuner2.onFinish(this::handleFinish);
  }

  private void handleFinish(AutoPID tuner, int motorId) {
    MustangNotifier.log(String.format("AutoTuner/MotorId%d", motorId), tuner.getTunedValues());
    System.out.println(Arrays.toString(tuner.getTunedValues()));
  }

  public static void add(TestMotor motor) {
    motors.add(motor);
  }

  public void periodic() {

    for (TestMotor motor : motors) {
      motor.updateSimulation();
    }
    for (AutoPID tuner : tuners) {
      if (tuner.isFinished()) {
        tuner2.loop();
        continue;
      }
      tuner.loop();
    }
  }
}
