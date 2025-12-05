// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot;

import static frc.team670.libs.oi.XboxJoysticButtons.Driver_ButtonB;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.team670.libs.logging.MustangNotifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RobotContainer {

  private static List<TestMotor> motors = new ArrayList<>();
  private List<AutoPID<?>> tuners = new ArrayList<>();
  private boolean started = false;

  public RobotContainer() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.1).withKA(0.01).withKV(0.12))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(80)
                    .withMotionMagicAcceleration(100));
    AutoPID<TalonFX> tuner = AutoPID.tuneTalonFX(25, config);
    tuner.start(20, 0.01, 0.005, 0.005, 0.005);
    tuner.onFinish(this::handleFinish);
    tuners.add(tuner);
    Driver_ButtonB.onTrue(
        () -> {
          started = true;
        });
  }

  private void handleFinish(AutoPID<?> tuner) {
    MustangNotifier.log("AutoTuner/MotorId", tuner.getTunedValues());
    MustangNotifier.log("AutoPID/done", true);
    System.out.println(Arrays.toString(tuner.getTunedValues()));
  }

  public static void add(TestMotor motor) {
    motors.add(motor);
  }

  public void periodic() {

    // ((TalonFX) tuners.get(0).getMotor()).setControl(new
    // MotionMagicVoltage(0).withPosition(1000));

    if (!started) {
      return;
    }

    for (TestMotor motor : motors) {
      motor.updateSimulation();
    }
    for (AutoPID<?> tuner : tuners) {

      tuner.loop();
      // System.out.println(Arrays.toString(tuner.getTunedValues()));
    }
  }
}
