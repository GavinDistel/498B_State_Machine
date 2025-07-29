package frc.robot;

public class Constants {

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
  }

  public static final class DrivetrainConstants {
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.94;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 300;
  }

  public static final class ClimberConstants {
    public static final double P = 320;
    public static final double I = 0;
    public static final double D = 0;
    public static final double G = 0;
    public static final double DeployMotionMagicAcceleration = 100; // 100
    public static final double DeployMotionMagicCruiseVelocity = 250; // 250
    public static final double RetractMotionMagicAcceleration = 0.44;
    public static final double RetractMotionMagicCruiseVelocity = 0.44;
    public static final double MotionMagicJerk = 200;
    public static final double cageStallCurrent = 60;
  }

  public static final class intakeConstants {
    public static final double P = 60;// 40
    public static final double I = 2;
    public static final double D = 2;// 1
    public static final double G = 0.275;// 0.5
    public static final double MotionMagicAcceleration = 50;// 100
    public static final double MotionMagicCruiseVelocity = 100;// 100
    public static final double MotionMagicJerk = 200;// 100
    public static final double stallCurrent = 150;

  }
}
