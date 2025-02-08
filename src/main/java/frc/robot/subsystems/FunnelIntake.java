package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.FunnelIntakeSetpoints;  
import frc.robot.Constants.WristSetpoints;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

public class FunnelIntake extends SubsystemBase {
  public enum Setpoint{
    FeederStation,
    Climb;
}
 // funnel setup
  private SparkFlex l_funnelMotor = new SparkFlex(FunnelConstants.FunnelLIntake, MotorType.kBrushless);
  private SparkFlex r_funnelMotor = new SparkFlex(FunnelConstants.FunnelRIntake, MotorType.kBrushless);
  private SparkMax funnelWrist = new SparkMax(FunnelConstants.FunnelWrist, MotorType.kBrushless);
  private SparkClosedLoopController funnelWristController = funnelWrist.getClosedLoopController();
  private RelativeEncoder funnelWristEncoder = funnelWrist.getEncoder();
  private double funnelWristCurrentTarget;

  public FunnelIntake() {
    l_funnelMotor.configure(
      Configs.FunnelIntakeSubsystem.l_funnelMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    r_funnelMotor.configure(
        Configs.FunnelIntakeSubsystem.r_funnelMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    funnelWrist.configure(
        Configs.FunnelIntakeSubsystem.funnelWristMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    funnelWristEncoder.setPosition(0);
  }

  private void moveToSetpoint(){
    funnelWristController.setReference(funnelWristCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void setWristPower(double power) {
    funnelWrist.set(power);
  }
  public void setIntakePower(double leftPower, double rightPower) {
    l_funnelMotor.set(leftPower);
    r_funnelMotor.set(rightPower);
  }

  public Command runIntakeCommand(){
        return this.startEnd(
            () -> this.setIntakePower(FunnelIntakeSetpoints.kForward, FunnelIntakeSetpoints.kReverse), () -> this.setIntakePower(0.0, 0.0));
  }
  

  public void periodic() {
    // This method will be called once per scheduler run
  }


    
}
