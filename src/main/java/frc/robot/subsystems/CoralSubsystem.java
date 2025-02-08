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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Constants.WristSetpoints;

public class CoralSubsystem extends SubsystemBase {

    public enum Setpoint{
        FeederStation,
        L1,
        L2,
        L3,
        L4;
    }

    //arm setup
    private SparkMax l_armMotor = new SparkMax(ArmConstants.ArmLeftCanID, MotorType.kBrushless);
    private SparkMax r_armMotor = new SparkMax(ArmConstants.ArmRightCanID, MotorType.kBrushless);
    private SparkClosedLoopController l_armController = l_armMotor.getClosedLoopController();
    private SparkClosedLoopController r_armController = r_armMotor.getClosedLoopController();
    private RelativeEncoder armEncoder = l_armMotor.getEncoder();


    //elevator setup
    private SparkFlex l_elevatorMotor = new SparkFlex(ElevatorConstants.LeftElevatorCanID, MotorType.kBrushless);
    private SparkFlex r_elevatorMotor = new SparkFlex(ElevatorConstants.RightElevatorCanID, MotorType.kBrushless);
    private SparkClosedLoopController l_elevatorController = l_elevatorMotor.getClosedLoopController(); 
    private SparkClosedLoopController r_elevatorController = r_elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = l_elevatorMotor.getEncoder();

    //wrist setup
    private SparkFlex wristMotor = new SparkFlex(ArmConstants.WristCanID, MotorType.kBrushless);
    private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    //intake setup
    private SparkFlex intakeMotor = new SparkFlex(ArmConstants.IntakeCanID, MotorType.kBrushless);

    
    private boolean wasReset = false;
    private double armCurrentTarget = ArmSetpoints.FeederStation;
    private double wristCurrentTarget = WristSetpoints.FeederStation;
    private double elevatorCurrentTarget = ElevatorSetpoints.FeederStation;

    public CoralSubsystem(){
        
        l_elevatorMotor.configure(
            Configs.CoralSubsystem.l_elevatorMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        r_elevatorMotor.configure(
            Configs.CoralSubsystem.r_elevatorMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        r_armMotor.configure(
            Configs.CoralSubsystem.r_armMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        l_armMotor.configure(
            Configs.CoralSubsystem.l_armMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        wristMotor.configure(
            Configs.CoralSubsystem.wristMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        intakeMotor.configure(
            Configs.CoralSubsystem.intakeMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        armEncoder.setPosition(0);
        elevatorEncoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    private void moveToSetpoint(){
        l_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        r_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        wristController.setReference(wristCurrentTarget, ControlType.kMAXMotionPositionControl);
        l_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        r_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

     /** Zero the arm encoder when the user button is pressed on the roboRIO */
    private void zeroOnUserButton() {
        if (!wasReset && RobotController.getUserButton()) {
         // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
         // constant zeroing while pressed
         wasReset = true;
         elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
         wasReset = false;
        }
    }

    private void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public Command setSetpointCommand(Setpoint setpoint){
        return this.runOnce(
            () -> {
                switch(setpoint){
                    case FeederStation:
                        armCurrentTarget = ArmSetpoints.FeederStation;
                        wristCurrentTarget = WristSetpoints.FeederStation;
                        elevatorCurrentTarget = ElevatorSetpoints.FeederStation;
                        break;
                    case L1:
                        armCurrentTarget = ArmSetpoints.L1;
                        wristCurrentTarget = WristSetpoints.L1;
                        elevatorCurrentTarget = ElevatorSetpoints.L1;
                        break;
                    case L2:
                        armCurrentTarget = ArmSetpoints.L2;
                        wristCurrentTarget = WristSetpoints.L2;
                        elevatorCurrentTarget = ElevatorSetpoints.L2;
                        break;
                    case L3:
                        armCurrentTarget = ArmSetpoints.L3;
                        wristCurrentTarget = WristSetpoints.L3;
                        elevatorCurrentTarget = ElevatorSetpoints.L3;
                        break;
                    case L4:
                        armCurrentTarget = ArmSetpoints.L4;
                        wristCurrentTarget = WristSetpoints.L4;
                        elevatorCurrentTarget = ElevatorSetpoints.L4;
                        break;
                }
            });
    }

    public Command runIntakeCommand(){
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
    }

   public void periodic() {
    moveToSetpoint();
    zeroOnUserButton();

    // Display subsystem values
    
    //SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    //SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
   // SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
   }
}