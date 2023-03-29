// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmForwardKinematicPosition;

public class Arm {
  private Shoulder m_shoulder = new Shoulder();
  private Elbow m_elbow = new Elbow();

  private double shoulderLength = ArmConstants.kShoulderLength;//meters
  private double elbowLength = ArmConstants.kElbowLength;

  private double q1; //units are wrong, convert to radians?? need to make them relative to ground
  private double q2;

  private double targetX;
  private double targetY;

  private double estimatedX;
  private double estimatedY;

  /** Creates a new Arm. */
  public Arm() {}

  private void setForwardKinematics(double shoulderAngle, double secondAngle) {
    m_shoulder.setTargetKinematicAngleRadians(shoulderAngle);
    m_elbow.setTargetKinematicAngleRadians(secondAngle);
  }

  private void setForwardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    m_shoulder.setTargetKinematicAngleRadians(forwardKinematicsPosition.getBaseAngleRadians());
    m_elbow.setTargetKinematicAngleRadians(forwardKinematicsPosition.getSecondAngleRadians());
  }

  public InstantCommand setForwardKinematicsCommand(ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new InstantCommand(() -> setForwardKinematics(forwardKinematicsPosition));
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
    calculateInverseKinematics(X, Y);
    //setInverseKinematics();
  }

  private void calculateQ2(double targetX, double targetY) {
    q2 = -Math.acos((shoulderLength*shoulderLength + elbowLength*elbowLength - targetX*targetX - targetY*targetY)/(-2*shoulderLength*elbowLength)) + 2*Math.PI;

    //other solution
    //q2 = Math.acos((-shoulderLength*shoulderLength - elbowLength*elbowLength + targetX*targetX + targetY*targetY)/(2*shoulderLength*elbowLength));
    SmartDashboard.putNumber("q2", Units.radiansToDegrees(q2));
  }

  private void calculateQ1(double targetX, double targetY) {
    // shoulderAngle = 
    //     Math.atan(targetX / targetY) -
    //     Math.atan((elbowLength * Math.sin(elbowAngle)) / (shoulderLength + elbowLength * Math.cos(elbowAngle)));

    q1 = -Math.atan2(targetY, targetX) - Math.atan((elbowLength*Math.sin(q2))/(shoulderLength + elbowLength*Math.cos(q2))) + Math.PI/2.0;
    SmartDashboard.putNumber("q1", Units.radiansToDegrees(q1));
  }

  private void calculateInverseKinematics(double targetX, double targetY) {
    calculateQ2(targetX, targetY);
    calculateQ1(targetX, targetY);
  }

  private void setInverseKinematics() {
    m_shoulder.setTargetKinematicAngleRadians(q1);
    m_elbow.setTargetKinematicAngleRadians(q2);
  }

  private void estimateCurrentXY() {
    estimatedX = shoulderLength*Math.sin(m_shoulder.getKinematicAngle()) + elbowLength*Math.sin(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle());
    estimatedY = shoulderLength*Math.cos(m_elbow.getKinematicAngle()) + elbowLength*Math.cos(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle());
    SmartDashboard.putNumber("Arm Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Arm Estimated Y", Units.metersToInches(estimatedY));
  }

  public void raiseCurrentPosition(double degrees) {
    if(m_elbow.getKinematicAngle() < 0) { //when second joint is -
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle()+Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle()-Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  public void lowerCurrentPosition(double degrees) {
    if(m_elbow.getKinematicAngle() > 0) { //when second joint is +
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle()+Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle()-Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  ///////////////////////////TRANSITIONS/////////////////////////////////////////////////////
  
  //Back to Back
  public Command BackToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_elbow.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToBackIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
      //new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()));
  }

  //Back to Transfer
  public Command BackToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    CommandBase sequence = new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
    sequence.addRequirements(m_shoulder, m_elbow);
    return sequence;
  }

  //Back to Front
  public Command BackToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Transfer to Back
  public Command TransferToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediatePosition)),
      //new WaitUntilCommand(() -> shoulder.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediate2Position)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Transfer to Front
  public Command TransferToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToFrontIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> shoulder.nearSetpoint() && elbow.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediatePosition)),
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediate2Position)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Front to Back
  public Command FrontToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediatePosition)),
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Front to Front
  public Command FrontToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Stow to Back
  public Command StowToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediatePosition)),
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //stow to front
  public Command StowToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  public void updateTelemetry() {
    // This method will be called once per scheduler run
    // estimateCurrentXY();
    // calculateInverseKinematics(estimatedX, estimatedY);
  }
}
