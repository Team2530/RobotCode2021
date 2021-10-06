// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {
  private static Pixy2 pixy;
  private static GameBall gameBall = new GameBall();

  public static void initialize() {
    pixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink
    pixy.init(); // Initializes the camera and prepares to send/receive data
    gameBall.lastBlockCount = 0;
    pixy.setLamp((byte) 1, (byte) 1);
  }

  public int getX() {
    return gameBall.ballX;
  }

  public int getY() {
    return gameBall.ballY;
  }
  
  @Override
  public void periodic() {

    // Update the pixy data
    Pixy.updateGameBall();
    System.out.println(String.format("Latest Game Ball Detected:\n\n%s", gameBall));
    // Uses the Pixy data to do things
    /*
     * if ( <button pressed> ) { this.driveTrain = driveTrain; this.shooter =
     * shooter; this.elevator = elevator; //take over drivetrain, elevator, and
     * shooter //stuff using above variables }
     */
  }

  public static GameBall updateGameBall() {
    // Gets the number of "blocks", identified targets, that match signature 1 on
    // the Pixy2,
    // does not wait for new data if none is available,
    // and limits the number of returned blocks to 25, for a slight increase in
    // efficiency
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			gameBall = new GameBall(); // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
    }
    gameBall.loadedBlock = largestBlock;
    gameBall.ballWidth = largestBlock.getWidth();
    gameBall.ballHeight = largestBlock.getHeight();
    gameBall.ballX = largestBlock.getX();
    gameBall.ballY = largestBlock.getY();
    gameBall.ballAngle = largestBlock.getAngle();
    gameBall.ballAge = largestBlock.getAge();
    gameBall.lastBlockCount = blockCount;
    return gameBall;
  }

  public double getArea() {
    return gameBall.ballWidth * gameBall.ballHeight;
  }
}

/*
Properties of our "Yellow Ball" (Game Ball)
ball height, width ...
*/
class GameBall {
  GameBall() {
    ballWidth = -1;
    ballHeight = -1;
    ballX = -1;
    ballY = -1;
    ballAngle = -1;
    ballAge = -1;
    lastBlockCount = -1;
  }
  Block loadedBlock;
  int ballWidth;
  int ballHeight;
  int ballX;
  int ballY;
  int ballAngle;
  int ballAge;
  int lastBlockCount;
}
