package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AutoControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * FTC DECODE Autonomous Program
 * 
 * Key Scoring Opportunities:
 * - LEAVE: Move off launch line (3 pts)
 * - CLASSIFIED ARTIFACTS: Score in GOAL through SQUARE to RAMP (3 pts each in AUTO)
 * - PATTERN: Match MOTIF colors on RAMP (2 pts per match in AUTO)
 * - BASE: Return to BASE ZONE at end (5-10 pts)
 * 
 * AprilTag IDs:
 * - Tag 20: Blue GOAL
 * - Tag 24: Red GOAL
 * - Tag 21-23: OBELISK (MOTIF indicators)
 */
@Autonomous(name="DECODE Main Auto", group="Competition")
public class MainAuto extends LinearOpMode {
    
    private AutoControl auto;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Alliance selection - CHANGE THIS BASED ON YOUR ALLIANCE
    private boolean isBlueAlliance = true; // Set to false for Red Alliance
    
    // AprilTag IDs
    private static final int BLUE_GOAL_TAG = 20;
    private static final int RED_GOAL_TAG = 24;
    private static final int MOTIF_TAG_GPP = 21;
    private static final int MOTIF_TAG_PGP = 22;
    private static final int MOTIF_TAG_PPG = 23;
    
    // Field positions (in mm) - adjust based on your field measurements
    private static final double LAUNCH_ZONE_X = 0;
    private static final double LAUNCH_ZONE_Y = 0;
    private static final double SPIKE_MARK_NEAR_X = 600;
    private static final double SPIKE_MARK_NEAR_Y = 300;
    private static final double GOAL_APPROACH_X = 1200;
    private static final double GOAL_APPROACH_Y = 600;
    private static final double BASE_ZONE_X = 100;
    private static final double BASE_ZONE_Y = 100;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize autonomous control
        auto = new AutoControl(this);
        auto.initialize();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        if (opModeIsActive()) {
            // Execute autonomous sequence
            runAutonomousSequence();
        }
        
        // Cleanup
        auto.cleanup();
    }
    
    /**
     * Main autonomous sequence
     */
    private void runAutonomousSequence() {
        telemetry.addData("Status", "Running Autonomous");
        telemetry.update();
        
        // Step 1: Detect MOTIF from OBELISK (optional but helpful)
        String motif = detectMotif();
        telemetry.addData("Detected MOTIF", motif);
        telemetry.update();
        
        // Step 2: LEAVE the launch line (3 points)
        leaveStartingPosition();
        
        // Step 3: Collect and score pre-loaded artifacts
        scorePreloadedArtifacts();
        
        // Step 4: Score additional artifacts from spike marks
        scoreSpikeMarkArtifacts();
        
        // Step 5: Attempt to create PATTERN on RAMP
        // (This requires precise shooting - adjust based on your robot's capabilities)
        attemptPatternScoring(motif);
        
        // Step 6: Return to BASE ZONE (5-10 points)
        returnToBase();
        
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
        telemetry.update();
    }
    
    /**
     * Detect the MOTIF by reading the OBELISK AprilTag
     * Returns: "GPP", "PGP", "PPG", or "UNKNOWN"
     */
    private String detectMotif() {
        telemetry.addData("Step", "Detecting MOTIF");
        telemetry.update();
        
        // Try to detect each MOTIF tag
        if (auto.isAprilTagDetected(MOTIF_TAG_GPP)) {
            return "GPP";
        } else if (auto.isAprilTagDetected(MOTIF_TAG_PGP)) {
            return "PGP";
        } else if (auto.isAprilTagDetected(MOTIF_TAG_PPG)) {
            return "PPG";
        }
        
        // If no tag detected, try rotating to find it
        auto.robot.controllerDrive(0, 0, 0.3, 1);
        sleep(1000);
        auto.robot.resetDrive();
        
        if (auto.isAprilTagDetected(MOTIF_TAG_GPP)) return "GPP";
        if (auto.isAprilTagDetected(MOTIF_TAG_PGP)) return "PGP";
        if (auto.isAprilTagDetected(MOTIF_TAG_PPG)) return "PPG";
        
        return "UNKNOWN";
    }
    
    /**
     * Step 1: Leave the starting position to earn LEAVE points (3 pts)
     */
    private void leaveStartingPosition() {
        telemetry.addData("Step", "Leaving launch line");
        telemetry.update();
        
        // Move forward off the launch line
        auto.moveTo(LAUNCH_ZONE_X + 400, LAUNCH_ZONE_Y, 0, 0.5, 2.0, 1.0);
        auto.wait(0.2);
    }
    
    /**
     * Step 2: Score pre-loaded artifacts (up to 3)
     * Each CLASSIFIED artifact = 3 points in AUTO
     */
    private void scorePreloadedArtifacts() {
        telemetry.addData("Step", "Scoring preloaded artifacts");
        telemetry.update();
        
        int goalTag = isBlueAlliance ? BLUE_GOAL_TAG : RED_GOAL_TAG;
        
        // Navigate to GOAL using AprilTag
        if (auto.navigateToAprilTag(goalTag, 24.0, 5.0)) {
            // Align with GOAL
            auto.alignWithAprilTag(goalTag, 2.0);
            
            // Score artifacts (adjust based on your robot's mechanism)
            auto.robot.initShooter();
            auto.wait(0.5);
            
            // Launch 3 preloaded artifacts
            for (int i = 0; i < 3; i++) {
                auto.robot.runIntake(); // Feed artifact to shooter
                auto.wait(0.3);
                auto.robot.stopIntake();
                auto.wait(0.5); // Wait between shots
            }
            
            auto.robot.shooter.setPower(0);
        } else {
            telemetry.addData("Warning", "Could not locate GOAL AprilTag");
            telemetry.update();
            
            // Fallback: Navigate to approximate GOAL position
            auto.moveTo(GOAL_APPROACH_X, GOAL_APPROACH_Y, 0, 0.6, 3.0, 1.0);
        }
    }
    
    /**
     * Step 3: Collect and score artifacts from spike marks
     */
    private void scoreSpikeMarkArtifacts() {
        telemetry.addData("Step", "Collecting spike mark artifacts");
        telemetry.update();
        
        // Navigate to nearest spike mark
        auto.moveTo(SPIKE_MARK_NEAR_X, SPIKE_MARK_NEAR_Y, 0, 0.5, 3.0, 1.0);
        
        // Run intake to collect artifacts
        auto.robot.runIntake();
        auto.wait(1.5);
        auto.robot.stopIntake();
        
        // Return to GOAL and score
        int goalTag = isBlueAlliance ? BLUE_GOAL_TAG : RED_GOAL_TAG;
        if (auto.navigateToAprilTag(goalTag, 24.0, 4.0)) {
            auto.robot.initShooter();
            auto.wait(0.3);
            
            // Launch collected artifacts
            auto.robot.runIntake();
            auto.wait(1.0);
            auto.robot.stopIntake();
            auto.robot.shooter.setPower(0);
        }
    }
    
    /**
     * Step 4: Attempt to create PATTERN on RAMP
     * This is advanced - requires precise shooting to match MOTIF
     */
    private void attemptPatternScoring(String motif) {
        telemetry.addData("Step", "Attempting PATTERN scoring");
        telemetry.addData("Target MOTIF", motif);
        telemetry.update();
        
        // This is a placeholder - implement based on your robot's capabilities
        // You would need to:
        // 1. Position robot for optimal shooting angle
        // 2. Adjust shooter power/angle for CLASSIFIED vs OVERFLOW
        // 3. Shoot artifacts in the correct color sequence
        
        // For now, just attempt to score more artifacts
        int goalTag = isBlueAlliance ? BLUE_GOAL_TAG : RED_GOAL_TAG;
        if (auto.alignWithAprilTag(goalTag, 2.0)) {
            // Shoot remaining artifacts
            auto.robot.initShooter();
            auto.wait(0.5);
            auto.robot.runIntake();
            auto.wait(1.0);
            auto.robot.stopIntake();
            auto.robot.shooter.setPower(0);
        }
    }
    
    /**
     * Step 5: Return to BASE ZONE for end-game points
     * Partially returned = 5 pts, Fully returned = 10 pts
     */
    private void returnToBase() {
        telemetry.addData("Step", "Returning to BASE");
        telemetry.update();
        
        // Navigate to BASE ZONE
        auto.moveTo(BASE_ZONE_X, BASE_ZONE_Y, 0, 0.7, 3.0, 1.0);
        
        // Ensure robot is fully in BASE ZONE
        auto.wait(0.5);
        auto.robot.resetDrive();
        
        telemetry.addData("Status", "In BASE ZONE");
        telemetry.update();
    }
}
