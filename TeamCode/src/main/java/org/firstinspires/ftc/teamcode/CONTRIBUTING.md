# Contributions Welcome 
We welcome contributions from every member of our team! These guidelines are in place to ensure a smooth, collaborative process and keep our code clean, consistent, and easy to maintain.

1. Code Management and Branching

We use a simple **branching model** to manage changes:
 1. `master` Branch (The "Gold" Standard): 
     * This branch *must always be stable* and runnable on the robot.
     * NEVER commit directly to master. All changes must go through a Pull Request. 
     * Code in this branch is deployed to the Robot Controller for competitions.
 2. Feature/Development Branches: 
    * For any new feature, bug fix, or experiment, create a new branch off of master. 
    * Naming Convention: Use `feature/[short-description]` or `fix/[short-description]`.
      * _Good Examples_: `feature/intake`, `fix/slide-speed`, `feature/path-planning`.

### The Contribution Workflow 
* **Pull**: Before starting, always run `git pull origin master` to get the latest stable code. 
* **Branch**: Create your new branch: `git checkout -b feature/your-name-and-task`.  
* **Code**: Write your code, following the conventions below. Commit your work frequently.
* **Push**: Push your branch to GitHub: `git push origin feature/your-name-and-task`. 
* **Pull Request (PR)**: Open a PR to merge your branch back into `master`. 

2. Naming Conventions 
Consistency is key for readability. Please follow these Java conventions in all your code: 

| Element |	Convention | Example                                      | Description | 
|---------|------------|----------------------------------------------|-------------| 
| **Classes** (OpModes, Hardware) | PascalCase | `BasicTeleOp, RobotDriveTrain`               | Capitalize the first letter of every word. | 
| **Methods/Functions** | camelCase | `initHardware(), setMotorPower()`            | Start with lowercase, capitalize subsequent words. | 
| **Variables** (General) | camelCase | `motorLeftFront1` `sensorDistance`           | Keep variable names descriptive. | 
| **Constants** (final) | UPPER_SNAKE_CASE | `DRIVE\_GEAR\_RATIO`, `CLAW\_OPEN\_POSITION` | Use all caps with underscores. | 
| **Hardware Map Names** | camelCase | `motorFrontLeft`, `imu`                     | Crucial: These names must exactly match the configuration file on the Control Hub. | 

3. Pull Request (PR) Process 
A Pull Request is how your code is reviewed and approved before becoming part of the stable master branch.

📝 What to Include in a PR
Every Pull Request description should contain the following: 
1. Summary: A one-sentence description of the change (e.g., "Adds the final code for the slide motor."). 
2. What does this PR do? (Detailed list) 
   * [ ] Implements feature X.
   * [ ] Fixes bug Y.
   * [ ] Updates documentation for Z.
3. Testing Completed: 
   * Describe how you tested the code. (e.g., "_Tested teleop driving for 5 minutes, no exceptions thrown._" or "_Verified the autonomous route runs from start to finish on the field._")

### ✅ Code Review 
* Before a PR can be merged into master, it must be reviewed and approved by at least one other team member. 
* The reviewer will check for functionality (does it work?), style (does it follow naming conventions?), and safety (will this damage the robot?). 
* Once approved and all required checks (if any) have passed, the PR can be merged by the reviewer or lead programmer. 

4. Documentation and Comments
* **Javadocs**: Every custom class and every major method should have a Javadoc comment explaining its purpose, inputs, and outputs. 
* **Inline Comments**: Use comments to explain non-obvious logic or to explain why a certain decision was made (e.g., // We use 0.8 power here to prevent motor stalling on the incline.). Avoid commenting code that is self-explanatory. 
* **Update Configuration**: If you add or rename any hardware component, you must update the corresponding entry in our physical robot configuration file on the Control Hub and make a note in the PR description.5. 

### ⚙️ Javadoc Template Example (Class Level) 
The class Javadoc should introduce the component, state its purpose, and list any hardware or configuration dependencies (which act as the "inputs" for the class itself). 

```
/**
 * **Purpose:** A unified control class for the robot's drive system,
 * encapsulating four motors and providing high-level movement methods.
 * This class abstracts away raw motor control, allowing OpModes to
 * focus only on desired robot behavior (e.g., drive, strafe, turn).
 *
 * **Inputs (Dependencies/Hardware):**
 * - Requires four configured DC motors: 'motorLeftFront', 'motorLeftBack', 'motorRightFront', 'motorRightBack'.
 * - Relies on the FTC 'HardwareMap' provided by the OpMode.
 * - (Optional) May depend on an internal 'IMU' sensor for accurate turning.
 *
 * **Outputs (Key Functionality):**
 * - Provides simple methods for tank drive or mechanized wheel drive.
 * - Manages motor run modes and power scaling.
 *
 * @author [Your Name or Team]
 * @version 1.0
 */
public class RobotDriveTrain {
    // ... class code here
}
 ``` 
### 🔩 Javadoc Template Example (Method Level)
Method Javadocs are where the standard tags like @param (input) and @return (output) truly shine.

```
/**
 * **Purpose:** Executes a straight-line autonomous movement using motor encoders.
 * The method blocks execution until the target distance is reached.
 *
 * @param distanceInInches The total distance the robot should travel forward
 * (positive) or backward (negative), measured in inches.
 * @param drivePower       The motor power level (from 0.0 to 1.0) to apply during
 * the movement. Use lower power for greater accuracy.
 *
 * @return **True** if the movement completed successfully; **False** if the
 * OpMode was stopped or an interrupt occurred before completion.
 */
public boolean driveStraight(double distanceInInches, double drivePower) {
    // ... method code here
    return true;
}
```
