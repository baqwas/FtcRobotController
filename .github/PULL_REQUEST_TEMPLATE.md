Before issuing a pull request, please see the contributing page.

---
name: TeamCode Contribution
about: Template for all new features and fixes to the TeamCode.
title: "[TYPE]: Brief, descriptive title of the change"
labels: enhancement
assignees: [Mentor or Lead Programmer]
---

## 🚀 Pull Request Summary

**This PR is for merging the branch `[branch name]` into `master`.**

### 🎯 Purpose
* **What is the goal of this change?** (e.g., Implement the Arm Subsystem, Fix a bug in Autonomous, Refactor motor initialization).
* **Why is this change necessary?** (e.g., The robot needs this new feature, The previous code was causing an error, We are improving readability).

---

## ⚙️ Detailed Changes

Please list all significant changes made in this PR.

- [ ] Added new file: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AprilTag.java`
- [ ] Modified OpMode: `BasicTeleOp` (now uses the new `HardwareClaw` class)
- [ ] Refactored method `initHardware()` in `RobotBase` for clarity.
- [ ] Fixed bug where `motorRight` was running in the wrong direction.

---

## ✅ Pre-Merge Checklist (Reviewer & Contributor)

The following items *must* be checked off before the PR can be merged into the `master` branch.

### Code and Logic Checks
- [ ] **Code compiles** without any errors or new warnings in Android Studio.
- [ ] **Naming conventions** were followed (e.g., PascalCase for Classes, camelCase for methods/variables).
- [ ] **Javadocs/Comments** are provided for all new classes and non-obvious methods.
- [ ] Code is formatted cleanly (Ctrl+Alt+L in Android Studio).

### Hardware & Testing Checks
- [ ] The code was **deployed to the Control Hub** and tested on the physical robot.
- [ ] **Tests run:**
    - [ ] TeleOp: Tested for [Duration] and verified [Key Functionality].
    - [ ] Autonomous: Verified [Autonomous Path/Action].
- [ ] The new or updated **Hardware Map configuration** on the Control Hub matches the names used in the code.

---

## 📸 Screenshots / Video (If Applicable)

*If this PR involves a visual change, like robot movement, an app change, or fixing a visual bug, please include a video or GIF.*

---

## 🔗 Related Issues

*If this PR closes a bug or implements a feature discussed elsewhere, link it here.*
* Closes # \[Issue Number]
* Relates to discussion in [Team Communication Channel/Thread Link]
