# Robot Software Task Documentation

This document provides a detailed overview of the robot software tasks and setup process for the team.  
It explains how to prepare the development environment, structure the project, and manage subsystem assignments.

---

## 🧭 Overview
This document serves as a guide for new and existing team members working on the robot’s software.  
It outlines how to:
- Set up the required tools and dependencies  
- Clone and manage the project repository  
- Understand the folder and code structure  
- Collaborate effectively during development

---

## ⚙️ Initial Setup

### 1. Install WPILib
Download and install the latest **WPILib** development environment:
- Visit [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html)
- Make sure `VS Code` and the **FRC Plugin** are correctly configured.

### 2. Fork the Main Swerve Template
- Go to the official **Main Swerve Template** repository on GitHub.  
- Click **Fork** to create your own copy.  
- Clone it locally to your development machine using:
  ```bash
  git clone https://github.com/Kelrot/junior-software-training-template.git
  ```

### 3. Open in VS Code
Open the cloned folder in **VS Code (WPILib)** and ensure the Gradle build completes without errors.

---

## 🧩 Project Structure

The project is organized into the following main sections:

| Folder / Module | Description |
| ---------------- | ------------ |
| `src/main/java/frc/robot/` | Root directory for robot code |
| `subsystems/` | Code for robot mechanisms (Drive, Elevator, Intake, etc.) |
| `commands/` | Command-based logic controlling subsystems |
| `utils/` | Utility classes such as wrappers, logging, or math helpers |


---

## 🧠 Development Guidelines

- Follow WPILib command-based design principles.  
- Use constants for tunable parameters — avoid hardcoded values in logic.  
- Commit frequently with meaningful messages.  
- Use pull requests for reviewing and merging changes.  
- Test each subsystem independently before full robot integration.

---

## 🧪 Simulation & Testing

1. Use **WPILib Simulation** to test robot behavior virtually.  
2. Verify motor direction, PID control, and trigger mappings.  
3. Once stable, deploy to the robot and validate physical performance.  
4. Document results in the shared Google Docs file for tracking progress.
