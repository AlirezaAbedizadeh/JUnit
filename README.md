# Autopilot System - JUnit 5 Test Suite

## Overview
This repository contains a comprehensive **testing suite** developed for an Autonomous Vehicle Autopilot system. The primary objective of this project is software quality assurance and testing, utilizing **JUnit 5** to validate the safety, reliability, and correctness of the vehicle's core control modules. 

*(Note: The core autopilot system logic was provided; this project focuses exclusively on designing, implementing, and executing the test cases).*

## System Under Test (SUT)
The testing suite evaluates the following components of the autopilot architecture:
* **Core Controllers**: Validating the decision-making logic of `AutoDriver`, `AdaptiveCruise`, `BrakeController`, `SteeringController`, and `GearBox`.
* **Sensor Integration**: Testing the system's response to environmental data from `LaneSensor`, `ObstacleSensor`, and `SpeedSensor` via `SensorSnapshot`.
* **Exception Handling**: Ensuring system stability and safe degradation during anomalies (e.g., verifying `IllegalGearChangeException` and `SensorFailureException` are thrown correctly).

## Testing Strategies Implemented
* **Comprehensive Unit Testing**: Thorough test coverage for individual control components using JUnit 5 (Jupiter).
* **Parameterized Testing**: Data-driven testing utilizing CSV datasets (e.g., `steering-cases.csv`) and `TestInputs` to evaluate multiple edge cases, boundary conditions, and complex input scenarios efficiently.
* **Safety-Critical Validation**: Simulating hardware failures (e.g., sensor disconnections) and invalid state transitions to guarantee the system handles faults safely.

## Technologies Used
* **Language**: Java
* **Testing Framework**: JUnit 5 (Jupiter)
* **Build Tool**: Maven

## Getting Started

### Prerequisites
* Java Development Kit (JDK) 11 or higher
* Maven

### Running the Tests
To execute the complete test suite and view the test reports, navigate to the project root directory and run:
```bash
mvn clean test
