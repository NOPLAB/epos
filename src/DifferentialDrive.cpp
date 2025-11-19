#include "DifferentialDrive.h"
#include <iostream>

DifferentialDrive::DifferentialDrive(unsigned short leftNodeId,
                                     unsigned short rightNodeId,
                                     int baseSpeed)
    : baseSpeed_(baseSpeed),
      leftNodeId_(leftNodeId),
      rightNodeId_(rightNodeId) {

    // Create motor controller instances with appropriate node IDs
    leftMotor_ = std::make_unique<EPOSController>(
        "EPOS4", "MAXON SERIAL V2", "USB", "USB0", 1000000, leftNodeId_);

    rightMotor_ = std::make_unique<EPOSController>(
        "EPOS4", "MAXON SERIAL V2", "USB", "USB0", 1000000, rightNodeId_);

    std::cout << "[DifferentialDrive] Created with left node ID: " << leftNodeId_
              << ", right node ID: " << rightNodeId_
              << ", base speed: " << baseSpeed_ << " RPM" << std::endl;
}

DifferentialDrive::~DifferentialDrive() {
    std::cout << "[DifferentialDrive] Destructor called, closing motors..." << std::endl;
    close();
}

bool DifferentialDrive::initialize() {
    std::cout << "[DifferentialDrive] Initializing motors..." << std::endl;

    // Initialize left motor
    if (!leftMotor_->initialize()) {
        std::cerr << "[DifferentialDrive] Failed to initialize left motor (Node ID: "
                  << leftNodeId_ << ")" << std::endl;
        std::cerr << "[DifferentialDrive] Error code: "
                  << leftMotor_->getLastErrorCode() << std::endl;
        return false;
    }
    std::cout << "[DifferentialDrive] Left motor initialized successfully" << std::endl;

    // Initialize right motor
    if (!rightMotor_->initialize()) {
        std::cerr << "[DifferentialDrive] Failed to initialize right motor (Node ID: "
                  << rightNodeId_ << ")" << std::endl;
        std::cerr << "[DifferentialDrive] Error code: "
                  << rightMotor_->getLastErrorCode() << std::endl;
        return false;
    }
    std::cout << "[DifferentialDrive] Right motor initialized successfully" << std::endl;

    // Activate velocity mode for both motors
    if (!leftMotor_->activateVelocityMode()) {
        std::cerr << "[DifferentialDrive] Failed to activate velocity mode for left motor"
                  << std::endl;
        return false;
    }

    if (!rightMotor_->activateVelocityMode()) {
        std::cerr << "[DifferentialDrive] Failed to activate velocity mode for right motor"
                  << std::endl;
        return false;
    }
    std::cout << "[DifferentialDrive] Velocity mode activated for both motors" << std::endl;

    // Set velocity profiles (acceleration and deceleration)
    unsigned int acceleration = 10000;  // RPM/s
    unsigned int deceleration = 10000;  // RPM/s

    if (!leftMotor_->setVelocityProfile(acceleration, deceleration)) {
        std::cerr << "[DifferentialDrive] Failed to set velocity profile for left motor"
                  << std::endl;
        return false;
    }

    if (!rightMotor_->setVelocityProfile(acceleration, deceleration)) {
        std::cerr << "[DifferentialDrive] Failed to set velocity profile for right motor"
                  << std::endl;
        return false;
    }
    std::cout << "[DifferentialDrive] Velocity profiles configured" << std::endl;

    std::cout << "[DifferentialDrive] Initialization complete!" << std::endl;
    return true;
}

bool DifferentialDrive::setMotorVelocities(int leftVelocity, int rightVelocity) {
    bool leftSuccess = leftMotor_->moveWithVelocity(leftVelocity);
    bool rightSuccess = rightMotor_->moveWithVelocity(rightVelocity);

    if (!leftSuccess) {
        std::cerr << "[DifferentialDrive] Failed to set left motor velocity to "
                  << leftVelocity << " RPM" << std::endl;
    }

    if (!rightSuccess) {
        std::cerr << "[DifferentialDrive] Failed to set right motor velocity to "
                  << rightVelocity << " RPM" << std::endl;
    }

    return leftSuccess && rightSuccess;
}

bool DifferentialDrive::moveForward(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    std::cout << "[DifferentialDrive] Moving forward at " << speed << " RPM" << std::endl;
    return setMotorVelocities(speed, speed);
}

bool DifferentialDrive::moveBackward(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    std::cout << "[DifferentialDrive] Moving backward at " << speed << " RPM" << std::endl;
    return setMotorVelocities(-speed, -speed);
}

bool DifferentialDrive::rotateLeft(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    std::cout << "[DifferentialDrive] Rotating left at " << speed << " RPM" << std::endl;
    // Left motor backward, right motor forward for in-place left rotation
    return setMotorVelocities(-speed, speed);
}

bool DifferentialDrive::rotateRight(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    std::cout << "[DifferentialDrive] Rotating right at " << speed << " RPM" << std::endl;
    // Left motor forward, right motor backward for in-place right rotation
    return setMotorVelocities(speed, -speed);
}

bool DifferentialDrive::curveLeft(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    int innerSpeed = speed / 2;  // Inner wheel (left) goes slower
    std::cout << "[DifferentialDrive] Curving left (left: " << innerSpeed
              << " RPM, right: " << speed << " RPM)" << std::endl;
    return setMotorVelocities(innerSpeed, speed);
}

bool DifferentialDrive::curveRight(int speed) {
    if (speed < 0) {
        speed = baseSpeed_;
    }
    int innerSpeed = speed / 2;  // Inner wheel (right) goes slower
    std::cout << "[DifferentialDrive] Curving right (left: " << speed
              << " RPM, right: " << innerSpeed << " RPM)" << std::endl;
    return setMotorVelocities(speed, innerSpeed);
}

bool DifferentialDrive::stop() {
    std::cout << "[DifferentialDrive] Stopping both motors" << std::endl;
    return setMotorVelocities(0, 0);
}

bool DifferentialDrive::getVelocities(int& leftVelocity, int& rightVelocity) {
    bool leftSuccess = leftMotor_->getVelocity(leftVelocity);
    bool rightSuccess = rightMotor_->getVelocity(rightVelocity);
    return leftSuccess && rightSuccess;
}

bool DifferentialDrive::getPositions(int& leftPosition, int& rightPosition) {
    bool leftSuccess = leftMotor_->getPosition(leftPosition);
    bool rightSuccess = rightMotor_->getPosition(rightPosition);
    return leftSuccess && rightSuccess;
}

bool DifferentialDrive::close() {
    std::cout << "[DifferentialDrive] Closing motors..." << std::endl;

    // Stop motors before closing
    stop();

    bool leftSuccess = leftMotor_->close();
    bool rightSuccess = rightMotor_->close();

    if (!leftSuccess) {
        std::cerr << "[DifferentialDrive] Failed to close left motor" << std::endl;
    }
    if (!rightSuccess) {
        std::cerr << "[DifferentialDrive] Failed to close right motor" << std::endl;
    }

    return leftSuccess && rightSuccess;
}

void DifferentialDrive::setBaseSpeed(int speed) {
    baseSpeed_ = speed;
    std::cout << "[DifferentialDrive] Base speed set to " << speed << " RPM" << std::endl;
}

int DifferentialDrive::getBaseSpeed() const {
    return baseSpeed_;
}
