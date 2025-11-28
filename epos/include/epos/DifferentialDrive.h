#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "epos/EPOSController.h"
#include <memory>

/**
 * @class DifferentialDrive
 * @brief Class for controlling a differential drive robot with two EPOS4 motor controllers
 *
 * This class manages two EPOSController instances (left and right motors) and provides
 * high-level control methods for differential drive robot motion including forward/backward
 * movement, rotation, and curved paths.
 */
class DifferentialDrive {
public:
    /**
     * @brief Constructor - initializes two motor controllers
     * @param leftNodeId Node ID for left motor (default: 1)
     * @param rightNodeId Node ID for right motor (default: 2)
     * @param baseSpeed Base speed in RPM (default: 500)
     */
    DifferentialDrive(unsigned short leftNodeId = 1,
                      unsigned short rightNodeId = 2,
                      int baseSpeed = 500);

    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~DifferentialDrive();

    /**
     * @brief Initialize both motor controllers
     * @return true if both motors initialized successfully
     */
    bool initialize();

    /**
     * @brief Move forward at specified speed
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool moveForward(int speed = -1);

    /**
     * @brief Move backward at specified speed
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool moveBackward(int speed = -1);

    /**
     * @brief Rotate left (in-place rotation)
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool rotateLeft(int speed = -1);

    /**
     * @brief Rotate right (in-place rotation)
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool rotateRight(int speed = -1);

    /**
     * @brief Curve left (left motor slower, right motor faster)
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool curveLeft(int speed = -1);

    /**
     * @brief Curve right (right motor slower, left motor faster)
     * @param speed Speed in RPM (uses baseSpeed if not specified)
     * @return true if command successful
     */
    bool curveRight(int speed = -1);

    /**
     * @brief Stop both motors
     * @return true if command successful
     */
    bool stop();

    /**
     * @brief Get current velocities of both motors
     * @param leftVelocity Output parameter for left motor velocity (RPM)
     * @param rightVelocity Output parameter for right motor velocity (RPM)
     * @return true if successful
     */
    bool getVelocities(int& leftVelocity, int& rightVelocity);

    /**
     * @brief Get current positions of both motors
     * @param leftPosition Output parameter for left motor position (quadcounts)
     * @param rightPosition Output parameter for right motor position (quadcounts)
     * @return true if successful
     */
    bool getPositions(int& leftPosition, int& rightPosition);

    /**
     * @brief Close both motor controllers
     * @return true if both closed successfully
     */
    bool close();

    /**
     * @brief Set base speed for robot movements
     * @param speed Speed in RPM
     */
    void setBaseSpeed(int speed);

    /**
     * @brief Get current base speed
     * @return Base speed in RPM
     */
    int getBaseSpeed() const;

private:
    std::unique_ptr<EPOSController> leftMotor_;   ///< Left motor controller
    std::unique_ptr<EPOSController> rightMotor_;  ///< Right motor controller
    int baseSpeed_;                                ///< Base speed in RPM
    unsigned short leftNodeId_;                    ///< Node ID for left motor
    unsigned short rightNodeId_;                   ///< Node ID for right motor

    /**
     * @brief Set velocity for both motors
     * @param leftVelocity Left motor velocity (RPM)
     * @param rightVelocity Right motor velocity (RPM)
     * @return true if both commands successful
     */
    bool setMotorVelocities(int leftVelocity, int rightVelocity);
};

#endif // DIFFERENTIAL_DRIVE_H
