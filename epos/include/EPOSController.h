#ifndef EPOS_CONTROLLER_H
#define EPOS_CONTROLLER_H

#include <string>
#include <iostream>

class EPOSController {
public:
    // Constructor
    EPOSController(
        const std::string& deviceName = "EPOS4",
        const std::string& protocolStackName = "MAXON SERIAL V2",
        const std::string& interfaceName = "USB",
        const std::string& portName = "USB0",
        unsigned int baudrate = 1000000,
        unsigned short nodeId = 1
    );

    // Destructor
    ~EPOSController();

    // Initialization
    bool initialize();

    // Position Control (Profile Position Mode)
    bool activatePositionMode();
    bool moveToPosition(long targetPosition, bool absolute = true, bool immediately = true);
    bool haltPositionMovement();
    bool setPositionProfile(unsigned int velocity, unsigned int acceleration, unsigned int deceleration);

    // Velocity Control (Profile Velocity Mode)
    bool activateVelocityMode();
    bool moveWithVelocity(long targetVelocity);
    bool haltVelocityMovement();
    bool setVelocityProfile(unsigned int acceleration, unsigned int deceleration);

    // State Management
    bool enable();
    bool disable();
    bool clearFault();
    bool getFaultState(bool& isFault);
    bool getEnableState(bool& isEnabled);

    // Motion Info
    bool getPosition(int& position);
    bool getVelocity(int& velocity);
    bool isTargetReached(bool& targetReached);

    // Device Control
    bool close();

    // Error handling
    unsigned int getLastErrorCode() const { return lastErrorCode_; }
    std::string getErrorString(unsigned int errorCode);

private:
    // Device parameters
    std::string deviceName_;
    std::string protocolStackName_;
    std::string interfaceName_;
    std::string portName_;
    unsigned int baudrate_;
    unsigned short nodeId_;

    // Device handle
    void* keyHandle_;

    // Error tracking
    unsigned int lastErrorCode_;

    // Helper functions
    void logError(const std::string& functionName, int result, unsigned int errorCode);
    void logInfo(const std::string& message);
    bool openDevice();
    bool prepareDevice();
};

#endif // EPOS_CONTROLLER_H
