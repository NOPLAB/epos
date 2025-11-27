#include "EPOSController.h"
#include "Definitions.h"
#include <cstring>
#include <sstream>
#include <iomanip>

#define MMC_SUCCESS 0
#define MMC_FAILED 1

EPOSController::EPOSController(
    const std::string& deviceName,
    const std::string& protocolStackName,
    const std::string& interfaceName,
    const std::string& portName,
    unsigned int baudrate,
    unsigned short nodeId)
    : deviceName_(deviceName),
      protocolStackName_(protocolStackName),
      interfaceName_(interfaceName),
      portName_(portName),
      baudrate_(baudrate),
      nodeId_(nodeId),
      keyHandle_(nullptr),
      lastErrorCode_(0) {
}

EPOSController::~EPOSController() {
    if (keyHandle_ != nullptr) {
        close();
    }
}

void EPOSController::logError(const std::string& functionName, int result, unsigned int errorCode) {
    std::cerr << "EPOSController: " << functionName << " failed (result=" << result
              << ", errorCode=0x" << std::hex << errorCode << std::dec << ")" << std::endl;
    lastErrorCode_ = errorCode;
}

void EPOSController::logInfo(const std::string& message) {
    std::cout << "EPOSController: " << message << std::endl;
}

std::string EPOSController::getErrorString(unsigned int errorCode) {
    char errorInfo[512];
    VCS_GetErrorInfo(errorCode, errorInfo, 512);
    return std::string(errorInfo);
}

bool EPOSController::openDevice() {
    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, deviceName_.c_str());
    strcpy(pProtocolStackName, protocolStackName_.c_str());
    strcpy(pInterfaceName, interfaceName_.c_str());
    strcpy(pPortName, portName_.c_str());

    logInfo("Opening device...");

    unsigned int errorCode = 0;
    keyHandle_ = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);

    if (keyHandle_ != nullptr && errorCode == 0) {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if (VCS_GetProtocolStackSettings(keyHandle_, &lBaudrate, &lTimeout, &errorCode) != 0) {
            if (VCS_SetProtocolStackSettings(keyHandle_, baudrate_, lTimeout, &errorCode) != 0) {
                if (VCS_GetProtocolStackSettings(keyHandle_, &lBaudrate, &lTimeout, &errorCode) != 0) {
                    if (baudrate_ == lBaudrate) {
                        delete[] pDeviceName;
                        delete[] pProtocolStackName;
                        delete[] pInterfaceName;
                        delete[] pPortName;
                        logInfo("Device opened successfully");
                        return true;
                    }
                }
            }
        }
    } else {
        keyHandle_ = nullptr;
    }

    delete[] pDeviceName;
    delete[] pProtocolStackName;
    delete[] pInterfaceName;
    delete[] pPortName;

    logError("VCS_OpenDevice", MMC_FAILED, errorCode);
    return false;
}

bool EPOSController::prepareDevice() {
    unsigned int errorCode = 0;
    int isFault = 0;

    // Check fault state
    if (VCS_GetFaultState(keyHandle_, nodeId_, &isFault, &errorCode) == 0) {
        logError("VCS_GetFaultState", MMC_FAILED, errorCode);
        return false;
    }

    // Clear fault if necessary
    if (isFault) {
        std::stringstream msg;
        msg << "Clearing fault, node = " << nodeId_;
        logInfo(msg.str());

        if (VCS_ClearFault(keyHandle_, nodeId_, &errorCode) == 0) {
            logError("VCS_ClearFault", MMC_FAILED, errorCode);
            return false;
        }
    }

    // Check enable state
    int isEnabled = 0;
    if (VCS_GetEnableState(keyHandle_, nodeId_, &isEnabled, &errorCode) == 0) {
        logError("VCS_GetEnableState", MMC_FAILED, errorCode);
        return false;
    }

    // Enable if necessary
    if (!isEnabled) {
        if (VCS_SetEnableState(keyHandle_, nodeId_, &errorCode) == 0) {
            logError("VCS_SetEnableState", MMC_FAILED, errorCode);
            return false;
        }
    }

    logInfo("Device prepared successfully");
    return true;
}

bool EPOSController::initialize() {
    if (!openDevice()) {
        return false;
    }

    if (!prepareDevice()) {
        return false;
    }

    return true;
}

bool EPOSController::activatePositionMode() {
    unsigned int errorCode = 0;
    std::stringstream msg;
    msg << "Activating profile position mode, node = " << nodeId_;
    logInfo(msg.str());

    if (VCS_ActivateProfilePositionMode(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_ActivateProfilePositionMode", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::setPositionProfile(unsigned int velocity, unsigned int acceleration, unsigned int deceleration) {
    unsigned int errorCode = 0;

    if (VCS_SetPositionProfile(keyHandle_, nodeId_, velocity, acceleration, deceleration, &errorCode) == 0) {
        logError("VCS_SetPositionProfile", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::moveToPosition(long targetPosition, bool absolute, bool immediately) {
    unsigned int errorCode = 0;
    std::stringstream msg;
    msg << "Moving to position = " << targetPosition << ", node = " << nodeId_;
    logInfo(msg.str());

    if (VCS_MoveToPosition(keyHandle_, nodeId_, targetPosition, absolute ? 1 : 0, immediately ? 1 : 0, &errorCode) == 0) {
        logError("VCS_MoveToPosition", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::haltPositionMovement() {
    unsigned int errorCode = 0;
    logInfo("Halting position movement");

    if (VCS_HaltPositionMovement(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_HaltPositionMovement", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::activateVelocityMode() {
    unsigned int errorCode = 0;
    std::stringstream msg;
    msg << "Activating profile velocity mode, node = " << nodeId_;
    logInfo(msg.str());

    if (VCS_ActivateProfileVelocityMode(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_ActivateProfileVelocityMode", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::setVelocityProfile(unsigned int acceleration, unsigned int deceleration) {
    unsigned int errorCode = 0;

    if (VCS_SetVelocityProfile(keyHandle_, nodeId_, acceleration, deceleration, &errorCode) == 0) {
        logError("VCS_SetVelocityProfile", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::moveWithVelocity(long targetVelocity) {
    unsigned int errorCode = 0;
    std::stringstream msg;
    msg << "Moving with target velocity = " << targetVelocity << " rpm, node = " << nodeId_;
    logInfo(msg.str());

    if (VCS_MoveWithVelocity(keyHandle_, nodeId_, targetVelocity, &errorCode) == 0) {
        logError("VCS_MoveWithVelocity", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::haltVelocityMovement() {
    unsigned int errorCode = 0;
    logInfo("Halting velocity movement");

    if (VCS_HaltVelocityMovement(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_HaltVelocityMovement", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::enable() {
    unsigned int errorCode = 0;
    logInfo("Enabling device");

    if (VCS_SetEnableState(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_SetEnableState", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::disable() {
    unsigned int errorCode = 0;
    logInfo("Disabling device");

    if (VCS_SetDisableState(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_SetDisableState", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::clearFault() {
    unsigned int errorCode = 0;
    logInfo("Clearing fault");

    if (VCS_ClearFault(keyHandle_, nodeId_, &errorCode) == 0) {
        logError("VCS_ClearFault", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::getFaultState(bool& isFault) {
    unsigned int errorCode = 0;
    int fault = 0;

    if (VCS_GetFaultState(keyHandle_, nodeId_, &fault, &errorCode) == 0) {
        logError("VCS_GetFaultState", MMC_FAILED, errorCode);
        return false;
    }

    isFault = (fault != 0);
    return true;
}

bool EPOSController::getEnableState(bool& isEnabled) {
    unsigned int errorCode = 0;
    int enabled = 0;

    if (VCS_GetEnableState(keyHandle_, nodeId_, &enabled, &errorCode) == 0) {
        logError("VCS_GetEnableState", MMC_FAILED, errorCode);
        return false;
    }

    isEnabled = (enabled != 0);
    return true;
}

bool EPOSController::getPosition(int& position) {
    unsigned int errorCode = 0;

    if (VCS_GetPositionIs(keyHandle_, nodeId_, &position, &errorCode) == 0) {
        logError("VCS_GetPositionIs", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::getVelocity(int& velocity) {
    unsigned int errorCode = 0;

    if (VCS_GetVelocityIs(keyHandle_, nodeId_, &velocity, &errorCode) == 0) {
        logError("VCS_GetVelocityIs", MMC_FAILED, errorCode);
        return false;
    }

    return true;
}

bool EPOSController::isTargetReached(bool& targetReached) {
    unsigned int errorCode = 0;
    int reached = 0;

    if (VCS_GetMovementState(keyHandle_, nodeId_, &reached, &errorCode) == 0) {
        logError("VCS_GetMovementState", MMC_FAILED, errorCode);
        return false;
    }

    targetReached = (reached != 0);
    return true;
}

bool EPOSController::close() {
    if (keyHandle_ == nullptr) {
        return true;
    }

    unsigned int errorCode = 0;
    logInfo("Closing device");

    if (VCS_CloseDevice(keyHandle_, &errorCode) != 0 && errorCode == 0) {
        keyHandle_ = nullptr;
        return true;
    }

    logError("VCS_CloseDevice", MMC_FAILED, errorCode);
    return false;
}
