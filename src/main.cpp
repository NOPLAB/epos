#include "EPOSController.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

// Global flag for signal handling
std::atomic<bool> g_running(true);

// Current target velocity
std::atomic<int> g_targetVelocity(0);

/**
 * @brief Signal handler for clean shutdown
 */
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\n[SIGNAL] Caught Ctrl+C, shutting down..." << std::endl;
    g_running = false;
  }
}

/**
 * @brief Configure terminal for non-blocking keyboard input
 * @param oldt Output parameter to store old terminal settings
 */
void configureTerminal(struct termios &oldt) {
  struct termios newt;

  // Get current terminal settings
  tcgetattr(STDIN_FILENO, &oldt);

  // Copy to new settings
  newt = oldt;

  // Disable canonical mode and echo
  newt.c_lflag &= ~(ICANON | ECHO);

  // Set minimum characters to read and timeout
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;

  // Apply new settings
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // Set stdin to non-blocking mode
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

/**
 * @brief Restore terminal to original settings
 * @param oldt Original terminal settings to restore
 */
void restoreTerminal(const struct termios &oldt) {
  // Restore original terminal settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  // Remove non-blocking flag
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

/**
 * @brief Clear the terminal screen
 */
void clearScreen() { std::cout << "\033[2J\033[1;1H"; }

/**
 * @brief Display current motor status
 */
void displayStatus(EPOSController &motor, int targetVel) {
  int currentVel = 0;
  int currentPos = 0;

  motor.getVelocity(currentVel);
  motor.getPosition(currentPos);

  clearScreen();

  std::cout << "========================================" << std::endl;
  std::cout << "    EPOS4 Motor Controller (Node 2)    " << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  std::cout << "Motor Status:" << std::endl;
  std::cout << "  Current Velocity: " << currentVel << " RPM" << std::endl;
  std::cout << "  Target Velocity:  " << targetVel << " RPM" << std::endl;
  std::cout << "  Position:         " << currentPos << " qc" << std::endl;
  std::cout << std::endl;

  std::cout << "Controls:" << std::endl;
  std::cout << "  W - Increase Speed (+100 RPM)" << std::endl;
  std::cout << "  S - Decrease Speed (-100 RPM)" << std::endl;
  std::cout << "  A - Reverse (-500 RPM)" << std::endl;
  std::cout << "  D - Forward (+500 RPM)" << std::endl;
  std::cout << "  SPACE - Stop (0 RPM)" << std::endl;
  std::cout << "  ESC - Exit" << std::endl;
  std::cout << std::endl;

  std::cout << "========================================" << std::endl;
  std::cout.flush();
}

int main(int argc, const char **argv) {
  // Set up signal handler for Ctrl+C
  signal(SIGINT, signalHandler);

  // Create motor controller for Node ID 2
  EPOSController motor("EPOS4", "MAXON SERIAL V2", "USB", "USB0", 1000000, 2);

  // Initialize motor
  std::cout << "Initializing EPOS4 Motor Controller (Node 2)..." << std::endl;
  if (!motor.initialize()) {
    std::cerr << "Failed to initialize motor. Error code: "
              << motor.getLastErrorCode() << std::endl;
    return 1;
  }

  // Activate velocity mode
  std::cout << "Activating velocity mode..." << std::endl;
  if (!motor.activateVelocityMode()) {
    std::cerr << "Failed to activate velocity mode. Error code: "
              << motor.getLastErrorCode() << std::endl;
    motor.close();
    return 1;
  }

  // Set velocity profile (acceleration and deceleration)
  std::cout << "Setting velocity profile..." << std::endl;
  if (!motor.setVelocityProfile(1000, 1000)) {
    std::cerr << "Failed to set velocity profile. Error code: "
              << motor.getLastErrorCode() << std::endl;
    motor.close();
    return 1;
  }

  std::cout << "Motor initialized successfully!" << std::endl;
  std::cout << "Starting keyboard control..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Configure terminal for non-blocking input
  struct termios oldt;
  configureTerminal(oldt);

  // Main control loop
  char key = 0;
  auto lastDisplayTime = std::chrono::steady_clock::now();
  const auto displayInterval =
      std::chrono::milliseconds(100); // Update display every 100ms

  while (g_running) {
    // Read keyboard input (non-blocking)
    key = 0;
    ssize_t bytesRead = read(STDIN_FILENO, &key, 1);

    if (bytesRead > 0) {
      int newVelocity = g_targetVelocity;

      // Process keyboard input
      switch (key) {
      case 'w':
      case 'W':
        // Increase speed by 100 RPM
        newVelocity += 100;
        if (newVelocity > 3000) newVelocity = 3000;  // Limit max speed
        g_targetVelocity = newVelocity;
        motor.moveWithVelocity(newVelocity);
        break;

      case 's':
      case 'S':
        // Decrease speed by 100 RPM
        newVelocity -= 100;
        if (newVelocity < -3000) newVelocity = -3000;  // Limit min speed
        g_targetVelocity = newVelocity;
        motor.moveWithVelocity(newVelocity);
        break;

      case 'a':
      case 'A':
        // Set reverse speed
        newVelocity = -500;
        g_targetVelocity = newVelocity;
        motor.moveWithVelocity(newVelocity);
        break;

      case 'd':
      case 'D':
        // Set forward speed
        newVelocity = 500;
        g_targetVelocity = newVelocity;
        motor.moveWithVelocity(newVelocity);
        break;

      case ' ': // Space
        // Stop motor
        newVelocity = 0;
        g_targetVelocity = newVelocity;
        motor.moveWithVelocity(0);
        break;

      case 27: // ESC key
        std::cout << "\nESC pressed, exiting..." << std::endl;
        g_running = false;
        break;

      default:
        // Ignore other keys
        break;
      }
    }

    // Update display at regular intervals
    auto now = std::chrono::steady_clock::now();
    if (now - lastDisplayTime >= displayInterval) {
      displayStatus(motor, g_targetVelocity);
      lastDisplayTime = now;
    }

    // Small sleep to prevent CPU spinning
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Cleanup
  std::cout << "\nShutting down motor..." << std::endl;
  motor.moveWithVelocity(0);  // Stop motor
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  motor.close();

  // Restore terminal settings
  restoreTerminal(oldt);

  clearScreen();
  std::cout << "Motor Controller terminated successfully." << std::endl;

  return 0;
}
