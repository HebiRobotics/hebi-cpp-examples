/**
 * This file demonstrates a simple teach repeat example with a 3 DOF arm.
 */

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

/// Macros needed for getting keyboard input without waiting user needing to press <enter>
#if defined(_WIN32) || defined(_WIN64) // On Windows
    #include <conio.h>
    #define hebi_getchar_init()
    #define hebi_getchar() _getch()
#elif defined(__APPLE__) || defined(TARGET_OS_OSX) // On OSX
/// TODO - figure out how to do non-blocking I/O on stdin on OSX
/// For now, you will need to press <enter> after a key to get `getchar` to return.
    #warning "non-blocking I/O is not implemented on OSX currently."
    #warning "You will need to press the <enter> key to leave/enter learning/playback mode."
    #define hebi_getchar_init()
    #define hebi_getchar() getchar()
#else // On Linux
    #define hebi_getchar_init() system ("/bin/stty raw")
    #define hebi_getchar() getchar()
#endif

#define HEBI_ENTER_LEARNING_MODE 'l'
#define HEBI_CAPTURE_WAYPOINT ' '
#define HEBI_WAYPOINT_PLAYBACK 'p'

#include <atomic>
#include <iostream> /// std::cout and std::cerr
#include <thread> // std::this_thread and std::thread

/// For printing string vectors
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& vec) {
    os << '{';
    size_t size = vec.size();
    for (size_t i = 0; i < size - 1; i++) {
        os << '"' << vec[i] << "\", ";
    }
    if (size > 0) {
        os << '"' << vec[size - 1] << '"';
    }
    return os << '}';
}

namespace hebi {
namespace example {

static std::atomic_bool zero_torque; /// Flag which determines whether or not to send zeroing torque commands
static std::atomic_bool quit_running; /// Flag used to tell background thread when to exit

/**
 * Structure of initialization parameters.
 */
struct CreateDesc {
    std::vector<std::string> moduleNames;
    std::vector<std::string> familyNames;
    std::vector<double> homePosition;       /// Home Position of the arm
    int commandLifetime;                    /// Lifetime of command (in milliseconds)
    double waypointTransitionTime;          /// Time (in seconds) to move from one waypoint to the next
    double feedbackFrequency;               /// Feedback frequency (in Hertz)
};

/**
 * Fetches the group from the provided description
 */
static bool createGroup(CreateDesc& createDesc, std::shared_ptr<Group>& group);

/**
 * Zeros the torques of the provided group. This makes moving the arm a lot easier (i.e. there is less resistance).
 */
static void sendZeroTorques(Group* group);

/**
 * Move through all the waypoints captured by the provided trajectory.
 * 
 * This function will move the arm from its position at the time this function is called to the first waypoint.
 */ 
static void moveThroughWayPoints(Group* group, trajectory::Trajectory* trajectory);

/**
 * Moves the arm from its current position to the provided waypoint in the provided time.
 */
static void moveToWayPoint(Group* group, Eigen::VectorXd* endPoint, double timeDuration);

/**
 * Records waypoints requested by user and returns a calculated trajectory to move through them.
 * 
 * @param transitionTime the time (in seconds) between each waypoint in the calculated trajectory
 */
static std::shared_ptr<hebi::trajectory::Trajectory> learnMotion(hebi::Group* group, double transitionTime);

static bool createGroup(CreateDesc& createDesc, std::shared_ptr<Group>& group) {
    Lookup lookup;
    group = lookup.getGroupFromNames(createDesc.familyNames, createDesc.moduleNames);

    if (group) {
        group->setCommandLifetimeMs(createDesc.commandLifetime);
        group->setFeedbackFrequencyHz(createDesc.feedbackFrequency);
        return true;
    }

    std::cerr << "Lookup::getGroupFromNames("
        << createDesc.moduleNames << ", "
        << createDesc.familyNames << ")" << " returned null" << std::endl;
    return false;
}

static void moveThroughWayPoints(Group* group, trajectory::Trajectory* trajectory) {
    const int numberOfModules = group->size();
    const double duration = trajectory->getDuration();

    GroupCommand cmd(numberOfModules);
    Eigen::VectorXd posCmd(numberOfModules);
    Eigen::VectorXd velCmd(numberOfModules);
    Eigen::VectorXd trqCmd(numberOfModules);

    /// NOTE: First thing to do is move the arm from its current position to the position of the first waypoint
    trajectory->getState(trajectory->getStartTime(), &posCmd, nullptr, nullptr);

    moveToWayPoint(
        group,
        &posCmd,
        duration / static_cast<double>(trajectory->getWaypointCount())
        /// timeDuration = duration to move through a single waypoint in the trajectory 
        /// This way, moving to the first waypoint is at the same speed as the subsequent
        /// waypoint to waypoint motion. 
    );

    const double period = 1.0 / static_cast<double>(group->getFeedbackFrequencyHz());
    auto waitMicros = static_cast<long long>(period * 1000.0 * 1000.0);

    for (double t = 0.0; t < duration; t += period) {
        trajectory->getState(t, &posCmd, &velCmd, &trqCmd);
        cmd.setPosition(posCmd);
        group->sendCommand(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(waitMicros));
    }
}

static void moveToWayPoint(Group* group, Eigen::VectorXd* endPoint, double timeDuration) {
    const int numberOfModules = group->size();
    GroupFeedback feedback(numberOfModules);
    GroupCommand cmd(numberOfModules);
    Eigen::VectorXd posCmd(numberOfModules);
    Eigen::VectorXd velCmd(numberOfModules);
    Eigen::VectorXd trqCmd(numberOfModules);

    if (!group->sendFeedbackRequest()) {
      return;
    }

    if (!group->getNextFeedback(feedback)) {
        return;
    }

    const size_t numberOfWaypoints = 2;
    Eigen::VectorXd time(numberOfWaypoints);
    time[0] = 0.0;
    time[1] = timeDuration;

    Eigen::MatrixXd positions(numberOfModules, numberOfWaypoints);
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(numberOfModules, numberOfWaypoints);
    Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(numberOfModules, numberOfWaypoints);

    Eigen::VectorXd currentPosition = feedback.getPosition();
    positions.col(0) = currentPosition;
    positions.col(1) = *endPoint;

    auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocity, &torque);

    const double duration = trajectory->getDuration();
    const double period = 1.0 / static_cast<double>(group->getFeedbackFrequencyHz());
    auto waitMicros = static_cast<long long>(period * 1000.0 * 1000.0);

    for (double t = 0.0; t < duration; t += period) {
        trajectory->getState(t, &posCmd, &velCmd, &trqCmd);
        cmd.setPosition(posCmd);
        group->sendCommand(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(waitMicros));
    }
}

static std::shared_ptr<hebi::trajectory::Trajectory> learnMotion(hebi::Group* group, double transitionTime) {
    std::cout
        << std::endl << "In learning mode."
        << std::endl << "Record a waypoint by pressing space bar."
        << std::endl << "Press 'p' to return to playback mode." << std::endl << std::endl;

    /// We want to zero torque while in learning mode
    zero_torque = true;
    const size_t numberOfModules = group->size();
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd waypointTimes;
    hebi::GroupFeedback feedback(numberOfModules);

    while (true) {
        char ch = hebi_getchar();
        switch (ch) {
            case HEBI_CAPTURE_WAYPOINT: {
                if (!group->sendFeedbackRequest()) {
                  std::cerr << "Group::sendFeedbackRequest failed" << std::endl;
                  continue;
                }

                if (!group->getNextFeedback(feedback)) {
                    std::cerr << "Group::getNextFeedback failed" << std::endl;
                    continue;
                }

                std::cout << "Recorded waypoint." << std::endl;
                waypoints.push_back(feedback.getPosition());
                break;
            }
            case HEBI_WAYPOINT_PLAYBACK: {
                if (waypoints.size() < 2) {
                    std::cerr << "You need to record at least 2 waypoints before returning to playback mode." << std::endl;
                    continue;
                }

                const size_t numberOfWaypoints = waypoints.size();
                Eigen::MatrixXd currentWaypoints(numberOfModules, numberOfWaypoints);
                waypointTimes.resize(numberOfWaypoints);

                for (size_t i = 0; i < numberOfWaypoints; i++) {
                    waypointTimes[i] = transitionTime * static_cast<double>(i);
                    currentWaypoints.col(i) = waypoints[i];
                }

                std::cout
                    << "Recorded " << numberOfWaypoints << " waypoints."
                    << std::endl << "Leaving learning mode." << std::endl << std::endl;
                 
                /// Since we will be playing back, don't zero torque.
                zero_torque = false;
                return hebi::trajectory::Trajectory::createUnconstrainedQp(waypointTimes, currentWaypoints, nullptr, nullptr);
            }
        }
    }
}

} // namespace example
} // namespace hebi

/**
 * Runs on background thread to keep torque zero.
 * This makes the arm easier to move (and play back though its waypoints)
 */
static void zeroTorqueProc(hebi::Group* group) {
    auto waitMicros = static_cast<long long>(5 * 1000); /// Zero torque every 5 milliseconds
    auto numberOfModules = group->size();
    hebi::GroupCommand command(numberOfModules);
    Eigen::VectorXd trq = Eigen::VectorXd::Zero(numberOfModules);

    while(true) {
        if (hebi::example::quit_running) {
            return;
        } else if (hebi::example::zero_torque) {
            command.setEffort(trq);
            group->sendCommand(command);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(waitMicros));
    }
}

int main(int argc, char* argv[]) {
    /// This is a macro that expands on Linux systems to enable behavior needed for `hebi_getchar()`
    hebi_getchar_init();

    /// Modify the structure below to suit your needs.
    hebi::example::CreateDesc desc = {
        { "base", "elbow", "shoulder" }, /// modules
        { "Nihon" },                     /// families - Note that you can...
                                         /// use 1 family if all modules are part of the same family

        { -3.50, -3.20, 1.414 },         /// home waypoint
        100,                             /// command lifetime (in ms)
        1.5,                             /// time to move from one waypoint to the next
        500.0                            /// Feedback frequency
    };

    std::shared_ptr<hebi::Group> group;

    if (!hebi::example::createGroup(desc, group)) {
        std::cerr << "Could not get group" << std::endl;
        return 1;
    }

    auto numberOfModules = group->size();
    Eigen::VectorXd homePosition(numberOfModules);

    /// Populate home position vector
    for (int i = 0; i < numberOfModules; i++) {
        homePosition[i] = desc.homePosition[i];
    }

    /// Move to home position
    hebi::example::moveToWayPoint(group.get(), &homePosition, desc.waypointTransitionTime);

    /// Constantly zero torque in the background
    std::thread zeroTorqueThread(zeroTorqueProc, group.get());
    zeroTorqueThread.detach();

    /// Learn Motion
    auto trajectory = hebi::example::learnMotion(group.get(), desc.waypointTransitionTime);

    /// Move through waypoints
    hebi::example::moveThroughWayPoints(group.get(), trajectory.get());

    while (true) {
        char ch = hebi_getchar();
        switch (ch) {
            case HEBI_ENTER_LEARNING_MODE: {
                trajectory = hebi::example::learnMotion(group.get(), desc.waypointTransitionTime);
                break;
            }
            case HEBI_WAYPOINT_PLAYBACK: {
                hebi::example::moveThroughWayPoints(group.get(), trajectory.get());
                goto example_end;
            }
            default: { break; }
        }
    }

example_end:
    hebi::example::quit_running = true;
    return 0;
}
