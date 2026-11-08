#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>

// ROS/UDP-free timing policy, tested independently. Times are steady seconds.
class ModeSwitchGuard {
public:
    double switch_wait = 3.0;
    double feedback_timeout = 0.3;
    double retry_period = 3.5;
    double last_sent_at = -std::numeric_limits<double>::infinity();
    int raw_feedback = -1;
    int spin_feedback_mode = 4;
    int max_retries = 2;
    int send_count = 0;
    int target = -1;
    int feedback = -1;
    double requested_at = 0.0;
    double feedback_at = -std::numeric_limits<double>::infinity();

    void validate() const {
        if (!std::isfinite(switch_wait) || switch_wait < 3.0
            || !std::isfinite(feedback_timeout) || feedback_timeout <= 0.0
            || !std::isfinite(retry_period) || retry_period < switch_wait
            || spin_feedback_mode < 1 || spin_feedback_mode > 4 || max_retries < 0)
            throw std::invalid_argument("Invalid mechanical wait, retry period or feedback timeout");
    }
    void mark_sent(double now) { last_sent_at = now; ++send_count; }
    bool should_retry(double now) const {
        // Fresh intermediate feedback confirms progress; never restart that motion.
        if (fresh(now) && raw_feedback == 1 && spin_feedback_mode != 1) return false;
        return target >= 0 && !ready(now) && send_count <= max_retries
            && now-last_sent_at >= retry_period;
    }

    int command_mode(int state_mode) const {
        if (state_mode == spin_feedback_mode) return 4;
        switch (state_mode) {
        case 0: return 0;
        case 2: return -1; // a configured alternate spin feedback must not accept both IDs
        // Crab/translation feedback IDs and units are not confirmed on this car.
        // No legacy mapping authorizes an unverified motion mode.
        case 3: return -1;
        case 4: return -1;
        default: return -1;
        }
    }
    bool fresh(double now) const {
        return now >= feedback_at && now-feedback_at <= feedback_timeout;
    }
    bool ready(double now) const {
        return target >= 0 && fresh(now) && feedback == target
            && now-requested_at >= switch_wait
            && now-last_sent_at >= switch_wait;
    }
    // Do not reverse an in-flight mechanical request. Stop is always independent.
    bool request(int mode, double now) {
        if (mode != 0 && mode != 4) return false;
        if (mode == target || (target >= 0 && !ready(now))) return false;
        target = mode;
        requested_at = now;
        last_sent_at = -std::numeric_limits<double>::infinity();
        send_count = 0;
        return true;
    }
    void observe(int state_mode, double now) {
        raw_feedback = state_mode;
        feedback = command_mode(state_mode);
        feedback_at = now;
        // No timestamp reset here: there is exactly one mechanical wait.
    }
    int ready_mode(double now) const { return ready(now) ? target : -1; }
};
