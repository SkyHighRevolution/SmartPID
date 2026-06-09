#ifndef AUTOTUNE_PID_H
#define AUTOTUNE_PID_H

#include <Arduino.h>

class AutoTunePID {
public:
    enum Mode {
        MANUAL = 0,
        AUTOMATIC = 1,
        TUNING = 2
    };

    enum Action {
        FORWARD = 0, // Direct Acting: Cooling (PV increase -> Output increase)
        REVERSE = 1  // Reverse Acting: Heating (PV increase -> Output decrease)
    };

    enum TuningRule {
        ZIEGLER_NICHOLS_PID = 0,
        ZIEGLER_NICHOLS_PI = 1,
        ZIEGLER_NICHOLS_P = 2
    };

    /**
     * @brief Construct a new AutoTunePID controller object.
     * 
     * @param input Pointer to the input process variable (PV).
     * @param output Pointer to the output control variable (CO).
     * @param setpoint Pointer to the setpoint target variable (SP).
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param action Control action direction (FORWARD/REVERSE).
     */
    AutoTunePID(double* input, double* output, double* setpoint,
                double kp, double ki, double kd, Action action = REVERSE);

    /**
     * @brief Executes the PID controller calculations or auto-tuning relay feedback.
     *        This function is non-blocking and respects the set sample time.
     * 
     * @return true if output was updated, false otherwise.
     */
    bool Compute();

    /**
     * @brief Set the Controller Mode (MANUAL, AUTOMATIC, or TUNING).
     * 
     * @param mode Mode to transition to.
     */
    void SetMode(Mode mode);
    Mode GetMode() const { return _mode; }

    /**
     * @brief Set the Controller Direction (FORWARD/REVERSE).
     * 
     * @param action Direction to set.
     */
    void SetControllerDirection(Action action);
    Action GetControllerDirection() const { return _action; }

    /**
     * @brief Updates the PID gain parameters manually.
     * 
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void SetTunings(double kp, double ki, double kd);
    double GetKp() const { return _kp; }
    double GetKi() const { return _ki; }
    double GetKd() const { return _kd; }

    /**
     * @brief Set the execution sample time.
     * 
     * @param sampleTime Sample time in milliseconds (or microseconds if SetTimeUnit is set to true).
     */
    void SetSampleTime(unsigned long sampleTime);
    unsigned long GetSampleTime() const { return _sampleTime; }

    /**
     * @brief Configure whether to track time in milliseconds or microseconds.
     * 
     * @param useMicroseconds Set to true for micros(), false for millis().
     */
    void SetTimeUnit(bool useMicroseconds);
    bool GetTimeUnit() const { return _useMicroseconds; }

    /**
     * @brief Set the Output limits for dynamic clamping (integral anti-windup limits).
     * 
     * @param minLimit Minimum output value.
     * @param maxLimit Maximum output value.
     */
    void SetOutputLimits(double minLimit, double maxLimit);
    double GetOutputMin() const { return _outMin; }
    double GetOutputMax() const { return _outMax; }

    /**
     * @brief Starts the Åström-Hägglund relay feedback tuning phase.
     * 
     * @param relayAmplitude Amplitude of the relay output oscillation around the bias.
     * @param hysteresis Hysteresis band to suppress sensor noise.
     * @param timeout Timeout for the tuning process in milliseconds/microseconds.
     * @param cyclesToTarget Number of full cycles to average for parameters extraction.
     */
    void StartTuning(double relayAmplitude, double hysteresis = 0.5, 
                     unsigned long timeout = 60000, int cyclesToTarget = 5);

    /**
     * @brief Abort the current tuning phase and revert to manual mode.
     */
    void CancelTuning();
    
    /**
     * @brief Check if the tuning phase has successfully completed.
     * 
     * @return true if completed, false otherwise.
     */
    bool IsTuningComplete() const { return _tuningComplete; }

    /**
     * @brief Set the tuning rule configuration (ZIEGLER_NICHOLS_PID, PI, or P).
     * 
     * @param rule Rule to apply.
     */
    void SetTuningRule(TuningRule rule) { _tuningRule = rule; }
    TuningRule GetTuningRule() const { return _tuningRule; }

private:
    double* _input;
    double* _output;
    double* _setpoint;

    double _kp;
    double _ki;
    double _kd;

    TuningRule _tuningRule;
    Action _action;
    Mode _mode;

    unsigned long _lastTime;
    unsigned long _sampleTime;
    bool _useMicroseconds;

    double _outMin;
    double _outMax;

    double _integralSum;
    double _lastInput;
    bool _isFirstRun;

    bool _tuningComplete;
    double _relayAmplitude;
    double _hysteresis;
    unsigned long _tuningTimeout;
    unsigned long _tuningStartTime;
    int _cyclesToTarget;

    bool _relayStateHigh;
    double _tuningBias;
    int _cycleCount;
    unsigned long _lastToggleTime;
    double _peakMax;
    double _peakMin;
    
    static const int MAX_TUNE_CYCLES = 10;
    double _amplitudes[MAX_TUNE_CYCLES];
    double _periods[MAX_TUNE_CYCLES];

    unsigned long GetCurrentTime() const;
    void InitializePID();
    void RunRelayFeedback();
    void FinishTuning();
};

#endif // AUTOTUNE_PID_H
