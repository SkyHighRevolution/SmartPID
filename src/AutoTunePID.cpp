#include "AutoTunePID.h"

AutoTunePID::AutoTunePID(double* input, double* output, double* setpoint,
                         double kp, double ki, double kd, Action action)
    : _input(input),
      _output(output),
      _setpoint(setpoint),
      _kp(kp),
      _ki(ki),
      _kd(kd),
      _tuningRule(ZIEGLER_NICHOLS_PID),
      _action(action),
      _mode(MANUAL),
      _lastTime(0),
      _sampleTime(100), // default 100ms
      _useMicroseconds(false),
      _outMin(0.0),
      _outMax(255.0),
      _integralSum(0.0),
      _lastInput(0.0),
      _isFirstRun(true),
      _tuningComplete(false),
      _relayAmplitude(10.0),
      _hysteresis(0.5),
      _tuningTimeout(60000),
      _tuningStartTime(0),
      _cyclesToTarget(5),
      _relayStateHigh(false),
      _tuningBias(0.0),
      _cycleCount(0),
      _lastToggleTime(0),
      _peakMax(-1e9),
      _peakMin(1e9) {
    
    for (int i = 0; i < MAX_TUNE_CYCLES; i++) {
        _amplitudes[i] = 0.0;
        _periods[i] = 0.0;
    }
}

bool AutoTunePID::Compute() {
    unsigned long now = GetCurrentTime();
    
    if (_mode == MANUAL) {
        return false;
    }
    
    if (_mode == TUNING) {
        // Check for timeout
        if (now - _tuningStartTime > _tuningTimeout) {
            CancelTuning();
            return false;
        }
        
        // Execute relay feedback calculations at sample rate
        unsigned long timeChange = now - _lastTime;
        if (timeChange >= _sampleTime) {
            RunRelayFeedback();
            _lastTime = now;
            return true;
        }
        return false;
    }
    
    // AUTOMATIC Mode (Active PID Loop)
    unsigned long timeChange = now - _lastTime;
    if (timeChange >= _sampleTime) {
        double dt = (double)timeChange / (_useMicroseconds ? 1000000.0 : 1000.0);
        if (dt <= 0.0) dt = 1e-6;
        
        double input = *_input;
        double setpoint = *_setpoint;
        
        // Error computation based on controller action
        double error;
        if (_action == FORWARD) {
            error = input - setpoint;
        } else {
            error = setpoint - input;
        }
        
        // 1. Proportional term
        double pTerm = _kp * error;
        
        // 2. Derivative on Measurement (kick prevention)
        double dTerm = 0.0;
        if (!_isFirstRun) {
            double inputDiff = input - _lastInput;
            if (_action == FORWARD) {
                dTerm = _kd * (inputDiff / dt);
            } else {
                dTerm = -_kd * (inputDiff / dt);
            }
        }
        
        // 3. Integral term with dynamic clamping (anti-windup)
        double trialIntegral = _integralSum + _ki * error * dt;
        double unsatOutput = pTerm + trialIntegral + dTerm;
        
        double outputVal = unsatOutput;
        bool clampIntegral = false;
        
        if (unsatOutput > _outMax) {
            outputVal = _outMax;
            if (error * _ki > 0.0) {
                clampIntegral = true;
            }
        } else if (unsatOutput < _outMin) {
            outputVal = _outMin;
            if (error * _ki < 0.0) {
                clampIntegral = true;
            }
        }
        
        if (!clampIntegral) {
            _integralSum = trialIntegral;
        }
        
        *_output = outputVal;
        
        // Save current variables
        _lastInput = input;
        _lastTime = now;
        _isFirstRun = false;
        
        return true;
    }
    
    return false;
}

void AutoTunePID::SetMode(Mode mode) {
    if (mode == _mode) return;
    
    if (mode == AUTOMATIC) {
        InitializePID();
    }
    
    _mode = mode;
}

void AutoTunePID::SetControllerDirection(Action action) {
    _action = action;
}

void AutoTunePID::SetTunings(double kp, double ki, double kd) {
    if (kp < 0.0 || ki < 0.0 || kd < 0.0) return;
    
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void AutoTunePID::SetSampleTime(unsigned long sampleTime) {
    if (sampleTime > 0) {
        _sampleTime = sampleTime;
    }
}

void AutoTunePID::SetTimeUnit(bool useMicroseconds) {
    if (useMicroseconds == _useMicroseconds) return;
    
    _useMicroseconds = useMicroseconds;
    if (_useMicroseconds) {
        _sampleTime *= 1000;
    } else {
        _sampleTime /= 1000;
        if (_sampleTime < 1) _sampleTime = 1;
    }
}

void AutoTunePID::SetOutputLimits(double minLimit, double maxLimit) {
    if (minLimit >= maxLimit) return;
    
    _outMin = minLimit;
    _outMax = maxLimit;
    
    if (_mode == AUTOMATIC) {
        if (*_output > _outMax) *_output = _outMax;
        else if (*_output < _outMin) *_output = _outMin;
        
        if (_integralSum > _outMax) _integralSum = _outMax;
        else if (_integralSum < _outMin) _integralSum = _outMin;
    }
}

void AutoTunePID::StartTuning(double relayAmplitude, double hysteresis, 
                             unsigned long timeout, int cyclesToTarget) {
    _relayAmplitude = relayAmplitude;
    _hysteresis = hysteresis;
    _tuningTimeout = timeout;
    _cyclesToTarget = cyclesToTarget;
    if (_cyclesToTarget > MAX_TUNE_CYCLES) {
        _cyclesToTarget = MAX_TUNE_CYCLES;
    }
    
    _tuningStartTime = GetCurrentTime();
    _tuningComplete = false;
    _cycleCount = 0;
    _peakMax = -1e9;
    _peakMin = 1e9;
    _tuningBias = *_output; // Bias is current output level
    
    // Set initial relay direction
    bool initiallyBelow = (*_input < *_setpoint);
    if (_action == REVERSE) {
        _relayStateHigh = initiallyBelow;
    } else {
        _relayStateHigh = !initiallyBelow;
    }
    
    // Set initial output
    if (_relayStateHigh) {
        *_output = _tuningBias + _relayAmplitude;
    } else {
        *_output = _tuningBias - _relayAmplitude;
    }
    if (*_output > _outMax) *_output = _outMax;
    if (*_output < _outMin) *_output = _outMin;
    
    _lastToggleTime = _tuningStartTime;
    
    for (int i = 0; i < MAX_TUNE_CYCLES; i++) {
        _amplitudes[i] = 0.0;
        _periods[i] = 0.0;
    }
    
    _mode = TUNING;
    _lastTime = GetCurrentTime();
}

void AutoTunePID::CancelTuning() {
    _mode = MANUAL;
    _tuningComplete = false;
}

unsigned long AutoTunePID::GetCurrentTime() const {
    return _useMicroseconds ? micros() : millis();
}

void AutoTunePID::InitializePID() {
    _lastInput = *_input;
    
    // Bumpless Transfer: Pre-load the integral sum to avoid jumps in control output
    double error;
    if (_action == FORWARD) {
        error = *_input - *_setpoint;
    } else {
        error = *_setpoint - *_input;
    }
    
    double pTerm = _kp * error;
    _integralSum = *_output - pTerm;
    
    if (_integralSum > _outMax) {
        _integralSum = _outMax;
    } else if (_integralSum < _outMin) {
        _integralSum = _outMin;
    }
    
    _isFirstRun = true;
    _lastTime = GetCurrentTime();
}

void AutoTunePID::RunRelayFeedback() {
    double input = *_input;
    double setpoint = *_setpoint;
    unsigned long now = GetCurrentTime();
    
    // Update maximum and minimum peaks
    if (input > _peakMax) _peakMax = input;
    if (input < _peakMin) _peakMin = input;
    
    bool toggle = false;
    
    // Check relay state and switch condition with hysteresis
    if (_action == REVERSE) {
        if (_relayStateHigh && (input > setpoint + _hysteresis)) {
            _relayStateHigh = false;
            toggle = true;
        } else if (!_relayStateHigh && (input < setpoint - _hysteresis)) {
            _relayStateHigh = true;
            toggle = true;
        }
    } else {
        if (_relayStateHigh && (input < setpoint - _hysteresis)) {
            _relayStateHigh = false;
            toggle = true;
        } else if (!_relayStateHigh && (input > setpoint + _hysteresis)) {
            _relayStateHigh = true;
            toggle = true;
        }
    }
    
    // Write relay output
    if (_relayStateHigh) {
        *_output = _tuningBias + _relayAmplitude;
    } else {
        *_output = _tuningBias - _relayAmplitude;
    }
    
    if (*_output > _outMax) *_output = _outMax;
    if (*_output < _outMin) *_output = _outMin;
    
    if (toggle) {
        static int toggleCount = 0;
        static unsigned long tMinus2 = 0;
        static unsigned long tMinus1 = 0;
        
        if (_cycleCount == 0 && toggleCount == 0) {
            tMinus1 = now;
            toggleCount = 1;
            _peakMax = input;
            _peakMin = input;
            return;
        }
        
        if (toggleCount == 1) {
            tMinus2 = tMinus1;
            tMinus1 = now;
            toggleCount = 2;
            _peakMax = input;
            _peakMin = input;
            return;
        }
        
        // Full cycle measurement completed (tMinus2 to now)
        double periodSec = (double)(now - tMinus2) / (_useMicroseconds ? 1000000.0 : 1000.0);
        double amplitudeVal = (_peakMax - _peakMin) / 2.0;
        
        if (_cycleCount < _cyclesToTarget && _cycleCount < MAX_TUNE_CYCLES) {
            _periods[_cycleCount] = periodSec;
            _amplitudes[_cycleCount] = amplitudeVal;
            _cycleCount++;
        }
        
        tMinus2 = tMinus1;
        tMinus1 = now;
        
        _peakMax = input;
        _peakMin = input;
        
        if (_cycleCount >= _cyclesToTarget) {
            FinishTuning();
            toggleCount = 0;
        }
    }
}

void AutoTunePID::FinishTuning() {
    double sumAmp = 0.0;
    double sumPer = 0.0;
    for (int i = 0; i < _cycleCount; i++) {
        sumAmp += _amplitudes[i];
        sumPer += _periods[i];
    }
    double avgAmp = sumAmp / _cycleCount;
    double avgPer = sumPer / _cycleCount;
    
    if (avgAmp < 1e-4) avgAmp = 1e-4;
    if (avgPer < 1e-4) avgPer = 1e-4;
    
    // Ultimate gain Ku and Ultimate period Tu calculation
    double Ku = (4.0 * _relayAmplitude) / (3.141592653589793 * avgAmp);
    double Tu = avgPer;
    
    // Ziegler-Nichols tuning rules calculation
    if (_tuningRule == ZIEGLER_NICHOLS_PID) {
        _kp = 0.6 * Ku;
        _ki = 2.0 * _kp / Tu;
        _kd = 0.125 * _kp * Tu;
    } else if (_tuningRule == ZIEGLER_NICHOLS_PI) {
        _kp = 0.45 * Ku;
        _ki = 1.2 * _kp / Tu;
        _kd = 0.0;
    } else { // ZIEGLER_NICHOLS_P
        _kp = 0.5 * Ku;
        _ki = 0.0;
        _kd = 0.0;
    }
    
    _tuningComplete = true;
    SetMode(AUTOMATIC);
}
