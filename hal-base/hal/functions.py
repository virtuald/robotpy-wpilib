import ctypes as C
import os as _os
import warnings

from .exceptions import HALError
from .constants import *

from hal_impl.types import *
from hal_impl.fndef import _RETFUNC, _THUNKFUNC, _VAR, _dll
from hal_impl import __hal_simulation__

def hal_wrapper(f):
    '''Decorator to support introspection. The wrapped function must be
       the same name as the wrapper function, but start with an underscore
    '''
    
    wrapped = globals()['_' + f.__name__]
    if hasattr(wrapped, 'fndata'):
        f.fndata = wrapped.fndata
    return f

def _STATUSFUNC(name, restype, *params, out=None, library=_dll,
                handle_missing=False, _inner_func=_RETFUNC):
    realparams = list(params)
    realparams.append(("status", C.POINTER(C.c_int32)))
    if restype is not None and out is not None:
        outindexes = [i for i, p in enumerate(params) if p[0] in out]
        def errcheck(rv, f, args):
            out = [rv]
            out.extend(args[i].value for i in outindexes)
            return tuple(out)
    else:
        errcheck = None
    _inner = _inner_func(name, restype, *realparams, out=out, library=library,
                        errcheck=errcheck, handle_missing=handle_missing)
    def outer(*args, **kwargs):
        status = C.c_int32(0)
        rv = _inner(*args, status=status, **kwargs)
        if status.value == 0:
            return rv
        elif status.value < 0:
            raise HALError(getHALErrorMessage(status.value))
        elif status.value > 0:
            warnings.warn(getHALErrorMessage(status.value), stacklevel=2)
            return rv
    
    # Support introspection for API validation
    if hasattr(_inner, 'fndata'):
        outer.fndata = _inner.fndata
    return outer

def _TSTATUSFUNC(*a, **k):
    return _STATUSFUNC(_inner_func=_THUNKFUNC, *a, **k)

def _CTRFUNC_errcheck(result, func, args):
    if result != 0:
        warnings.warn(getHALErrorMessage(result))
    return args

def _CTRFUNC(name, *params, out=None, library=_dll, handle_missing=False):
    return _RETFUNC(name, C.c_int, ("handle", TalonSRX_ptr), *params, out=out, library=library,
                    handle_missing=handle_missing, errcheck=_CTRFUNC_errcheck)

#############################################################################
# Semaphore.hpp
#############################################################################

initializeMutexNormal = _RETFUNC("initializeMutexNormal", MUTEX_ID)
deleteMutex = _RETFUNC("deleteMutex", None, ("sem", MUTEX_ID))
takeMutex = _RETFUNC("takeMutex", C.c_int8, ("sem", MUTEX_ID))
tryTakeMutex = _RETFUNC("tryTakeMutex", C.c_bool, ("sem", MUTEX_ID))
giveMutex = _RETFUNC("giveMutex", C.c_int8, ("sem", MUTEX_ID))

initializeMultiWait = _RETFUNC("initializeMultiWait", MULTIWAIT_ID)
deleteMultiWait = _RETFUNC("deleteMultiWait", None, ("sem", MULTIWAIT_ID))
takeMultiWait = _RETFUNC("takeMultiWait", C.c_int8, ("sem", MULTIWAIT_ID),("mutex", MUTEX_ID))
giveMultiWait = _RETFUNC("giveMultiWait", C.c_int8, ("sem", MULTIWAIT_ID))

#############################################################################
# HAL
#############################################################################

getPort = _RETFUNC("getPort", Port_ptr, ("pin", C.c_uint8))
getPortWithModule = _RETFUNC("getPortWithModule", Port_ptr, ("module", C.c_uint8), ("pin", C.c_uint8))
freePort = _RETFUNC("freePort", None, ("port", Port_ptr))

_getHALErrorMessage = _RETFUNC("getHALErrorMessage", C.c_char_p, ("code", C.c_int32))
@hal_wrapper
def getHALErrorMessage(code):
    return _getHALErrorMessage(code).decode('utf_8')

getFPGAVersion = _STATUSFUNC("getFPGAVersion", C.c_uint16)
getFPGARevision = _STATUSFUNC("getFPGARevision", C.c_uint32)
getFPGATime = _STATUSFUNC("getFPGATime", C.c_uint64)

getFPGAButton = _STATUSFUNC("getFPGAButton", C.c_bool)

_HALSetErrorData = _RETFUNC("HALSetErrorData", C.c_int, ("errors", C.c_char_p), ("errorsLength", C.c_int), ("wait_ms", C.c_int))
@hal_wrapper
def HALSetErrorData(errors, wait_ms):
    errors = errors.encode('utf-8')
    return _HALSetErrorData(errors, len(errors), wait_ms)

HALGetControlWord = _RETFUNC("HALGetControlWord", C.c_int, ("data", HALControlWord_ptr), out=["data"])

HALGetAllianceStation = _RETFUNC("HALGetAllianceStation", C.c_int, ("allianceStation", C.POINTER(C.c_int)), out=["allianceStation"])

_HALGetJoystickAxes = _RETFUNC("HALGetJoystickAxes", C.c_int, ("joystickNum", C.c_uint8), ("axes", HALJoystickAxes_ptr))
@hal_wrapper
def HALGetJoystickAxes(joystickNum):
    axes = HALJoystickAxes()
    _HALGetJoystickAxes(joystickNum, axes)
    return axes.axes[0:axes.count]

_HALGetJoystickPOVs = _RETFUNC("HALGetJoystickPOVs", C.c_int, ("joystickNum", C.c_uint8), ("povs", HALJoystickPOVs_ptr))
@hal_wrapper
def HALGetJoystickPOVs(joystickNum):
    povs = HALJoystickPOVs()
    _HALGetJoystickPOVs(joystickNum, povs)
    return povs.povs[0:povs.count]

_HALGetJoystickButtons = _RETFUNC("HALGetJoystickButtons", C.c_int, ("joystickNum", C.c_uint8), ("buttons", HALJoystickButtons_ptr))
@hal_wrapper
def HALGetJoystickButtons(joystickNum):
    buttons = HALJoystickButtons()
    _HALGetJoystickButtons(joystickNum, buttons)
    return buttons

_HALGetJoystickDescriptor = _RETFUNC("HALGetJoystickDescriptor", C.c_int, ("joystickNum", C.c_uint8), ("descriptor", HALJoystickDescriptor_ptr))
@hal_wrapper
def HALGetJoystickDescriptor(joystickNum):
    descriptor = HALJoystickDescriptor()
    _HALGetJoystickDescriptor(joystickNum, descriptor)
    return descriptor

HALGetJoystickIsXbox = _RETFUNC("HALGetJoystickIsXbox", C.c_int, ("joystickNum", C.c_uint8))
HALGetJoystickType = _RETFUNC("HALGetJoystickType", C.c_int, ("joystickNum", C.c_uint8))
HALGetJoystickName = _RETFUNC("HALGetJoystickName", C.c_char_p, ("joystickNum", C.c_uint8))
HALGetJoystickAxisType = _RETFUNC("HALGetJoystickAxisType", C.c_int, ("joystickNum", C.c_uint8), ("axis", C.c_uint8))
HALSetJoystickOutputs = _RETFUNC("HALSetJoystickOutputs", C.c_int, ("joystickNum", C.c_uint8), ("outputs", C.c_uint32), ("leftRumble", C.c_uint16), ("rightRumble", C.c_uint16))
HALGetMatchTime = _RETFUNC("HALGetMatchTime", C.c_int, ("matchTime", C.POINTER(C.c_float)), out=["matchTime"])

HALSetNewDataSem = _RETFUNC("HALSetNewDataSem", None, ("sem", MULTIWAIT_ID))

HALGetSystemActive = _STATUSFUNC("HALGetSystemActive", C.c_bool)
HALGetBrownedOut = _STATUSFUNC("HALGetBrownedOut", C.c_bool)

_HALInitialize = _RETFUNC("HALInitialize", C.c_int, ("mode", C.c_int, 0))
@hal_wrapper
def HALInitialize(mode = 0):
    rv = _HALInitialize(mode)
    if not rv:
        raise HALError("Could not initialize HAL")

HALNetworkCommunicationObserveUserProgramStarting = _RETFUNC("HALNetworkCommunicationObserveUserProgramStarting", None)
HALNetworkCommunicationObserveUserProgramDisabled = _RETFUNC("HALNetworkCommunicationObserveUserProgramDisabled", None)
HALNetworkCommunicationObserveUserProgramAutonomous = _RETFUNC("HALNetworkCommunicationObserveUserProgramAutonomous", None)
HALNetworkCommunicationObserveUserProgramTeleop = _RETFUNC("HALNetworkCommunicationObserveUserProgramTeleop", None)
HALNetworkCommunicationObserveUserProgramTest = _RETFUNC("HALNetworkCommunicationObserveUserProgramTest", None)

_HALReport = _RETFUNC("HALReport", C.c_uint32, ("resource", C.c_uint8), ("instanceNumber", C.c_uint8), ("context", C.c_uint8, 0), ("feature", C.c_char_p, None))
@hal_wrapper
def HALReport(resource, instanceNumber, context = 0, feature = None):
    if feature is not None:
        feature = feature.encode('utf-8')
    return _HALReport(resource, instanceNumber, context, feature)

def HALIsSimulation():
    return __hal_simulation__

#############################################################################
# Accelerometer
#############################################################################

setAccelerometerActive = _RETFUNC("setAccelerometerActive", None, ("active", C.c_bool))
setAccelerometerRange = _RETFUNC("setAccelerometerRange", None, ("range", C.c_int))
getAccelerometerX = _RETFUNC("getAccelerometerX", C.c_double)
getAccelerometerY = _RETFUNC("getAccelerometerY", C.c_double)
getAccelerometerZ = _RETFUNC("getAccelerometerZ", C.c_double)

#############################################################################
# Analog
#############################################################################

# Analog output functions
initializeAnalogOutputPort = _STATUSFUNC("initializeAnalogOutputPort", AnalogPort_ptr, ("port", Port_ptr))
freeAnalogOutputPort = _RETFUNC("freeAnalogOutputPort", None, ("analog_port", AnalogPort_ptr))
setAnalogOutput = _STATUSFUNC("setAnalogOutput", None, ("analog_port", AnalogPort_ptr), ("voltage", C.c_double))
getAnalogOutput = _STATUSFUNC("getAnalogOutput", C.c_double, ("analog_port", AnalogPort_ptr))
checkAnalogOutputChannel = _RETFUNC("checkAnalogOutputChannel", C.c_bool, ("pin", C.c_uint32))

# Analog input functions
initializeAnalogInputPort = _STATUSFUNC("initializeAnalogInputPort", AnalogPort_ptr, ("port", Port_ptr))
freeAnalogInputPort = _RETFUNC("freeAnalogInputPort", None, ("analog_port", AnalogPort_ptr))
checkAnalogModule = _RETFUNC("checkAnalogModule", C.c_bool, ("module", C.c_uint8))
checkAnalogInputChannel = _RETFUNC("checkAnalogInputChannel", C.c_bool, ("pin", C.c_uint32))

setAnalogSampleRate = _STATUSFUNC("setAnalogSampleRate", None, ("samples_per_second", C.c_double))
getAnalogSampleRate = _STATUSFUNC("getAnalogSampleRate", C.c_float)
setAnalogAverageBits = _STATUSFUNC("setAnalogAverageBits", None, ("analog_port", AnalogPort_ptr), ("bits", C.c_uint32))
getAnalogAverageBits = _STATUSFUNC("getAnalogAverageBits", C.c_uint32, ("analog_port", AnalogPort_ptr))
setAnalogOversampleBits = _STATUSFUNC("setAnalogOversampleBits", None, ("analog_port", AnalogPort_ptr), ("bits", C.c_uint32))
getAnalogOversampleBits = _STATUSFUNC("getAnalogOversampleBits", C.c_uint32, ("analog_port", AnalogPort_ptr))
getAnalogValue = _STATUSFUNC("getAnalogValue", C.c_int16, ("analog_port", AnalogPort_ptr))
getAnalogAverageValue = _STATUSFUNC("getAnalogAverageValue", C.c_int32, ("analog_port", AnalogPort_ptr))
getAnalogVoltsToValue = _STATUSFUNC("getAnalogVoltsToValue", C.c_int32, ("analog_port", AnalogPort_ptr), ("voltage", C.c_double))
getAnalogVoltage = _STATUSFUNC("getAnalogVoltage", C.c_float, ("analog_port", AnalogPort_ptr))
getAnalogAverageVoltage = _STATUSFUNC("getAnalogAverageVoltage", C.c_float, ("analog_port", AnalogPort_ptr))
getAnalogLSBWeight = _STATUSFUNC("getAnalogLSBWeight", C.c_uint32, ("analog_port", AnalogPort_ptr))
getAnalogOffset = _STATUSFUNC("getAnalogOffset", C.c_int32, ("analog_port", AnalogPort_ptr))

isAccumulatorChannel = _STATUSFUNC("isAccumulatorChannel", C.c_bool, ("analog_port", AnalogPort_ptr))
initAccumulator = _STATUSFUNC("initAccumulator", None, ("analog_port", AnalogPort_ptr))
resetAccumulator = _STATUSFUNC("resetAccumulator", None, ("analog_port", AnalogPort_ptr))
setAccumulatorCenter = _STATUSFUNC("setAccumulatorCenter", None, ("analog_port", AnalogPort_ptr), ('center', C.c_int32))
setAccumulatorDeadband = _STATUSFUNC("setAccumulatorDeadband", None, ("analog_port", AnalogPort_ptr), ("deadband", C.c_int32))
getAccumulatorValue = _STATUSFUNC("getAccumulatorValue", C.c_int64, ("analog_port", AnalogPort_ptr))
getAccumulatorCount = _STATUSFUNC("getAccumulatorCount", C.c_uint32, ("analog_port", AnalogPort_ptr))
getAccumulatorOutput = _STATUSFUNC("getAccumulatorOutput", None, ("analog_port", AnalogPort_ptr), ("value", C.POINTER(C.c_int64)), ("count", C.POINTER(C.c_uint32)), out=["value", "count"])

initializeAnalogTrigger = _STATUSFUNC("initializeAnalogTrigger", AnalogTrigger_ptr, ("port", Port_ptr), ("index", C.POINTER(C.c_uint32)), out=["index"])
cleanAnalogTrigger = _STATUSFUNC("cleanAnalogTrigger", None, ("analog_trigger", AnalogTrigger_ptr))
setAnalogTriggerLimitsRaw = _STATUSFUNC("setAnalogTriggerLimitsRaw", None, ("analog_trigger", AnalogTrigger_ptr), ("lower", C.c_int32), ("upper", C.c_int32))
setAnalogTriggerLimitsVoltage = _STATUSFUNC("setAnalogTriggerLimitsVoltage", None, ("analog_trigger", AnalogTrigger_ptr), ("lower", C.c_double), ("upper", C.c_double))
setAnalogTriggerAveraged = _STATUSFUNC("setAnalogTriggerAveraged", None, ("analog_trigger", AnalogTrigger_ptr), ("use_averaged_value", C.c_bool))
setAnalogTriggerFiltered = _STATUSFUNC("setAnalogTriggerFiltered", None, ("analog_trigger", AnalogTrigger_ptr), ("use_filtered_value", C.c_bool))
getAnalogTriggerInWindow = _STATUSFUNC("getAnalogTriggerInWindow", C.c_bool, ("analog_trigger", AnalogTrigger_ptr))
getAnalogTriggerTriggerState = _STATUSFUNC("getAnalogTriggerTriggerState", C.c_bool, ("analog_trigger", AnalogTrigger_ptr))
getAnalogTriggerOutput = _STATUSFUNC("getAnalogTriggerOutput", C.c_bool, ("analog_trigger", AnalogTrigger_ptr), ("type", C.c_int))

#############################################################################
# Compressor
#############################################################################

initializeCompressor = _RETFUNC("initializeCompressor", PCM_ptr, ("module", C.c_uint8))
checkCompressorModule = _RETFUNC("checkCompressorModule", C.c_bool, ("module", C.c_uint8))

getCompressor = _STATUSFUNC("getCompressor", C.c_bool, ("pcm", PCM_ptr))

setClosedLoopControl = _STATUSFUNC("setClosedLoopControl", None, ("pcm", PCM_ptr), ("value", C.c_bool))
getClosedLoopControl = _STATUSFUNC("getClosedLoopControl", C.c_bool, ("pcm", PCM_ptr))

getPressureSwitch = _STATUSFUNC("getPressureSwitch", C.c_bool, ("pcm", PCM_ptr))
getCompressorCurrent = _STATUSFUNC("getCompressorCurrent", C.c_float, ("pcm", PCM_ptr))
getCompressorCurrentTooHighFault = _STATUSFUNC("getCompressorCurrentTooHighFault", C.c_bool, ("pcm", PCM_ptr))
getCompressorCurrentTooHighStickyFault = _STATUSFUNC("getCompressorCurrentTooHighStickyFault", C.c_bool, ("pcm", PCM_ptr))
getCompressorShortedStickyFault = _STATUSFUNC("getCompressorShortedStickyFault", C.c_bool, ("pcm", PCM_ptr))
getCompressorShortedFault = _STATUSFUNC("getCompressorShortedFault", C.c_bool, ("pcm", PCM_ptr))
getCompressorNotConnectedStickyFault = _STATUSFUNC("getCompressorNotConnectedStickyFault", C.c_bool, ("pcm", PCM_ptr))
getCompressorNotConnectedFault = _STATUSFUNC("getCompressorNotConnectedFault", C.c_bool, ("pcm", PCM_ptr))
clearAllPCMStickyFaults = _STATUSFUNC("clearAllPCMStickyFaults", None, ("pcm", PCM_ptr))


#############################################################################
# Digital
#############################################################################

initializeDigitalPort = _STATUSFUNC("initializeDigitalPort", DigitalPort_ptr, ("port", Port_ptr))
freeDigitalPort = _RETFUNC("freeDigitalPort", None, ("digital_port", DigitalPort_ptr))
checkPWMChannel = _RETFUNC("checkPWMChannel", C.c_bool, ("digital_port", DigitalPort_ptr))
checkRelayChannel = _RETFUNC("checkRelayChannel", C.c_bool, ("digital_port", DigitalPort_ptr))

setPWM = _STATUSFUNC("setPWM", None, ("digital_port", DigitalPort_ptr), ("value", C.c_ushort))
allocatePWMChannel = _STATUSFUNC("allocatePWMChannel", C.c_bool, ("digital_port", DigitalPort_ptr))
freePWMChannel = _STATUSFUNC("freePWMChannel", None, ("digital_port", DigitalPort_ptr))
getPWM = _STATUSFUNC("getPWM", C.c_ushort, ("digital_port", DigitalPort_ptr))
latchPWMZero = _STATUSFUNC("latchPWMZero", None, ("digital_port", DigitalPort_ptr))
setPWMPeriodScale = _STATUSFUNC("setPWMPeriodScale", None, ("digital_port", DigitalPort_ptr), ("squelch_mask", C.c_uint32))

allocatePWM = _STATUSFUNC("allocatePWM", PWM_ptr)
freePWM = _STATUSFUNC("freePWM", None, ("pwm", PWM_ptr))
setPWMRate = _STATUSFUNC("setPWMRate", None, ("rate", C.c_double))
setPWMDutyCycle = _STATUSFUNC("setPWMDutyCycle", None, ("pwm", PWM_ptr), ("duty_cycle", C.c_double))
setPWMOutputChannel = _STATUSFUNC("setPWMOutputChannel", None, ("pwm", PWM_ptr), ("pin", C.c_uint32))

setRelayForward = _STATUSFUNC("setRelayForward", None, ("digital_port", DigitalPort_ptr), ("on", C.c_bool))
setRelayReverse = _STATUSFUNC("setRelayReverse", None, ("digital_port", DigitalPort_ptr), ("on", C.c_bool))
getRelayForward = _STATUSFUNC("getRelayForward", C.c_bool, ("digital_port", DigitalPort_ptr))
getRelayReverse = _STATUSFUNC("getRelayReverse", C.c_bool, ("digital_port", DigitalPort_ptr))

allocateDIO = _STATUSFUNC("allocateDIO", C.c_bool, ("digital_port", DigitalPort_ptr), ("input", C.c_bool))
freeDIO = _STATUSFUNC("freeDIO", None, ("digital_port", DigitalPort_ptr))
setDIO = _STATUSFUNC("setDIO", None, ("digital_port", DigitalPort_ptr), ("value", C.c_short))
getDIO = _STATUSFUNC("getDIO", C.c_bool, ("digital_port", DigitalPort_ptr))
getDIODirection = _STATUSFUNC("getDIODirection", C.c_bool, ("digital_port", DigitalPort_ptr))
pulse = _STATUSFUNC("pulse", None, ("digital_port", DigitalPort_ptr), ("pulse_length", C.c_double))
isPulsing = _STATUSFUNC("isPulsing", C.c_bool, ("digital_port", DigitalPort_ptr))
isAnyPulsing = _STATUSFUNC("isAnyPulsing", C.c_bool)

setFilterSelect = _STATUSFUNC("setFilterSelect", None, ("digital_port", DigitalPort_ptr), ("filter_idx", C.c_int))
getFilterSelect = _STATUSFUNC("getFilterSelect", C.c_int, ("digital_port", DigitalPort_ptr))
setFilterPeriod = _STATUSFUNC("setFilterPeriod", None, ("filter_idx", C.c_int), ("value", C.c_uint32))
getFilterPeriod = _STATUSFUNC("getFilterPeriod", C.c_uint32, ("filter_idx", C.c_int))

initializeCounter = _STATUSFUNC("initializeCounter", Counter_ptr, ("mode", C.c_int), ("index", C.POINTER(C.c_uint32)), out=["index"])
freeCounter = _STATUSFUNC("freeCounter", None, ("counter", Counter_ptr))
setCounterAverageSize = _STATUSFUNC("setCounterAverageSize", None, ("counter", Counter_ptr), ("size", C.c_int32))
setCounterUpSource = _STATUSFUNC("setCounterUpSource", None, ("counter", Counter_ptr), ("pin", C.c_uint32), ("analog_trigger", C.c_bool))
setCounterUpSourceEdge = _STATUSFUNC("setCounterUpSourceEdge", None, ("counter", Counter_ptr), ("rising_edge", C.c_bool), ("falling_edge", C.c_bool))
clearCounterUpSource = _STATUSFUNC("clearCounterUpSource", None, ("counter", Counter_ptr))
setCounterDownSource = _STATUSFUNC("setCounterDownSource", None, ("counter", Counter_ptr), ("pin", C.c_uint32), ("analog_trigger", C.c_bool))
setCounterDownSourceEdge = _STATUSFUNC("setCounterDownSourceEdge", None, ("counter", Counter_ptr), ("rising_edge", C.c_bool), ("falling_edge", C.c_bool))
clearCounterDownSource = _STATUSFUNC("clearCounterDownSource", None, ("counter", Counter_ptr))
setCounterUpDownMode = _STATUSFUNC("setCounterUpDownMode", None, ("counter", Counter_ptr))
setCounterExternalDirectionMode = _STATUSFUNC("setCounterExternalDirectionMode", None, ("counter", Counter_ptr))
setCounterSemiPeriodMode = _STATUSFUNC("setCounterSemiPeriodMode", None, ("counter", Counter_ptr), ("high_semi_period", C.c_bool))
setCounterPulseLengthMode = _STATUSFUNC("setCounterPulseLengthMode", None, ("counter", Counter_ptr), ("threshold", C.c_double))
getCounterSamplesToAverage = _STATUSFUNC("getCounterSamplesToAverage", C.c_int32, ("counter", Counter_ptr))
setCounterSamplesToAverage = _STATUSFUNC("setCounterSamplesToAverage", None, ("counter", Counter_ptr), ("samples_to_average", C.c_int))
resetCounter = _STATUSFUNC("resetCounter", None, ("counter", Counter_ptr))
getCounter = _STATUSFUNC("getCounter", C.c_int32, ("counter", Counter_ptr))
getCounterPeriod = _STATUSFUNC("getCounterPeriod", C.c_double, ("counter", Counter_ptr))
setCounterMaxPeriod = _STATUSFUNC("setCounterMaxPeriod", None, ("counter", Counter_ptr), ("max_period", C.c_double))
setCounterUpdateWhenEmpty = _STATUSFUNC("setCounterUpdateWhenEmpty", None, ("counter", Counter_ptr), ("enabled", C.c_bool))
getCounterStopped = _STATUSFUNC("getCounterStopped", C.c_bool, ("counter", Counter_ptr))
getCounterDirection = _STATUSFUNC("getCounterDirection", C.c_bool, ("counter", Counter_ptr))
setCounterReverseDirection = _STATUSFUNC("setCounterReverseDirection", None, ("counter", Counter_ptr), ("reverse_direction", C.c_bool))

initializeEncoder = _STATUSFUNC("initializeEncoder", Encoder_ptr,
        ("port_a_module", C.c_uint8), ("port_a_pin", C.c_uint32), ("port_a_analog_trigger", C.c_bool),
        ("port_b_module", C.c_uint8), ("port_b_pin", C.c_uint32), ("port_b_analog_trigger", C.c_bool),
        ("reverse_direction", C.c_bool), ("index", C.POINTER(C.c_int32)), out=["index"])
freeEncoder = _STATUSFUNC("freeEncoder", None, ("encoder", Encoder_ptr))
resetEncoder = _STATUSFUNC("resetEncoder", None, ("encoder", Encoder_ptr))
getEncoder = _STATUSFUNC("getEncoder", C.c_int32, ("encoder", Encoder_ptr))
getEncoderPeriod = _STATUSFUNC("getEncoderPeriod", C.c_double, ("encoder", Encoder_ptr))
setEncoderMaxPeriod = _STATUSFUNC("setEncoderMaxPeriod", None, ("encoder", Encoder_ptr), ("max_period", C.c_double))
getEncoderStopped = _STATUSFUNC("getEncoderStopped", C.c_bool, ("encoder", Encoder_ptr))
getEncoderDirection = _STATUSFUNC("getEncoderDirection", C.c_bool, ("encoder", Encoder_ptr))
setEncoderReverseDirection = _STATUSFUNC("setEncoderReverseDirection", None, ("encoder", Encoder_ptr), ("reverse_direction", C.c_bool))
setEncoderSamplesToAverage = _STATUSFUNC("setEncoderSamplesToAverage", None, ("encoder", Encoder_ptr), ("samples_to_average", C.c_uint32))
getEncoderSamplesToAverage = _STATUSFUNC("getEncoderSamplesToAverage", C.c_uint32, ("encoder", Encoder_ptr))
setEncoderIndexSource = _STATUSFUNC("setEncoderIndexSource", None, ("encoder", Encoder_ptr), ("pin", C.c_uint32), ("analogTrigger", C.c_bool), ("activeHigh", C.c_bool), ("edgeSensitive", C.c_bool))

getLoopTiming = _STATUSFUNC("getLoopTiming", C.c_uint16)

spiInitialize = _TSTATUSFUNC("spiInitialize", None, ("port", C.c_uint8))

_spiTransaction = _THUNKFUNC("spiTransaction", C.c_int32, ("port", C.c_uint8),
                           ("data_to_send", C.POINTER(C.c_uint8)), ("data_received", C.POINTER(C.c_uint8)), ("size", C.c_uint8))
@hal_wrapper
def spiTransaction(port, data_to_send):
    size = len(data_to_send)
    send_buffer = (C.c_uint8 * size)(*data_to_send)
    recv_buffer = (C.c_uint8 * size)()
    rv = _spiTransaction(port, send_buffer, recv_buffer, size)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))
    return recv_buffer[:rv]

_spiWrite = _THUNKFUNC("spiWrite", C.c_int32, ("port", C.c_uint8), ("data_to_send", C.POINTER(C.c_uint8)), ("send_size", C.c_uint8))
@hal_wrapper
def spiWrite(port, data_to_send):
    send_size = len(data_to_send)
    buffer = (C.c_uint8 * send_size)(*data_to_send)
    rv = _spiWrite(port, buffer, send_size)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))
    return rv

_spiRead = _THUNKFUNC("spiRead", C.c_int32, ("port", C.c_uint8), ("buffer", C.POINTER(C.c_uint8)), ("count", C.c_uint8))
@hal_wrapper
def spiRead(port, count):
    buffer = (C.c_uint8 * count)()
    rv = _spiRead(port, buffer, count)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))
    return buffer[:]

spiClose = _THUNKFUNC("spiClose", None, ("port", C.c_uint8))
spiSetSpeed = _THUNKFUNC("spiSetSpeed", None, ("port", C.c_uint8), ("speed", C.c_uint32))
#spiSetBitsPerWord = _THUNKFUNC("spiSetBitsPerWord", None, ("port", C.c_uint8), ("bpw", C.c_uint8))
spiSetOpts = _THUNKFUNC("spiSetOpts", None, ("port", C.c_uint8), ("msb_first", C.c_int), ("sample_on_trailing", C.c_int), ("clk_idle_high", C.c_int))
spiSetChipSelectActiveHigh = _TSTATUSFUNC("spiSetChipSelectActiveHigh", None, ("port", C.c_uint8))
spiSetChipSelectActiveLow = _TSTATUSFUNC("spiSetChipSelectActiveLow", None, ("port", C.c_uint8))
spiGetHandle = _THUNKFUNC("spiGetHandle", C.c_int32, ("port", C.c_uint8));
spiSetHandle = _THUNKFUNC("spiSetHandle", None, ("port", C.c_uint8), ("handle", C.c_int32))

spiInitAccumulator = _TSTATUSFUNC('spiInitAccumulator', None, ("port", C.c_uint8),
                                 ('period', C.c_uint32), ('cmd', C.c_uint32), ('xfer_size', C.c_uint8),
                                 ('valid_mask', C.c_uint32), ('valid_value', C.c_uint32), ('data_shift', C.c_uint8),
                                 ('data_size', C.c_uint8), ('is_signed', C.c_bool), ('big_endian', C.c_bool))
spiFreeAccumulator = _TSTATUSFUNC('spiFreeAccumulator', None, ("port", C.c_uint8))
spiResetAccumulator = _TSTATUSFUNC('spiResetAccumulator', None, ("port", C.c_uint8))
spiSetAccumulatorCenter = _TSTATUSFUNC('spiSetAccumulatorCenter', None, ("port", C.c_uint8), ('center', C.c_int32))
spiSetAccumulatorDeadband = _TSTATUSFUNC('spiSetAccumulatorDeadband', None, ("port", C.c_uint8), ('deadband', C.c_int32))
spiGetAccumulatorLastValue = _TSTATUSFUNC('spiGetAccumulatorLastValue', C.c_int32, ("port", C.c_uint8))
spiGetAccumulatorValue = _TSTATUSFUNC('spiGetAccumulatorValue', C.c_int64, ("port", C.c_uint8))
spiGetAccumulatorCount = _TSTATUSFUNC('spiGetAccumulatorCount', C.c_uint32, ("port", C.c_uint8)) 
spiGetAccumulatorAverage = _TSTATUSFUNC('spiGetAccumulatorAverage', C.c_double, ("port", C.c_uint8)) 
spiGetAccumulatorOutput = _TSTATUSFUNC('spiGetAccumulatorOutput', None, ("port", C.c_uint8), ('value', C.POINTER(C.c_int64)), ('count', C.POINTER(C.c_uint32)), out=['value', 'count'])



i2CInitialize = _TSTATUSFUNC("i2CInitialize", None, ("port", C.c_uint8))

_i2CTransaction = _THUNKFUNC("i2CTransaction", C.c_int32, ("port", C.c_uint8), ("device_address", C.c_uint8),
                           ("data_to_send", C.POINTER(C.c_uint8)), ("send_size", C.c_uint8),
                           ("data_received", C.POINTER(C.c_uint8)), ("receive_size", C.c_uint8))
@hal_wrapper
def i2CTransaction(port, device_address, data_to_send, receive_size):
    send_size = len(data_to_send)
    send_buffer = (C.c_uint8 * send_size)(*data_to_send)
    recv_buffer = (C.c_uint8 * receive_size)()
    rv = _i2CTransaction(port, device_address, send_buffer, send_size, recv_buffer, receive_size)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))
    return recv_buffer[:]

_i2CWrite = _THUNKFUNC("i2CWrite", C.c_int32, ("port", C.c_uint8), ("device_address", C.c_uint8), ("data_to_send", C.POINTER(C.c_uint8)), ("send_size", C.c_uint8))
@hal_wrapper
def i2CWrite(port, device_address, data_to_send):
    send_size = len(data_to_send)
    buffer = (C.c_uint8 * send_size)(*data_to_send)
    rv = _i2CWrite(port, device_address, buffer, send_size)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))

_i2CRead = _THUNKFUNC("i2CRead", C.c_int32, ("port", C.c_uint8), ("device_address", C.c_uint8), ("buffer", C.POINTER(C.c_uint8)), ("count", C.c_uint8))
@hal_wrapper
def i2CRead(port, device_address, count):
    buffer = (C.c_uint8 * count)()
    rv = _i2CRead(port, device_address, buffer, count)
    if rv < 0:
        raise IOError(_os.strerror(C.get_errno()))
    return buffer[:]

i2CClose = _THUNKFUNC("i2CClose", None, ("port", C.c_uint8))

#############################################################################
# Interrupts
#############################################################################

_InterruptHandlerFunction = C.CFUNCTYPE(None, C.c_uint32, C.c_void_p)
_interruptHandlers = {}

initializeInterrupts = _STATUSFUNC("initializeInterrupts", Interrupt_ptr, ("interrupt_index", C.c_uint32), ("watcher", C.c_bool))
_cleanInterrupts = _STATUSFUNC("cleanInterrupts", None, ("interrupt", Interrupt_ptr))
@hal_wrapper
def cleanInterrupts(interrupt):
    _cleanInterrupts(interrupt)

    # remove references to function handlers
    _interruptHandlers.pop(interrupt, None)

waitForInterrupt = _STATUSFUNC("waitForInterrupt", C.c_uint32, ("interrupt", Interrupt_ptr), ("timeout", C.c_double), ("ignorePrevious", C.c_bool))
enableInterrupts = _STATUSFUNC("enableInterrupts", None, ("interrupt", Interrupt_ptr))
disableInterrupts = _STATUSFUNC("disableInterrupts", None, ("interrupt", Interrupt_ptr))
readRisingTimestamp = _STATUSFUNC("readRisingTimestamp", C.c_double, ("interrupt", Interrupt_ptr))
readFallingTimestamp = _STATUSFUNC("readFallingTimestamp", C.c_double, ("interrupt", Interrupt_ptr))
requestInterrupts = _STATUSFUNC("requestInterrupts", None, ("interrupt", Interrupt_ptr), ("routing_module", C.c_uint8), ("routing_pin", C.c_uint32), ("routing_analog_trigger", C.c_bool))

_attachInterruptHandler = _STATUSFUNC("attachInterruptHandler", None, ("interrupt", Interrupt_ptr), ("handler", _InterruptHandlerFunction), ("param", C.c_void_p))
@hal_wrapper
def attachInterruptHandler(interrupt, handler):
    # While attachInterruptHandler provides a param parameter, we use ctypes
    # magic instead to uniquify the callback handler

    # create bounce function to drop param
    cb_func = _InterruptHandlerFunction(lambda mask, param: handler(mask))

    # keep reference to it
    handlers = _interruptHandlers.setdefault(interrupt, [])
    handlers.append(cb_func)

    # actually attach
    _attachInterruptHandler(interrupt, cb_func, None)

setInterruptUpSourceEdge = _STATUSFUNC("setInterruptUpSourceEdge", None, ("interrupt", Interrupt_ptr), ("rising_edge", C.c_bool), ("falling_edge", C.c_bool))

#############################################################################
# Notifier
#############################################################################

_NotifierProcessQueueFunction = C.CFUNCTYPE(None, C.c_uint64, C.c_void_p)
_notifierProcessQueueFunctions = {}

_initializeNotifier = _STATUSFUNC("initializeNotifier", Notifier_ptr, ("processQueue", _NotifierProcessQueueFunction), ('param', C.c_void_p))
@hal_wrapper
def initializeNotifier(processQueue):
    # While initializeNotifier provides a param parameter, we use ctypes
    # magic instead to uniquify the callback handler

    # create bounce function to drop param
    cb_func = _NotifierProcessQueueFunction(lambda mask, param: processQueue(mask))

    # initialize notifier
    notifier = _attachInterruptHandler(processQueue, cb_func, None)

    # keep reference to bounce function
    handlers = _notifierProcessQueueFunctions[notifier] = cb_func

_cleanNotifier = _STATUSFUNC("cleanNotifier", None, ("notifier", Notifier_ptr))
@hal_wrapper
def cleanNotifier(notifier):
    _cleanNotifier(notifier)

    # remove reference to process queue function
    _notifierProcessQueueFunctions.pop(notifier, None)

getNotifierParam = _STATUSFUNC("getNotifierParam", C.c_void_p, ("notifier", Notifier_ptr))
updateNotifierAlarm = _STATUSFUNC("updateNotifierAlarm", None, ("notifier", Notifier_ptr), ("triggerTime", C.c_uint64))
stopNotifierAlarm = _STATUSFUNC("stopNotifierAlarm", None, ("notifier", Notifier_ptr))

#############################################################################
# PDP
#############################################################################
initializePDP = _RETFUNC("initializePDP", None, ("module", C.c_uint8))
getPDPTemperature = _STATUSFUNC("getPDPTemperature", C.c_double, ("module", C.c_uint8))
getPDPVoltage = _STATUSFUNC("getPDPVoltage", C.c_double, ("module", C.c_uint8))
getPDPChannelCurrent = _STATUSFUNC("getPDPChannelCurrent", C.c_double, ("module", C.c_uint8), ("channel", C.c_uint8))
getPDPTotalCurrent = _STATUSFUNC("getPDPTotalCurrent", C.c_double, ("module", C.c_uint8))
getPDPTotalPower = _STATUSFUNC("getPDPTotalPower", C.c_double, ("module", C.c_uint8))
getPDPTotalEnergy = _STATUSFUNC("getPDPTotalEnergy", C.c_double, ("module", C.c_uint8))
resetPDPTotalEnergy = _STATUSFUNC("resetPDPTotalEnergy", None, ("module", C.c_uint8))
clearPDPStickyFaults = _STATUSFUNC("clearPDPStickyFaults", None, ("module", C.c_uint8))

#############################################################################
# Power
#############################################################################
getVinVoltage = _STATUSFUNC("getVinVoltage", C.c_float)
getVinCurrent = _STATUSFUNC("getVinCurrent", C.c_float)
getUserVoltage6V = _STATUSFUNC("getUserVoltage6V", C.c_float)
getUserCurrent6V = _STATUSFUNC("getUserCurrent6V", C.c_float)
getUserActive6V = _STATUSFUNC("getUserActive6V", C.c_bool)
getUserCurrentFaults6V = _STATUSFUNC("getUserCurrentFaults6V", C.c_int)
getUserVoltage5V = _STATUSFUNC("getUserVoltage5V", C.c_float)
getUserCurrent5V = _STATUSFUNC("getUserCurrent5V", C.c_float)
getUserActive5V = _STATUSFUNC("getUserActive5V", C.c_bool)
getUserCurrentFaults5V = _STATUSFUNC("getUserCurrentFaults5V", C.c_int)
getUserVoltage3V3 = _STATUSFUNC("getUserVoltage3V3", C.c_float)
getUserCurrent3V3 = _STATUSFUNC("getUserCurrent3V3", C.c_float)
getUserActive3V3 = _STATUSFUNC("getUserActive3V3", C.c_bool)
getUserCurrentFaults3V3 = _STATUSFUNC("getUserCurrentFaults3V3", C.c_int)

#############################################################################
# Solenoid
#############################################################################

initializeSolenoidPort = _STATUSFUNC("initializeSolenoidPort", SolenoidPort_ptr, ("port", Port_ptr))
freeSolenoidPort = _RETFUNC('freeSolenoidPort', None, ('port', SolenoidPort_ptr))
checkSolenoidModule = _RETFUNC("checkSolenoidModule", C.c_bool, ("module", C.c_uint8))

getSolenoid = _STATUSFUNC("getSolenoid", C.c_bool, ("solenoid_port", SolenoidPort_ptr))
getAllSolenoids = _STATUSFUNC("getAllSolenoids", C.c_uint8, ("solenoid_port", SolenoidPort_ptr))
setSolenoid = _STATUSFUNC("setSolenoid", None, ("solenoid_port", SolenoidPort_ptr), ("value", C.c_bool))

getPCMSolenoidBlackList = _STATUSFUNC("getPCMSolenoidBlackList", C.c_int, ("solenoid_port", SolenoidPort_ptr))
getPCMSolenoidVoltageStickyFault = _STATUSFUNC("getPCMSolenoidVoltageStickyFault", C.c_bool, ("solenoid_port", SolenoidPort_ptr))
getPCMSolenoidVoltageFault = _STATUSFUNC("getPCMSolenoidVoltageFault", C.c_bool, ("solenoid_port", SolenoidPort_ptr))
clearAllPCMStickyFaults_sol = _STATUSFUNC("clearAllPCMStickyFaults_sol", None, ("solenoid_port", SolenoidPort_ptr))

#############################################################################
# TalonSRX
#############################################################################
TalonSRX_Create1 = _RETFUNC("c_TalonSRX_Create1", TalonSRX_ptr, ("deviceNumber", C.c_int))
TalonSRX_Create2 = _RETFUNC("c_TalonSRX_Create2", TalonSRX_ptr, ("deviceNumber", C.c_int), ("controlPeriodMs", C.c_int))
TalonSRX_Create3 = _RETFUNC("c_TalonSRX_Create3", TalonSRX_ptr, ("deviceNumber", C.c_int), ("controlPeriodMs", C.c_int), ('enablePeriodMs', C.c_int))
TalonSRX_Destroy = _RETFUNC("c_TalonSRX_Destroy", None, ("handle", TalonSRX_ptr))
TalonSRX_Set = _RETFUNC("c_TalonSRX_Set", None, ("handle", TalonSRX_ptr), ("value", C.c_double))
TalonSRX_SetParam = _CTRFUNC("c_TalonSRX_SetParam", ("paramEnum", C.c_int), ("value", C.c_double))
TalonSRX_RequestParam = _CTRFUNC("c_TalonSRX_RequestParam", ("paramEnum", C.c_int))
TalonSRX_GetParamResponse = _CTRFUNC("c_TalonSRX_GetParamResponse", ("paramEnum", C.c_int), ("value", C.POINTER(C.c_double)), out=["value"])
TalonSRX_GetParamResponseInt32 = _CTRFUNC("c_TalonSRX_GetParamResponseInt32", ("paramEnum", C.c_int), ("value", C.POINTER(C.c_int)), out=["value"])
TalonSRX_SetPgain = _CTRFUNC("c_TalonSRX_SetPgain", ("slotIdx", C.c_int), ("gain", C.c_double))
TalonSRX_SetIgain = _CTRFUNC("c_TalonSRX_SetIgain", ("slotIdx", C.c_int), ("gain", C.c_double))
TalonSRX_SetDgain = _CTRFUNC("c_TalonSRX_SetDgain", ("slotIdx", C.c_int), ("gain", C.c_double))
TalonSRX_SetFgain = _CTRFUNC("c_TalonSRX_SetFgain", ("slotIdx", C.c_int), ("gain", C.c_double))
TalonSRX_SetIzone = _CTRFUNC("c_TalonSRX_SetIzone", ("slotIdx", C.c_int), ("zone", C.c_int))
TalonSRX_SetCloseLoopRampRate = _CTRFUNC("c_TalonSRX_SetCloseLoopRampRate", ("slotIdx", C.c_int), ("closeLoopRampRate", C.c_int))
TalonSRX_SetVoltageCompensationRate = _CTRFUNC("c_TalonSRX_SetVoltageCompensationRate", ("voltagePerMs", C.c_double))
TalonSRX_SetSensorPosition = _CTRFUNC("c_TalonSRX_SetSensorPosition", ("pos", C.c_int))
TalonSRX_SetForwardSoftLimit = _CTRFUNC("c_TalonSRX_SetForwardSoftLimit", ("forwardLimit", C.c_int))
TalonSRX_SetReverseSoftLimit = _CTRFUNC("c_TalonSRX_SetReverseSoftLimit", ("reverseLimit", C.c_int))
TalonSRX_SetForwardSoftEnable = _CTRFUNC("c_TalonSRX_SetForwardSoftEnable", ("enable", C.c_int))
TalonSRX_SetReverseSoftEnable = _CTRFUNC("c_TalonSRX_SetReverseSoftEnable", ("enable", C.c_int))
TalonSRX_GetPgain = _CTRFUNC("c_TalonSRX_GetPgain", ("slotIdx", C.c_int), ("gain", C.POINTER(C.c_double)), out=["gain"])
TalonSRX_GetIgain = _CTRFUNC("c_TalonSRX_GetIgain", ("slotIdx", C.c_int), ("gain", C.POINTER(C.c_double)), out=["gain"])
TalonSRX_GetDgain = _CTRFUNC("c_TalonSRX_GetDgain", ("slotIdx", C.c_int), ("gain", C.POINTER(C.c_double)), out=["gain"])
TalonSRX_GetFgain = _CTRFUNC("c_TalonSRX_GetFgain", ("slotIdx", C.c_int), ("gain", C.POINTER(C.c_double)), out=["gain"])
TalonSRX_GetIzone = _CTRFUNC("c_TalonSRX_GetIzone", ("slotIdx", C.c_int), ("zone", C.POINTER(C.c_int)), out=["zone"])
TalonSRX_GetCloseLoopRampRate = _CTRFUNC("c_TalonSRX_GetCloseLoopRampRate", ("slotIdx", C.c_int), ("closeLoopRampRate", C.POINTER(C.c_int)), out=["closeLoopRampRate"])
TalonSRX_GetVoltageCompensationRate = _CTRFUNC("c_TalonSRX_GetVoltageCompensationRate", ("voltagePerMs", C.POINTER(C.c_double)), out=["voltagePerMs"])
TalonSRX_GetForwardSoftLimit = _CTRFUNC("c_TalonSRX_GetForwardSoftLimit", ("forwardLimit", C.POINTER(C.c_int)), out=["forwardLimit"])
TalonSRX_GetReverseSoftLimit = _CTRFUNC("c_TalonSRX_GetReverseSoftLimit", ("reverseLimit", C.POINTER(C.c_int)), out=["reverseLimit"])
TalonSRX_GetForwardSoftEnable = _CTRFUNC("c_TalonSRX_GetForwardSoftEnable", ("enable", C.POINTER(C.c_int)), out=["enable"])
TalonSRX_GetReverseSoftEnable = _CTRFUNC("c_TalonSRX_GetReverseSoftEnable", ("enable", C.POINTER(C.c_int)), out=["enable"])
TalonSRX_GetPulseWidthRiseToFallUs = _CTRFUNC("c_TalonSRX_GetPulseWidthRiseToFallUs", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_IsPulseWidthSensorPresent = _CTRFUNC("c_TalonSRX_IsPulseWidthSensorPresent", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_SetStatusFrameRate = _CTRFUNC("c_TalonSRX_SetStatusFrameRate", ("frameEnum", C.c_int), ("periodMs", C.c_int))
TalonSRX_ClearStickyFaults = _CTRFUNC("c_TalonSRX_ClearStickyFaults")
TalonSRX_ChangeMotionControlFramePeriod = _RETFUNC("c_TalonSRX_ChangeMotionControlFramePeriod", None, ("handle", TalonSRX_ptr), ("periodMs", C.c_int))
TalonSRX_ClearMotionProfileTrajectories = _RETFUNC("c_TalonSRX_ClearMotionProfileTrajectories", None, ("handle", TalonSRX_ptr))
TalonSRX_GetMotionProfileTopLevelBufferCount = _RETFUNC("c_TalonSRX_GetMotionProfileTopLevelBufferCount", C.c_int, ("handle", TalonSRX_ptr))
TalonSRX_IsMotionProfileTopLevelBufferFull = _RETFUNC("c_TalonSRX_IsMotionProfileTopLevelBufferFull", C.c_int, ("handle", TalonSRX_ptr))
TalonSRX_PushMotionProfileTrajectory = _CTRFUNC("c_TalonSRX_PushMotionProfileTrajectory", ("targPos", C.c_int), ("targVel", C.c_int), ("profileSlotSelect", C.c_int), ("timeDurMs", C.c_int), ("velOnly", C.c_int), ("isLastPoint", C.c_int), ("zeroPos", C.c_int))
TalonSRX_ProcessMotionProfileBuffer = _RETFUNC("c_TalonSRX_ProcessMotionProfileBuffer", None, ("handle", TalonSRX_ptr))
TalonSRX_GetMotionProfileStatus = _CTRFUNC("c_TalonSRX_GetMotionProfileStatus", ("flags", C.POINTER(C.c_int)), ("profileSlotSelect", C.POINTER(C.c_int)), ("targPos", C.POINTER(C.c_int)), ("targVel", C.POINTER(C.c_int)), ("topBufferRemaining", C.POINTER(C.c_int)), ("topBufferCnt", C.POINTER(C.c_int)), ("btmBufferCnt", C.POINTER(C.c_int)), ("outputEnable", C.POINTER(C.c_int)), out=["flags", "profileSlotSelect", "targPos", "targVel", "topBufferRemaining", "topBufferCnt", "btmBufferCnt", "outputEnable"])
TalonSRX_GetFault_OverTemp = _CTRFUNC("c_TalonSRX_GetFault_OverTemp", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_UnderVoltage = _CTRFUNC("c_TalonSRX_GetFault_UnderVoltage", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_ForLim = _CTRFUNC("c_TalonSRX_GetFault_ForLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_RevLim = _CTRFUNC("c_TalonSRX_GetFault_RevLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_HardwareFailure = _CTRFUNC("c_TalonSRX_GetFault_HardwareFailure", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_ForSoftLim = _CTRFUNC("c_TalonSRX_GetFault_ForSoftLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFault_RevSoftLim = _CTRFUNC("c_TalonSRX_GetFault_RevSoftLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_OverTemp = _CTRFUNC("c_TalonSRX_GetStckyFault_OverTemp", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_UnderVoltage = _CTRFUNC("c_TalonSRX_GetStckyFault_UnderVoltage", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_ForLim = _CTRFUNC("c_TalonSRX_GetStckyFault_ForLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_RevLim = _CTRFUNC("c_TalonSRX_GetStckyFault_RevLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_ForSoftLim = _CTRFUNC("c_TalonSRX_GetStckyFault_ForSoftLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetStckyFault_RevSoftLim = _CTRFUNC("c_TalonSRX_GetStckyFault_RevSoftLim", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetAppliedThrottle = _CTRFUNC("c_TalonSRX_GetAppliedThrottle", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetCloseLoopErr = _CTRFUNC("c_TalonSRX_GetCloseLoopErr", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFeedbackDeviceSelect = _CTRFUNC("c_TalonSRX_GetFeedbackDeviceSelect", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetModeSelect = _CTRFUNC("c_TalonSRX_GetModeSelect", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetLimitSwitchEn = _CTRFUNC("c_TalonSRX_GetLimitSwitchEn", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetLimitSwitchClosedFor = _CTRFUNC("c_TalonSRX_GetLimitSwitchClosedFor", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetLimitSwitchClosedRev = _CTRFUNC("c_TalonSRX_GetLimitSwitchClosedRev", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetSensorPosition = _CTRFUNC("c_TalonSRX_GetSensorPosition", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetSensorVelocity = _CTRFUNC("c_TalonSRX_GetSensorVelocity", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetCurrent = _CTRFUNC("c_TalonSRX_GetCurrent", ("param", C.POINTER(C.c_double)), out=["param"])
TalonSRX_GetBrakeIsEnabled = _CTRFUNC("c_TalonSRX_GetBrakeIsEnabled", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetEncPosition = _CTRFUNC("c_TalonSRX_GetEncPosition", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetEncVel = _CTRFUNC("c_TalonSRX_GetEncVel", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetEncIndexRiseEvents = _CTRFUNC("c_TalonSRX_GetEncIndexRiseEvents", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetQuadApin = _CTRFUNC("c_TalonSRX_GetQuadApin", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetQuadBpin = _CTRFUNC("c_TalonSRX_GetQuadBpin", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetQuadIdxpin = _CTRFUNC("c_TalonSRX_GetQuadIdxpin", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetAnalogInWithOv = _CTRFUNC("c_TalonSRX_GetAnalogInWithOv", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetAnalogInVel = _CTRFUNC("c_TalonSRX_GetAnalogInVel", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetTemp = _CTRFUNC("c_TalonSRX_GetTemp", ("param", C.POINTER(C.c_double)), out=["param"])
TalonSRX_GetBatteryV = _CTRFUNC("c_TalonSRX_GetBatteryV", ("param", C.POINTER(C.c_double)), out=["param"])
TalonSRX_GetResetCount = _CTRFUNC("c_TalonSRX_GetResetCount", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetResetFlags = _CTRFUNC("c_TalonSRX_GetResetFlags", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetFirmVers = _CTRFUNC("c_TalonSRX_GetFirmVers", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetPulseWidthPosition = _CTRFUNC("c_TalonSRX_GetPulseWidthPosition", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetPulseWidthVelocity = _CTRFUNC("c_TalonSRX_GetPulseWidthVelocity", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetPulseWidthRiseToRiseUs = _CTRFUNC("c_TalonSRX_GetPulseWidthRiseToRiseUs", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_IsValid = _CTRFUNC("c_TalonSRX_GetActTraj_IsValid", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_ProfileSlotSelect = _CTRFUNC("c_TalonSRX_GetActTraj_ProfileSlotSelect", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_VelOnly = _CTRFUNC("c_TalonSRX_GetActTraj_VelOnly", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_IsLast = _CTRFUNC("c_TalonSRX_GetActTraj_IsLast", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetOutputType = _CTRFUNC("c_TalonSRX_GetOutputType", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetHasUnderrun = _CTRFUNC("c_TalonSRX_GetHasUnderrun", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetIsUnderrun = _CTRFUNC("c_TalonSRX_GetIsUnderrun", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetNextID = _CTRFUNC("c_TalonSRX_GetNextID", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetBufferIsFull = _CTRFUNC("c_TalonSRX_GetBufferIsFull", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetCount = _CTRFUNC("c_TalonSRX_GetCount", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_Velocity = _CTRFUNC("c_TalonSRX_GetActTraj_Velocity", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_GetActTraj_Position = _CTRFUNC("c_TalonSRX_GetActTraj_Position", ("param", C.POINTER(C.c_int)), out=["param"])
TalonSRX_SetDemand = _CTRFUNC("c_TalonSRX_SetDemand", ("param", C.c_int))
TalonSRX_SetOverrideLimitSwitchEn = _CTRFUNC("c_TalonSRX_SetOverrideLimitSwitchEn", ("param", C.c_int))
TalonSRX_SetFeedbackDeviceSelect = _CTRFUNC("c_TalonSRX_SetFeedbackDeviceSelect", ("param", C.c_int))
TalonSRX_SetRevMotDuringCloseLoopEn = _CTRFUNC("c_TalonSRX_SetRevMotDuringCloseLoopEn", ("param", C.c_int))
TalonSRX_SetOverrideBrakeType = _CTRFUNC("c_TalonSRX_SetOverrideBrakeType", ("param", C.c_int))
TalonSRX_SetModeSelect = _CTRFUNC("c_TalonSRX_SetModeSelect", ("param", C.c_int))
TalonSRX_SetModeSelect2 = _CTRFUNC("c_TalonSRX_SetModeSelect2", ("modeSelect", C.c_int), ("demand", C.c_int))
TalonSRX_SetProfileSlotSelect = _CTRFUNC("c_TalonSRX_SetProfileSlotSelect", ("param", C.c_int))
TalonSRX_SetRampThrottle = _CTRFUNC("c_TalonSRX_SetRampThrottle", ("param", C.c_int))
TalonSRX_SetRevFeedbackSensor = _CTRFUNC("c_TalonSRX_SetRevFeedbackSensor", ("param", C.c_int))

#############################################################################
# Utilities
#############################################################################
HAL_NO_WAIT = _VAR("HAL_NO_WAIT", C.c_int32)
HAL_WAIT_FOREVER = _VAR("HAL_WAIT_FOREVER", C.c_int32)

delayTicks = _RETFUNC("delayTicks", None, ("ticks", C.c_int32))
delayMillis = _RETFUNC("delayMillis", None, ("ms", C.c_double))
delaySeconds = _RETFUNC("delaySeconds", None, ("s", C.c_double))
