#
# All data that passes through the HAL comes to/from here
#

from hal import constants
import sys
import copy

import logging
logger = logging.getLogger('hal.data')

#: Dictionary of all robot data (input and output data)
hal_data = {
    # don't fill this out, fill out the version in reset_hal_data
}

#: A dictionary with only hal input data
#: -> this allows you to create a dictionary that you can do update_hal_data
#:    with, without worrying that you're overwriting any values that the robot
#:    might set something and cause a weird race condition
hal_in_data = {
    # don't fill this out
}


#: A multiwait object. Use hal.giveMultiWait() to set this when
#: driver station related data has been updated
hal_newdata_sem = None



class NotifyDict(dict):
    '''
        Allows us to listen to changes in the dictionary -- 
        note that we don't wrap everything, because for our
        purposes we don't care about the rest
        
        We only use these for some keys in the hal_data dict, 
        as not all keys are useful to listen to
    '''
    __slots__ = ['cbs']
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cbs = {}
        
    def register(self, k, cb, notify=False):
        '''
            register a function to be called when an item is set 
            with in this dictionary. We raise a key error if the 
            key passed is not a key that the dictionary contains.

            :param k:        Key to be registered for call back. The key must be a
                             valid key with in the dictionary
            :param cb:       Function to be called if k is set. This function needs 
                             to take at least 2 parameters
            :param notify:   Calls the function cb after registering k                
        '''
        if k not in self:
            raise KeyError("Cannot register for non-existant key '%s'" % k)
        self.cbs.setdefault(k, []).append(cb)
        if notify:
            cb(k, self[k])
        
    def __setitem__(self, k, v):
        '''
           Overrides __setitem__. If k has any callback functions defined they are
           called from here
           
           :param k: key to be set
           :param v: value to be set
        '''     
        super().__setitem__(k, v)
        
        # Call the callbacks
        for cb in self.cbs.get(k, []):
            try:
                cb(k, v)
            except:
                logger.exception("BAD INTERNAL ERROR")
    

class IN:
    '''Marks a variable in the dict as something the simulator can set'''
    def __init__(self, value):
        self.value = value
  
class OUT:
    '''Marks a variable in the dict as something the robot will set'''
    def __init__(self, value):
        self.value = value

def _reset_hal_data(hooks):
    '''
        Intended to be used by the test runner or simulator. Don't call this
        directly, instead call hal_impl.reset_hal()
        
        Subject to change until the simulator is fully developed, as the
        usefulness of some of this isn't immediately clear yet.
        
        Generally, non-hal components should only be modifying this 
        dict, and shouldn't add new keys, nor delete existing keys.
        
        TODO: add comments stating which parameters are input, output, expected types
        
        TODO: initialization isn't consistent yet. Need to decide whether to
              use None, or an explicit initialization key
              
        :param hooks: A :class:`SimHooks` or similar instance
        
        .. warning:: Don't put invalid floats in here, or this structure
                     is no longer JSON serializable!
    '''
    global hal_data, hal_newdata_sem
    hal_newdata_sem = None
    
    hal_data.clear()
    hal_in_data.clear()
    
    hal_data.update({

        'alliance_station': IN(constants.kHALAllianceStationID_red1),

        'time': {
            'has_source': IN(False),

            # Used to compute getFPGATime
            'program_start': OUT(hooks.getTime()),

            # Used to compute getMatchTime -- set to return value of getFPGATime()
            'match_start': OUT(None),
        },
        


        # You should not modify these directly, instead use the mode_helpers!
        'control': {
            'has_source':   IN(False),
            'enabled':      OUT(False),
            'autonomous':   OUT(False),
            'test':         OUT(False),
            'eStop':        OUT(False),
            'fmsAttached': IN(False),
            'dsAttached':  OUT(False),
        },
                     
        # key:   resource type
        # value: list of instance numbers
        'reports': NotifyDict({}),

        # See driver station notes above

        # Joysticks are stored numbered 0-5.
        # buttons are stored as booleans
        # axes are stored as values between -1 and 1
        # povs are stored as integer values
        'joysticks': [{
            'has_source': IN(False),
            'buttons': IN([None] + [False]*12), # numbered 1-12 -- 0 is ignored
            'axes':    IN([0]*constants.kMaxJoystickAxes),  # x is 0, y is 1, .. 
            'povs':    IN([-1]*constants.kMaxJoystickPOVs)  # integers
        
        } for _ in range(6)],

        

        'fpga_button': IN(False),
        'error_data': OUT(None),

        # built-in accelerometer on roboRIO
        'accelerometer': {
            'has_source': IN(False),        # built-in accelerometer only
            'active':   OUT(False),         # built-in accelerometer only
            'range':    OUT(0),             # built-in accelerometer only
            
            # These should be used by other simulation components when 
            # appropriate
            'x':        IN(0),
            'y':        IN(0),
            'z':        IN(0),
        },
              
        # Generic robot information to be used by sim components when
        # appropriate... some components can add custom data here.
        #
        # The way this works is that each device will define its own key(s)
        # here, and that you set the values that it's expecting. Refer to the
        # sim device's documentation to figure out the right key.
        #
        # By convention, it should be devicename_bus_port_thing, such as
        # "adxrs450_i2c_0_angle"
        #
        # Sim devices should read these values with defaults of 0, as they may
        # not exist.
        'robot': {
        },

        # global for all
        'analog_sample_rate': OUT(1024.0), # need a better default

        # 8 analog channels, each is a dictionary.

        'analog_out': [NotifyDict({
            'initialized': OUT(False),
            'voltage': OUT(0.0),

        }) for _ in range(8)],

        # TODO: make this easier to use
        'analog_in': [NotifyDict({
            'has_source':       IN(False),
            'initialized':      OUT(False),
            'avg_bits':         OUT(0),
            'oversample_bits':  OUT(0),
            'value':            IN(0),
            'avg_value':        IN(0),
            'voltage':          IN(0),
            'avg_voltage':      IN(0),
            'lsb_weight':       IN(1),    # TODO: better default
            'offset':           IN(65535),    # TODO: better default

            'accumulator_initialized': OUT(False),
            'accumulator_center':   OUT(0),
            'accumulator_value':    IN(0),
            'accumulator_count':    IN(1), # don't make zero, or divide by zero error occurs
            'accumulator_deadband': OUT(0),

        }) for _ in range(8)],

        'analog_trigger': [{
            'has_source':   IN(False),
            'initialized':  OUT(False),
            'port':         OUT(0),
            # trigger values
            'trig_lower':   OUT(None),
            'trig_upper':   OUT(None),
            'trig_type':    OUT(None), # 'averaged' or 'filtered'
            'trig_state':   OUT(False), # TODO: might be BOTH

        } for _ in range(8)],

        # compressor control is here
        'compressor': NotifyDict({
            'has_source':           IN(False),
            'initialized':          OUT(False),
            'on':                   IN(False),
            'closed_loop_enabled':  OUT(False),
            'pressure_switch':      IN(False),
            'current':              IN(0.0)
        }),
                
        # digital stuff here
        
        # pwm contains dicts with keys: value, period_scale
        # -> value isn't sane
        'pwm': [NotifyDict({
            'initialized':  OUT(False),
            'type':         OUT(None),   # string value set by HALReport: jaguar, victor, talon, etc
            'raw_value':    OUT(0), # raw value that is used by wpilib represents the hardware PWM value
            'value':        OUT(0),      # value is the PWM value that user set from -1 to 1 (but it might not be exactly what you expect)
            'period_scale': OUT(None),
            'zero_latch':   OUT(False),

        }) for _ in range(20)],

        'pwm_loop_timing': IN(40), # this is the value the roboRIO returns
               
        # for pwm attached to a DIO
        'd0_pwm':       OUT([None]*6), # dict with keys: duty_cycle, pin
        'd0_pwm_rate':  OUT(None),
                
        'relay': [NotifyDict({
            'initialized': OUT(False),
            'fwd':         OUT(False),
            'rev':         OUT(False),

        }) for _ in range(8)],

        #Keep track of used MXP dio ports
        'mxp': [{
            'initialized': OUT(False),

        } for _ in range(16)],
                
        'dio': [NotifyDict({
            'has_source':   IN(False),
            'initialized':  OUT(False),
            'value':        IN(False), # technically both
            'pulse_length': OUT(None),
            'is_input':     OUT(False),
            'filter_idx':   OUT(None), # is None or filter number
            
        }) for _ in range(26)],
        
        # Digital glitch filter:    
        'filter': [NotifyDict({
            'enabled': OUT(False),
            'period': OUT(False),
        }) for _ in range(3)],
        
        'encoder': [{
            'has_source':         IN(False),
            'initialized':        OUT(False),
            'config':             OUT({}), # dictionary of pins/modules
            'count':              IN(0),
            'period':             IN(sys.float_info.max),
            'max_period':         OUT(0),
            'direction':          IN(False),
            'reverse_direction':  OUT(False),
            'samples_to_average': OUT(0),

        } for _ in range(4)],
        
        # There is a lot of config involved here... 
        'counter': [{
            'has_source':         IN(False),
            'initialized':        OUT(False),
            'count':              IN(0),
            'period':             OUT(sys.float_info.max),
            'max_period':         OUT(0),
            'direction':          IN(False),
            'reverse_direction':  OUT(False),
            'samples_to_average': OUT(0),
            'mode':               OUT(0),
            'average_size':       OUT(0),
            
            'up_source_channel':  OUT(0),
            'up_source_trigger':  OUT(False),
            'down_source_channel': OUT(0),
            'down_source_trigger': OUT(False),
            
            'update_when_empty':  OUT(False),
            
            'up_rising_edge':     OUT(False),
            'up_falling_edge':    OUT(False),
            'down_rising_edge':   OUT(False),
            'down_falling_edge':  OUT(False),
            
            'pulse_length_threshold': OUT(0),
            
        } for _ in range(8)],
        
        'user_program_state': OUT(None), # starting, disabled, autonomous, teleop, test

        'power': {
            'has_source':       IN(False),
            'vin_voltage':      IN(0),
            'vin_current':      IN(0),
            'user_voltage_6v':  IN(6.0),
            'user_current_6v':  IN(0),
            'user_active_6v':   IN(False),
            'user_faults_6v':   IN(0),
            'user_voltage_5v':  IN(5.0),
            'user_current_5v':  IN(0),
            'user_active_5v':   IN(False),
            'user_faults_5v':   IN(0),
            'user_voltage_3v3': IN(3.3),
            'user_current_3v3': IN(0),
            'user_active_3v3':  IN(False),
            'user_faults_3v3':  IN(0),
        },

        # solenoid values are True, False 
        'solenoid': [NotifyDict({
            'initialized': OUT(False),
            'value':       OUT(None)
        }) for _ in range(8)],

        'pdp': {
            'has_source':    IN(False),
            'temperature':   IN(0),
            'voltage':       IN(0),
            'current':       IN([0]*16),
            'total_current': IN(0),
            'total_power':   IN(0),
            'total_energy':  IN(0)
        },
        
        # The key is the device number as an integer. The value is a dictionary
        # that is specific to each CAN device
        'CAN': NotifyDict(),
    })
    
    # Ok, filter out the data into a 'both' and 'in' dictionary, removing
    # the OUT and IN objects
    _filter_hal_data(hal_data, hal_in_data)

    
def _filter_hal_data(both_dict, in_dict):
    
    for k, v in both_dict.items():
        
        if isinstance(v, IN):
            # strip 
            both_dict[k] = v.value
            in_dict[k] = copy.deepcopy(v.value)
            
        elif isinstance(v, OUT):
            # strip
            both_dict[k] = v.value
        
        elif isinstance(v, dict):
            
            v_in = {}
            _filter_hal_data(v, v_in)
            if len(v_in) > 0:
                in_dict[k] = v_in
        
        elif isinstance(v, list):
        
            v_in = _filter_hal_list(v)
            if v_in:
                in_dict[k] = v_in
        
        else:
            raise ValueError("Must be dict, list, IN or OUT; %s: %s" % (k, v))

def _filter_hal_list(both_list):
    
    in_list = []
    
    for v in both_list:
        if not isinstance(v, dict):
            raise ValueError("lists can only contain dicts, otherwise must be contained in IN or OUT")
        
        v_in = {}
        _filter_hal_data(v, v_in)
        
        if len(v_in) != 0:
            in_list.append(v_in)
    
    assert len(in_list) == 0 or len(in_list) == len(both_list)
    return in_list

def update_hal_data(in_dict, out_dict=hal_data):
    '''Given a dictionary of inputs, update the hal_data'''
    for k, v in in_dict.items():
        if isinstance(v, dict):
            update_hal_data(v, out_dict[k])
        elif isinstance(v, list):
            v_out = out_dict[k]
            for i, vv in enumerate(v):
                if isinstance(vv, dict):
                    update_hal_data(vv, v_out[i])
                else:
                    # This works, lists of lists are not allowed
                    v_out[i] = vv
        else:
            out_dict[k] = v
