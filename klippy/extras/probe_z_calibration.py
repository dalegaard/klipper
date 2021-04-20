import logging

class ProbeZCalibrationHelper:
    def __init__(self, config):
        self.state = None

        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
            self.handle_connect)

        ppins = self.printer.lookup_object('pins')
        pin = config.get('nozzle_pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        mcu.register_config_callback(self._build_config)
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        self.mcu_probe = EndstopWrapper(config, self.mcu_endstop)
        query_endstops = self.printer.load_object(config, 'query_endstops')
        query_endstops.register_endstop(self.mcu_endstop, 'nozzle')
        self.printer.lookup_object('pins').register_chip('nozzle', self)

        self.calibration_site = [
            config.getfloat('calibration_x'),
            config.getfloat('calibration_y'),
            None,
        ]
        self.z_hop = config.getfloat('z_hop', 30)
        self.z_backoff = -config.getfloat('z_backoff', 0)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PROBE_Z_CALIBRATE', self.cmd_PROBE_Z_CALIBRATE,
                                    desc=self.cmd_PROBE_Z_CALIBRATE_help)

    def handle_connect(self):
        pass

    def _build_config(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.mcu_endstop.add_stepper(stepper)

    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise pins.error("Nozzle virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe


    cmd_PROBE_Z_CALIBRATE_help = "Automatically calibrate the probe's z_offset using the nozzle endstop"
    def cmd_PROBE_Z_CALIBRATE(self, gcmd):
        if self.state is not None:
            raise self.printer.command_error("Already performing PROBE_Z_CALIBRATE")
            return
        state = CalibrationState(self, gcmd)
        state.start()

class EndstopWrapper:
    def __init__(self, config, endstop):
        self.mcu_endstop = endstop
        self.position_endstop = -config.getfloat('z_backoff', 0)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

    def get_position_endstop(self):
        return self.position_endstop

class CalibrationState:
    def __init__(self, helper, gcmd):
        self.helper = helper
        self.gcmd = gcmd
        self.printer = helper.printer
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode = helper.gcode
        self.phoming = self.printer.lookup_object('homing')
        self.probe = probe = self.printer.lookup_object('probe')

    def start(self):
        self.toolhead.manual_move(self.helper.calibration_site, 50)

        # Perform nozzle zeroing
        pos = self.toolhead.get_position()
        pos[2] = -10
        self.phoming.probing_move(self.helper.mcu_probe, pos, 5)
        curpos = self.toolhead.get_position()
        self.toolhead.manual_move([None, None, curpos[2]+1], 10)
        self.phoming.probing_move(self.helper.mcu_probe, pos, 0.1)
        nozzle_zero = self.toolhead.get_position()[2] + self.helper.z_backoff

        self.toolhead.manual_move([None, None, nozzle_zero+self.helper.z_hop], 50)

        # Perform probe
        offsets = self.probe.get_offsets()
        site = list(self.helper.calibration_site)
        site[0] -= offsets[0]
        site[1] -= offsets[1]
        self.toolhead.manual_move(site, 50)
        probe_zero = self.probe.run_probe(self.gcmd)[2]

        self.toolhead.manual_move([None, None, probe_zero+self.helper.z_hop], 50)

        offset = probe_zero - nozzle_zero

        self.gcmd.respond_info(
                "z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above value and restart the printer." % (offset,)
        )
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.probe.name, 'z_offset', "%.3f" % (offset,))

def load_config(config):
    return ProbeZCalibrationHelper(config)
