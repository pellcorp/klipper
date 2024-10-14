# Perform an easy auto offset calibration with BLTouch or Probe and a Load Cell Probe
#
# Initially developed for use with BLTouch
#
# Copyright (C) 2022 Marc Hillesheim <marc.hillesheim@outlook.de>
# based on https://github.com/hawkeyexp/auto_offset_z
#
# Modified and adopted by CryoZ <hellion.zz@gmail.com>
#
# Version 0.0.2 / 18.04.2024
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Example config:
#
# [lc_auto_z_offset]
# center_xy_position: 100.0,150.0
# secondary_probe: lc

# Example run:
# LC_AUTO_Z_OFFSET NOMOVE=1 SET=0 SAVE=0 SAMPLES=3
# Where:
# NOMOVE param - disable moving to center_xy_position
# SET param - setting calculated z_offset
# SAVE param - saving z_offset to probe config, SAVE_CONFIG required
# Other params - goes unmodified to probe run command

import math

class AutoZOffsetCalibration:

    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos_center, y_pos_center = config.getfloatlist("center_xy_position",
                                                         count=2)
        self.center_x_pos, self.center_y_pos = x_pos_center, y_pos_center
        self.z_hop = config.getfloat("z_hop", default=10.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        zconfig = config.getsection('stepper_z')
        self.max_z = zconfig.getfloat('position_max', note_valid=False)
        self.speed = config.getfloat('speed', 10.0, above=0.)
        self.offsetadjust = config.getfloat('offsetadjust', 0)
        self.offset_min = config.getfloat('offset_min', -2)
        self.offset_max = config.getfloat('offset_max', 2)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.gcode.register_command("LC_AUTO_Z_OFFSET",
                                    self.cmd_AUTO_Z_OFFSET,
                                    desc=self.cmd_AUTO_Z_OFFSET_help)

        # check if a bltouch is installed
        if config.has_section("bltouch"):
            bltouch = config.getsection('bltouch')
            self.x_offset = bltouch.getfloat('x_offset', note_valid=False)
            self.y_offset = bltouch.getfloat('y_offset', note_valid=False)
            # check if a possible valid offset is set for bltouch
            if ((self.x_offset == 0) and (self.y_offset == 0)):
                raise config.error(
                    "LC_AutoZOffset: Check the x and y offset [bltouch] - it seems both are zero and the BLTouch can't be at the same position as the nozzle :-)"
                )

        # check if a probe is installed
        elif config.has_section("probe"):
            probe = config.getsection('probe')
            self.x_offset = probe.getfloat('x_offset', note_valid=False)
            self.y_offset = probe.getfloat('y_offset', note_valid=False)
            # check if a possible valid offset is set for probe
            if ((self.x_offset == 0) and (self.y_offset == 0)):
                raise config.error(
                    "LC_AutoZOffset: Check the x and y offset from [probe] - it seems both are 0 and the Probe can't be at the same position as the nozzle :-)"
                )
        else:
            raise config.error(
                "LC_AutoZOffset: No bltouch or probe in configured in your system - check your setup."
            )

        self.secondary_probe = config.get("secondary_probe", None)
        if self.secondary_probe is None:
            raise config.error("LC_AutoZOffset: No secondary probe")

    # custom round operation based mathematically instead of python default cutting off
    def rounding(self, n, decimals=0):
        expoN = n * 10**decimals
        if abs(expoN) - abs(math.floor(expoN)) < 0.5:
            return math.floor(expoN) / 10**decimals
        return math.ceil(expoN) / 10**decimals

    def cmd_AUTO_Z_OFFSET(self, gcmd):
        # check if all axes are homed
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        curpos = toolhead.get_position()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        paramoffsetadjust = gcmd.get_float('OFFSETADJUST', default=0)
        nomove = gcmd.get_int('NOMOVE', default=0)
        opt_set = gcmd.get_int('SET', default=0)
        opt_save = gcmd.get_int('SAVE', default=0)
        if ('x' not in kin_status['homed_axes']
                or 'y' not in kin_status['homed_axes']
                or 'z' not in kin_status['homed_axes']):
            raise gcmd.error("You must home X, Y and Z axes first")

        gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET",
                                                      "SET_GCODE_OFFSET",
                                                      {'Z': 0})
        self.gcode_move.cmd_SET_GCODE_OFFSET(gcmd_offset)

        gcmd.respond_info("LC_AutoZOffset: Probing main probe ...")
        if nomove == 0:
            toolhead.manual_move([
                self.center_x_pos - self.x_offset,
                self.center_y_pos - self.y_offset
            ], self.speed)
        main_session = self.printer.lookup_object('probe').start_probe_session(gcmd)
        main_session.run_probe(gcmd)
        z_probe = main_session.pull_probed_results()[0]
        main_session.end_probe_session()
        # Perform Z Hop
        if self.z_hop:
            toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

        gcmd.respond_info("LC_AutoZOffset: Probing nozzle probe ...")
        if nomove == 0:
            toolhead.manual_move([self.center_x_pos, self.center_y_pos],
                                 self.speed)
        else:
            toolhead.manual_move(
                [curpos[0] + self.x_offset, curpos[1] + self.y_offset],
                self.speed)
        nozzle_session =  self.printer.lookup_object(self.secondary_probe).start_probe_session(gcmd)
        nozzle_session.run_probe(gcmd)
        z_nozzle = nozzle_session.pull_probed_results()[0]
        nozzle_session.end_probe_session()
        # Perform Z Hop
        if self.z_hop:
            toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)
        if nomove > 0:
            toolhead.manual_move([curpos[0], curpos[1]], self.speed)
            gcmd.respond_info("LC_AutoZOffset: Moving nozzle back")

        diffbedendstop = z_probe[2] - z_nozzle[2]
        if paramoffsetadjust != 0:
            offset = self.rounding(diffbedendstop + paramoffsetadjust, 3)
            gcmd.respond_info(
                "LC_AutoZOffset:\nNozzle: %.3f\nProbe: %.3f\nDiff: %.3f\nParam Manual Adjust: %.3f\nTotal Calculated Offset: %.3f"
                % (
                    z_nozzle[2],
                    z_probe[2],
                    diffbedendstop,
                    paramoffsetadjust,
                    offset,
                ))
        else:
            offset = self.rounding(diffbedendstop + self.offsetadjust, 3)
            gcmd.respond_info(
                "LC_AutoZOffset:\nNozzle: %.3f\nProbe: %.3f\nDiff: %.3f\nConfig Manual Adjust: %.3f\nTotal Calculated Offset: %.3f"
                % (
                    z_nozzle[2],
                    z_probe[2],
                    diffbedendstop,
                    self.offsetadjust,
                    offset,
                ))

        # failsave
        if offset < self.offset_min or offset > self.offset_max:
            raise gcmd.error(
                "LC_AutoZOffset: Your calculated offset is out of config limits! (Min: %.3f mm | Max: %.3f mm) - abort..."
                % (self.offset_min, self.offset_max))

        if opt_set > 0:
            self.set_offset(offset)
        if opt_save > 0:
            self.save_offset(offset)

    cmd_AUTO_Z_OFFSET_help = "Test probe and nozzle sensor to calculate Z-offset for probe"

    def set_offset(self, offset):
        # reset possible existing offset to zero
        gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET",
                                                      "SET_GCODE_OFFSET",
                                                      {'Z': 0})
        self.gcode_move.cmd_SET_GCODE_OFFSET(gcmd_offset)
        # set new offset
        gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET",
                                                      "SET_GCODE_OFFSET",
                                                      {'Z': offset})
        self.gcode_move.cmd_SET_GCODE_OFFSET(gcmd_offset)

    def save_offset(self, offset):
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            self.gcode.respond_info(
                "probe: z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer." % (offset))
            configfile.set('probe', 'z_offset', "%.3f" % (offset, ))


def load_config(config):
    return AutoZOffsetCalibration(config)
