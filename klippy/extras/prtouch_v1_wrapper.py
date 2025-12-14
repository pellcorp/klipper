# prtouch support
#
# Copyright (C) 2018-2021  Creality <wangyulong878@sina.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time
import socket
from . import probe
import math
import random
import threading
import datetime
import os
PR_VERSION = 307


# COMMANDS

# READ_PRES C=100
# TEST_SWAP
# DEAL_AVGS
# TRIG_TEST C=10
# SELF_CHECK_PRTOUCH
# START_STEP_PRTOUCH DIR=1 SPD=10 DIS=10
# NOZZLE_CLEAR HOT_MIN_TEMP=140 HOT_MAX_TEMP=260 BED_MAX_TEMP=100
# CHECK_BED_MESH AUTO_G29=1
# PRTOUCH_READY
# FORCE_MOVE STEPPER=stepper_x DISTANCE=5 VELOCITY=10
# PID_CALIBRATE HEATER=extruder TARGET=210
# PID_CALIBRATE HEATER=heater_bed TARGET=60
# SAFE_DOWN_Z DOWN_DIS=10 UP_DIS=5


PR_ERR_CODE_PRES_READ_DATA_TIMEOUT  = {'code':'key520', 'msg':'PR_ERR_CODE_PRES_READ_DATA_TIMEOUT: The data read interval is too large, need=11ms, actual={0}.', 'values':[]}
PR_ERR_CODE_PRES_VAL_IS_CONSTANT    = {'code':'key521', 'msg':'PR_ERR_CODE_PRES_VAL_IS_CONSTANT: The pressure data for channel={0} is incorrect. The value is constant {1}.', 'values':[]}
PR_ERR_CODE_PRES_NOT_BE_SENSED      = {'code':'key522', 'msg':'PR_ERR_CODE_PRES_NOT_BE_SENSED: The pressure data in channel={0} cannot be properly sensed.', 'values':[]}
PR_ERR_CODE_PRES_LOST_RUN_DATA      = {'code':'key523', 'msg':'PR_ERR_CODE_PRES_LOST_RUN_DATA: The pressure data is lost when the probe is over and waiting for the data to be sent back.', 'values':[]}
PR_ERR_CODE_PRES_NOISE_TOO_BIG      = {'code':'key524', 'msg':'PR_ERR_CODE_PRES_NOISE_TOO_BIG: Sensor data noise is too big, channel={0}.', 'values':[]}
PR_ERR_CODE_HAVE_LOST_STEP          = {'code':'key526', 'msg':'PR_ERR_CODE_HAVE_LOST_STEP: Z-axis motor step loss was found.', 'values':[]}
PR_ERR_CODE_STEP_LOST_RUN_DATA      = {'code':'key527', 'msg':'PR_ERR_CODE_STEP_LOST_RUN_DATA: The motor step data is lost when the probe is over and waiting for data return', 'values':[]}
PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT = {'code':'key529', 'msg':'PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT: G28 Z try probe out of times.', 'values':[]}
PR_ERR_CODE_SWAP_PIN_DETECTI        = {'code':'key532', 'msg':'PR_ERR_CODE_SWAP_PIN_DETECTI: The synchronization pin test failed, pres_swap_pin={0}, step_swap_pin={1}.', 'values':[]} 
PR_ERR_CODE_PR_NOT_TRIGGER          = {'code':'key533', 'msg':'PR_ERR_CODE_PR_NOT_TRIGGER: Prtouch Z Move But loadcell No Trigger', 'values':[]} 


MAX_PRES_CNT = 4
MAX_STEP_CNT = 4
MAX_BUF_LEN = 32


# PRTouch "endstop" wrapper
class PRTouchEndstopWrapper:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.steppers = []
        self.dbg_msg = {}
        self.shut_down = False
        self.run_step_prtouch_flag = False
        self.jump_probe_ready, self.has_save_sys_acc = False, False
        self.safe_move_z_tri_call_back, self.safe_move_z_all_cnt = None, 0
        self.mm_per_step, self.pres_tri_time, self.step_tri_time, self.pres_tri_chs, self.pres_buf_cnt = 0, 0, 0, 0, 0
        self.rdy_pos = None
        self.ovr_det_hold, self.mm_per_step = 0, None
        self.ver_step, self.ver_pres, self.ver_prth = 'V0.0', 'V0.0', ('V' + str(PR_VERSION / 100))
        self.sys_max_velocity, self.sys_max_accel, self.sys_max_z_velocity, self.sys_max_z_accel = 0, 0, 0, 0

        self.step_res, self.pres_res, self.steppers = [], [], []
        self.read_swap_prtouch_cmd, self.start_step_prtouch_cmd = None, None
        self.write_swap_prtouch_cmd, self.read_pres_prtouch_cmd, self.start_pres_prtouch_cmd = None, None, None
        self.bed_mesh, self.toolhead, self.bed_mesh, self.pheaters, self.heater_hot, self.heater_bed = None, None, None, None, None, None
        # 0. Base Cfg
        self.use_adc            = config.getboolean('use_adc', default=False)
        # 1. Tri Cfg
        self.tri_acq_ms         = config.getint('tri_acq_ms', default=(1 if self.use_adc else 12), minval=1, maxval=1000) 
        self.tri_send_ms        = config.getint('tri_send_ms', default=10, minval=1, maxval=1000) 
        self.tri_need_cnt       = config.getint('tri_need_cnt', default=1, minval=1, maxval=MAX_PRES_CNT) 
        self.tri_try_max_times  = config.getint('tri_try_max_times', default=10, minval=0) 
        self.cal_hftr_cut       = config.getfloat('cal_hftr_cut', default=10, minval=0.01, maxval=100.)        
        self.cal_lftr_k1        = config.getfloat('cal_lftr_k1', default=(0.65 if self.use_adc else 0.85))   

        self.tri_min_hold       = config.getintlist('tri_min_hold', default=([3, 3] if self.use_adc else [2000, 20000]))
        self.tri_max_hold       = config.getintlist('tri_max_hold', default=([3072, 3072] if self.use_adc else [6000, 60000]))              
        self.tri_hftr_cut       = config.getfloatlist('tri_hftr_cut', default=[2, 1])
        self.tri_lftr_k1        = config.getfloatlist('tri_lftr_k1', default=([0.50, 0.50] if self.use_adc else [0.70, 0.30]))

        # 2. Debug Cfg
        self.show_msg           = config.getboolean('show_msg', default=False)
        self.tri_wave_ip        = config.get('tri_wave_ip', None)        
        # 3. Shake Z Cfg
        self.shake_cnt          = config.getint('shake_cnt', default=8, minval=1, maxval=512)
        self.shake_range        = config.getfloat('shake_range', default=0.5, minval=0.1, maxval=2)
        # 4. Clear Nozzle Cfg
        self.hot_min_temp       = config.getfloat('hot_min_temp', default=140, minval=80, maxval=200)
        self.hot_max_temp       = config.getfloat('hot_max_temp', default=200, minval=180, maxval=300)
        self.bed_max_temp       = config.getfloat('bed_max_temp', default=60, minval=45, maxval=100)
        self.pa_clr_dis_mm      = config.getfloat('pa_clr_dis_mm', default=30, minval=2, maxval=100)
        self.pa_clr_down_mm     = config.getfloat('pa_clr_down_mm', default=-0.15, minval=-1, maxval=1)
        self.clr_noz_start_x    = config.getfloat('clr_noz_start_x', default=0, minval=0, maxval=1000)
        self.clr_noz_start_y    = config.getfloat('clr_noz_start_y', default=0, minval=0, maxval=1000)
        self.clr_noz_len_x      = config.getfloat('clr_noz_len_x', default=0, minval=self.pa_clr_dis_mm + 6, maxval=1000)
        self.clr_noz_len_y      = config.getfloat('clr_noz_len_y', default=0, minval=0, maxval=1000)    
        self.clr_xy_spd         = config.getfloat('clr_xy_spd', default=2.0, minval=0.1, maxval=10) 
        self.clr_noz_quick      = config.getboolean('clr_noz_quick', default=False)
        self.clr_quick_times    = config.getint('clr_quick_times', default=5, minval=1, maxval=10) 
        self.clr_quick_high     = config.getfloat('clr_quick_high', default=2.0, minval=0.1, maxval=10) 
        self.clr_xy_quick_spd   = config.getfloat('clr_xy_quick_spd', default=100.0, minval=10.0, maxval=300.0) 
        self.clr_quick_react_dis = config.getfloat('clr_quick_react_dis', default=5.0, minval=0.0, maxval=15.0) 
        # 5. Speed Cfg
        self.tri_z_down_spd     = config.getfloatlist('speeds', default=([10, 10] if self.use_adc else [2.5, 1.0])) # speed
        self.tri_z_up_spd       = config.getfloat('lift_speed', self.tri_z_down_spd[0] * (1.0 if self.use_adc else 2.0), minval=0.1, maxval=100)
        self.rdy_xy_spd         = config.getfloat('rdy_xy_spd', default=(300 if self.use_adc else 200), minval=1, maxval=1000)
        self.rdy_z_spd          = config.getfloat('rdy_z_spd', default=self.tri_z_up_spd, minval=1, maxval=50)
        self.acc_ctl_mm         = config.getfloat('acc_ctl_mm', (0.5 if self.use_adc else 0.25), minval=0, maxval=10)
        self.low_spd_nul        = config.getint('low_spd_nul', 5, minval=1, maxval=10)
        self.send_step_duty     = config.getint('send_step_duty', 16, minval=0, maxval=10)
        self.run_max_velocity   = config.getfloat('run_max_velocity', default=500, minval=1, maxval=5000)
        self.run_max_accel      = config.getfloat('run_max_accel', default=500, minval=1, maxval=50000)  
        self.run_max_z_velocity = config.getfloat('run_max_z_velocity', default=20, minval=1, maxval=5000)
        self.run_max_z_accel    = config.getfloat('run_max_z_accel', default=200, minval=1, maxval=50000) 
        # 6. Gap Cfg
        self.check_bed_mesh_max_err = config.getfloat('check_bed_mesh_max_err', default=0.2, minval=0.01, maxval=1)
        self.stored_profs       = config.get_prefix_sections('prtouch')
        self.stored_profs       = self.stored_profs[1] if len(self.stored_profs) == 2 else None
        # 6. Other Cfg
        self.need_self_check    = config.getboolean('need_self_check', default=True) 
        self.bed_max_err        = config.getfloat('bed_max_err', (5 if self.use_adc else 5), minval=1, maxval=10)  
        self.max_z              = config.getsection('stepper_z').getfloat('position_max', default=300, minval=10, maxval=500)        
        self.min_z              = config.getsection('stepper_z').getfloat('position_min', default=-10, minval=-20, maxval=100)        
        self.g29_down_min_z     = config.getfloat('g29_down_min_z', default=25, minval=25, maxval=500)
        self.probe_min_3err     = config.getfloat('probe_min_3err', default=0.1, minval=0.01, maxval=1)
        self.step_base          = config.getint('step_base', 1, minval=1, maxval=10) 
        self.g28_wait_cool_down = config.getboolean('g28_wait_cool_down', default=False)
        self.best_above_z       = config.getfloat('best_above_z', default=(4 if self.use_adc else 3), minval=2, maxval=10) # above 
        self.safe_z_dead_dis    = config.getfloat('safe_z_dead_dis', default=2, minval=0, maxval=10)
        # 7. Fan Cfg
        self.fan_heat_min_spd   = config.getfloat('fan_heat_min_spd', default=0.3, minval=0, maxval=255) 
        self.fan_heat_max_spd   = config.getfloat('fan_heat_max_spd', default=1.0, minval=0, maxval=255)

        self.gcode              = self.printer.lookup_object('gcode')
        self.lost_step_dis      = config.getfloat('lost_step_dis', default=0.5, minval=0, maxval=10)
        self.tilt_corr_dis      = config.getfloat('tilt_corr_dis', default=0, minval=-0.1, maxval=0.1)

        self.noz_ex_com         = config.getfloat('noz_ex_com', default=0.0, minval=0, maxval=1)    # Nozzle expansion compensation
        self.sys_time_duty      = config.getfloat('sys_time_duty', default=0.001, minval=0.00001, maxval=0.010)
        self.is_corexz          = str(config.getsection('printer').get('kinematics')) == 'corexz'

        # 快速抖动采样测试
        self.shake_sample_num         = config.getint('shake_sample_num', default=100, minval=1, maxval=1000)
        self.shake_z_velocity         = config.getint('shake_z_velocity', default=100, minval=1, maxval=1000)
        self.shake_z_accel            = config.getint('shake_z_accel', default=500, minval=1, maxval=10000)
        self.shake_num                = config.getint('shake_num', default=20, minval=1, maxval=100)
        self.check_std_max            = config.getint('check_std_max', default=self.tri_min_hold[0], minval=1, maxval=8000)
        self.check_std_mim            = config.getint('check_std_mim', default=250, minval=1, maxval=8000)


        # 1. Load Swap Pins
        self.step_swap_pin = config.get('step_swap_pin')
        self.pres_swap_pin = config.get('pres_swap_pin')
        # 1. Load Pres Pins
        self.pres_cnt = config.getint('pres_cnt', 1, 1, MAX_PRES_CNT) 
        self.pres_clk_pins, self.pres_sdo_pins, self.pres_adc_pins = [], [], []
        for i in range(self.pres_cnt):
            if self.use_adc:
                self.pres_adc_pins.append(config.get('pres%d_adc_pins' % i))
            else:
                self.pres_clk_pins.append(config.get('pres%d_clk_pins' % i))
                self.pres_sdo_pins.append(config.get('pres%d_sdo_pins' % i))
        # 2. Load Step Pins
        self.z_step_pins, self.z_dir_pins = [], []
        for s in ['stepper_z', 'stepper_x' if self.is_corexz else 'stepper_z1', 'stepper_z2', 'stepper_z3']:
            if config.has_section(s):
                sec = config.getsection(s)
                self.z_step_pins.append(sec.get('step_pin'))
                self.z_dir_pins.append(sec.get('dir_pin'))
            pass
        # 3. Creat Step And Pres Oid
        self.ppins = self.printer.lookup_object('pins')    
        self.step_mcu, self.pres_mcu = self.ppins.chips['mcu'], self.ppins.chips['mcu']
        if self.step_swap_pin and self.pres_swap_pin:
            self.step_mcu = self.ppins.parse_pin(self.step_swap_pin, True, True)['chip']
            self.pres_mcu = self.ppins.parse_pin(self.pres_swap_pin, True, True)['chip']
        self.step_oid, self.pres_oid = self.step_mcu.create_oid(), self.pres_mcu.create_oid()

        self.step_mcu.register_config_callback(self._build_step_config)
        self.pres_mcu.register_config_callback(self._build_pres_config)

        self.gcode.register_command('TEST_PRTH', self.cmd_TEST_PRTH, desc=self.cmd_TEST_PRTH_help)
        self.gcode.register_command('READ_PRES', self.cmd_READ_PRES, desc=self.cmd_READ_PRES_help)
        self.gcode.register_command('TEST_SWAP', self.cmd_TEST_SWAP, desc=self.cmd_TEST_SWAP_help)
        self.gcode.register_command('DEAL_AVGS', self.cmd_DEAL_AVGS, desc=self.cmd_DEAL_AVGS_help)
        self.gcode.register_command('TRIG_TEST', self.cmd_TRIG_TEST, desc=self.cmd_TRIG_TEST_help)    
        self.gcode.register_command('CHECK_BED_MESH', self.cmd_CHECK_BED_MESH, desc=self.cmd_CHECK_BED_MESH_help)
        self.gcode.register_command('PRTOUCH_READY', self.cmd_PRTOUCH_READY, desc=self.cmd_PRTOUCH_READY_help)     
        self.gcode.register_command('NOZZLE_CLEAR', self.cmd_NOZZLE_CLEAR, desc=self.cmd_NOZZLE_CLEAR_help)       
        self.gcode.register_command('SAFE_DOWN_Z', self.cmd_SAFE_DOWN_Z, desc=self.cmd_SAFE_DOWN_Z_help)   
        self.gcode.register_command('SAFE_MOVE_Z', self.cmd_SAFE_MOVE_Z, desc=self.cmd_SAFE_MOVE_Z_help)    
        self.gcode.register_command('ACCURATE_HOME_Z', self.cmd_ACCURATE_HOME_Z, desc=self.cmd_ACCURATE_HOME_Z_help) 
        self.gcode.register_command('SELF_CHECK_PRTOUCH', self.cmd_SELF_CHECK_PRTOUCH, desc=self.cmd_SELF_CHECK_PRTOUCH_help)
        self.gcode.register_command('START_STEP_PRTOUCH', self.cmd_START_STEP_PRTOUCH, desc=self.cmd_START_STEP_PRTOUCH_help)
        self.gcode.register_command('TRIG_BED_TEST', self.cmd_TRIG_BED_TEST, desc=self.cmd_TRIG_BED_TEST_help)    

        self.step_mcu.register_response(self._handle_step_debug_prtouch, "debug_prtouch", self.step_oid)
        self.step_mcu.register_response(self._handle_result_run_step_prtouch, "result_run_step_prtouch", self.step_oid)
        
        self.pres_mcu.register_response(self._handle_pres_debug_prtouch, "debug_prtouch", self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_run_pres_prtouch, "result_run_pres_prtouch", self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_read_pres_prtouch, "result_read_pres_prtouch", self.pres_oid)
        
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
        self.run_sta_status = 1
        self.z_set_status = False
        self.nozzle_clear_z_out_of_range = False
        self.After_G28Z_skip_bed_tilt_flag = False
        self.trig_info = [-1,-1,-1,-1]
        self.Tri_bed_test_z = []
        self.save_pres_file = None
        pass

    def _build_step_config(self):
        self.bed_mesh   = self.printer.lookup_object('bed_mesh')
        self.toolhead   = self.printer.lookup_object("toolhead")
        self.probe      = self.printer.lookup_object("probe")
        self.pheaters   = self.printer.lookup_object('heaters')
        self.heater_hot = self.printer.lookup_object('extruder').heater
        self.heater_bed = self.printer.lookup_object('heater_bed').heater

        if 'heater_fan' in self.printer.objects:
            self.fan_heat_obj = self.printer.lookup_object('heater_fan')
        if 'fan' in self.printer.objects:
            self.fan_mode_obj = self.printer.lookup_object('fan')

        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max

        self.home_x = min_x + (max_x - min_x) / 2
        self.home_y = min_y + (max_y - min_y) / 2

        if self.clr_noz_start_x <= 0 or self.clr_noz_start_y <= 0 or self.clr_noz_len_x <= 0 or self.clr_noz_len_y <= 0:
            self.clr_noz_start_x = (max_x - min_x) * 1 / 3 + min_x
            self.clr_noz_start_y = max_y - 6
            self.clr_noz_len_x = (max_x - min_x) * 1 / 3
            self.clr_noz_len_y = 5

        random.seed(time.time()) 
        self.rdy_pos = [[min_x, min_y, self.bed_max_err], [min_x, max_y, self.bed_max_err],
                        [max_x, max_y, self.bed_max_err], [max_x, min_y, self.bed_max_err]]

        self.step_mcu.add_config_cmd('config_step_prtouch oid=%d step_cnt=%d swap_pin=%s sys_time_duty=%u' 
                                     % (self.step_oid, len(self.z_step_pins), self.ppins.parse_pin(self.step_swap_pin, True, True)['pin'], int(self.sys_time_duty * 100000)))

        for i in range(len(self.z_step_pins)):
            step_par = self.ppins.parse_pin(self.z_step_pins[i], True, True)
            dir_par = self.ppins.parse_pin(self.z_dir_pins[i], True, True)
            if self.is_corexz:
                if i == 0:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], not dir_par['invert'], step_par['invert']))
                else:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))
            else:
                self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))
            
        self.read_swap_prtouch_cmd = self.step_mcu.lookup_query_command('read_swap_prtouch oid=%c', 'result_read_swap_prtouch oid=%c sta=%c', oid=self.step_oid)
        self.start_step_prtouch_cmd = self.step_mcu.lookup_command('start_step_prtouch oid=%c dir=%c send_ms=%c step_cnt=%u step_us=%u acc_ctl_cnt=%u low_spd_nul=%c send_step_duty=%c auto_rtn=%c', cq=None)
        self.manual_get_steps_cmd = self.step_mcu.lookup_query_command('manual_get_steps oid=%c index=%c', 'result_manual_get_steps oid=%c index=%c tri_time=%u tick0=%u tick1=%u tick2=%u tick3=%u step0=%u step1=%u step2=%u step3=%u', oid=self.step_oid)
        pass

    def _build_pres_config(self):
        self.pres_mcu.add_config_cmd('config_pres_prtouch oid=%d use_adc=%d pres_cnt=%d swap_pin=%s sys_time_duty=%u' 
                                     % (self.pres_oid, self.use_adc, self.pres_cnt, self.ppins.parse_pin(self.pres_swap_pin, True, True)['pin'], int(self.sys_time_duty * 100000)))
        for i in range(self.pres_cnt):
            if self.use_adc:
                adc_par = self.ppins.parse_pin(self.pres_adc_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (self.pres_oid, i, adc_par['pin'], adc_par['pin']))
            else:
                clk_par = self.ppins.parse_pin(self.pres_clk_pins[i], True, True)
                sdo_par = self.ppins.parse_pin(self.pres_sdo_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (self.pres_oid, i, clk_par['pin'], sdo_par['pin']))
   
        self.write_swap_prtouch_cmd = self.pres_mcu.lookup_query_command('write_swap_prtouch oid=%c sta=%c', 'resault_write_swap_prtouch oid=%c', oid=self.pres_oid)
        self.read_pres_prtouch_cmd = self.pres_mcu.lookup_command('read_pres_prtouch oid=%c acq_ms=%u cnt=%u', cq=None)
        self.start_pres_prtouch_cmd = self.pres_mcu.lookup_command('start_pres_prtouch oid=%c tri_dir=%c acq_ms=%c send_ms=%c need_cnt=%c tri_hftr_cut=%u tri_lftr_k1=%u min_hold=%u max_hold=%u', cq=None)
        self.deal_avgs_prtouch_cmd = self.pres_mcu.lookup_query_command('deal_avgs_prtouch oid=%c base_cnt=%c', 'result_deal_avgs_prtouch oid=%c ch0=%i ch1=%i ch2=%i ch3=%i', oid=self.pres_oid)
        
        self.manual_get_pres_cmd = self.pres_mcu.lookup_query_command('manual_get_pres oid=%c index=%c', 'resault_manual_get_pres oid=%c index=%c tri_time=%u tri_chs=%c buf_cnt=%u tick_0=%u ch0_0=%i ch1_0=%i ch2_0=%i ch3_0=%i tick_1=%u ch0_1=%i ch1_1=%i ch2_1=%i ch3_1=%i', oid=self.pres_oid)
        pass
    
    #region Handles
    def _handle_step_debug_prtouch(self, params):
        # self.print_msg('HD_STEP_DEBUG', str(params))
        self.ver_step = 'V' + str(params['version'] / 100)
        pass

    def _handle_result_run_step_prtouch(self, params):
        # self.print_msg('HD_RUN_STEP', str(params))
        self.step_tri_time = params['tri_time'] / 10000.
        for i in range(4):
            sdir = {'tick': params['tick%d' % i] / 10000., 'step': params['step%d' % i], 'index': params['index']}
            self.step_res.append(sdir)
        if self.safe_move_z_tri_call_back != None and len(self.step_res) == MAX_BUF_LEN:
            run_dis = (self.safe_move_z_all_cnt - self.step_res[-1]['step']) * self.mm_per_step
            self.safe_move_z_tri_call_back(run_dis)
            self.safe_move_z_tri_call_back = None
            self.print_msg('SAFE_MOVE_Z', 'tri_dis = %f' % run_dis, True)
        if self.run_sta_status == 0 and self.safe_move_z_tri_call_back != None :
            run_dis = (self.safe_move_z_all_cnt - self.step_res[-1]['step']) * self.mm_per_step
            self.safe_move_z_tri_call_back(run_dis)
            self.safe_move_z_tri_call_back = None
            self.run_sta_status = 1
        pass
    
    def _handle_pres_debug_prtouch(self, params):
        # self.print_msg('HD_PRES_DEBUG', str(params))
        self.ver_pres = 'V' + str(params['version'] / 100)
        pass

    def _handle_result_run_pres_prtouch(self, params):
        # self.print_msg('HD_RUN_PRES', str(params))
        self.pres_tri_time = params['tri_time'] / 10000.
        self.pres_tri_chs = params['tri_chs']
        self.pres_buf_cnt = params['buf_cnt']
        for i in range(2):
            rdir = {'tick':params['tick_%d' % i] / 10000., 'ch0':params['ch0_%d' % i], 'ch1':params['ch1_%d' % i], 'ch2':params['ch2_%d' % i], 'ch3':params['ch3_%d' % i], 'index': params['index']}
            self.pres_res.append(rdir)
        pass 

    def _handle_result_read_pres_prtouch(self, params):
        # self.print_msg('HD_READ_PRES', str(params), True)
        self.pres_res.append(params)
        pass
    #endregion

    #region System
    def get_mcu(self):
        return self.step_mcu
    
    def get_position_endstop(self):
        return 0
    
    def add_stepper(self, stepper):
        self.mm_per_step = self.steppers[0].get_step_dist() * self.step_base
        if stepper in self.steppers:
            return
        self.steppers.append(stepper)

    def get_steppers(self):
        return list(self.steppers)

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        return True

    def home_wait(self, home_end_time):
        return True
    
    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def query_endstop(self, print_time):
        return False
    
    def probe_prepare(self, hmove):
        pass

    def probe_finish(self, hmove):
        pass
    #endregion

    #region Prtouch
    def print_msg(self, title, msg, force=False):
        logging.info('[%s] %s' , title, msg)
        if not self.show_msg and not force:
            return
        if title != 'SHOW_WAVE':
            self.gcode.respond_info('[' + title + ']' + msg)
        if self.tri_wave_ip is None:
            return
        if title not in self.dbg_msg:   
            self.dbg_msg[title] = len(self.dbg_msg)
        ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ss.sendto((str(self.dbg_msg[title]) + '$' + title + '$' + time.strftime("%H:%M:%S ")  + '$' +  msg + '\n').encode(), (str(self.tri_wave_ip), 21021))
        ss.close() 
        pass

    def print_ary(self, title, ary, lent=32, pt_cnt=3, force=False):
        st = '['
        for i in range(len(ary) - lent, len(ary)):
            st = st + ("%." + ("%df, " % pt_cnt)) % (ary[i])      
        st += ']'      
        logging.info('[%s] %s' , title, st)
        if self.show_msg or force:
            self.print_msg(title, st, force)
        pass

    def print_res(self, title='None'):
        t_buf, p_buf = [], []
        for i in range(len(self.step_res)):
            t_buf.append(self.step_res[i]['tick'])
            p_buf.append(self.step_res[i]['step'])   
        self.print_ary('STEP_TICK', t_buf, len(t_buf))
        self.print_ary('STEP_DATA', p_buf, len(p_buf))

        t_buf, p_buf = [], [[], [], [], []]
        for i in range(len(self.pres_res)):
            t_buf.append(self.pres_res[i]['tick'])
            for j in range(self.pres_cnt):
                p_buf[j].append(self.pres_res[i]['ch%d' % j])

        self.print_ary('PRES_TICK', t_buf, len(t_buf))
        for i in range(self.pres_cnt):
            self.send_wave_tri(i, p_buf[i])
            self.print_ary('PRES_CH%d' % i, p_buf[i], len(p_buf[i]))
            pass
        pass

    
    def ck_and_raise_error(self, ck, err_code, vals=[]):  
        if not ck:
            return
        self.print_msg('RAISE_ERROR', "Start Raise Error")
        # 出错默认抬升热床原理碰头一段距离（默认：25mm）
        self.enable_steps()
        # now_pos = self.toolhead.get_position()
        # self.print_msg('RAISE_ERROR', "Start Raise Error Z Pos:{}".format(now_pos))
        down_min_z = 5
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(down_min_z, 10) #self.tri_z_down_spd[0]
        self.step_res = []
        self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
        t_last = time.time()
        while (time.time() - t_last < (down_min_z/10 + 10) and (len(self.step_res) != MAX_BUF_LEN)):
            self.delay_s(0.010)
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        # self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
        self.disable_steps()

        err_code['values'] = vals
        self.print_msg('RAISE_ERROR', str(err_code), True)
        raise self.printer.command_error(str(err_code))


    def send_wave_tri(self, ch, ary):
        msg = '%d' % ch
        for i in range(len(ary)):
            msg += ',%d' % ary[i]
        msg += ',%d' % self.trig_info[ch]
        self.print_msg('SHOW_WAVE', msg)
        self.trig_info[ch] = -1
        pass

    def delay_s(self, delay_s):
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        if not self.printer.is_shutdown():
            self.toolhead.get_last_move_time()
            eventtime = reactor.pause(eventtime + delay_s)
            pass

    def get_linear2(self, p1, p2, po, is_base_x):
        if (math.fabs(p1[0] - p2[0]) < 0.001 and is_base_x) or (math.fabs(p1[1] - p2[1]) < 0.001 and not is_base_x):
            return po
        a = (p2[2] - p1[2]) / ((p2[0] - p1[0]) if is_base_x else (p2[1] - p1[1]))
        b = p1[2] - (p1[0] if is_base_x else p1[1]) * a
        po[2] = a * (po[0] if is_base_x else po[1]) + b
        return po

    def ck_g28ed(self):
        for i in range(3):
            if self.toolhead.kin.limits[i][0] > self.toolhead.kin.limits[i][1]:
                self.gcode.run_script_from_command('G28')
                break
        pass

    def set_step_par(self, load_sys=True):
        if load_sys:
            self.toolhead.max_velocity = self.sys_max_velocity
            self.toolhead.max_accel = self.sys_max_accel
            self.toolhead.kin.max_z_velocity = self.sys_max_z_velocity
            self.toolhead.kin.max_z_accel = self.sys_max_z_accel
            self.has_save_sys_acc = False
            return
        if not self.has_save_sys_acc:
            self.sys_max_velocity = self.toolhead.max_velocity
            self.sys_max_accel = self.toolhead.max_accel
            self.sys_max_z_velocity = self.toolhead.kin.max_z_velocity
            self.sys_max_z_accel = self.toolhead.kin.max_z_accel

            self.toolhead.max_velocity = self.run_max_velocity
            self.toolhead.max_accel = self.run_max_accel
            self.toolhead.kin.max_z_velocity = self.run_max_z_velocity
            self.toolhead.kin.max_z_accel = self.run_max_z_accel
            self.has_save_sys_acc = True
        pass    

    def enable_steps(self):
        self.print_msg('ENABLE_STEPS', 'Start enable_steps()...')
        for stepper in self.toolhead.get_kinematics().get_steppers():
            print_time = self.toolhead.get_last_move_time()
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable = stepper_enable.lookup_enable(stepper.get_name())
            was_enable = enable.is_motor_enabled()
            if not was_enable:
                enable.motor_enable(print_time)
                self.toolhead.dwell(0.100)
                self.delay_s(0.5) 
        pass 

    def disable_steps(self):
        self.print_msg('DISABLE_STEPS', 'Start disable_steps()...')
        for stepper in self.toolhead.get_kinematics().get_steppers():
            print_time = self.toolhead.get_last_move_time()
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable = stepper_enable.lookup_enable(stepper.get_name())
            was_enable = enable.is_motor_enabled()
            if was_enable:
                enable.motor_disable(print_time)
                self.toolhead.dwell(0.100)
                self.delay_s(0.5) 
        pass 

    def move(self, pos, speed, wait=True):
        
        if not self.shut_down:
            gcmd = 'G1 F%d X%.3f Y%.3f Z%.3f' % (speed * 60, pos[0], pos[1], pos[2]) if len(pos) >= 3 else 'G1 F%d X%.3f Y%.3f' % (speed * 60, pos[0], pos[1])
            self.print_msg('PRTOUCH_MOVE', 'Start Move, gcmd=' + gcmd)
            self.gcode.run_script_from_command(gcmd)
            if wait:
                self.toolhead.wait_moves()
            pass
    #endregion

    def set_fan_speed(self, fan_name='None', fan_spd=0.):
        self.print_msg('SET_FAN_SPEED', 'fan_name=%s, fan_spd=%f' % (fan_name, fan_spd))
        if fan_name == 'heater_fan':
            for key in self.printer.objects:
                if 'heater_fan' in key:
                    self.printer.objects[key].fan.set_speed_from_command(fan_spd)
                    break

        if fan_name == 'fan':
            if 'fan' in self.printer.objects:
                self.printer.lookup_object('fan').fan.set_speed_from_command(fan_spd)
            else:
                self.gcode.run_script_from_command('M106 P0 S%d' % (fan_spd * 255))
                self.gcode.run_script_from_command('M106 P2 S%d' % (fan_spd * 255))
        pass

    def set_hot_temps(self, temp, wait=False, err=5):
        self.print_msg('SET_HOT_TEMPS', 'temp=%.2f, wait=%d, err=%f'% (temp, wait, err))
        self.pheaters.set_temperature(self.heater_hot, temp, False)
        if wait:
            while not self.shut_down and abs(self.heater_hot.target_temp - self.heater_hot.smoothed_temp) > err and self.heater_hot.target_temp > 0:
                self.delay_s(0.100) 
        pass

    def set_bed_temps(self, temp, wait=False, err=5):
        self.print_msg('SET_BED_TEMPS', 'temp=%.2f, wait=%d, err=%f'% (temp, wait, err))
        # self.pheaters.set_temperature(self.heater_bed, temp, False)           
        self.gcode.run_script_from_command('M140 S%d' % (temp))
        if wait:
            while not self.shut_down and abs(self.heater_bed.target_temp - self.heater_bed.smoothed_temp) > err and self.heater_bed.target_temp > 0:
                self.delay_s(0.100)          
        pass

    def shake_motor(self, cnt):
        self.print_msg('SHAKE_MOTOR', 'cnt=%d' % cnt)
        now_pos = self.toolhead.get_position()
        max_z_velocity = self.toolhead.kin.max_z_velocity
        for i in range(int(cnt / 2)):
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] - self.shake_range, now_pos[1] - self.shake_range, now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.25)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] + self.shake_range, now_pos[1] - self.shake_range, now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.25)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] + self.shake_range, now_pos[1] + self.shake_range, now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.25)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] - self.shake_range, now_pos[1] + self.shake_range, now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.25)))
            while len(self.toolhead.move_queue.queue) >= 5:
                self.delay_s(0.010)
        self.move(now_pos, self.rdy_z_spd)
        pass

    def quick_shake_motor(self, cnt):
        now_pos = self.toolhead.get_position()

        temp_max_z_velocity = self.toolhead.kin.max_z_velocity
        temp_max_z_accel = self.toolhead.kin.max_z_accel

        self.toolhead.kin.max_z_velocity = self.shake_z_velocity
        self.toolhead.kin.max_z_accel = self.shake_z_accel

        self.print_msg('SHAKE_MOTOR', 'shake_cnt={} shake_z_velocity={} shake_z_accel:{} shake_range:{} max_z_v:{} max_z_accel:{}'.format(cnt,self.shake_z_velocity,self.shake_z_accel,self.shake_range,temp_max_z_velocity,temp_max_z_accel))
        for i in range(cnt):
            if i % 2 == 0:
                # self.gcode.run_script_from_command('G1')
                self.gcode.run_script_from_command('G1 Z%.2f F%d' % (now_pos[2], int(60 * self.shake_z_velocity)))
            else:
                self.gcode.run_script_from_command('G1 Z%.2f F%d' % (now_pos[2] + self.shake_range, int(60 * self.shake_z_velocity)))
            # while len(self.toolhead.move_queue.queue) >= 5:
            #     self.delay_s(0.010)

        self.toolhead.kin.max_z_velocity = temp_max_z_velocity
        self.toolhead.kin.max_z_accel =    temp_max_z_accel

        self.move(now_pos, self.rdy_z_spd)
        pass
    
    # 检查z轴下探过程脉冲步数是否丢包
    # return True(成功) False(失败)
    def ck_and_manual_get_step(self):
        if len(self.step_res) == MAX_BUF_LEN:
            return True
        self.print_msg('CK_AND_MANUAL_GET_STEP', 'need={}, recv={}, step_res={}'.format(MAX_BUF_LEN, len(self.step_res),self.step_res))
        for i in range(0, MAX_BUF_LEN, 4):
            if len(self.step_res) > i and self.step_res[i]['index'] == i:
                continue
            self.print_msg('CK_AND_MANUAL_GET_STEP', 'Re trans index=%d' % i)
            params = self.manual_get_steps_cmd.send([self.step_oid, i])
            self.step_tri_time = params['tri_time'] / 10000.
            for j in range(4):
                sdir = {'tick': params['tick%d' % j] / 10000., 'step': params['step%d' % j], 'index': params['index']}
                self.step_res.insert(i + j, sdir)
        if len(self.step_res) != MAX_BUF_LEN:
            # self.ck_and_raise_error(True, PR_ERR_CODE_STEP_LOST_RUN_DATA, ["step_res len:{}".format(len(self.step_res))])
            return False
        return True

    # 检查z轴下探过程压力数据是否丢包
    # return True(成功) False(失败)
    def ck_and_manual_get_pres(self):
        if len(self.pres_res) == MAX_BUF_LEN:
            return True
        self.print_msg('CK_AND_MANUAL_GET_PRES', 'need={}, recv={}, pres_res:{}'.format(MAX_BUF_LEN, len(self.pres_res),self.pres_res))
        for i in range(0, MAX_BUF_LEN, 2):
            if len(self.pres_res) > i and self.pres_res[i]['index'] == i:
                continue
            self.print_msg('CK_AND_MANUAL_GET_PRES', 'Re trans index=%d' % i)
            params = self.manual_get_pres_cmd.send([self.step_oid, i])
            self.pres_tri_time = params['tri_time'] / 10000.
            self.pres_tri_chs = params['tri_chs']
            self.pres_buf_cnt = params['buf_cnt']
            for j in range(2):
                rdir = {'tick':params['tick_%d' % j] / 10000., 'ch0': params['ch0_%d' % j], 'ch1': params['ch1_%d' % j], 'ch2': params['ch2_%d' % j], 'ch3': params['ch3_%d' % j], 'index': params['index']}
                self.pres_res.insert(i + j, rdir)   
        if len(self.pres_res) != MAX_BUF_LEN:
            # self.ck_and_raise_error(True, PR_ERR_CODE_PRES_LOST_RUN_DATA, ["pres_res len:{}".format(len(self.pres_res))])
            return False
        return True

    def get_valid_ch(self):
        now_pos = self.toolhead.get_position()
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max  
        l_chs = []
        l_chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - min_y) ** 2) if (self.pres_tri_chs & 0x01) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - max_x) ** 2 + (now_pos[1] - min_y) ** 2) if (self.pres_tri_chs & 0x02) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - max_y) ** 2) if (self.pres_tri_chs & 0x04) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - max_y) ** 2 + (now_pos[1] - max_y) ** 2) if (self.pres_tri_chs & 0x08) else max_x * max_y * 2)
        valid_ch = l_chs.index(min(l_chs))         
        self.print_msg('VALID_CH', 'Tri_mark=%d best_ch=%d Chs=' % (self.pres_tri_chs, valid_ch) + '  ' + str(l_chs))
        return valid_ch, l_chs       

    def cal_tri_data(self, start_step, start_pos_z, step_res, pres_res, oft_z=0):   

        # 0. Send Debut Msg.
        # self.print_res()        
        # for i in range(self.pres_cnt):
        #     tmp_buf = []
        #     for j in range(len(pres_res)):
        #         tmp_buf.append(pres_res[j]['ch%d' % i])
        #     self.send_wave_tri(i, tmp_buf)
        #     pass
        # 1. Get Best Tri Ch
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        _valid_ch, l_chs = self.get_valid_ch()
        out_mms = []
        for valid_ch in range(len(l_chs)):
            if l_chs[valid_ch] == max_x * max_y * 2:
                continue
            # 2. Copy Pres Tick And Data.
            pres_t_buf, pres_d_buf = [], []
            for i in range(len(pres_res)):
                pres_t_buf.append(pres_res[i]['tick'] - self.pres_tri_time)       
                pres_d_buf.append(pres_res[i]['ch%d' % valid_ch])
                pass

            # 3. Copy Step Tick And Data.
            step_t_buf, step_d_buf = [], []
            for i in range(len(step_res)):
                step_t_buf.append(step_res[i]['tick'] - self.step_tri_time)
                step_d_buf.append(step_res[i]['step'])
                pass

            # 4. Filter The Pres Data
            if not self.use_adc:
                # for i in range(len(pres_d_buf) - 2):
                #     d3_abs = [math.fabs(pres_d_buf[i]), math.fabs(pres_d_buf[i+1]), math.fabs(pres_d_buf[i+2])]
                #     pres_d_buf[i] = pres_d_buf[i + d3_abs.index(min(d3_abs))]

                # z-score过滤毛刺算法
                # 阈值一般选2或者3
                threshold = 2
                # 计算数据标准方
                pres_data_len =len(pres_d_buf)
                pres_data_sum =sum(pres_d_buf)
                value_mean = pres_data_sum/pres_data_len
                pres_data_sum = 0
                for j in range(pres_data_len):
                    pres_data_sum += math.pow(pres_d_buf[j]-value_mean,2)
                # 标准差
                std_diff =  math.sqrt(pres_data_sum/pres_data_len)
                for i in range(1,len(pres_d_buf)-2):
                    if math.fabs(pres_d_buf[i]-value_mean)/std_diff > threshold:
                        pres_d_buf[i] = pres_d_buf[i-1]

                rc = 1. / 2. / math.pi / self.cal_hftr_cut
                coff = rc / (rc + 1. / (1000 / self.tri_acq_ms))
                _pres_d_buf = [p for p in pres_d_buf]
                pres_d_buf = [0]

                for i in range(1, len(_pres_d_buf)):
                    pres_d_buf.append((_pres_d_buf[i] - _pres_d_buf[i - 1] + pres_d_buf[-1]) * coff)
    
            for i in range(1, len(pres_d_buf)):
                pres_d_buf[i] = pres_d_buf[i - 1] * (1 - self.cal_lftr_k1) + pres_d_buf[i] * self.cal_lftr_k1
            self.print_ary('CAL_TRI_DATA_FILTER_CH%d' % valid_ch, pres_d_buf, len(pres_d_buf), 0)

            # 5. Cal The Pres Tri Index And Tick
            vals_p = [x for x in pres_d_buf]
            min_val, max_val = min(vals_p), max(vals_p)
            for i in range(len(vals_p)):
                vals_p[i] = (vals_p[i] - min_val) / (max_val - min_val)
            angle = math.atan((vals_p[-1] - vals_p[0]) / len(vals_p))
            sin_angle, cos_angle = math.sin(-angle), math.cos(-angle)
            for i in range(len(vals_p)):
                vals_p[i] = (i - 0) * sin_angle + (vals_p[i] - 0) * cos_angle + 0
            # self.print_ary('SINCOS', 'sincos = ', vals_p, len(vals_p), 2)
            pres_tri_index = vals_p.index(min(vals_p))
            pres_tri_tick = pres_t_buf[pres_tri_index]
            # 6. Cal The Step Tri Index And Tick
            step_tri_index = MAX_BUF_LEN - 1
            step_tri_tick = step_t_buf[step_tri_index]
            for i in range(len(step_t_buf) - 1):
                if ((step_t_buf[i] <= pres_tri_tick <= step_t_buf[i + 1]) or (step_t_buf[i] == pres_tri_tick)) and step_d_buf[i] != 0:
                    step_tri_index = i
                    step_tri_tick = step_t_buf[step_tri_index]
                    break
            out_step = step_d_buf[-1]
            if 0 < step_tri_index < len(step_res) - 1:
                out_step = step_d_buf[step_tri_index] + (step_d_buf[step_tri_index + 1] - step_d_buf[step_tri_index]) * (pres_tri_tick - step_tri_tick) / (step_t_buf[step_tri_index + 1] - step_t_buf[step_tri_index])
            cal_tri_z = (start_step - out_step) * self.mm_per_step
            out_val_mm = start_pos_z - cal_tri_z + oft_z
            self.print_msg('TRI_PRES_MSG', 'start_pres={:.3f}, tri_pres={:.3f}, end_pres={:.3f}, pres_tri_index={:.3f}, pres_tri_tick={:.3f}'.format(pres_d_buf[0], pres_d_buf[pres_tri_index], pres_d_buf[-1], pres_tri_index, pres_tri_tick))
            self.print_msg('TRI_STEP_MSG', 'start_step={:.3f}, tri_step={:.3f}, end_step={:.3f}, step_tri_index={:.3f}, step_tri_tick={:.3f}'.format(start_step,    out_step,                   step_d_buf[-1], step_tri_index, step_tri_tick))
            self.print_msg('TRI_OUT_MM', 'mm_pre_step={:.3f}, out_mm={:.3f} start_pos_z={:.3f} cal_tri_z={:.3f} oft_z={:.3f}'.format(self.mm_per_step, out_val_mm,start_pos_z,cal_tri_z,oft_z))
            out_mms.append(out_val_mm)
            self.trig_info[valid_ch]=pres_tri_index
        self.print_res()   
        self.print_ary('OUT_MMS', out_mms, len(out_mms), 2)
        return sum(out_mms) / len(out_mms) # out_val_mm

    def get_step_cnts(self, run_dis, run_spd):
        if not self.mm_per_step:
            self.get_mm_per_step()
        step_cnt = int(run_dis / self.mm_per_step)
        if step_cnt == 0:
            msg = "run_dis:%s mm_per_step:%s step_cnt:%s run_spd:%s" % (run_dis, self.mm_per_step, int(run_dis / self.mm_per_step), run_spd)
            self.print_msg('get_step_cnts', msg)
            return step_cnt, 0, 0
        step_us = int(((run_dis / run_spd) * 1000 * 1000) / step_cnt)
        acc_ctl_cnt = int(self.acc_ctl_mm / self.mm_per_step)
        self.print_msg('GET_STEP_CNTS', 'run_dis=%.2f, run_spd=%.2f, step_cnt=%d, step_us=%d acc_ctl_cnt=%d' % (run_dis, run_spd, step_cnt, step_us, acc_ctl_cnt))
        return step_cnt, step_us, acc_ctl_cnt
    
    def get_best_rdy_z(self, rdy_x, rdy_y, base_pos=None):
        if not base_pos:
            base_pos = self.rdy_pos
        p_left = [base_pos[0][0], rdy_y, 0]
        p_right = [base_pos[2][0], rdy_y, 0]
        p_mid = [rdy_x, rdy_y, 0]
        p_left = self.get_linear2(base_pos[0], base_pos[1], p_left, False)
        p_right = self.get_linear2(base_pos[2], base_pos[3], p_right, False)
        p_mid = self.get_linear2(p_left, p_right, p_mid, True)
        self.print_msg('GET_BEST_RDY_Z', "Src=%s, x=%.2f, y=%.2f, cal_z=%.2f" % (('RDY' if base_pos == self.rdy_pos else 'GAP'), rdy_x, rdy_y, p_mid[2]))
        return p_mid[2] if p_mid[2] < self.bed_max_err else self.bed_max_err
    
    def get_mm_per_step(self):
        if self.mm_per_step:
            return
        for stepper in self.toolhead.get_kinematics().get_steppers():
            if stepper.is_active_axis('z'):
                self.mm_per_step = self.step_base * stepper.get_step_dist()                
                self.print_msg('GET_MM_PER_STEP', str(stepper.get_step_dist()))
        pass

    def run_to_next(self, nextpos, wait=True):
        if self.After_G28Z_skip_bed_tilt_flag == False:
            self.print_msg('RUN_TO_NEXT', 'nextpos=%s, wait=%d' % (str(nextpos), wait))
            nextpos_z = self.get_best_rdy_z(nextpos[0], nextpos[1]) + self.best_above_z
        # 归零后第一次调用，z太高到最大网床倾斜高度
        else:
            nextpos_z = self.bed_max_err
            self.print_msg('RUN_TO_NEXT', 'After G28Z first run fun(run_to_next) skip bed tilt flag:{} nextpos:{} cal_nextpos_z:{}'.format(self.After_G28Z_skip_bed_tilt_flag,nextpos,nextpos_z))
        self.move(nextpos[:2] + [nextpos_z], self.rdy_xy_spd, wait)
        pass

    # TODO 传感器自检
    # return True(自检成功) False(自检存在错误)
    def env_self_check(self,error_num=1,force=False):
        # 1. PR_ERR_CODE_SWAP_PIN_DETECTI
        # 下位机设置了同步线为高电平时，调用start_step_prtouch_cmd接口时，无法运行电机，因此测试时
        # 此项检查同步线信号是否正常。通过发送指令让同步线写高点平、低电平，并读取io电平校验是否正确

        # 如有异常。重新检测一次
        for check_times in range(10):
            static_check_error_state = 0
            self.delay_s(0.2)
            self.print_msg('WHY_DEBUG', "Static Check Sensor data! times:{}".format(check_times))
            self.write_swap_prtouch_cmd.send([self.pres_oid, 1])
            params1 = self.read_swap_prtouch_cmd.send([self.step_oid])
            self.write_swap_prtouch_cmd.send([self.pres_oid, 0])
            params0 = self.read_swap_prtouch_cmd.send([self.step_oid])
            if not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1:
                self.print_msg('DEBUG', '--Self Test 1 = PR_ERR_CODE_SWAP_PIN_DETECTI, Error!! params0:{} params1:{}--'.format(params0,params1), force) 
                static_check_error_state = 1
                continue
                # self.ck_and_raise_error(True, PR_ERR_CODE_SWAP_PIN_DETECTI)
            else: 
                self.print_msg('DEBUG', '--Self Test 1 = PR_ERR_CODE_SWAP_PIN_DETECTI, Pass!!--', force) 

            if static_check_error_state == 1:
                self.ck_and_raise_error(True, PR_ERR_CODE_SWAP_PIN_DETECTI)
                return False

            # 以下之间方案不适用于压电陶瓷传感器
            if not self.need_self_check and not force or self.use_adc:
                return True

            # 2. PR_ERR_CODE_PRES_READ_DATA_TIMEOUT
            # 此项检查读取压力传感器数据是否超时，判断通讯、传感器采样率是否正常。默认应变片使用HX711 12ms采集一个数据
            # 采集压力传感器数据前，要进行一次求平均去皮操作。8为采集8个数据
            # self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])
            self.print_msg('AVGS_RESAULT', str(params))
            self.pres_res = []
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 32 + 8])
            start_tick_s = time.time()
            # 大概等1s,期间判断接收到的数据长度是否达到指定长度
            while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * 64)) and len(self.pres_res) < 32:
                self.delay_s(0.010)
            # 发送停止读取压力传感器数据指令
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 0])
            # 校验通讯是否丢包，如果错误，发送错误
            if len(self.pres_res) < 32:
                static_check_error_state = 2
                continue
            # self.ck_and_raise_error(len(self.pres_res) < 32, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT)

            pnt_tick, pnt_vals = [], [[], [], [], []]
            # 开头和结尾各去掉4个数据
            for i in range(4, len(self.pres_res) - 4):
                pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
                for j in range(self.pres_cnt):
                    pnt_vals[j].append(self.pres_res[i]['ch%d' % j])
            tr = 0
            for i in range(1, len(pnt_tick)):
                tr += pnt_tick[i] - pnt_tick[i - 1]
            # self.print_msg('SELF_pres_res', "pres_res:{}".format(self.pres_res))
            self.print_msg('SELF_CHECK_TICK', "TICK:{}".format(pnt_tick))
            self.print_msg('SELF_CHECK_DATA', "DATA:{}".format(pnt_vals))
            # 发送数据上位机显示波形
            for i in range(len(pnt_vals)):
                self.send_wave_tri(i,pnt_vals[i])
            # 校验采样率 判断采样间隔时间是否超过2倍触发时间（12ms）
            if tr / (len(pnt_tick) - 1) > 2 * self.tri_acq_ms:
                static_check_error_state = 2
                self.print_msg('DEBUG', '--Self Test 2 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Error!!-- error time:{} Normal:{}'.format(tr / (len(pnt_tick) - 1) , 2 * self.tri_acq_ms), force)
                continue
                # self.ck_and_raise_error(tr / (len(pnt_tick) - 1) > 2 * self.tri_acq_ms, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT)
            else:
                self.print_msg('DEBUG', '--Self Test 2 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Pass!!--', force)
            
            # 3. PR_ERR_CODE_PRES_VAL_IS_CONSTANT
            # 此项检查压力传感器是否出现一个恒定值情况。检查传感器是否正常
            # 用于确定哪个通道数据异常
            IS_CONSTANT_CH = []
            for i in range(self.pres_cnt): 
                # sums, avg = 0, sum(pnt_vals[i]) / len(pnt_vals[i])
                sums = 0
                for j in range(1,len(pnt_vals[i])):
                    # pnt_vals[i][j] -= avg
                    # 判断相邻两个数据都为相同数据
                    if pnt_vals[i][j] == pnt_vals[i][j-1]:
                        sums += 1
                    else:
                        sums = 0
                # 判断超过一半数据连续相等。说明存在数据异常
                if sums>len(pnt_vals[i])/2:
                    IS_CONSTANT_CH.append(i)   
            if len(IS_CONSTANT_CH) >= error_num:
                static_check_error_state = 3
                self.print_msg('DEBUG', '--Self Test 3 = PR_ERR_CODE_PRES_VAL_IS_CONSTANT, Error!!-- IS_CONSTANT_CH:{}'.format(IS_CONSTANT_CH), force)    
                continue
                # self.ck_and_raise_error(True, PR_ERR_CODE_PRES_VAL_IS_CONSTANT,IS_CONSTANT_CH)
            else:
                self.print_msg('DEBUG', '--Self Test 3 = PR_ERR_CODE_PRES_VAL_IS_CONSTANT, Pass!!--', force)    

            # 这项容易误触发。旁边如果环境存在震动。容易误判
            # 这项检查传感器数据是否存在较大噪声。
            # 4. PR_ERR_CODE_PRES_NOISE_TOO_BIG
            # 用于确定哪个通道数据异常
            TOO_BIG_CH = []
            for i in range(self.pres_cnt):
                big_cnt = 0
                for j in range(len(pnt_vals[i])):
                    big_cnt += (1 if abs(pnt_vals[i][j]) > (self.tri_min_hold[0]*2 if not self.use_adc else 200) else 0)
                if big_cnt > len(pnt_vals[i])/2:
                    TOO_BIG_CH.append(i)
            if len(TOO_BIG_CH) >= error_num:
                static_check_error_state = 4
                self.print_msg('DEBUG', '--Self Test 4 = PR_ERR_CODE_PRES_NOISE_TOO_BIG, Error!!-- TOO_BIG_CH:{}'.format(TOO_BIG_CH), force=force)
                continue
                # self.ck_and_raise_error(True, PR_ERR_CODE_PRES_NOISE_TOO_BIG,TOO_BIG_CH)
            else:
                self.print_msg('DEBUG', '--Self Test 4 = PR_ERR_CODE_PRES_NOISE_TOO_BIG, Pass!!--', force=force)
                break
        
        # 根据错误状态报错
        if static_check_error_state == 2:
            self.ck_and_raise_error(True, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT)
        elif static_check_error_state == 3:
            self.ck_and_raise_error(True, PR_ERR_CODE_PRES_VAL_IS_CONSTANT,IS_CONSTANT_CH)
        elif static_check_error_state == 4:
            self.ck_and_raise_error(True, PR_ERR_CODE_PRES_NOISE_TOO_BIG,TOO_BIG_CH)
        if static_check_error_state != 0:
            return False

        # 抖动检测
        for check_times in range(2):
            self.delay_s(0.2)
            self.print_msg('WHY_DEBUG', "Shake Check Sensor data! times:{}".format(check_times)) 
            # 5. PR_ERR_CODE_PRES_NOT_BE_SENSED
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
            now_pos = self.toolhead.get_position()
            self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
            self.pres_res = []
            self.prtouch_shake_check_error = False
            # 采集数据长度。100个
            datalen = self.shake_sample_num  #100
            # 由于从发送采集数据开始到shake_motor期间有时延，这是已经采集了一部分传感器数据。如果采集数据量太少则采集不到振动部分的结果
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, datalen+25])
            # self.shake_motor(self.shake_cnt)
            self.quick_shake_motor(self.shake_num) #20
            while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * datalen)) and len(self.pres_res) < datalen:
                self.delay_s(0.010)
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 0])
            if len(self.pres_res) < 32:
                self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Error!!-- read {} pres data len {} < 32'.format(datalen,len(self.pres_res)), force) 
                if check_times != 0:
                    self.ck_and_raise_error(True, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT) 
                    return False   
                continue

            pnt_tick, pnt_vals = [], [[], [], [], []]
            # 剔除电机振动前的采样数据，大约在第25个开始
            for i in range(25, len(self.pres_res)):
                pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
                for j in range(self.pres_cnt):
                    pnt_vals[j].append(self.pres_res[i]['ch%d' % j])
            # 发送数据上位机显示波形
            for i in range(len(pnt_vals)):
                self.send_wave_tri(i,pnt_vals[i])
                self.print_msg('SELF_CHECK_DATA', "SHAKE DATA CH[{}]:{}".format(i,pnt_vals[i]))


            # 方差判断法,由于传感器静止状态采集到的数据皆为底噪。波动比较小因此标准差比较小 250以内
            # 如果让热床快速抖动起来，传感器数据呈现较为有规律的上下波动标准差比较大 K1设备（250-2000） K1max设备（250-4000）
            # 异常情况：
            # 1、传感器线没插入座子（四条线断开）。标准差很小250以内
            # 2、抖动过程中，外部连续点击热床（比如手点击热床）或者不均匀外力持续按压(比如手按着热床)。标准差会超过2000
            NO_SENSED_CH = []
            # 判断标准差
            for i in range(self.pres_cnt):
                # 排序
                pnt_vals[i].sort()
                pnt_vals[i] = pnt_vals[i][3:(len(pnt_vals[i])-3)]
                # 计算数据标准方
                pres_data_len =len(pnt_vals[i])
                pres_data_sum =sum(pnt_vals[i])
                pres_mean = pres_data_sum/pres_data_len
                pres_data_sum = 0
                for j in range(pres_data_len):
                    pres_data_sum += math.pow(pnt_vals[i][j]-pres_mean,2)
                # 标准差
                pres_std =  math.sqrt(pres_data_sum/pres_data_len)
                # 判断传感器线是否断开及是否有不均匀外力压着热床
                if pres_std < self.check_std_mim:
                    NO_SENSED_CH.append(i)
                # self.print_msg('DEBUG',"pres_res[{}]:{}".format(i,pnt_vals[i]))
                self.print_msg('DEBUG'," pres_std[{}]:{:.2f} normal:[{}]".format(i,pres_std,self.check_std_mim))
            if len(NO_SENSED_CH)>=4:
                self.prtouch_shake_check_error = True
            # 如果抖动出现异常，再重新抖动采集一次，如果还有异常跳过去探测一次检测
            if len(NO_SENSED_CH) != 0:
                self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_NOT_BE_SENSED, Error!!-- Error CH:{}'.format(NO_SENSED_CH), force) 
                continue
            else:
                break
        # 如果存在任意通道抖动检测出异常则进入点击探测验证。如果四个通道都异常直接报错，不点击
        if len(NO_SENSED_CH) > 0 and len(NO_SENSED_CH) < 4:
            # 如果抖动检测到有问题，则z轴探测一次
            use_tri_z_down_spd = self.tri_z_down_spd[0]
            use_tri_hftr_cut = self.tri_hftr_cut[0]
            use_tri_lftr_k1 = self.tri_lftr_k1[0]
            use_tri_min_hold = self.tri_min_hold[0]
            use_tri_max_hold = self.tri_max_hold[0]
            # 连续尝试探测2次。如果都检测异常则上报
            for move_times in range(2):
                self.enable_steps()
                self.print_msg('WHY DEBUG', 'check sensor sensitivity')
                self.step_res, self.pres_res = [], []
                params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])
                self.print_msg('AVGS_RESAULT', str(params))
                step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(self.max_z * 1.2, use_tri_z_down_spd * (1.2 if self.use_adc else 2.0))
                self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                                int(use_tri_hftr_cut * 1000),  int(use_tri_lftr_k1 * 1000), int(use_tri_min_hold), int(use_tri_max_hold)])
                self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
                t_last = time.time()
                while (time.time() - t_last < (self.max_z * 2.0) / (use_tri_z_down_spd * (1.2 if self.use_adc else 2.0))) and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                    self.delay_s(0.010)
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
                self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])

                # 长度为0说明下探过程中没有压力传感器未被触发,继续下探一次
                if len(self.pres_res) == 0 or len(self.step_res) == 0:
                    self.print_msg('WHY DEBUG', '--Self Test 5 bed move max_z*1.2 is not probe,move again--')
                    continue
                
                # 抬升
                self.step_res = []
                z_up_mm = self.best_above_z * 2
                step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(z_up_mm, use_tri_z_down_spd * (1.2 if self.use_adc else 2.0))
                self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
                t_last = time.time()
                while (time.time() - t_last < (z_up_mm/use_tri_z_down_spd + 5) and (len(self.step_res) != MAX_BUF_LEN)):
                    self.delay_s(0.010)
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])

                # 判断数据合法性
                if len(self.pres_res) != MAX_BUF_LEN:
                    self.print_msg('WHY DEBUG', '--Self Test 5 pres data len {} < 32--'.format(len(self.pres_res)))
                    continue

                # 压力数据处理
                pnt_vals = [[], [], [], []]
                NO_SENSED_CH = []
                for j in range(self.pres_cnt):
                    for i in range(len(self.pres_res)):
                        pnt_vals[j].append(self.pres_res[i]['ch%d' % j])
                    # self.print_msg('DEBUG'," pnt_vals[{}][-1]:{} normal:[{}]".format(j,pnt_vals[j],self.check_std_mim))
                # 高通滤波
                rc = 1. / 2. / math.pi / self.cal_hftr_cut
                coff = rc / (rc + 1. / (1000 / self.tri_acq_ms))
                NO_SENSED_CH = []
                for j in range(len(pnt_vals)):
                    pres_d_buf = [0]
                    for i in range(1, len(pnt_vals[j])):
                        pres_d_buf.append(round((pnt_vals[j][i] - pnt_vals[j][i - 1] + pres_d_buf[-1]) * coff,2))
                    pnt_vals[j] = pres_d_buf
                    # 判断最后个数据小于拔出线后的经验阈值，说明线断了
                    if (pnt_vals[j][-1] < self.check_std_mim):
                        NO_SENSED_CH.append(j)
                    self.print_msg('DEBUG'," pnt_vals[{}][-1]:{} normal:[{}]".format(j,pnt_vals[j],self.check_std_mim))

                if len(NO_SENSED_CH)>=error_num:
                    self.prtouch_shake_check_error = True
                    continue
                else:
                    self.prtouch_shake_check_error = False


                # 发送数据上位机显示波形
                for i in range(len(pnt_vals)):
                    self.send_wave_tri(i,pnt_vals[i])
                # 退出探测
                break
        
        if len(NO_SENSED_CH) != 0 and self.prtouch_shake_check_error == True:
            self.ck_and_raise_error(True, PR_ERR_CODE_PRES_NOT_BE_SENSED,NO_SENSED_CH)
            return False
        self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_NOT_BE_SENSED, Pass!!--', force) 
        return True

    def probe_ready(self):
        self.print_msg('PROBE_READY', 'Start probe_ready()...')
        self.After_G28Z_skip_bed_tilt_flag = False
        if self.jump_probe_ready:
            self.jump_probe_ready = False
            return False
        self.ck_g28ed()
        for i in range(len(self.rdy_pos)):
            self.rdy_pos[i][2] = self.bed_max_err
        now_pos = self.toolhead.get_position()
        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.rdy_z_spd)
        for i in range(4):
            self.print_msg('PROBE_READY', 'Start Probe Point=%s' % str(self.rdy_pos[i]))
            self.move(self.rdy_pos[i], self.rdy_xy_spd)   
            self.rdy_pos[i][2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:probe_ready run_step_prtouch() ERROR")
                return False
        self.print_msg('RDY_POS', "[00=%.2f, 01=%.2f, 11=%.2f, 10=%.2f]" % (self.rdy_pos[0][2], self.rdy_pos[1][2], self.rdy_pos[2][2], self.rdy_pos[3][2]))
        return True

    def report_run_dis(self, run_dis):
        self.v_sd.run_dis = run_dis

    def safe_move_z(self, run_sta, run_dis, run_spd, run_rdo, tri_call_back = None):
        self.print_msg('SAFE_MOVE_Z', 'run_sta=%d, run_dis=%f, run_spd=%f, run_rdo=%f' % (run_sta, run_dis, run_spd, run_rdo))
        run_dir = 1 if run_dis > 0 else 0
        run_dis = math.fabs(run_dis)   

        if tri_call_back:
            self.safe_move_z_tri_call_back = tri_call_back    
        else:
            self.safe_move_z_tri_call_back = self.report_run_dis  

        self.start_step_prtouch_cmd.send([self.step_oid, run_dir, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        self.start_pres_prtouch_cmd.send([self.pres_oid, run_dir, 0, 0, 0, 0, 0, 0, 0])

        if run_sta == 1:
            self.step_res, self.pres_res = [], []
            self.enable_steps()            
            if run_spd == 0 or run_dis == 0 or run_rdo == 0:
                return
            
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            step_cnt, step_us, acc_ctl = self.get_step_cnts(run_dis, run_spd)
    
            if step_cnt == 0 or step_us == 0 or acc_ctl == 0:
                return
            self.safe_move_z_all_cnt = step_cnt
            self.start_pres_prtouch_cmd.send([self.pres_oid, run_dir, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                              int(self.tri_hftr_cut[0] * 1000), int(self.tri_lftr_k1[0] * 1000) if self.use_adc else int(self.tri_lftr_k1[0] * 1000 / run_rdo), 
                                              self.tri_min_hold[0] if self.use_adc else int(self.tri_min_hold[0] * run_rdo), self.tri_max_hold[0] if self.use_adc else int(self.tri_max_hold[0] * run_rdo)])
            self.start_step_prtouch_cmd.send([self.step_oid, run_dir, self.tri_send_ms, step_cnt, step_us, acc_ctl, self.low_spd_nul, self.send_step_duty, 0])
        pass
    
    # 精准探测
    # 连续两次探测过程数据丢包则会报False
    # return out_mm flag(True[成功] False[失败])
    # out_mm 输出探测结果与目标高度差，如果flag为False 默认输出999
    def run_step_prtouch(self, down_min_z, probe_min_3err, rt_last=False, pro_cnt=3, crt_cnt=3, fast_probe=False, re_g28=False):
        use_tri_times = 0
        ck_step_error_cnt = 0
        ck_pres_error_cnt = 0
        use_tri_z_down_spd = self.tri_z_down_spd[0]
        use_tri_hftr_cut = self.tri_hftr_cut[0]
        use_tri_lftr_k1 = self.tri_lftr_k1[0]
        use_tri_min_hold = self.tri_min_hold[0]
        use_tri_max_hold = self.tri_max_hold[0]

        self.print_msg('RUN_STEP_PRTOUCH', "tri_z_down_spd=%.2f, tri_hftr_cut=%.2f, tri_lftr_k1=%.2f, tri_min_hold=%.0f, tri_max_hold=%.0f"
                            % (use_tri_z_down_spd, use_tri_hftr_cut, use_tri_lftr_k1, use_tri_min_hold, use_tri_max_hold))
        
        res_z = []
        now_pos = self.toolhead.get_position()
        lost_min_cnt = 0
        FORCE_MOVE_MAX = 0
        Z_RefreshFlag = False
        while True:
            self.step_res, self.pres_res = [], []
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(down_min_z, use_tri_z_down_spd)
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                              int(use_tri_hftr_cut * 1000), int(use_tri_lftr_k1 * 1000), int(use_tri_min_hold), int(use_tri_max_hold)])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while (time.time() - t_last < (down_min_z / use_tri_z_down_spd + 2)) and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])
            if len(self.step_res) == 0 or len(self.pres_res) == 0:
                if re_g28:
                    self.move(now_pos[:2] + [100, now_pos[3]], self.rdy_z_spd)
                    self.run_G28_Z(True)
                    self.probe_ready()
                    self.move(now_pos, self.rdy_z_spd)
                else:
                    FORCE_MOVE_MAX += 1
                    # 只有G28Z和擦嘴才使能刷新z高度平面
                    if self.z_set_status == True:
                        # self.print_msg('WHY DEBUG',"run FORCE_MOVE STEPPER=stepper_z DISTANCE=5 VELOCITY=5 times:{}".format(FORCE_MOVE_MAX))
                        # self.gcode.run_script_from_command('FORCE_MOVE STEPPER=stepper_z DISTANCE=5 VELOCITY=5')

                        # 出现下探过程没触发时，以结束位置为新的高度平面 
                        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
                        # 抬升一下z轴
                        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.tri_z_up_spd)
                        self.toolhead.wait_moves()
                        now_pos = self.toolhead.get_position()
                        Z_RefreshFlag = True
                        self.print_msg('WHY DEBUG',"Prtouch Z Move {}mm but loadcell No Trigger times:{},reset z level and prtouch z again!".format(down_min_z,FORCE_MOVE_MAX))
                    else:
                        self.print_msg('WHY DEBUG',"Prtouch Z Move {}mm but loadcell No Trigger times:{}".format(down_min_z,FORCE_MOVE_MAX))
                        self.ck_and_raise_error(True, PR_ERR_CODE_PR_NOT_TRIGGER, ["Z Move {}mm but loadcell No Trigger times:{}".format(down_min_z,FORCE_MOVE_MAX)])
                continue
            
            if not self.ck_and_manual_get_step():
                ck_step_error_cnt += 1
                self.print_msg('WHY ERROR',"FUN:run_step_prtouch ck_and_manual_get_step() ERROR Times:{}",ck_step_error_cnt)
                # 连续连续2次出现丢包就报error
                if ck_step_error_cnt > 2:
                    self.ck_and_raise_error(True, PR_ERR_CODE_STEP_LOST_RUN_DATA, ["step_res len:{}".format(len(self.step_res))])
                    return 999,False
            else:
                ck_step_error_cnt = 0

            if not self.ck_and_manual_get_pres():
                ck_pres_error_cnt += 1
                self.print_msg('WHY ERROR',"FUN:run_step_prtouch ck_and_manual_get_pres() ERROR Times:{}",ck_pres_error_cnt)
                # 连续连续2次出现丢包就报error
                if ck_pres_error_cnt > 2:
                    self.ck_and_raise_error(True, PR_ERR_CODE_PRES_LOST_RUN_DATA, ["pres_res len:{}".format(len(self.pres_res))])
                    return 999,False
            else:
                ck_pres_error_cnt = 0

            # self.ck_and_manual_get_step()
            # self.ck_and_manual_get_pres()

            # 擦嘴时探测高度小于最小z范围
            if self.nozzle_clear_z_out_of_range == True:
                self.print_msg('WHY DEBUG',"nozzle clear z less than z_min:{},reset z level and prtouch z again!".format(self.min_z))
                self.nozzle_clear_z_out_of_range = False
                Z_RefreshFlag = True

            # 下探过程中由于没有触发刷新过z轴的参考0平面后
            # 的第一次触发位置会被重新设置为z参考0面高度。
            if Z_RefreshFlag == True:
                self.print_msg('WHY DEBUG',"refresh z home position now_pos{}".format(now_pos))
                # 以结束位置为新的高度平面 
                self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
                # 抬升一下z轴
                self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.tri_z_up_spd)
                self.toolhead.wait_moves()
                now_pos = self.toolhead.get_position()
                # 中途误触发测到的高度数据不可用，清零
                res_z.clear()
                use_tri_times = 0
                Z_RefreshFlag = False
                continue

            can_rt = False
            if len(self.pres_res) >= MAX_BUF_LEN or use_tri_times > 20:
                step_par_down, pres_par_down, tri_time_down = [x for x in self.step_res], [x for x in self.pres_res], [self.pres_tri_time, self.step_tri_time]                
                res_z.append(self.cal_tri_data(step_cnt_down, now_pos[2], step_par_down, pres_par_down, -lost_min_cnt * self.mm_per_step))
                can_rt = True if (len(res_z) == 2 and not self.use_adc and math.fabs(res_z[0] - res_z[1]) < 0.05 and crt_cnt != pro_cnt) else False
                can_rt = True if can_rt else (len(res_z) >= (crt_cnt - 1)) and (max(res_z) - min(res_z) <= probe_min_3err)

            else:
                if use_tri_times <= 20 and use_tri_times >= 5:
                    rd = (use_tri_times - 5) / 15.
                    use_tri_z_down_spd = (self.tri_z_down_spd[1] - self.tri_z_down_spd[0]) * rd + self.tri_z_down_spd[0]
                    use_tri_hftr_cut = (self.tri_hftr_cut[1] - self.tri_hftr_cut[0]) * rd + self.tri_hftr_cut[0]
                    use_tri_lftr_k1 = (self.tri_lftr_k1[1] - self.tri_lftr_k1[0]) * rd + self.tri_lftr_k1[0]
                    use_tri_min_hold = (self.tri_min_hold[1] - self.tri_min_hold[0]) * rd + self.tri_min_hold[0]
                    use_tri_max_hold = (self.tri_max_hold[1] - self.tri_max_hold[0]) * rd + self.tri_max_hold[0]
                if use_tri_times % 5 == 0:
                    self.shake_motor(self.shake_cnt)

            up_min_z = (step_cnt_down - self.step_res[-1]['step']) * self.mm_per_step
            self.print_msg("WHY DEBUG","run_step_prtouch up_min_z:{} can_rt:{}".format(up_min_z,can_rt))
            step_cnt_up, step_us_up, acc_ctl_cnt = self.get_step_cnts(up_min_z, self.tri_z_up_spd)
            step_cnt_up = int(step_cnt_down - self.step_res[-1]['step'])

            if fast_probe and pro_cnt > 2 and len(res_z) > 0: 
                if len(res_z) == 1 and up_min_z > self.best_above_z / 2:
                    _step_cnt_up = int((self.best_above_z / 2) / self.mm_per_step)
                    lost_min_cnt = step_cnt_up - _step_cnt_up
                    step_cnt_up = int(_step_cnt_up)
                elif len(res_z) >= pro_cnt or can_rt:
                    step_cnt_up += int(lost_min_cnt)

            self.step_res, self.pres_res = [], []
            self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_up, step_us_up, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while (time.time() - t_last < (down_min_z / use_tri_z_down_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            # 抬升不用ck
            # self.ck_and_manual_get_step()
            if len(res_z) >= pro_cnt or can_rt:
                break
            use_tri_times += 1
        # res_z.sort()
        # self.print_ary('RES_Z', res_z, len(res_z))
        # return res_z[int((len(res_z) - 1) / 2)] if len(res_z) != 2 else (res_z[0] + res_z[1]) / 2
        # 从小到大排序
        self.print_ary('RES_Z', res_z, len(res_z))
        self.Tri_bed_test_z = [round(i,3) for i in res_z]
        res_z.sort()
        # res_z长度 != 2 选择中位值，res_z长度 = 2 计算均值
        out_mm = res_z[int((len(res_z) - 1) / 2)] if len(res_z) != 2 else (res_z[0] + res_z[1]) / 2
        self.print_msg('WHY DEBUG',"OUT_RES_Z:{:.3f}".format(out_mm))
        return out_mm,True

    def bed_mesh_post_proc(self, last_point):
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max

        res1 = self.bed_mesh.bmc.probe_helper.results
        res1.append(last_point)

        self.print_msg('WHY DEBUG', 'bed mesh last point=:{}'.format(last_point))
        self.print_msg('BED_MESH_POST_PROC', 'Before=' + str(res1))

        # 1. Check Error Point
        for r in range(3):
            res_yx = []
            for l in range(y_cnt):
                res_yx.append((res1[int(l * x_cnt):int(l * x_cnt + x_cnt)])[::int(1 if (l % 2 == 0) else -1)])

            err_ps = []
            err_ps_post = []
            # max_tan = self.bed_max_err * 2 * 1.2 / min([(max_x - min_x), (max_y - min_y)])
            max_tan = 3 / min([(max_x - min_x), (max_y - min_y)])
            for i in range(len(res_yx)):
                # self.print_msg('RE_PROBE_POINT', "res_yx[{}]:{}".format(i,res_yx[i]))
                for j in range(len(res_yx[i])):
                    mid_p = res_yx[i][j]
                    rdn_p = [res_yx[i - 1][j] if i >= 1 else mid_p, res_yx[i + 1][j] if i < x_cnt - 1 else mid_p,
                            res_yx[i][j - 1] if j >= 1 else mid_p, res_yx[i][j + 1] if j < y_cnt - 1 else mid_p]
                    err_cnt = 0
                    for m in range(len(rdn_p)):
                        if rdn_p[m] != mid_p and (math.fabs(rdn_p[m][2] - mid_p[2])) / max([math.fabs(rdn_p[m][0] - mid_p[0]), math.fabs(rdn_p[m][1] - mid_p[1])]) > max_tan:
                            err_cnt += 1 
                    if err_cnt >= 1 and mid_p not in err_ps:
                        err_ps.append(mid_p)
                        # i为行数 j为列数
                        err_ps_post.append([i,j])
                
            self.print_msg('RE_PROBE_POINT', "max_tan:{} len:{} err_ps:{}".format(max_tan,len(err_ps),err_ps))
            for i in range(len(err_ps)):
                self.run_to_next(err_ps[i])
                err_ps[i][2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:
                    self.print_msg('WHY ERROR',"FUN:bed_mesh_post_proc run_step_prtouch() ERROR")
                    return False
            # 异常点重测后的结果，更新进入热床数据
            for i in range(len(err_ps)):
                pos = 0
                # 偶数行
                if err_ps_post[i][0] % 2 ==0:
                    pos = (err_ps_post[i][1]+err_ps_post[i][0]*x_cnt)
                # 基数行
                else:
                    pos = (err_ps_post[i][0]*x_cnt+x_cnt-err_ps_post[i][1]-1)
                self.print_msg('RE_PROBE_POINT', "new_res1[{}]:{}  old_res1{}:{}".format(pos,err_ps[i],pos,res1[pos]))
                res1[pos] = err_ps[i]

        # 2. Resize tilt
        for i in range(len(res1)):
            res1[i][2] += (0 + self.noz_ex_com)
            res1[i][2] += (1 - (res1[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis

        # 3. Resize Mid Z
        mid_z = self.noz_ex_com
        if (x_cnt % 2 == 1) and (y_cnt % 2 == 1):
            mid_z = res1[int(len(res1) / 2)][2]
        else:
            if (x_cnt % 2 == 0) and (y_cnt % 2 == 0):
                axi = [[x_cnt / 2 - 1, y_cnt / 2 - 1], [x_cnt / 2, y_cnt / 2 - 1], [x_cnt / 2 - 1, y_cnt / 2], [x_cnt / 2, y_cnt / 2]]
            elif x_cnt % 2 == 1:
                axi = [[int(x_cnt / 2), y_cnt / 2 - 1], [int(x_cnt / 2), y_cnt / 2]]
            elif y_cnt % 2 == 1:
                axi = [[x_cnt / 2 - 1, int(y_cnt / 2)], [x_cnt / 2, int(y_cnt / 2)]]

            mid_z = 0
            
            for i in range(len(axi)):
                mid_z += res1[int(axi[i][0] + axi[i][1] * x_cnt)][2]
            mid_z /= len(axi)

        if mid_z < self.noz_ex_com:
            mid_z = self.noz_ex_com - mid_z
            for i in range(len(res1)):
                res1[i][2] += mid_z

        self.print_msg('BED_MESH_POST_PROC', 'After=' + str(res1))
        del res1[-1]
        return True

    def check_bed_mesh(self, auto_g29=True):
        self.print_msg('CK_BED_MESH', 'Start check_bed_mesh({})...'.format(auto_g29))
        self.ck_g28ed()  
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)
        self.print_msg('CK_BED_MESH', 'Start probe_ready()...')
        self.probe_ready()
        if not self.run_step_prtouch_flag:
            self.print_msg('WHY ERROR',"FUN:check_bed_mesh probe_ready() ERROR")
            return False
        self.jump_probe_ready = True
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        if mesh:
            self.bed_mesh.set_mesh(mesh)
            err_cnt, errs = 0, []

            for i in range(4):
                mesh_z = self.bed_mesh.z_mesh.calc_z(self.rdy_pos[i][0], self.rdy_pos[i][1]) - self.noz_ex_com - ((1 - (self.rdy_pos[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis)
                errs.append(abs(self.rdy_pos[i][2] - mesh_z))
                err_cnt += (1 if errs[i] > self.check_bed_mesh_max_err else 0)
                self.print_msg('CK_BED_MESH', 'P%d = [x=%.2f, y=%.2f, mest_z=%.2f, probe_z=%.2f, err_z=%.2f]' % (i, self.rdy_pos[i][0], self.rdy_pos[i][1], mesh_z, self.rdy_pos[i][2], errs[i]))    
        
            save_version = 0 if not self.stored_profs else self.stored_profs.getint('version', default=0)
            self.print_msg('CK_BED_MESH', 'Now Version=%d, Cfg Version=%d' % (PR_VERSION, save_version))
            err_cnt = err_cnt if save_version == PR_VERSION else 4
            if err_cnt < 2:
                self.print_msg('DEBUG', "check_bed_mesh: Pass!!")
                return True
            self.print_msg('DEBUG', 'check_bed_mesh: Due to the great change of the hot bed or version, it needs to be re-leveled. ' + str([[errs[0], errs[1],errs[2], errs[3]], self.check_bed_mesh_max_err]))    
            pass
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.gcode.run_script_from_command('BED_MESH_CALIBRATE')

        configfile = self.printer.lookup_object('configfile')
        configfile.set('prtouch default', 'version', PR_VERSION)

        self.gcode.run_script_from_command('CXSAVE_CONFIG')
        return True
    
    # TODO 擦嘴程序
    # return True(成功) False(失败)
    def clear_nozzle(self, hot_min_temp, hot_max_temp, bed_max_temp):
        self.print_msg('CLEAR_NOZZLE', 'Start clear_nozzle(), hot_min_temp=%.2f, hot_max_temp=%.2f, bed_max_temp=%.2f' % (hot_min_temp, hot_max_temp, bed_max_temp))
        min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y
        # max_x, max_y = self.clr_noz_start_x + self.clr_noz_len_x, self.clr_noz_start_y + self.clr_noz_len_y
        self.set_bed_temps(temp=bed_max_temp, wait=False)
        self.set_hot_temps(temp=hot_min_temp, wait=False, err=10)
        self.ck_g28ed()
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)
        self.set_step_par(load_sys=False)
        random.seed(time.time())  
        cur_pos = self.toolhead.get_position()
        src_pos = [min_x + random.uniform(0, self.clr_noz_len_x - self.pa_clr_dis_mm - 5), 
                   min_y + random.uniform(0, self.clr_noz_len_y), self.bed_max_err, cur_pos[3]]
        end_pos = [src_pos[0] + self.pa_clr_dis_mm, src_pos[1], src_pos[2], src_pos[3]]
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=10)
        self.set_hot_temps(temp=hot_min_temp + 40, wait=False, err=10)         

        self.print_msg('CLEAR_NOZZLE', 'Start Src Pos Probe...')
        self.move([src_pos[0], src_pos[1], src_pos[2]], self.rdy_xy_spd)
        self.z_set_status = True
        for i in range(3):
            src_pos[2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:clear_nozzle run_step_prtouch() ERROR")
                return False
            if src_pos[2] <= self.min_z:
                self.nozzle_clear_z_out_of_range = True
                self.print_msg('WHY DEBUG',"nozzle clear src_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!".format(src_pos[2],self.min_z,self.nozzle_clear_z_out_of_range))
                continue
            break
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)

        self.print_msg('CLEAR_NOZZLE', 'Start End Pos Probe...')
        self.move([end_pos[0], end_pos[1], end_pos[2]], self.rdy_xy_spd)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        for i in range(3):
            end_pos[2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:clear_nozzle run_step_prtouch() ERROR")
                return False
            if end_pos[2] <= self.min_z:
                self.nozzle_clear_z_out_of_range = True
                self.print_msg('WHY DEBUG',"nozzle clear end_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!".format(end_pos[2],self.min_z,self.nozzle_clear_z_out_of_range))
                continue
            break
        self.z_set_status = False

        self.print_msg('CLEAR_NOZZLE', 'Down And Wait Temp For %.2f...' % hot_max_temp)
        self.move([src_pos[0], src_pos[1], self.bed_max_err], self.rdy_xy_spd)
        self.move([src_pos[0], src_pos[1], src_pos[2] - self.pa_clr_down_mm], self.rdy_z_spd)
        self.set_hot_temps(temp=hot_max_temp, wait=True, err=10) 

        self.print_msg('CLEAR_NOZZLE', 'Start Clear The Noz...')
        self.move(end_pos[:2] + [end_pos[2] + self.pa_clr_down_mm], self.clr_xy_spd)
        self.set_fan_speed('fan', 1.0)

        self.print_msg('CLEAR_NOZZLE', 'Start Cool Down The Noz...')
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)

        self.print_msg('CLEAR_NOZZLE', 'Start Finish Clear...')
        self.move([end_pos[0] + self.pa_clr_dis_mm, end_pos[1], end_pos[2] + self.bed_max_err], self.clr_xy_spd)
        self.set_fan_speed('fan', 0.)
        self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)
        self.set_step_par(load_sys=True)
        self.bed_mesh.set_mesh(mesh)
        # self.gcode.run_script_from_command('G28 Z')
        return True
    #  快速擦嘴程序
    # return True(成功) False(失败)
    def clear_nozzle_quick(self, hot_min_temp, hot_max_temp, bed_max_temp):
        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start clear_nozzle_quick(), hot_min_temp=%.2f, hot_max_temp=%.2f, bed_max_temp=%.2f' % (hot_min_temp, hot_max_temp, bed_max_temp))
        start_x, start_y = self.clr_noz_start_x, self.clr_noz_start_y
        max_x = self.clr_noz_start_x + self.clr_noz_len_x
        self.set_bed_temps(temp=bed_max_temp, wait=False)
        self.set_hot_temps(temp=hot_min_temp, wait=False, err=10)
        self.ck_g28ed()
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)

        self.set_step_par(load_sys=True)
        max_accel = self.toolhead.max_accel 
        max_accel_to_decel = self.toolhead.max_accel_to_decel
        square_corner_velocity = self.toolhead.square_corner_velocity 
        self.toolhead.max_accel = 20000
        self.toolhead.max_accel_to_decel = 10000
        self.toolhead.square_corner_velocity = self.clr_xy_quick_spd

        #mesh = self.bed_mesh.get_mesh()
        #self.bed_mesh.set_mesh(None)
        #self.set_step_par(load_sys=False)
        random.seed(time.time())  
        cur_pos = self.toolhead.get_position()
        src_pos = [start_x ,start_y, self.bed_max_err, cur_pos[3]]
        end_pos = [max_x , start_y, src_pos[2], src_pos[3]]
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=10)
        self.set_hot_temps(temp=hot_min_temp + 40, wait=False, err=10)         

        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Src Pos Probe...')
        self.move([src_pos[0], src_pos[1], src_pos[2]+1], self.rdy_xy_spd)
        self.z_set_status = True
        for i in range(3):
            src_pos[2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('ERROR',"FUN:clear_nozzle_quick run_step_prtouch() ERROR")
                return False
            if src_pos[2] <= self.min_z:
                self.nozzle_clear_z_out_of_range = True
                self.print_msg('WHY DEBUG',"nozzle clear src_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!".format(src_pos[2],self.min_z,self.nozzle_clear_z_out_of_range))
                continue
            break
        if self.use_adc:
                self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.gcode.respond_info("[CLEAR_NOZZLE_QUICK] src_pos[2]:{}".format(src_pos[2]))

        self.print_msg('CLEAR_NOZZLE_QUICK', 'Start End Pos Probe...')
        self.move([end_pos[0], end_pos[1], end_pos[2]+1], self.rdy_xy_spd)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)

        for i in range(3):
            end_pos[2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True) 
            if not self.run_step_prtouch_flag:
                self.print_msg('ERROR',"FUN:clear_nozzle_quick run_step_prtouch() ERROR")
                return False
            if end_pos[2] <= self.min_z:
                self.nozzle_clear_z_out_of_range = True
                self.print_msg('WHY DEBUG',"nozzle clear end_pos_z:{} less than z_min:{},resetflag:{} z level and prtouch z again!".format(end_pos[2],self.min_z,self.nozzle_clear_z_out_of_range))
                continue
            break
        self.z_set_status = False
        
        self.gcode.respond_info("[CLEAR_NOZZLE_QUICK] end_pos[2]:{}".format(end_pos[2]))

        self.print_msg('CLEAR_NOZZLE_QUICK', 'move to start position... ')
        if end_pos[2] > 1.6 and src_pos[2] > 1.6:
            self.move([end_pos[0], end_pos[1], end_pos[2] - 0.2], self.rdy_xy_spd)
            self.set_hot_temps(temp=hot_max_temp, wait=True, err=10) 

            react_dis = self.clr_quick_react_dis
            random_y = random.uniform(-1,0.5)
            self.move([end_pos[0], end_pos[1], end_pos[2] + 1], self.rdy_xy_spd)
            self.move([end_pos[0] - react_dis, end_pos[1] + random_y, end_pos[2] + 1], self.clr_xy_quick_spd)
            self.move([end_pos[0] - react_dis, end_pos[1] + random_y, end_pos[2] - self.clr_quick_high], self.rdy_z_spd)

            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Clear The Noz... 1')
        
            for i in range(self.clr_quick_times):
                self.move([src_pos[0] + react_dis, src_pos[1] + random_y, src_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                self.move([end_pos[0] - react_dis, end_pos[1] + random_y, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                if i%2 == 1:
                    self.move([end_pos[0] - react_dis, end_pos[1] + random_y - 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                    self.gcode.run_script_from_command('G4 S2')
                    self.gcode.run_script_from_command('M400')
                else :
                    self.move([end_pos[0] - react_dis, end_pos[1] + random_y +0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                    self.gcode.run_script_from_command('G4 S2')

            self.move([end_pos[0], end_pos[1], end_pos[2] ], self.rdy_xy_spd)
            self.move([end_pos[0], end_pos[1], end_pos[2] - 0.2], self.rdy_xy_spd)

            self.set_fan_speed('fan', 1.0)
            self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)
            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Cool Down The Noz...')


            self.print_msg('CLEAR_NOZZLE_QUICK', 'Start Clear The Noz... 2')
            self.move([end_pos[0], end_pos[1], end_pos[2] + 1], self.rdy_xy_spd)
            self.move([end_pos[0] - react_dis, end_pos[1] + random_y, end_pos[2] + 1], self.clr_xy_quick_spd)
            self.move([end_pos[0] - react_dis, end_pos[1] + random_y, end_pos[2] - self.clr_quick_high], self.rdy_z_spd)
            for a in range(3):
                self.move([src_pos[0] + react_dis, src_pos[1] + random_y, src_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                self.move([end_pos[0] - react_dis, end_pos[1] + random_y ,end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                if i%2 == 1:
                    self.move([end_pos[0] - react_dis, end_pos[1] + random_y - 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                    self.gcode.run_script_from_command('G4 S2')
                    self.gcode.run_script_from_command('M400')
                else :
                    self.move([end_pos[0] - react_dis, end_pos[1] + random_y + 0.5, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
                    self.gcode.run_script_from_command('G4 S2')

            self.move([end_pos[0]- react_dis, end_pos[1] + random_y, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd, False)
            self.move([end_pos[0]- react_dis, end_pos[1] + random_y - 4, end_pos[2] - self.clr_quick_high], self.clr_xy_quick_spd)

            self.move([src_pos[0] + react_dis, src_pos[1] + random_y - 4, 5], 600)
            self.set_fan_speed('fan', 0.)
            self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)
            
            self.toolhead.square_corner_velocity = square_corner_velocity
            self.toolhead.max_accel = max_accel
            self.toolhead.max_accel_to_decel = max_accel_to_decel
            self.bed_mesh.set_mesh(mesh)
        else:
            #普通擦嘴
            min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y
            self.set_step_par(load_sys=False)
            random.seed(time.time())  
            src_pos = [min_x + random.uniform(0, self.clr_noz_len_x - self.pa_clr_dis_mm - 5), 
                    min_y + random.uniform(0, self.clr_noz_len_y), src_pos[2], src_pos[3]]
            end_pos = [src_pos[0] + self.pa_clr_dis_mm, src_pos[1], end_pos[2], end_pos[3]]

            self.print_msg('CLEAR_NOZZLE', 'Down And Wait Temp For %.2f...' % hot_max_temp)
            self.move([src_pos[0], src_pos[1], self.bed_max_err], self.rdy_xy_spd)
            self.move([src_pos[0], src_pos[1], src_pos[2] - self.pa_clr_down_mm], self.rdy_z_spd)
            self.set_hot_temps(temp=hot_max_temp, wait=True, err=10) 

            self.print_msg('CLEAR_NOZZLE', 'Start Clear The Noz...')
            self.move(end_pos[:2] + [end_pos[2] + self.pa_clr_down_mm], self.clr_xy_spd)
            self.set_fan_speed('fan', 1.0)

            self.print_msg('CLEAR_NOZZLE', 'Start Cool Down The Noz...')
            self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)

            self.print_msg('CLEAR_NOZZLE', 'Start Finish Clear...')
            self.move([end_pos[0] + self.pa_clr_dis_mm, end_pos[1], end_pos[2] + self.bed_max_err], self.clr_xy_spd)
            self.set_fan_speed('fan', 0.)
            self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)
            self.set_step_par(load_sys=True)
            self.bed_mesh.set_mesh(mesh)
        # self.gcode.run_script_from_command('G28 Z')
        return True

    # TODO Z轴回0 G28_Z
    # return True(Z归零成功) False(Z归零失败)
    def run_G28_Z(self, accurate=False): 
        self.print_msg('RUN_G28_Z', 'Start run_G28_Z({})...'.format(accurate))
        self.enable_steps()
        self.get_mm_per_step()
        self.g29_cnt = 0
        self.jump_probe_ready = False
        self.After_G28Z_skip_bed_tilt_flag = True
        self.toolhead.wait_moves()
        if not self.bed_mesh.get_mesh() and self.bed_mesh.pmgr.profiles.get('default', None):
            self.gcode.run_script_from_command('BED_MESH_PROFILE LOAD=\"default\"')
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)   
        self.set_step_par(load_sys=False)
        now_pos = self.toolhead.get_position()    
        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max        
        # now_pos = [min_x + (max_x - min_x) / 2, min_y + (max_y - min_y) / 2, now_pos[2], now_pos[3]]
        now_pos = [self.home_x, self.home_y, now_pos[2], now_pos[3]]
        # 1. Check hot temp
        target_temp = self.heater_hot.target_temp
        if self.g28_wait_cool_down and self.heater_hot.smoothed_temp > (self.hot_min_temp + 5):
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to cool down[%.2f -> %.2f]...' % (target_temp, self.hot_min_temp))
            self.set_fan_speed('fan', 1.0)
            self.set_hot_temps(temp=self.hot_min_temp, wait=True, err=5) 
            self.set_fan_speed('fan', 0.0)
        # 2. First probe z
        random.seed(int(time.time()))
        now_pos0 = [now_pos[0] + (1 if (int(time.time() * 1000) % 2 == 0) else -1) * random.uniform(4, 8),
                    now_pos[1] + (1 if (int(time.time() * 100) % 2 == 0) else -1) * random.uniform(4, 8), 0, now_pos[3]]
        now_pos1 = [now_pos[0] + random.uniform(-1.0, +1.0), now_pos[1] + random.uniform(-1.0, +1.0), self.best_above_z * 2, now_pos[3]] 
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        self.move(now_pos0, self.rdy_xy_spd / 5)
        # 自检异常。退出G28_Z
        if not self.env_self_check():
            self.print_msg('WHY ERROR',"FUN:G28_Z env_self_check() ERROR")
            return False

        out_mms, lev_z = [], 0
        use_tri_times = 0
        use_tri_z_down_spd = self.tri_z_down_spd[0]
        use_tri_hftr_cut = self.tri_hftr_cut[0]
        use_tri_lftr_k1 = self.tri_lftr_k1[0]
        use_tri_min_hold = self.tri_min_hold[0]
        use_tri_max_hold = self.tri_max_hold[0]

        move_down_cnt = 0
        recored_Coarse_Probe_mm = []
        ck_step_error_cnt = 0
        ck_pres_error_cnt = 0
        while True: 
            self.print_msg('RUN_G28_Z', "use_tri_times=%d, use_tri_z_down_spd=%.2f, use_tri_hftr_cut=%.2f, use_tri_lftr_k1=%.2f, use_tri_min_hold=%.0f, use_tri_max_hold=%.0f"
                            % (use_tri_times,use_tri_z_down_spd,use_tri_hftr_cut,use_tri_lftr_k1,use_tri_min_hold,use_tri_max_hold))
            self.step_res, self.pres_res = [], []
            self.enable_steps()
            self.print_msg('RUN_G28_Z', 'Start Coarse Probe, Test Index=%d...' % use_tri_times)
            self.step_res, self.pres_res = [], []
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
            step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(self.max_z * 1.2, use_tri_z_down_spd * (1.2 if self.use_adc else 2.0))
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                              int(use_tri_hftr_cut * 1000),  int(use_tri_lftr_k1 * 1000), int(use_tri_min_hold), int(use_tri_max_hold)])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while (time.time() - t_last < (self.max_z * 2.0) / (use_tri_z_down_spd * (1.2 if self.use_adc else 2.0))) and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])
 
            if len(self.pres_res) == 0 or len(self.step_res) == 0:
                if move_down_cnt < 2:
                    move_down_cnt += 1
                    self.toolhead.set_position(now_pos0[:2] + [0, now_pos0[3]], homing_axes=[2])
                    self.move(now_pos0[:2] + [self.best_above_z * 2, now_pos0[3]], self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))
                else:
                    self.delay_s(0.10)
                continue

            # 默认一个错误的输出高度999
            out_mm = 999
            if not self.ck_and_manual_get_step():
                ck_step_error_cnt += 1
                self.print_msg('WHY ERROR',"FUN:G28_Z ck_and_manual_get_step() ERROR Times:{}",ck_step_error_cnt)
                # 连续连续2次出现丢包就报error
                if ck_step_error_cnt > 2:
                    self.ck_and_raise_error(True, PR_ERR_CODE_STEP_LOST_RUN_DATA, ["step_res len:{}".format(len(self.step_res))])
                    return False
            else:
                ck_step_error_cnt = 0
                # 粗探计算z轴高度
                out_mm = (self.toolhead.get_position()[2] - (step_cnt_down - self.step_res[-1]['step']) * self.mm_per_step) \
                        if len(self.pres_res) != MAX_BUF_LEN else self.cal_tri_data(step_cnt_down, self.toolhead.get_position()[2], self.step_res, self.pres_res)               
            # 设置当前高度为home点
            self.toolhead.set_position(now_pos0[:2] + [0, now_pos0[3]], homing_axes=[2])
            # 基于当前高度抬升喷头6mm
            # self.move(now_pos0[:2] + [self.best_above_z * 2 if self.pres_buf_cnt > MAX_BUF_LEN else 1, now_pos0[3]], self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))
            # why修改 热床传感器触发后，上面设置触发位置为起始平面高度。此句作用为抬升6mm后，再进行下面的擦嘴或者精探动作
            # 如果人为或者误差多次导致热床传感器快速触发。热床会一直移动远离喷头，要是本身热床距离外边框比较近，探测次数在范围内。
            # 这里可能会导致热床撞边框。目前这个问题没有很好的解法
            self.move(now_pos0[:2] + [self.best_above_z * 2, now_pos0[3]], self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))
            # if self.pres_buf_cnt > MAX_BUF_LEN: why修改
            recored_Coarse_Probe_mm.append(out_mm)
            out_mms.append(math.fabs(out_mm))
            if len(out_mms) > 3:
                del out_mms[0]
            _out_mms = [m for m in out_mms]
            _out_mms.sort()
            self.print_ary('G28_FIRST_MMS', _out_mms, len(_out_mms), 3)
            if len(_out_mms) == 3 and (_out_mms[0] + _out_mms[1] < 2.0):
                break

            use_tri_times += 1
            if self.tri_try_max_times != 0:
                if use_tri_times == self.tri_try_max_times:
                    self.print_msg('WHY ERROR', "FUN:G28_Z Coarse Probe Out of max {} times:{}. all data:{}".format(self.tri_try_max_times,use_tri_times,recored_Coarse_Probe_mm))
                    # 如果粗探连续超过10次都没有确定。说明热床或者探测有问题。报探测超时
                    self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)
                    return False
            if use_tri_times <= 20 and use_tri_times >= 5:
                rd = (use_tri_times - 5) / 15.
                use_tri_z_down_spd = (self.tri_z_down_spd[1] - self.tri_z_down_spd[0]) * rd + self.tri_z_down_spd[0]
                use_tri_hftr_cut = (self.tri_hftr_cut[1] - self.tri_hftr_cut[0]) * rd + self.tri_hftr_cut[0]
                use_tri_lftr_k1 = (self.tri_lftr_k1[1] - self.tri_lftr_k1[0]) * rd + self.tri_lftr_k1[0]
                use_tri_min_hold = (self.tri_min_hold[1] - self.tri_min_hold[0]) * rd + self.tri_min_hold[0]
                use_tri_max_hold = (self.tri_max_hold[1] - self.tri_max_hold[0]) * rd + self.tri_max_hold[0]
            if use_tri_times % 5 == 0:
                self.toolhead.set_position(now_pos0[:2] + [0, now_pos0[3]], homing_axes=[2])
                self.move(now_pos0[:2] + [self.best_above_z * 2, now_pos0[3]], self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))
                self.shake_motor(self.shake_cnt)
            if use_tri_times == 19:
                self.print_msg('WHY ERROR', "FUN:G28_Z Coarse Probe Out of max 19 times:{}. all data:{}".format(use_tri_times,recored_Coarse_Probe_mm))
                # 如果没设置最大触发次数，强制粗探连续超过20次都没有确定。说明热床或者探测有问题。报探测超时
                self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)
                return False
        if not accurate:
            if self.g28_wait_cool_down:
                self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to recovery[%.2f -> %.2f]...' % (self.hot_min_temp, target_temp))
                self.set_hot_temps(temp=target_temp, wait=False, err=5)
            return True   
        # 3. Normal probe z    
        use_tri_times = 0
        while True: 
            self.print_msg('RUN_G28_Z', 'Start Precision Probe, index=%d...' % use_tri_times)
            self.move(self.toolhead.get_position()[:2] + [now_pos1[2]], self.rdy_z_spd)
            self.move(now_pos1, self.rdy_xy_spd / 5)
            self.shake_motor(self.shake_cnt)
            self.z_set_status = True
            res_z,self.run_step_prtouch_flag = self.run_step_prtouch(self.max_z * 1.2, self.probe_min_3err, False, 5, 3, True)
            self.z_set_status = False
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:run_G28_Z run_step_prtouch() ERROR")
                return False
            self.toolhead.set_position(now_pos1[:2] + [self.toolhead.get_position()[2] - res_z, now_pos[3]], homing_axes=[2])
            self.print_msg('RUN_G28_Z', 'Start Result Check Probe...')
            self.move(now_pos1[:2] + [self.bed_max_err, now_pos[3]], self.rdy_z_spd)
            ck_z,self.run_step_prtouch_flag = self.run_step_prtouch(self.max_z * 1.2, self.probe_min_3err, False, 3, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:run_G28_Z run_step_prtouch() ERROR")
                return False
            use_tri_times += 1
            if math.fabs(ck_z) < 1.0:
                break
            else:
                if use_tri_times == 2:
                    self.print_msg('WHY ERROR', "FUN:G28_Z Precision Probe Out of max 2 times:{}. ck_z data:{}".format(use_tri_times,ck_z))
                    self.ck_and_raise_error(True, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)
                    return False

        # 4. Set hot temp to old target
        if self.g28_wait_cool_down:
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to recovery[%.2f -> %.2f]...' % (self.hot_min_temp, target_temp))
            self.set_hot_temps(temp=target_temp, wait=True if target_temp > self.hot_min_temp else False, err=5)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.set_step_par(load_sys=True)
        self.bed_mesh.set_mesh(mesh)   
        return True

    def run_re_g29s(self, last_point):
        self.print_msg('RUN_RE_G29S', 'Start run_re_g29s()...')        

        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']

        res1 = self.bed_mesh.bmc.probe_helper.results
        res1.append(last_point)

        now_pos = self.toolhead.get_position()
        # self.best_above_z *= 2
        # self.rdy_z_spd *= 0.75
        # self.tri_z_down_spd *= 0.75
        # self.tri_z_up_spd *= 0.75
        # self.low_spd_nul *= 2
        # self.run_max_velocity = int(self.run_max_velocity / 2)
        # self.run_max_accel = int(self.run_max_accel / 2)
        # self.run_max_z_velocity = int(self.run_max_z_velocity / 2)
        # self.run_max_z_accel = int(self.run_max_z_accel / 2)

        self.move(now_pos[:2] + [5, now_pos[3]], self.rdy_z_spd)

        self.run_G28_Z(True)
        self.probe_ready()
        for i in range(len(res1)):
            self.print_msg('RUN_RE_G29S_ONE', 'Re Probe = ' + str(res1[i]))   
            if i % x_cnt == 0:
                self.shake_motor(self.shake_cnt / 4)
            self.run_to_next(res1[i], True)
            res1[i][2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:run_re_g29s run_step_prtouch() ERROR")
                return False

        self.print_msg('RUN_RE_G29S', 'Start Step Lost Check...')
        self.move([self.home_x, self.home_y, self.get_best_rdy_z(self.home_x, self.home_y, self.rdy_pos) + self.best_above_z], self.rdy_xy_spd)
        res_z,self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
        if not self.run_step_prtouch_flag:
            self.print_msg('WHY ERROR',"FUN:run_re_g29s run_step_prtouch() ERROR")
            return False
        self.print_msg('RUN_RE_G29S', 'Step Lost Check, need=0, tri=%.2f' % res_z) 
        # res_z = 10
        if math.fabs(res_z) > self.lost_step_dis and self.lost_step_dis > 0:
            self.run_G28_Z(True)
            self.probe_ready()
            for i in range(len(res1)):
                self.print_msg('RUN_RE_G29S_TWO', 'Re Probe = ' + str(res1[i]))   
                if i % x_cnt == 0:
                    self.shake_motor(self.shake_cnt / 4)
                now_pos = self.toolhead.get_position()
                self.move(now_pos[:2] + [5], self.rdy_z_spd)
                self.move([self.home_x, self.home_y, 5], self.rdy_xy_spd)
                res_z,self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:
                    self.print_msg('WHY ERROR',"FUN:run_re_g29s run_step_prtouch() ERROR")
                    return False
                now_pos = self.toolhead.get_position()
                self.toolhead.set_position(now_pos[:2] + [now_pos[2] - res_z, now_pos[3]], homing_axes=[2])
                self.run_to_next(res1[i], True)
                res1[i][2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
                if not self.run_step_prtouch_flag:
                    self.print_msg('WHY ERROR',"FUN:run_re_g29s run_step_prtouch() ERROR")
                    return False
            self.run_G28_Z(True)

        # self.best_above_z = int(self.best_above_z / 2)
        # self.rdy_z_spd /= 0.75
        # self.tri_z_down_spd /= 0.75
        # self.tri_z_up_spd /= 0.75
        # self.low_spd_nul = int(self.low_spd_nul / 2)
        # self.run_max_velocity = int(self.run_max_velocity * 2)
        # self.run_max_accel = int(self.run_max_accel * 2)
        # self.run_max_z_velocity = int(self.run_max_z_velocity * 2)
        # self.run_max_z_accel = int(self.run_max_z_accel * 2)

        last_z = res1[-1][2]
        del res1[-1]
        return last_z

    # return True(成功) False(失败)
    def run_G29_Z(self):
        self.print_msg('RUN_G29_Z', 'Start run_G29_Z()...')
        self.set_step_par(load_sys=False)
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count'] 
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count'] 
        self.toolhead.wait_moves()    
        now_pos = self.toolhead.get_position()
        if (int(self.g29_cnt) % int(x_cnt)) == 0:
            self.shake_motor(self.shake_cnt / 4)
        self.print_msg('RUN_G29_Z', 'g29_cnt=%d, x=%.2f, y=%.2f, z=%.2f' % (self.g29_cnt, now_pos[0], now_pos[1], now_pos[2]))
        if self.g29_cnt == 0:
            if self.use_adc:
                self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
            self.set_fan_speed('fan', 0.0)
            if self.probe_ready():
                self.run_to_next(now_pos, True)
                self.shake_motor(self.shake_cnt / 4)
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:run_G29_Z probe_ready() ERROR")
                return False
        self.g29_cnt += 1

        now_pos[2],self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True, re_g28=True)
        if not self.run_step_prtouch_flag:
            self.print_msg('WHY ERROR',"FUN:run_G29_Z run_step_prtouch() ERROR")
            return False
        
        if self.g29_cnt == x_cnt * y_cnt:
            self.g29_cnt = 0
            self.print_msg('RUN_G29_Z', 'Start Step Lost Check...')
            self.move([self.home_x, self.home_y, self.get_best_rdy_z(self.home_x, self.home_y, self.rdy_pos) + self.best_above_z], self.rdy_xy_spd)
            res_z,self.run_step_prtouch_flag = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            self.print_msg('RUN_G29_Z', 'Step Lost Check, need=0, tri=%.2f' % res_z) 
            if not self.run_step_prtouch_flag:
                self.print_msg('WHY ERROR',"FUN:run_G29_Z run_step_prtouch() ERROR")
                return False

            # G29_Z后 home点归零时 z轴探测到的高度大于运行的最小丢步距离判定为丢步
            if math.fabs(res_z) > self.lost_step_dis and self.lost_step_dis > 0:
                self.print_msg('WHY ERROR',"FUN:run_G29_Z check last point res_z:({}) out of lost_step_dis:{}".format(math.fabs(res_z),self.lost_step_dis))
                self.ck_and_raise_error(True, PR_ERR_CODE_HAVE_LOST_STEP,["RUN_G29_Z check last point res_z:({}) out of lost_step_dis:{}".format(math.fabs(res_z),self.lost_step_dis)])
                return False
            if not self.bed_mesh_post_proc(now_pos):
                self.print_msg('WHY ERROR',"FUN:run_G29_Z bed_mesh_post_proc ERROR")
                return False
            self.move(self.toolhead.get_position()[:2] + [10], self.rdy_z_spd)
            self.move([self.home_x, self.home_y, 10], self.rdy_xy_spd)
            if self.use_adc:
                self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.set_step_par(load_sys=True)
        return now_pos
    
    # TODO 读取压力数据
    cmd_READ_PRES_help = "Read The Press Vals."
    def cmd_READ_PRES(self, gcmd):
        self.print_msg('VERSION', 'ARM(PRTH)=%s, MCU(PRES)=%s, MCU(STEP)=%s' % (self.ver_prth, self.ver_pres, self.ver_step))
        self.pres_res = []
        read_cnt = gcmd.get_int('C', 1)
        avg_flag = gcmd.get_int('F', 0)
        if avg_flag:
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
        self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, read_cnt])
        start_tick_s = time.time()
        while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * read_cnt)) and len(self.pres_res) < read_cnt:
            self.delay_s(0.010)
        pnt_vals = [[], [], [], []]
        pnt_tick = []
        for i in range(len(self.pres_res)):
            pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
            for j in range(self.pres_cnt):
                pnt_vals[j].append(self.pres_res[i]['ch%d' % j])

        self.print_ary('READ_PRES_TICKS', pnt_tick, len(pnt_tick), 3, True)        
        for i in range(self.pres_cnt):
            self.print_ary('READ_PRES_CH%d' % i, pnt_vals[i], len(pnt_vals[i]), 0, True)    
            self.send_wave_tri(i, pnt_vals[i])
        pass

    # TODO 测试同步线
    cmd_TEST_SWAP_help = "Test The Swap Pin."
    def cmd_TEST_SWAP(self, gcmd):
        self.write_swap_prtouch_cmd.send([self.pres_oid, 1])
        params1 = self.read_swap_prtouch_cmd.send([self.pres_oid])
        self.write_swap_prtouch_cmd.send([self.pres_oid, 0])
        params0 = self.read_swap_prtouch_cmd.send([self.pres_oid])       
        self.print_msg('SWAP_TEST', 'params0:{} params1:{}'.format(params0,params1))
        if not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1:
            self.print_msg('SWAP_TEST', '!!!Swap Test ERROR!!!', True)
        else:
            self.print_msg('SWAP_TEST', '---Swap Test Success---', True)
        pass 

    cmd_DEAL_AVGS_help = "Read And Cal The Avgs."
    def cmd_DEAL_AVGS(self, gcmd):
        read_cnt = gcmd.get_int('C', 8)
        params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, read_cnt])
        self.print_msg('AVGS_RESAULT', str(params), True)
        pass

    cmd_NOZZLE_CLEAR_help = "Clear the nozzle on bed."
    def cmd_NOZZLE_CLEAR(self, gcmd):
        hot_min_temp = gcmd.get_float('HOT_MIN_TEMP', self.hot_min_temp)
        hot_max_temp = gcmd.get_float('HOT_MAX_TEMP', self.hot_max_temp)
        bed_max_temp = gcmd.get_float('BED_MAX_TEMP', self.bed_max_temp)
        self.gcode.run_script_from_command('RESTORE_E_CURRENT')
        if self.clr_noz_quick:
            self.clear_nozzle_quick(hot_min_temp, hot_max_temp, bed_max_temp)
        else :
            self.clear_nozzle(hot_min_temp, hot_max_temp, bed_max_temp)

    cmd_CHECK_BED_MESH_help = "Check the bed mesh."
    def cmd_CHECK_BED_MESH(self, gcmd):
        self.check_bed_mesh(gcmd.get_int('AUTO_G29', 0) > 0)
        pass

    cmd_START_STEP_PRTOUCH_help = "Start the step prtouch."
    def cmd_START_STEP_PRTOUCH(self, gcmd):
        self.enable_steps()
        self.get_mm_per_step()
        run_dir = gcmd.get_int('DIR', 0)
        run_spd = gcmd.get_float('SPD', 10)
        run_dis = gcmd.get_float('DIS', 10)
        self.step_res = []
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(run_dis, run_spd)
        self.start_step_prtouch_cmd.send([self.step_oid, run_dir, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
        t_last = time.time()
        while (time.time() - t_last < (run_dis / run_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN):
            self.delay_s(0.010)
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        pass

    cmd_PRTOUCH_READY_help = "Test the ready point."
    def cmd_PRTOUCH_READY(self, gcmd):
        self.probe_ready()
        pass

    cmd_SAFE_MOVE_Z_help = "Safe move z"  
    def cmd_SAFE_MOVE_Z(self, gcmd):
        self.v_sd.run_dis = 0.0
        run_sta = gcmd.get_int('STA', 0)
        run_dis = gcmd.get_float('DIS', +10)        
        run_spd = gcmd.get_float('SPD', 5.0)
        run_rdo = gcmd.get_float('RDO', 1.0)
        if run_sta == 0:
            self.run_sta_status = 0
        self.safe_move_z(run_sta, run_dis, run_spd, run_rdo)
        pass

    cmd_ACCURATE_HOME_Z_help = "Coarse home z"  
    def cmd_ACCURATE_HOME_Z(self, gcmd):
        self.ck_g28ed()
        self.run_G28_Z(True)
        pass

    cmd_SAFE_DOWN_Z_help = "Safe down z before G28"
    def cmd_SAFE_DOWN_Z(self, gcmd):
        self.get_mm_per_step()
        self.enable_steps()
        down_dis = gcmd.get_float('DOWN_DIS', 5)   
        up_dis = gcmd.get_float('UP_DIS', 0)
        
        if down_dis != 0:
            if self.safe_z_dead_dis != 0 and down_dis > self.safe_z_dead_dis + 1:
                self.step_res, self.pres_res = [], []
                step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(self.safe_z_dead_dis, self.tri_z_down_spd)
                self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
                t_last = time.time()
                while ((time.time() - t_last) < ((self.safe_z_dead_dis / 1.0) + 10)) and (len(self.step_res) != MAX_BUF_LEN):
                    self.delay_s(0.010)
                self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
                self.ck_and_manual_get_step()
                down_dis -= self.safe_z_dead_dis             
            self.step_res, self.pres_res = [], []
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
            step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(down_dis, self.tri_z_down_spd)
            self.start_pres_prtouch_cmd.send([self.pres_oid, 1, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                              int(self.tri_hftr_cut * 1000), int(self.tri_lftr_k1 * 1000),
                                              self.tri_min_hold if self.use_adc else (self.tri_min_hold * 2), self.tri_max_hold if self.use_adc else (self.tri_max_hold * 2)])
            self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while ((time.time() - t_last) < ((down_dis / self.tri_z_down_spd) + 10)) and (len(self.step_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 1, 0, 0, 0, 0, 0, 0, 0])
            self.print_res('cmd_SAFE_DOWN_Z')     
            self.ck_and_manual_get_step()       

        if up_dis != 0:
            if self.safe_z_dead_dis != 0 and up_dis > self.safe_z_dead_dis + 1:
                self.step_res, self.pres_res = [], []
                step_cnt_up, step_us_up, acc_ctl_cnt = self.get_step_cnts(self.safe_z_dead_dis, self.tri_z_down_spd)
                self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_up, step_us_up, 0, self.low_spd_nul, self.send_step_duty, 0])
                t_last = time.time()
                while ((time.time() - t_last) < ((self.safe_z_dead_dis / 1.0) + 10)) and (len(self.step_res) != MAX_BUF_LEN):
                    self.delay_s(0.010)
                self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
                self.ck_and_manual_get_step()
                up_dis -= self.safe_z_dead_dis 
            self.step_res, self.pres_res = [], []
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
            step_cnt_up, step_us_up, acc_ctl_cnt = self.get_step_cnts(up_dis, self.tri_z_down_spd)
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, 
                                              int(self.tri_hftr_cut * 1000), int(self.tri_lftr_k1 * 1000),
                                              self.tri_min_hold if self.use_adc else (self.tri_min_hold * 2), self.tri_max_hold if self.use_adc else (self.tri_max_hold * 2)])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_up, step_us_up, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while ((time.time() - t_last) < ((up_dis / self.tri_z_down_spd) + 10)) and (len(self.step_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])
            self.ck_and_manual_get_step()
        pass

    cmd_TRIG_TEST_help = "Test The Tri is Normal"
    def cmd_TRIG_TEST(self, gcmd):  
        self.enable_steps()          
        self.get_mm_per_step()
        run_cnt = gcmd.get_int('C', 1)
        self.run_step_prtouch(20, 0, False, run_cnt, run_cnt, True) # (self.max_z*1.2, 0, False, run_cnt, run_cnt, True)
        pass
    cmd_TRIG_BED_TEST_help = "Test The Tri in bed mesh"
    def cmd_TRIG_BED_TEST(self, gcmd): 
        # 暂时先关闭网床数据
        if not self.bed_mesh.get_mesh() and self.bed_mesh.pmgr.profiles.get('default', None):
            self.gcode.run_script_from_command('BED_MESH_PROFILE LOAD=\"default\"')
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)  
        now_time  = datetime.datetime.now().strftime('[%Y-%m-%d_%H_%M_%S]')
        file_flag = gcmd.get_int('F', 1)
        run_cnt = gcmd.get_int('C', 1)
        run_spd = gcmd.get_float('S', 1.0)
        if file_flag == 1:
            dirs = "/usr/share/klipper/pres_data/"
            _path = dirs + now_time+"_Tri_bedmesh_data_{}_{}.csv".format(run_cnt,run_spd)
            # 文件夹不存在则新建
            if not os.path.exists(dirs):
                os.makedirs(dirs)
            self.save_pres_file = open(_path,mode="w")
            Tri_z_str = ""
            for i in range(run_cnt):
                Tri_z_str += ",Tri_Z{}".format(i)
            Tri_z_str += ",MAX,MIN,(MAX-MIN)"
            self.save_pres_file.writelines("Time,Speed,Pos_X,Pos_Y,Pos_Z"+Tri_z_str+"\n")
        elif file_flag == 0:
            if self.save_pres_file != None:
                self.save_pres_file.close()
                self.save_pres_file = None
        
        # 计算网床采样点位置
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count'] 
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']
        # 计算跨度
        x_dist = (max_x - min_x) / (x_cnt - 1)
        y_dist = (max_y - min_y) / (y_cnt - 1)
        # floor distances down to next hundredth
        x_dist = math.floor(x_dist * 100) / 100
        y_dist = math.floor(y_dist * 100) / 100
        if x_dist < 1. or y_dist < 1.:
            raise self.printer.command_error("""{"code":"key555", "msg":"cmd_TRIG_BED_TEST bed_mesh: min/max points too close together", "values": []}""")
        # 按照路径计算采样点位置
        max_x = min_x + x_dist * (x_cnt - 1)
        pos_y = min_y
        points = []
        for i in range(y_cnt):
            for j in range(x_cnt):
                if not i % 2:
                    # move in positive directon
                    pos_x = min_x + j * x_dist
                else:
                    # move in negative direction
                    pos_x = max_x - j * x_dist
                # rectangular bed, append
                points.append([pos_x, pos_y,2])
            pos_y += y_dist

        self.enable_steps()          
        self.get_mm_per_step()
        old_speed = self.tri_z_down_spd
        # self.print_msg("WHY DEBUG","tri_z_down_spd:{}".format(self.tri_z_down_spd))
        self.tri_z_down_spd = (run_spd,run_spd)
        old_up_speed = self.tri_z_up_spd
        self.tri_z_up_spd = 5
        for i in range(len(points)):
            self.run_to_next(points[i], True)
            now_pos = self.toolhead.get_position()
            self.print_msg("WHY DEBUG","Start Tri Point:{} Pos:{}".format(i,now_pos[:-1]))
            self.run_step_prtouch(20, 0, False, run_cnt, run_cnt, True) # (self.max_z*1.2, 0, False, run_cnt, run_cnt, True)
            # str_t = ''
            tri_z_max = max(self.Tri_bed_test_z)
            tri_z_min = min(self.Tri_bed_test_z)
            diff_max_min = tri_z_max - tri_z_min
            now_time  = datetime.datetime.now().strftime('[%Y/%m/%d-%H:%M:%S]')
            str_t = str(self.Tri_bed_test_z).strip("[]")
            save_str = "{},{},{},{},{:.3f},{:.3f},{:.3f}".format(now_time,self.tri_z_down_spd[0],str(now_pos[:-1]).strip("[]"),str_t,tri_z_max,tri_z_min,diff_max_min)
            self.print_msg("WHY DEBUG","speed:{} pos:{} tri_z:{} Max:{:.3f} Min:{:.3f} (Max-Min):{:.3f}".format(self.tri_z_down_spd[0],now_pos[:-1],self.Tri_bed_test_z,tri_z_max,tri_z_min,diff_max_min),True)
            self.save_pres_file.writelines(save_str+"\n")
            self.Tri_bed_test_z = []

        if self.save_pres_file != None:
            self.save_pres_file.close()
            self.save_pres_file = None
        self.tri_z_up_spd = old_up_speed
        self.tri_z_down_spd = old_speed
        self.bed_mesh.set_mesh(mesh)  
        pass

    cmd_SELF_CHECK_PRTOUCH_help = "Self check the pres."
    def cmd_SELF_CHECK_PRTOUCH(self, gcmd):
        self.env_self_check()
        pass

    cmd_TEST_PRTH_help = "For Debug Cmd"
    def cmd_TEST_PRTH(self, gcmd):
        # self.env_self_check(force=True)
        # self.print_msg('%f' % self.noz_ex_com)
        # zero_z = gcmd.get_float('Z', 0)
        # self.shake_motor(self.shake_cnt)
        # spd = gcmd.get_float('S', 0)
        # self.set_fan_speed('heater_fan', spd)
        # self.set_fan_speed('fan', 1.0) 
        # read_cnt = gcmd.get_int('C', 1)
        # for i in range(read_cnt):
        #     self.gcode.run_script_from_command('BED_MESH_CALIBRATE')
        #     self.gcode.run_script_from_command('BED_MESH_OUTPUT')

        # x_cnt = self.bed_mesh.bmc.mesh_config['x_count']
        # y_cnt = self.bed_mesh.bmc.mesh_config['y_count']
        # min_x, min_y = self.bed_mesh.bmc.mesh_min
        # max_x, max_y = self.bed_mesh.bmc.mesh_max

        # x_len = (max_x - min_x) / x_cnt
        # y_len = (max_y - min_y) / y_cnt

        # res1 = []
        # for i in range(x_cnt * y_cnt):
        #     res1.append([int((i % x_cnt) * x_len + min_x), int(int(i / y_cnt) * int(y_len) + min_y), 0.01 * i])
        pass

    def change_hot_min_temp(self, temp):
        self.hot_min_temp = temp
