#!/usr/bin/env python

import argparse
import os
import signal
import sys

from ros_abstraction_layer import ral
from std_msgs.msg import Float32
from PyQt5 import QtCore, QtWidgets

# Load defaults from the same YAML used by mtm_device_crtk.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_HAPTICS_YAML = os.path.join(_SCRIPT_DIR, 'mtm_haptics_gains.yaml')


def _load_yaml_defaults():
    import yaml
    defaults = {
        'kp_pos': 120.0,
        'kd_pos': 0.0,
        'kp_rot': 2.5,
        'kd_rot': 0.05,
        'max_force': 2.0,
        'max_torque': 0.0,
        'linear_deadband': 0.001,
        'angular_deadband': 0.01,
    }
    try:
        with open(_HAPTICS_YAML, 'r') as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict) and 'haptics_gains' in data:
            defaults.update({k: float(v) for k, v in data['haptics_gains'].items() if k in defaults})
    except Exception as e:
        print('[mtm_haptics_gain_gui] Warning: could not load {}: {}'.format(_HAPTICS_YAML, e))
    return defaults


class HapticsGainGui(QtWidgets.QWidget):
    def __init__(self, g_ral, arm_name):
        super(HapticsGainGui, self).__init__()
        self._ral = g_ral
        self._arm_name = arm_name
        self._topic_prefix = '/{}/haptics/gains'.format(arm_name)

        self._publishers = {}
        self._spinboxes = {}

        yaml_defaults = _load_yaml_defaults()
        self._fields = {
            'kp_pos':           {'default': yaml_defaults['kp_pos'],           'min': 0.0, 'max': 500.0,  'step': 1.0},
            'kd_pos':           {'default': yaml_defaults['kd_pos'],           'min': 0.0, 'max': 50.0,   'step': 0.01},
            'kp_rot':           {'default': yaml_defaults['kp_rot'],           'min': 0.0, 'max': 50.0,   'step': 0.1},
            'kd_rot':           {'default': yaml_defaults['kd_rot'],           'min': 0.0, 'max': 10.0,   'step': 0.01},
            'max_force':        {'default': yaml_defaults['max_force'],        'min': 0.0, 'max': 20.0,   'step': 0.1},
            'max_torque':       {'default': yaml_defaults['max_torque'],       'min': 0.0, 'max': 10.0,   'step': 0.1},
            'linear_deadband':  {'default': yaml_defaults['linear_deadband'],  'min': 0.0, 'max': 0.05,   'step': 0.0005},
            'angular_deadband': {'default': yaml_defaults['angular_deadband'], 'min': 0.0, 'max': 0.5,    'step': 0.001},
        }

        self._setup_ros()
        self._setup_ui()
        self.setWindowTitle('MTM Haptics Gain Tuner ({})'.format(self._arm_name))
        self._publish_all()

    def _setup_ros(self):
        for field_name in self._fields:
            topic = '{}/{}'.format(self._topic_prefix, field_name)
            self._publishers[field_name] = self._ral.publisher(topic, Float32, queue_size=1)

    def _switch_arm(self, arm_name):
        self._arm_name = arm_name
        self._topic_prefix = '/{}/haptics/gains'.format(arm_name)
        self.setWindowTitle('MTM Haptics Gain Tuner ({})'.format(arm_name))
        self._topic_label.setText('Publishing to: {}'.format(self._topic_prefix))
        # Shut down old publishers and create new ones on updated topics.
        for pub in self._publishers.values():
            pub.unregister()
        self._publishers.clear()
        self._setup_ros()
        self._publish_all()

    def _setup_ui(self):
        main_layout = QtWidgets.QVBoxLayout()

        arm_layout = QtWidgets.QHBoxLayout()
        arm_layout.addWidget(QtWidgets.QLabel('Arm:'))
        self._arm_selector = QtWidgets.QComboBox()
        self._arm_selector.addItems(['MTMR', 'MTML'])
        self._arm_selector.setCurrentText(self._arm_name)
        self._arm_selector.currentTextChanged.connect(self._switch_arm)
        arm_layout.addWidget(self._arm_selector)
        arm_layout.addStretch()
        main_layout.addLayout(arm_layout)

        self._topic_label = QtWidgets.QLabel('Publishing to: {}'.format(self._topic_prefix))
        self._topic_label.setWordWrap(True)
        main_layout.addWidget(self._topic_label)

        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel('Parameter'), 0, 0)
        grid.addWidget(QtWidgets.QLabel('Value'), 0, 1)

        row = 1
        for field_name, cfg in self._fields.items():
            label = QtWidgets.QLabel(field_name)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setMinimum(cfg['min'])
            spin.setMaximum(cfg['max'])
            spin.setSingleStep(cfg['step'])
            spin.setValue(cfg['default'])
            spin.valueChanged.connect(self._make_value_cb(field_name))

            self._spinboxes[field_name] = spin
            grid.addWidget(label, row, 0)
            grid.addWidget(spin, row, 1)
            row += 1

        main_layout.addLayout(grid)

        button_layout = QtWidgets.QHBoxLayout()
        reset_btn = QtWidgets.QPushButton('Reset Defaults')
        reset_btn.clicked.connect(self._reset_defaults)
        publish_btn = QtWidgets.QPushButton('Republish All')
        publish_btn.clicked.connect(self._publish_all)
        button_layout.addWidget(reset_btn)
        button_layout.addWidget(publish_btn)

        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)
        self.resize(520, 320)

    def _make_value_cb(self, field_name):
        def _on_value_changed(value):
            self._publish_value(field_name, value)

        return _on_value_changed

    def _publish_value(self, field_name, value):
        msg = Float32()
        msg.data = float(value)
        self._publishers[field_name].publish(msg)

    def _publish_all(self):
        for field_name, spin in self._spinboxes.items():
            self._publish_value(field_name, spin.value())

    def _reset_defaults(self):
        for field_name, cfg in self._fields.items():
            self._spinboxes[field_name].setValue(cfg['default'])
        self._publish_all()


def parse_args():
    parser = argparse.ArgumentParser(description='PyQt GUI for tuning MTM haptics gains')
    parser.add_argument('--arm', default='MTMR', help='MTM arm prefix, e.g. MTMR or MTML')
    return parser.parse_args()


def main():
    args = parse_args()

    g_ral = ral('mtm_haptics_gain_gui')

    app = QtWidgets.QApplication(sys.argv)

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    win = HapticsGainGui(g_ral, args.arm)
    win.show()

    # Let the Qt event loop drive; periodically yield to ral/ROS callbacks.
    timer = QtCore.QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
