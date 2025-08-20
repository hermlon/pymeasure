#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2025 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

import pytest

from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x import Mode, OnOff
from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x_sc import (
    SDM3065XSC,
    LoopMode,
    CycleMode,
    VoltageChannelModeVoltage,
    VoltageChannelModeResistance,
    VoltageChannelModeCapacitance,
    VoltageChannelMode,
    CurrentChannelMode,
    VoltageRange,
    ResistanceRange,
    CapacitanceRange,
    CurrentRange,
    ChannelSpeed,
    VoltageChannelConfig,
)
from pymeasure.units import ureg


@pytest.fixture(scope="module")
def sdm3065xsc(connected_device_address):
    instr = SDM3065XSC(connected_device_address)
    return instr


@pytest.mark.parametrize("sc_enabled", [True, False])
def test_scan_card_enabled(sdm3065xsc, sc_enabled):
    sdm3065xsc.sc_enabled = True
    assert sdm3065xsc.sc_enabled
    sdm3065xsc.sc_enabled = False
    assert not sdm3065xsc.sc_enabled


@pytest.mark.parametrize("sc_start", [True, False])
def test_scan_card_start(sdm3065xsc, sc_start):
    sdm3065xsc.sc_enabled = True
    sdm3065xsc.sc_start = True
    assert sdm3065xsc.sc_start
    sdm3065xsc.sc_start = False
    assert not sdm3065xsc.sc_start


@pytest.mark.parametrize("loop_mode", list(LoopMode))
def test_function(sdm3065xsc, loop_mode):
    sdm3065xsc.sc_loop_mode = loop_mode
    assert sdm3065xsc.sc_loop_mode == loop_mode


@pytest.mark.parametrize("delay", [0, 1.0, 0.8])
def test_delay(sdm3065xsc, delay):
    sdm3065xsc.sc_delay = delay
    assert sdm3065xsc.sc_delay == delay


@pytest.mark.parametrize("cycle_mode", list(CycleMode))
def test_cycle_mode(sdm3065xsc, cycle_mode):
    sdm3065xsc.sc_cycle_mode = cycle_mode
    assert sdm3065xsc.sc_cycle_mode == cycle_mode


@pytest.mark.parametrize("count", [1, 5])
def test_count(sdm3065xsc, count):
    sdm3065xsc.sc_count = count
    assert sdm3065xsc.sc_count == count


@pytest.mark.parametrize(
    "ch_low,ch_high", [(low, high) for high in range(1, 17) for low in range(1, high + 1)]
)
def test_sc_limit(sdm3065xsc, ch_low, ch_high):
    sdm3065xsc.sc_ch_limit_low = ch_low
    sdm3065xsc.sc_ch_limit_high = ch_high
    assert sdm3065xsc.sc_ch_limit_low == ch_low
    assert sdm3065xsc.sc_ch_limit_high == ch_high


@pytest.mark.parametrize("ch", range(1, 2))
def test_channel_voltage(sdm3065xsc, ch):
    print("hi")
    import time

    # print(getattr(sdm3065xsc, f"ch_{ch}").voltage)
    sdm3065xsc.sc_enabled = True
    sdm3065xsc.sc_cycle_mode = CycleMode.MANUAL
    sdm3065xsc.sc_count = 2
    sdm3065xsc.sc_ch_limit_low = 1
    sdm3065xsc.sc_ch_limit_high = 2
    sdm3065xsc.sc_start = True
    time.sleep(2)
    print(sdm3065xsc.ch_1.voltage)
    print(sdm3065xsc.ch_1.voltage)
    sdm3065xsc.ch_1.switch = False
    # sdm3065xsc.sc_start = False


@pytest.mark.parametrize("ch", range(1, 13))
@pytest.mark.parametrize("switch", list(OnOff))
@pytest.mark.parametrize("mode", list(VoltageChannelMode))
@pytest.mark.parametrize("count", range(1, 3))
def test_voltage_channel_config(sdm3065xsc, ch, switch, mode, count):
    channel = getattr(sdm3065xsc, f"ch_{ch}")
    sdm3065xsc.sc_enabled = True
    conf = VoltageChannelConfig(switch, mode, count=count)
    channel.config = conf
    assert str(channel.config) == str(conf)


# TODO: test other channel modes, possibly extract test code to fixture
