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
from itertools import product, chain

from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x import Mode, OnOff
from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x_sc import (
    SDM3065XSC,
    LoopMode,
    CycleMode,
    VoltageChannelModeVoltage,
    VoltageChannelModeResistance,
    VoltageChannelModeCapacitance,
    VoltageChannelModeFrequency,
    VoltageChannelModeNoRange,
    CurrentChannelMode,
    VoltageRange,
    ResistanceRange,
    CapacitanceRange,
    CurrentRange,
    ChannelSpeed,
    VoltageChannelConfig,
    CurrentChannelConfig,
)
from pymeasure.units import ureg


@pytest.fixture(scope="module")
def sdm3065xsc(connected_device_address):
    instr = SDM3065XSC(connected_device_address)
    # enable for all following scancard tests to work
    instr.sc_enabled = True
    return instr


@pytest.mark.parametrize("sc_enabled", [True, False])
def test_scan_card_enabled(sdm3065xsc, sc_enabled):
    sdm3065xsc.sc_enabled = True
    assert sdm3065xsc.sc_enabled
    sdm3065xsc.sc_enabled = False
    assert not sdm3065xsc.sc_enabled
    sdm3065xsc.sc_enabled = True


@pytest.mark.parametrize("sc_start", [True, False])
def test_scan_card_start(sdm3065xsc, sc_start):
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


def test_channel_voltage(sdm3065xsc):
    sdm3065xsc.reset()
    sdm3065xsc.sc_cycle_mode = CycleMode.MANUAL
    sdm3065xsc.sc_count = 1
    sdm3065xsc.sc_ch_limit_low = 1
    sdm3065xsc.sc_ch_limit_high = 2

    sdm3065xsc.scan()

    assert sdm3065xsc.ch_1.voltage > 0
    assert sdm3065xsc.ch_2.voltage > 0
    assert sdm3065xsc.ch_1.voltage == 0
    assert sdm3065xsc.ch_2.voltage == 0


@pytest.fixture(scope="module", params=range(1, 13))
def voltage_channel(request, sdm3065xsc):
    ch = request.param
    channel = getattr(sdm3065xsc, f"ch_{ch}")
    yield channel


@pytest.fixture(scope="module", params=range(13, 17))
def current_channel(request, sdm3065xsc):
    ch = request.param
    channel = getattr(sdm3065xsc, f"ch_{ch}")
    yield channel


@pytest.mark.parametrize("count", range(1, 3))
@pytest.mark.parametrize(
    "mode,mrange,speed",
    chain(
        product(list(VoltageChannelModeNoRange), [None], [None]),
        product(list(VoltageChannelModeVoltage), list(VoltageRange), list(ChannelSpeed)),
        product(list(VoltageChannelModeResistance), list(ResistanceRange), list(ChannelSpeed)),
        product(list(VoltageChannelModeCapacitance), list(CapacitanceRange), [None]),
        product(list(VoltageChannelModeFrequency), list(VoltageRange), [None]),
    ),
)
@pytest.mark.parametrize("switch", list(OnOff))
def test_voltage_channel_config(sdm3065xsc, voltage_channel, switch, mode, mrange, speed, count):
    if int(voltage_channel.id) in range(7, 13) and mode == VoltageChannelModeResistance.R4WIRE:
        pytest.skip("4W not supported on channels 7 to 12")
    conf = VoltageChannelConfig(switch, mode, mrange, speed, count=count)
    voltage_channel.config = conf
    assert str(voltage_channel.config) == str(conf)


@pytest.mark.parametrize("count", range(1, 3))
@pytest.mark.parametrize("speed", list(ChannelSpeed))
@pytest.mark.parametrize("mode,mrange", product(list(CurrentChannelMode), list(CurrentRange)))
@pytest.mark.parametrize("switch", list(OnOff))
def test_current_channel_config(sdm3065xsc, current_channel, switch, mode, mrange, speed, count):
    conf = CurrentChannelConfig(switch, mode, mrange, speed, count=count)
    current_channel.config = conf
    assert str(current_channel.config) == str(conf)


def test_voltage_channel_4w(sdm3065xsc):
    sdm3065xsc.ch_1.config = VoltageChannelConfig(
        OnOff.ON, VoltageChannelModeResistance.R4WIRE, ResistanceRange.AUTO, ChannelSpeed.FAST
    )
    sdm3065xsc.ch_6.config = VoltageChannelConfig(
        OnOff.ON, VoltageChannelModeResistance.R4WIRE, ResistanceRange.AUTO, ChannelSpeed.FAST
    )
    sdm3065xsc.ch_7.config = VoltageChannelConfig(
        OnOff.ON, VoltageChannelModeVoltage.VOLTAGE_AC, ResistanceRange.AUTO, ChannelSpeed.FAST
    )
