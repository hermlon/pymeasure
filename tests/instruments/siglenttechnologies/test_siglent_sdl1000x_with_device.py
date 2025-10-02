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

from pymeasure.instruments.siglenttechnologies.siglent_sdl1000x import SDL1000X, StaticMode
from pymeasure.units import ureg


@pytest.fixture(scope="module")
def sdl1000x(connected_device_address):
    instr = SDL1000X(connected_device_address)
    return instr


def test_get_instrument_id(sdl1000x):
    assert "Siglent Technologies,SDL10" in sdl1000x.id


def test_enable(sdl1000x):
    sdl1000x.enable = True
    assert sdl1000x.enable
    sdl1000x.reset()
    assert not sdl1000x.enable


@pytest.mark.parametrize("mode", list(StaticMode))
def test_static_mode(sdl1000x, mode):
    sdl1000x.static_mode = mode
    assert sdl1000x.static_mode == mode


def test_current(sdl1000x):
    sdl1000x.static_mode = StaticMode.CURRENT
    sdl1000x.current = 500 * ureg.mA
    assert sdl1000x.current == 0.5 * ureg.A


@pytest.mark.parametrize("ran", [5, 30])
def test_current_irange(sdl1000x, ran):
    sdl1000x.static_mode = StaticMode.CURRENT
    sdl1000x.current = 500 * ureg.mA
    sdl1000x.current_irange = ran
    assert sdl1000x.current_irange == ran


@pytest.mark.parametrize("ran", [36, 150])
def test_current_vrange(sdl1000x, ran):
    sdl1000x.static_mode = StaticMode.CURRENT
    sdl1000x.current = 500 * ureg.mA
    sdl1000x.current_vrange = ran
    assert sdl1000x.current_vrange == ran


def test_current_slope(sdl1000x):
    sdl1000x.static_mode = StaticMode.CURRENT
    sdl1000x.current_slope_rise = 0.4
    sdl1000x.current_slope_fall = 0.5
    assert sdl1000x.current_slope_rise == 0.4
    assert sdl1000x.current_slope_fall == 0.5
