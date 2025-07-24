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

from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x import SDM3065X, Mode
from pymeasure.units import ureg

@pytest.fixture(scope='module')
def sdm3065x(connected_device_address):
    instr = SDM3065X(connected_device_address)
    return instr

def test_get_instrument_id(sdm3065x):
    assert 'Siglent Technologies,SDM3065X' in sdm3065x.id


@pytest.mark.parametrize('mode', list(Mode))
def test_set_mode(sdm3065x, mode):
    sdm3065x.mode = mode
    assert sdm3065x.mode == mode

@pytest.mark.parametrize('mode', [Mode.CURRENT_DC, Mode.CURRENT_AC])
def test_get_current(sdm3065x, mode):
    sdm3065x.mode = mode
    assert sdm3065x.current

@pytest.mark.parametrize('mode', [Mode.VOLTAGE_DC, Mode.VOLTAGE_AC])
def test_get_voltage(sdm3065x, mode):
    sdm3065x.mode = mode
    assert sdm3065x.voltage