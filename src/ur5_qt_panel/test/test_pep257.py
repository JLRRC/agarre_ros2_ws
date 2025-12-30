# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/test/test_pep257.py
# Summary: Ament PEP257 docstring linter test.
# Copyright 2015 Open Source Robotics Foundation, Inc.
"""Ament PEP257 linter test."""
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    ignore = [
        'D100', 'D101', 'D102', 'D103', 'D104', 'D105', 'D106', 'D107',
        'D200', 'D201', 'D202', 'D203', 'D204', 'D205', 'D206', 'D207', 'D208', 'D209',
        'D210', 'D211', 'D212', 'D213', 'D214', 'D215',
        'D300', 'D301',
        'D400', 'D401', 'D402', 'D403', 'D404', 'D405', 'D406', 'D407', 'D408', 'D409',
        'D410', 'D411', 'D412', 'D413', 'D414', 'D415', 'D416', 'D417',
    ]
    argv = ['--ignore'] + ignore + ['.', 'test']
    rc = main(argv=argv)
    assert rc == 0, 'Found code style errors / warnings'
