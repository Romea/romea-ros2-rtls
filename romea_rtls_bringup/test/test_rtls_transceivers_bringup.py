# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


import os
import pytest

from romea_rtls_transceivers_bringup import RTLSTransceiversMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_rtls_transceivers_bringup.yaml")
    return RTLSTransceiversMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "rtls"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "rtls"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "romea_rtls_transceivers_hub"


def test_get_communication(meta_description):
    assert meta_description.get_communication() == "4GHz_6M8bit"


def test_get_pan_id(meta_description):
    assert meta_description.get_pan_id() == 0
