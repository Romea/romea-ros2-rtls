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
from romea_common_bringup import MetaDescription, robot_urdf_prefix, device_namespace
from romea_rtls_transceiver_bringup import RTLSTransceiverMetaDescription
from romea_rtls_transceiver_description import transceiver_urdf


class RTLSTransceiversMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription(
            "rtls_transceivers", meta_description_file_path
        )

    def get_name(self):
        return self.meta_description.get("name")

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_communication(self):
        return self.meta_description.get_or("communication", "configuration", None)

    def get_pan_id(self):
        return self.meta_description.get_or("pan_id", "configuration", None)

    def get_transceivers(self):
        return self.meta_description.get("transceivers", "configuration")


def get_configuration_param(meta_description, transceiver_meta_description, what):
    global_param = getattr(meta_description, "get_" + what)()
    transceiver_param = getattr(transceiver_meta_description, "get_" + what)()

    if global_param is not None:
        if transceiver_param != global_param:
            raise AttributeError(
                "RTLS system has not the same "
                + what
                + " than transceiver called "
                + transceiver_meta_description.get_name()
            )

    return global_param


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = RTLSTransceiversMetaDescription(meta_description_file_path)
    directory = os.path.dirname(meta_description_file_path)

    transceivers_urdf = ""
    for transceiver in meta_description.get_transceivers():

        transceiver_meta_description = RTLSTransceiverMetaDescription(
            directory + "/" + transceiver + "rtls_transceiver.yaml"
        )

        ros_namespace = device_namespace(
            robot_namespace,
            meta_description.get_name(),
            transceiver_meta_description.get_name(),
        )

        transceiver_pan_id = (
            get_configuration_param(
                meta_description, transceiver_meta_description, "pan_id"
            ),
        )

        transceiver_communication = (
            get_configuration_param(
                meta_description, transceiver_meta_description, "communication"
            ),
        )

        transceivers_urdf += transceiver_urdf(
            robot_urdf_prefix(robot_namespace),
            mode,
            transceiver_meta_description.get_name(),
            transceiver_meta_description.get_type(),
            transceiver_communication,
            transceiver_pan_id,
            transceiver_meta_description.get_pan_id(),
            transceiver_meta_description.get_id(),
            transceiver_meta_description.get_mode(),
            transceiver_meta_description.get_parent_link(),
            transceiver_meta_description.get_xyz(),
            ros_namespace,
        )

    return transceivers_urdf
