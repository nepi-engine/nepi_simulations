#!/usr/bin/env python

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# Ported verbatim from nepi_setup's resources/bash/nepi_utils/sync_yaml_files.py
# (see nepi_engine_ws/nepi_setup/resources/docker/nepi_docker_sync.sh, which
# calls this same helper via nepi_bash_utils' sync_yaml_files) so gazebo_setup's
# sync_yaml_files bash function in nepi_gazebo_bash_utils has something to
# shell out to.

# This script loads echos yaml keys and values from file

success=0

import os
import sys
import shutil
import yaml

def read_yaml_2_dict(file_path):
    dict_from_file = dict()
    success=0
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
                if dict_from_file is None:
                    dict_from_file = dict()
                success = 1
        except Exception as e:
           pass

    return success, dict_from_file


def write_dict_to_file(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success=0
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = 1
    except Exception as e:
        pass
    return success

overwrite = False
success = 0
if len(sys.argv) > 1:
    SOURCE_YAML_FILE = sys.argv[1]
    TARGET_YAML_FILE = sys.argv[2]
    if os.path.exists(SOURCE_YAML_FILE):
        if os.path.exists(TARGET_YAML_FILE) == False:
            print("Target file not found " + str(TARGET_YAML_FILE) + " will create")
            shutil.copyfile(SOURCE_YAML_FILE, TARGET_YAML_FILE)
            if os.path.exists(TARGET_YAML_FILE) == True:
                success=1
        else:
            [success,source_dict] = read_yaml_2_dict(SOURCE_YAML_FILE)
            if success == 1:
                success = 0
                [success,target_dict] = read_yaml_2_dict(TARGET_YAML_FILE)
                if success == 1:
                    for key in source_dict.keys():
                        if key not in target_dict.keys():
                            target_dict[key] = source_dict[key]
                    success=write_dict_to_file(target_dict, TARGET_YAML_FILE)


else:
     print("Source file not found " + str(SOURCE_YAML_FILE))



print(str(success))
