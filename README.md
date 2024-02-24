<!--
NEPI Dual-Use License
Project: nepi_edge_sdk_ai

This license applies to any user of NEPI Engine software

Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
see https://github.com/numurus-nepi/nepi_edge_sdk_ai

This software is dual-licensed under the terms of either a NEPI software developer license
or a NEPI software commercial license.

The terms of both the NEPI software developer and commercial licenses
can be found at: www.numurus.com/licensing-nepi-engine

Redistributions in source code must retain this top-level comment block.
Plagiarizing this software to sidestep the license obligations is illegal.

Contact Information:
====================
- https://www.numurus.com/licensing-nepi-engine
- mailto:nepi@numurus.com

-->
# nepi_edge_sdk_ai #
This repository provides the _nepi_edge_sdk_ai_ ROS node and support files. The _nepi_edge_sdk_ai_ runs NEPI’s AI Model Management (AIM) system, which provides a framework for storing, connecting, and running AI models from an onboard AI model library, as well as publishing AIM results to downstream consumers like a NEPI automation script of connected control system. The AIM system manages which of the available AI models is active at a given time, which sensor stream is provided as input to the active model, and any additional parameters related to AI model execution. The AIM system also supports external AI process orchestration such as active model selection and on-the-fly data input switching. The AIM system supports AI model output logging and ensures AI model outputs adhere to a standard ROS 2D label topic format, ensuring a consistent interface for both on-board and off-board consumers of the AIM data output streams.

On bootup, the AIM system queries all the models stored in the AIM Onboard AI Model Library located on the user storage drive mounted in the NEPI File System at:

	/mnt/nepi_storage/ai_models

This folder is also accessible and editable from a PC’s File Manager application connected to the NEPI user storage drive at:

	smb://192.168.179.103/nepi_storage/ai_models

NOTE: Models deployed into the AIM’s AI Model Library must be supported by one of the AIM AI frameworks installed on your NEPI device and placed into the appropriate library subfolders for the specific AI framework you intend to run your models with. To see a list of AIM supported AI frameworks, required files, and AIM Model Library deployment folders, see the document “NEPI Engine – Supported AI Frameworks and Models” available at:

	https://nepi.com/documentation/

### Build and Install ###
This repository is typically built as part of the _nepi_base_ws_ catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the _nepi_base_ws_ source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the _nepi_base_ws_ container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.