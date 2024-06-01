.. assignment2 documentation master file, created by
   sphinx-quickstart on Fri May 31 19:33:02 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to assignment2's documentation!
=======================================

This document provides an overview of the assignment2 code and its functionalities.

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

**Assignment2 documentation!**
===============================
This is the documentation of the Assignment2 package! The assignment\_2\_2023 repository contains various files and scripts that are integral to a specific assignment or project. This report aims to provide a comprehensive explanation of the repository's contents, highlighting the purpose and functionality of each file.

**File Overview**
==================

* **CMakeLists.txt**
   * **Purpose:** An essential file in a ROS (Robot Operating System) project, defining how the project should be built.
   * **Functionality:** Specifies project name, required packages, dependencies, compilation flags, and other build configurations.
   * **Usage:** CMake uses this file to generate build scripts for different build systems (e.g., Make, Ninja) to compile the project.

* **package.xml**
   * **Purpose:** An XML file providing information about the ROS package.
   * **Functionality:** Includes metadata such as package name, version, maintainer details, license information, and package dependencies.
   * **Usage:** Crucial for package management, distribution, and ensuring compatibility between different ROS packages.

* **src/ directory**
   * **Purpose:** Contains the main source code files for the project.
   * **Functionality:** Scripts implement project functionalities, algorithms, data processing, and communication with external devices or services.
   * **Usage:** Scripts within this directory are responsible for the core operations and may interact with other ROS nodes and modules.

* **launch/ directory**
   * **Purpose:** Stores ROS launch files, defining configurations and parameters for launching ROS nodes.
   * **Functionality:** Launch files allow for simultaneous execution of multiple ROS nodes with specific settings.
   * **Usage:** Used to start and manage the project's ROS nodes, set parameters, and establish connections between nodes.

* **config/ directory**
   * **Purpose:** Contains configuration files for the project.
   * **Functionality:** Stores parameters, settings, or calibration data used by project scripts or ROS nodes.
   * **Usage:** Configuration files allow for easy customization of the project's behavior without modifying the source code directly.

* **Other Files**
   Additional files such as README.md, LICENSE, or documentation files may also exist, providing instructions, project descriptions, licensing information, or other relevant documentation.

**Assignment2 Modules**
===========================

Action client node
*******************

.. automodule:: action_client_node
   :members:
   
   
Position node
****************

.. automodule:: position_node
   :members:


Target service node
**********************

.. automodule:: target_service_node
   :members:


**Conclusion**
==============

The assignment\_2\_2023 repository comprises various files and scripts that are vital for the project's functionality. The CMakeLists.txt file defines the project's build configurations, while the package.xml file contains essential metadata and dependency information.

The src/ directory houses the main source code files, implementing the project's core functionalities. The launch/ directory contains launch files for managing and starting ROS nodes, and the config/ directory stores configuration files for customizing the project's behavior.

To gain a deeper understanding of the project, it is crucial to review and analyze the contents of each file, along with any accompanying documentation or instructions provided within the repository.


