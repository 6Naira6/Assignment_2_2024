.. _index:

Assignment 2 Documentation
======================================

Overview
--------
This project implements a ROS-based system that includes:

1. **Action Client**:
   - Allows users to dynamically set a goal (x, y) or cancel it.
   - Tracks the robot’s progress using feedback from the action server.
   - Logs the robot’s position and velocity using data from the ``/odom`` topic.
   - Publishes the last set target to a ROS topic for tracking.

2. **Service Node**:
   - Maintains a record of the last target set by the user.
   - Responds to service calls by returning the most recently set target coordinates.

Installation
------------
1. Clone the repository into your catkin workspace:

   .. code-block:: bash

      cd ~/workspace/src
      git clone https://github.com/6Naira6/Assignment_2_2024.git

2. Build the workspace:

   .. code-block:: bash

      cd ~/workspace
      catkin_make
      source devel/setup.bash

3. Ensure all required dependencies are installed:

   .. code-block:: bash

      sudo apt-get install ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-std-msgs ros-noetic-std-srvs ros-noetic-nav-msgs

How to Launch
-------------

**Step 1: Launch the Project**

Start the action client and service nodes:

.. code-block:: bash

   roslaunch assignment_2_2024 assignment1.launch

or:

.. code-block:: bash

   roslaunch assignment_2_2024 assignment1_pos.launch

**Step 2: Set a Goal**

Interact with the action client by entering commands in the terminal:

- **Set a new goal**:

  .. code-block:: none

     set x y

  Example:

  .. code-block:: none

     set 5 5

- **Cancel the current goal**:

  .. code-block:: none

     cancel

**Step 3: Retrieve the Last Target**

Call the service to get the last set target:

.. code-block:: bash

   rosservice call /get_last_target true

The output will look like:

.. code-block:: none

   success: True
   message: "5.0, 5.0"

**Step 4: Monitor Logs**

- Monitor the robot’s position and velocity:

  .. code-block:: bash

     rostopic echo /odom

- View the feedback from the action server:

  .. code-block:: bash

     rostopic echo /reaching_goal/feedback

Proof of Work
-------------
.. image:: static/WithCommands.gif
    :alt: Demo with commands

.. image:: static/WithData.gif
    :alt: Demo with data


.. toctree::
   :maxdepth: 2
   :caption: Contents

   modules
