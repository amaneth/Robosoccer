# soccercode

## Launch

See src/launchers launch files.

## Outline

  - [X] Raspberry Pi Video capture node ('raspicam', C++, modified a bit from https://github.com/fpasteau/raspicam_node)
  - [X] blob_detector node for detecting blobs from a list of colors ('blob_detector', Python)
    - looks for a areas of colors from a list of HSV color ranges
    - publishes the name, location and sizes of the top ten biggest blobs.

  - [~] Preprocessing node (`blob_mapping`, Python)
    - subscribes to topic from blob_detector
    - converts x/y value and area messages then 
      maps them to real space (angle from robot and distance)
    - it also tries to find markers by checking pairs of colors
    - observed markers are compiled together and published for robot_localization.

  - [~] Localization node (`robot_localization`, Python)
    - receives name, location and size of observed_markers 
    - will calculate its position from observed stuff
    - holds the positions of the markers/goals on the field
    - also will calculate position of the ball
    - sends out location and ball_position estimate with Id

  - [ ] Master node (`master_sender`, Python)
    - receives locations of robots (also it's own)
    - averages or takes position of ball from closest robot
    - Plans best trajectories for robots and sends out commands to `/robot` nodes to achieve it:
      - *Naive strategy*
        - Look for closest robot to ball
        - Calculate optimal position to kick the ball towards the goal
        - Calculate best path, and first step on that path
        - Send actuator action to achieve that path
        - Works iteratively!



