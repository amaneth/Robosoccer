# Robosoccer

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


