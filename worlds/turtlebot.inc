define kinect ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define hokuyo ranger
(
  sensor
  (
    range_min 0.0
    range_max 2.0
    fov 270.25
    samples 1081
  )

  color "black"
  size [ 0.05 0.05 0.1 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  #kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  hokuyo(pose [ -0.1 0.0 -0.11 0.0 ])

)
