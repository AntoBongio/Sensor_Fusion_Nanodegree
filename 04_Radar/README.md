
# Radar Target Generation and Detection

The radar’s capability to determine the targets at long range with accurate velocity and spatial information make it an important sensor for self driving applications. Additionally, its capability to sense objects in dark and poor weather (rain, fog) conditions also help it cover the domains where LIDAR or camera may fail.

Given a Radar withthe following characteristics

<img src="media/radar_specs.png" width="700" height="300" />

We want to:
- Configure the FMCW waveform based on the system requirements.
- Define the range and velocity of target and simulate its displacement.
- For the same simulation loop process the transmit and receive signal to determine the beat signal
- Perform Range FFT on the received signal to determine the Range
- Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.

The figure below summarises the pipeline.

<img src="media/project_pipeline.png" width="1000" height="500" />


## Final result




