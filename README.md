#Tracking tester

ROS node, which given annotated frames, tests a detecting/tracking system
measures its performance and returns results in a `.csv` file.
Together with `stopwatch` these tools create data which can be parsed by
`vot-report-generator`.

## Usage
```
-v, --visualize				Print frames on screen, optional
-f, --fps					fps of playback, optional - default is wait mode
-i, --input					path to directory containing frames and annotations
-o, --output				path where output should be saved
```
