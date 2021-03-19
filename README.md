# Tracking tester

Copyright (c) 2020-2021 [Antmicro](https://www.antmicro.com)

*For an example on how to use this repository check* [ros-tracking-policy-examples repository](https://github.com/antmicro/ros-tracking-nodes-policy-examples).

ROS node, which given annotated frames, tests a detecting/tracking system measures its performance and returns results in a `.csv` file.
It also uses `stopwatch` node from [ros-tracking-nodes-stopwatch repository](https://github.com/antmicro/ros-tracking-nodes-stopwatch) to gather additional measurements.

### Node usage

The node is created to run one test and dies afterwards.

* `-v`, `--visualize` - Print frames on screen, optional
* `-f`, `--fps` - fps of playback, optional - default is wait mode
* `-i`, `--input` - path to directory containing frames and annotations
* `-o`, `--output` - path where output should be saved, .csv file extension, optional

Tested node should subscribe to topic `tracking_tester/frame` and publish to topic `tracking_tester/bbox`.

### Input format

There should be an image file of common format (readable by OpenCV) for each frame and one annotation in the input directory.

Image names should be in format `number.extension` where `number` is an arbitrary integer (possibly with leading zeros).
Frames are processed in increasing number order (sorted numerically, not lexicographically).

The annotation file should be in [ALOV dataset](http://alov300pp.joomlafree.it/dataset-resources.html) format, that is:

* there is one line for each frame
* each line is in form `ord X1 Y1 X2 Y2 X3 Y3 X4 Y4`
* `ord` is an integer describing frame number, starting from one
* the other 8 numbers describe corners of the rectangular bounding box, (current version assumes `Y1 = Y2`, `Y3 = Y4`, `X1 = X3`, `X2 = X4`)

It is assumed that the target is visible in every frame.

### Output

The result of one test is a `.csv` file (with a header) in following format:

`frame_number,time,iou,left,top,width,height,realLeft,realTop,r ealWidth,realHeight`

Where:

* `time` is `ros::Time` as a real number.
* `left`, `top`, `width`, `height` specify bounding box received from tested policy.
* `realLeft`, `realTop`, `realWidth`, `realHeight` describe ground-truth bounding box from the dataset.
* `iou` is Intersection over Union of these two bounding boxes.

Other files might be written to the directory containing provided output file.

## Automated tester script

There is a Python3 script `test.py` supplemented, which runs tester and generates output, which can be parsed by `vot-report-generator`.

### Script and config

The script can test multiple policies at once. For each policy to be tested, there should be four consecutive lines in `test.config` file.

```
policy_name
an integer - fps of playback or zero for wait mode
a command which will be exectued before a test
a command which will be executed after a test
```

Example:

```
my_detector                             # name of the policy
60                                      # run at 60 fps
./detector-node --threshold 0.7         # start the node before every test
rosnode kill detector-node              # kill the node after every test
...
detector2                               # name a different of the policy
0                                       # use wait mode
rosservice call /detector2/node-init    # the node is already running, just initialize it
rosservice call /detector2/node-reset   # reset the node after test
```

### Script usage

The script call is following:

```
python3 test.py dataset/sequence1 dataset/sequence2 ... out_path --passes 5 --config-path sample.config
```

* `dataset/sequence1 dataset/sequence2 ...` - paths to directories with sequences in ALOV format
* `out_path` - path to empty diretory - where the results should be saved
* `--passes` - number of passes over all datasets for all policies
* `--config-path` - path to the test configuration file

### Output

The script generates a directory for each tested policy as well as a metadata file. 
In each of created directories there are `.csv` files in following format: `<test_name>_<pass_number>.csv`.
The metadata file contains lines in following format `<property>=<value>`, like `test1=hills`.

## Video sequences generation

This tester requires video sequences with annotations in [ALOV dataset](http://alov300pp.joomlafree.it/dataset-resources.html) format.
Those can be generated using the [video2dataset tool](https://github.com/antmicro/video2dataset).

## Licensing

The sources are published under the Apache 2.0 License, except for files located in the `third-party/` directory.
For those files the license is either enclosed in the file header or in a separate LICENSE file.
