# Tracking tester

ROS node, which given annotated frames, tests a detecting/tracking system measures its performance and returns results in a `.csv` file.
It also uses `stopwatch` node to gather additional measurements.

### Node usage

The node is created to run one test and dies afterwards.

* `-v`, `--visualize` - Print frames on screen, optional
* `-f`, `--fps` - fps of playback, optional - default is wait mode
* `-i`, `--input` - path to directory containing frames and annotations
* `-o`, `--output` - path where output should be saved, .csv file extension, optional

## test.py

There is a Python3 script supplemented, which runs tester and generates output, which can be parsed by `vot-report-generator`.

### Config

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

* `--in_paths` - paths to directories with datasets in ALOV format 
* `--out_path` - path to empty diretory - where should results be saved
* `--passes` - number of passes over all datasets for all policies

