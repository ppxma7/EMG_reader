# EMG_reader
- This is in-development code
- MATLAB code to record EMG and display force traces
- Specific to OT BIO elettronica EMG systems
- Please contact Michael Asghar for any questions / problems with the code. I would avoid playing around with the communication protocols near the top of the scripts, as these are very specific to OT. Feel free to mess about with difference force traces etc.

### Installation

- Tested on:

MATLAB Version: 25.2.0.3150157 (R2025b) Update 4
MATLAB License Number: 646274
Operating System: Microsoft Windows 11 Home Version 10.0 (Build 26200)

- There are several versions of EMG_reader, specific to the OT Bio elettronica EMG system. EMG_reader works with Sessantaquattro+, Muovi+ and Novecento+.

- `experiment_muovi6.m` is the script that works with the Muovi+ (wireless) system. This expects 2x64ch Muovis, and communicates via the Syncstation. This script is specific to the ePhys lab, and expects two force channels.

- `experiment_sessanta2_singleforce.m` works with the Sessantaquattro+. This script is specific to the Derby chair and expects a single force channel.

- `experiment_novecento` is the script that works with the novecento (wired) system in the ePhys lab. Again, this expects a multi-channel force rig (L and R). This system can theoretically accommodate up to 10 probes. Currently the script is set to work with 6x BIO64HD probes.

### Example running novecento

- At the top of each script, you can set your file saving preferences, e.g. subject number, study condition. The `force_dir` is important as depending on if you set it to push or pull, it will reverse the force direction.

- Set the `datapath` variable correctly otherwise it won't save - to where you want to save.

- Set `mvcLeft` and `mvcRight` to [] to calculate MVC. With the code running, hit 'm' and it will perform an MVC. Note this number down and set the values for mvcLeft (or mvcRight).

- The code allows many force traces to be drawn (`task_shape`) - you can also set your own custom versions in the `run_task` function. Hit 't' after the MVC values have been populated and it will start the task. The code only saves data when a task or MVC is complete - do not quit any window before it is complete.

### Calibration in ePhys lab

- Several load cells have been calibrated using known weights. These are noted down as comments in the load cell calibration section near the top of the script. To ignore any calibration, set `force_scale_L` and `force_scale_R` to 1. Otherwise, it must be set to a value, e.g. for the S-type cells on the TA/MG rig, the value is 0.01905.

#### Highly specific notes

- For the fatigue task, you must press 'q' to quit, don't close the window or it won't save.

- If you want to use the AUX inputs instead of the load cell inputs on the back of the novecento, you can, just set the force_scale to 10/65536. You will then need to use your own amplifier, e.g. forza. A version of the novecento script using a forza-B is made, called `experiment_novecento_forza`.





