## Notices ##

* Compilation error under Ubuntu 16.04 LTS Xenial: [Issue #4](https://github.com/yzrobot/cloud_annotation_tool/issues/4)

* :boom: [L-CAS 3D Point Cloud Annotation Tool :two:](https://github.com/yzrobot/cloud_annotation_tool/tree/devel) is released. :boom:

# L-CAS 3D Point Cloud Annotation Tool #

[![Build Status](https://travis-ci.org/yzrobot/cloud_annotation_tool.svg?branch=master)](https://travis-ci.org/yzrobot/cloud_annotation_tool)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/ecd31982b8ef4e21b096d7ded0979bb8)](https://www.codacy.com/app/yzrobot/cloud_annotation_tool?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=yzrobot/cloud_annotation_tool&amp;utm_campaign=Badge_Grade)
[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

![screenshot](/images/screenshot.png)

* Maintainer status: maintained
* Author: Zhi Yan
* License: CC BY-NC-SA 4.0
* Dataset: [https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/)

The tool provides a semi-automatic labeling function, means the 3D point cloud data (loaded from the PCD file) is first clustered to provide candidates for labeling, each candidate being a point cluster. Then, the user annotating the data, can label each object by indicating candidate's ID, category, and visibility. A flowchart of this process is shown below.

![flowchart](/images/flowchart.png)

*The quickest way to activate the optional steps is to modify the source code and recompile. :scream:*

## Compiling (tested on Ubuntu 14.04/16.04) ##

### Prerequisites ###

* Qt 4.x: `sudo apt-get install libqt4-dev qt4-qmake`
* VTK 5.x: `sudo apt-get install libvtk5-dev`
* PCL 1.7: `sudo apt-get install libpcl-1.7-all-dev`

### Build and run ###

* `mkdir build`
* `cd build`
* `cmake ..`
* `make`
* `./cloud_annotation_tool`

## Test examples ##

[lcas_simple_data.zip](lcas_simple_data.zip) contains 172 consecutive frames (in .pcd file) with 2 fully annotated pedestrians.

## Citation ##

If you are considering using this tool and the data provided, please reference the following:

```
bibtex coming soon!
```
