# L-CAS 3D Point Cloud Annotation Tool #

[![Build Status][travis-img]][travis-link]
[![Codacy Badge][codacy-img]][codacy-link]

![alt tag](https://github.com/LCAS/cloud_annotation_tool/blob/master/images/screenshot.png)

* Maintainer status: maintained
* Author: Zhi Yan
* License: CC BY-NC-SA 4.0
* Dataset: [https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-3d-point-cloud-people-dataset/)

The tool provides a semi-automatic labeling function, means the 3D point cloud data (loaded from the PCD file) is first clustered to provide candidates for labeling, each candidate being a point cluster. Then, the user annotating the data, can label each object by indicating candidate's ID, category, and visibility. A flowchart of this process is shown below.

![alt tag](https://github.com/LCAS/cloud_annotation_tool/blob/master/images/flowchart.png)
