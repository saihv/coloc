# CoLoC

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/82f2d7a3d4904abe8abb577aea4e9fdc)](https://app.codacy.com/app/saihv/coloc?utm_source=github.com&utm_medium=referral&utm_content=saihv/coloc&utm_campaign=Badge_Grade_Dashboard)

CoLoC is a computer vision based pipeline for collaborative localization between multiple cameras. The targeted application for this software is collaborative localization for micro aerial vehicles equipped with monocular cameras, as described in the following papers:

[1] Sai Vemprala and Srikanth Saripalli. "Monocular Vision based Collaborative Localization for Micro Aerial Vehicle Swarms." 2018 IEEE International Conference on Unmanned Aircraft Systems (ICUAS), pp, 315-323. IEEE, 2018. (https://arxiv.org/abs/1804.02510)  
[2] Sai Vemprala and Srikanth Saripalli. "Vision based collaborative localization for multirotor vehicles." 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 1653-1658. IEEE, 2016.

The code is still a work in progress, with ROS integration, final refinements and testing underway. Install and setup instructions will be added soon.

Libraries/modules used within this project:

1. OpenMVG: https://github.com/openMVG/openMVG  (Base API, detection, matching, structure from motion and localization algorithms)  
2. OpenCV: https://github.com/opencv/opencv  (Image processing and CV related utilities)  
3. KORAL: https://github.com/komrad36/koral  (For GPU based feature description and matching)  
4. dlib: https://github.com/davisking/dlib  (For performing single variable bounded minimization as part of the covariance intersection algorithm)  

All credits for the individual components coming from these projects go to the original author(s).
