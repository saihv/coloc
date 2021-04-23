# CoLoC

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/82f2d7a3d4904abe8abb577aea4e9fdc)](https://app.codacy.com/app/saihv/coloc?utm_source=github.com&utm_medium=referral&utm_content=saihv/coloc&utm_campaign=Badge_Grade_Dashboard)

CoLoC is a computer vision based pipeline for collaborative localization between multiple cameras. The targeted application for this software is collaborative localization for micro aerial vehicles equipped with monocular cameras, as described in our paper "Collaborative Localization for Micro Aerial Vehicles". If you find our paper or this code useful in your research, please cite our work as follows:

@article{vemprala2021collaborative,
  author={Vemprala, Sai H. and Saripalli, Srikanth},
  journal={IEEE Access}, 
  title={Collaborative Localization for Micro Aerial Vehicles}, 
  year={2021},
  pages={1-16},
  doi={10.1109/ACCESS.2021.3074537}}

The code is intended as a basic demonstration of the various modules used in this pipeline. 

Libraries/modules used within this project:

1. OpenMVG: https://github.com/openMVG/openMVG  (Base API, detection, matching, structure from motion and localization algorithms)  
2. OpenCV: https://github.com/opencv/opencv  (Image processing and CV related utilities)  
3. KORAL: https://github.com/komrad36/koral  (For GPU based feature description and matching)  
4. dlib: https://github.com/davisking/dlib  (For performing single variable bounded minimization as part of the covariance intersection algorithm)  

All credits for the individual components coming from these projects go to the original author(s).
