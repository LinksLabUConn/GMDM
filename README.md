# GMDM: Generalized Multi-Speed Dubins Motion Model C++ Library

This code implements a novel motion model, called Generalized Multi-Speed Dubins Motion Model (GMDM), which extends the Dubins model by considering multiple speeds. While the Dubins model produces time-optimal paths under a constant-speed constraint, these paths could be suboptimal if this constraint is relaxed to include multiple speeds. This is because a constant speed results in a large minimum turning radius, thus producing paths with longer maneuvers and larger travel times. In contrast, multi-speed relaxation allows for slower speed sharp turns, thus producing more direct paths with shorter maneuvers and smaller travel times. Furthermore, the inability of the Dubins model to reduce speed could result in fast maneuvers near obstacles, thus producing paths with high collision risks. 

In this regard, GMDM provides the motion planners the ability to jointly optimize time and risk by allowing the change of speed along the path. GMDM is built upon the six Dubins path types considering the change of speed on path segments. It is theoretically established that GMDM provides full reachability of the configuration space for any speed selections. Furthermore, it is shown that the Dubins model is a  specific case of GMDM for constant speeds. The solutions of GMDM are analytical and suitable for real-time applications. Published results show that in obstacle-free environments, GMDM produces near time-optimal paths with significantly lower travel times than the Dubins model while having similar computation times. In obstacle-rich environments, GMDM produces time-risk optimized paths with substantially lower collision risks. 

<p align="center">
  <img src="figs/figure1.png" height = "255"/>
</p>
<p align="center">
Fig. Comparison of GMDM with the Dubins paths. Left: horter-time path than Dubins in obstacle-free environments. Right: Safer path than Dubins in obstacle-rich environments.
</p>

## Usage
Code is well-documented and header files should be self-explanatory. An example program is provided that reproduces the results in Figure 6 of the IEEE TRO published article. The GMDM C++ Library has no particular dependencies. A CMakeLists.txt file and some build scripts are provided for convenience. The library was tested and evaluated in Ubuntu 22.04 LTS. A python script matplotlib and pandas dependencies is provided to visualize the GMDM paths.

### Instructions
- Run `./build_debug.sh` to build a debug version, or `./build_release.sh` to build a release version.
- Run `./run.sh` to run the C++ and python programs in sequence.
- Run `./clean.sh` to remove generated files and folders from building and running.

## Citation
If you use the results presented in this paper or the code from this repository, please cite the following:
```
@ARTICLE{10938343,
  author={Wilson, James P. and Gupta, Shalabh and Wettergren, Thomas A.},
  journal={IEEE Transactions on Robotics}, 
  title={Generalized Multispeed Dubins Motion Model}, 
  year={2025},
  volume={41},
  number={},
  pages={2861-2878},
  doi={10.1109/TRO.2025.3554436}}
```
Please kindly star :star: this project if it helps you!

## License
[MIT](LICENSE) © Links Lab UConn

## Contact
James P. Wilson (james.wilson@uconn.edu)
