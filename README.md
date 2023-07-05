# Dobot_project
## 최종 테스트 영상
<img src="./data/11174b88-e936-4fd0-8ded-8efd62ea2e1e.gif" alt="이미지" width="640" height="320"> <br>

## Rule
[Rule](https://github.com/wjdgus2235/Dobot_project/blob/master/data/Rule.pdf)

## 최종 code 
[code](https://github.com/wjdgus2235/Dobot_project/blob/master/src/dobot.py)

### camera calibration
```shell
$ python3 3_calibration.py --dir calibration_checkerboard/ --width 8 --height 6 --square_size 0.0295
```
#### (calibration_matrix.npy, distortion_coefficients.npy) are created

### aruco pose_estimation
```shell
$ python3 4_pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
```
