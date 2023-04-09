# Orientation_tracking_using_UKF

This project was completed as a requirement for the course ESE650: Learning in Robotics at the University of Pennsylvania. The primary objective of this project was to implement a Quaternion-based orientation tracking algorithm using an Unscented Kalman filter, as described in the research paper "Unscented-Kalman filter". The Quaternion representation of orientation provides more efficient computation compared to other methods.

However, due to the non-linear relationship between the estimated orientation and the measurements, traditional Kalman filters are unsuitable for this task. Therefore, we utilized the Unscented Kalman filter, which allows for the implementation of non-linear process and measurement models. The Unscented Kalman filter is more accurate than the Extended Kalman filter and provides a more robust orientation tracking solution.

# Results


![1_1](https://user-images.githubusercontent.com/42107613/230778873-75bf8a14-2006-4ae0-9641-a467b236dbb8.jpg)
![2_2](https://user-images.githubusercontent.com/42107613/230778874-0145b190-8999-4bc2-951a-a22e907fb23a.jpg)
![4_4](https://user-images.githubusercontent.com/42107613/230778875-b7cbcd1b-573e-413d-92db-8116cb318671.jpg)
![5_5](https://user-images.githubusercontent.com/42107613/230778876-cf96abc0-0e63-4b3c-903f-19ee958fc936.jpg)
![6_6](https://user-images.githubusercontent.com/42107613/230778877-13cc2b8c-ad6d-442f-a76f-600ea31cc837.jpg)
