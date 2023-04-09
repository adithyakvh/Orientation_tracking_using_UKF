# Orientation_tracking_using_UKF

This project was completed as a requirement for the course ESE650: Learning in Robotics at the University of Pennsylvania. The primary objective of this project was to implement a Quaternion-based orientation tracking algorithm using an Unscented Kalman filter, as described in the research paper "Unscented-Kalman filter". The Quaternion representation of orientation provides more efficient computation compared to other methods.

However, due to the non-linear relationship between the estimated orientation and the measurements, traditional Kalman filters are unsuitable for this task. Therefore, we utilized the Unscented Kalman filter, which allows for the implementation of non-linear process and measurement models. The Unscented Kalman filter is more accurate than the Extended Kalman filter and provides a more robust orientation tracking solution.

# Results

![1_1](https://user-images.githubusercontent.com/42107613/230778374-38230acc-8223-4858-8513-0f8336fd3c5c.jpg)


![1](https://user-images.githubusercontent.com/42107613/230778061-5ec8a2ee-5ef1-422e-beb7-456a93a77649.JPG)
![2](https://user-images.githubusercontent.com/42107613/230778065-bcb7920a-f2c6-458a-b0a3-8975e3621322.JPG)
![4](https://user-images.githubusercontent.com/42107613/230778070-3345afb2-0e3b-4884-946f-2c6db928efca.JPG)
![5](https://user-images.githubusercontent.com/42107613/230778076-b873ded7-a718-4fd4-9b1d-7eaa5dd57665.JPG)
![6](https://user-images.githubusercontent.com/42107613/230778082-fe17566e-bfd4-4080-931f-0218b9ea26f6.JPG)
