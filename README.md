# Averaging Weighted Quaternions
A repository explaining how to find the average quaternion given several weighted quaternions. As I had trouble finding example code that solves this problem, I think that this will be helpful to others. I worked on that problem in a robot related project, therefore I used datatypes like `tf::Quaternion` or `tf::Point` which are taken from the [`tf` package](http://wiki.ros.org/tf "Transform package of ROS") associated with _ROS_ ([Robot Operating System](http://www.ros.org/ "ROSlaunch the Website")). I am pretty sure you can easily replace them with the respective [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page "Eigen and Friends")-versions. Also I am not sure whether I added all necessary includes, so bear with me. Nevertheless, I hope this will help people with the same problem!

Credits go to [this answer on SO](https://stackoverflow.com/a/27410865/4397853 "See Jonathans answer...") which basically sums up steps given in [this paper](http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf "Averaging Quaternions"). Also there is a [python implementation](https://github.com/christophhagen/averaging-quaternions) that is doing the same.

## The Algorithm in short with bad mathematic notation
Given a list of ![equation](https://latex.codecogs.com/gif.latex?n) quaternions ![equation](https://www.codecogs.com/eqnedit.php?latex=\mathcal{W}(A,f)&space;=&space;(T,\bar{f})) with corresponding weights ![equation](https://latex.codecogs.com/gif.latex?w_i) we want to find a single quaternion ![equation](https://latex.codecogs.com/gif.latex?q_{\text{avg}}) that represents their weighted average.

First we construct a matrix ![equation](https://latex.codecogs.com/gif.latex?Q) which contains the weighted quaternions (represented as column vectors) as outlined below:

![equation](https://latex.codecogs.com/gif.latex?Q%20%26%3D%20%5Bq_i%20w_i%20%5C%2C%20%7C%20%5C%2C%20%5Cforall%20i%5D%20%3D%20%5Bq_1w_i%2C%20q_2w_2%2C%20%5Cldots%2C%20q_nw_n%5D%20%3D%20%5Cbegin%7Bbmatrix%7D%20a_1w_1%20%26%20a_2w_2%20%26%20%5Cldots%20%26%20a_nw_n%5C%5C%20b_1w_1%20%26%20b_2w_2%20%26%20%5Cldots%20%26%20b_nw_n%5C%5C%20c_1w_1%20%26%20c_2w_2%20%26%20%5Cldots%20%26%20c_nw_n%5C%5C%20d_1w_1%20%26%20d_2w_2%20%26%20%5Cldots%20%26%20d_nw_n%20%5Cend%7Bbmatrix%7D)

Then we multiply this matrix with its transposed version resulting in a ![equation](https://latex.codecogs.com/gif.latex?(4&space;\times&space;n)&space;\cdot&space;(n&space;\times&space;4)&space;=&space;(4&space;\times&space;4)) Matrix ![equation](https://latex.codecogs.com/gif.latex?Q_2%3DQ%20Q%5ET). Then we have to find the largest Eigenvalue of ![equation](https://latex.codecogs.com/gif.latex?Q_2) and take the corresponding Eigenvector:
![equation](https://latex.codecogs.com/gif.latex?%5Ctext%7Bmax%5C_eigenvector%7D%3D%5C%7B%5Ctext%7Beigenvector%7D%5C%2C%7C%5C%2Cmax%28%5Ctext%7Beigenvalue%7D%29%3B%5Cforall%5C%2C%5Ctext%7B%28eigenvalue%2C%20eigenvector%20%29%20of%20%7DQ_2%5C%7D)

Then our averaged quaternion is simply:

![equation](https://latex.codecogs.com/gif.latex?q_%7B%5Ctext%7Bavg%7D%7D%20%3D%20%5Cfrac%7B%5Ctext%7Bmax%5C_eigenvector%7D%7D%7B%7C%7C%5Ctext%7Bmax%5C_eigenvector%7D%7C%7C%7D)

Usage: Include the averaging_quaternions.hpp, call math_quat::getAverageQuaternion Method.

