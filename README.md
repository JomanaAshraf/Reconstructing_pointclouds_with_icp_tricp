# Reconstructing_pointclouds_with_icp_tricp
This is an implementation for reconstructing pointclouds using **Iterative Closest Point** and **Trimmed Iterative Closest Point** algorithms.

The algorithms used in the `/src/icp_tricp.cpp` file are:
* Finding Nearest Neighbour by using kd-tree nanoflann.
* Estimating the rotation by using SVD.
* Iterative Closest Point Algorithm.
* Trimmed Iterative Closest Point Algorithm.

In order to run the program, this command should be used:

```
./icp_tricp (path_of_pc1) (path_of_pc2) (rotation_angle) (translation) (max_itererations) (output_filename)
```
The previous command includes the path to the first and second point clouds, the required rotational angle in degree which will be applied to the **z-axis** of `pointcloud 2` also, the translation is the distance between the two point clouds which is applied in the **x-axis** of the second point cloud.Consequently, the max iterations are defined and the name of the output file.

## Output From the Program
The program will show 3 windows:
1. The two point clouds in one window where the **pc1** will be in **white** and **pc2** will be in **cyan**.
2. The ICP window which will show the two pointclouds after applying the icp algorithm.
3. The TRICP window which will show the two pointclouds after applying the tricp algorithm.
4. Two text files
    * output_file_icp.txt
    *  output_file_tricp.txt
    
### The output file will contain:

* The number of iterations taken
* Processing time in seconds
* Mean square error
* Rotation error
* Translation error

## Evaluation

The Evaluation of the icp and tricp algorithms are implemented on three different dataset which can be accessed from `/dataset` that contains **horse.pcd**, **wolf.pcd**, and **lion.pcd**. Different rotations and translations are applied to each pair to compare between the two algorithms according to the processing time, MSE, number of iterations, rotation error, and translation error.

### Wolf Point Cloud
___

| P.O.C | Ground Truth | ICP | Tricp |
|:-----:|:-----------: | :------:|:----:|
| Rot: 0<br />trans:20m|![wolf_0_20](https://user-images.githubusercontent.com/74091020/216834280-e520c9a9-e74b-4978-a116-049b560b490f.png)|![icp_wolf_0_20](https://user-images.githubusercontent.com/74091020/216834344-89e3c017-fa8f-499b-a905-2c7301866a39.png)|![tricp_wolf_0_20](https://user-images.githubusercontent.com/74091020/216834534-a5f1bd02-3d96-4266-b4db-d4a5ddcf8ef9.png)|
| Rot: 5<br />trans:50m|![wolf_5_50](https://user-images.githubusercontent.com/74091020/216834764-4583c0cd-cdf2-4206-85af-2e80eaa0f5f0.png)|![icp_wolf_5_50](https://user-images.githubusercontent.com/74091020/216834914-79698e1a-75d3-4cee-9499-3615e68fba22.png)|![tricp_wolf_5_50](https://user-images.githubusercontent.com/74091020/216834974-d0b090e0-119a-4df5-b849-d31860f80a47.png)|
| Rot: 10<br />trans:100m|![wolf_10_100](https://user-images.githubusercontent.com/74091020/216835272-b6616ed5-0c3a-470c-acf1-b68ff4d1d30e.png)|![icp_10_100](https://user-images.githubusercontent.com/74091020/216835304-a5e62b4d-fa71-4efa-b718-74c351ee7324.png)|![tricp_wolf_10_100](https://user-images.githubusercontent.com/74091020/216835319-38d9a6ed-48e9-450c-9120-2c79ec9abd4e.png)|
| Rot: 20<br />trans:50m|![wolf_20_50](https://user-images.githubusercontent.com/74091020/216835399-82d535f3-8112-4bc8-9c0f-d79ff2b5a8cf.png)|![icp_wolf_20_50](https://user-images.githubusercontent.com/74091020/216835448-bc928d18-414a-4158-ad4f-201f10e2af5e.png)|![tricp_wolf_20_50](https://user-images.githubusercontent.com/74091020/216835463-3db8ba3d-6bbe-49bb-8fd0-0378463cfc02.png)|

The following table will present the evaluation between the different parameters for both algorithms:
| P.O.C | No_of_Iterations| Processing Time | MSE | Rotation Error | Translation Error |
|:-----:| :--------------:| :--------------:|:---:|:--------------:|:-----------------:|
| **Icp**<br/>Rot: 0<br />trans:20m|100|3.01373 s|5195.12|12.4004|11.1202|
| **Tricp**<br/>Rot: 0<br />trans:20m|26|0.462693 s|12.4109|5.48527|4.39881|
| **Icp**<br/>Rot: 5<br />trans:50m|78|2.39915 s|5195.11|12.3278|11.0334|
| **Tricp**<br/>Rot: 5<br />trans:50m|22|0.382936 s|12.4104|5.48191|3.78654|
| **Icp**<br/>Rot: 10<br />trans:100m|99|3.06214 s|5195.11|13.2591|10.9824|
| **Tricp**<br/>Rot: 10<br />trans:100m|29|0.54177 s|12.4103|5.71628|4.08619|

The number of point clouds used in this example is 2040 for each point cloud and as shown from the above tables that the tricp is more effective in aligning the two point clouds together with 60% matching factor also the mean square error is smaller in tricp than in icp despite that the alignment of the icp is pretty good also.

### Horse Point Cloud
___
| P.O.C | Ground Truth | ICP | Tricp |
|:-----:|:-----------: | :------:|:----:|
| Rot: 0<br />trans:20m|![horse_0_20](https://user-images.githubusercontent.com/74091020/216796479-aaf7949b-7645-4562-bcae-36749f42115e.png)|![icp_horse_0_20](https://user-images.githubusercontent.com/74091020/216796481-8c26c711-8894-466d-a153-d2c3d6a90c9b.png)|![tricp_horse_0_20](https://user-images.githubusercontent.com/74091020/216796482-5464b515-15c8-4c4c-9241-35a61da8db95.png)|
| Rot: 5<br />trans:200m|![horse_5_200](https://user-images.githubusercontent.com/74091020/216797158-a431b8dd-2b4b-44a4-ae12-1ffeeb09ff5e.png)|![icp_horse_5_200](https://user-images.githubusercontent.com/74091020/216797173-4a1d2b40-57f2-43e2-b729-658fe6a47561.png)|![tricp_horse_5_200](https://user-images.githubusercontent.com/74091020/216797176-db73bb65-f531-4458-967d-25e54df2079c.png)|
| Rot: 10<br />trans:50m|![horse_10_50](https://user-images.githubusercontent.com/74091020/216796837-a182187e-4277-4c25-8fd4-22a50ae0ab26.png)|![icp_horse_10_50](https://user-images.githubusercontent.com/74091020/216796841-42ad8c5a-a399-424a-b1e4-2c8f6a46536d.png)|![tricp_horse_10_50](https://user-images.githubusercontent.com/74091020/216796843-5e2ebb69-6f51-4468-b0ba-2f7858650388.png)|
| Rot: 20<br />trans:100m|![horse_20_100](https://user-images.githubusercontent.com/74091020/216796991-1ebd73c8-5537-41ca-97a3-d6a91b13873a.png)|![icp_horse_20_100](https://user-images.githubusercontent.com/74091020/216797012-05a0c892-9116-4d72-9027-25ea0c167830.png)|![tricp_horse_20_100](https://user-images.githubusercontent.com/74091020/216797018-c841fb9f-8453-477d-9c38-f1326193ebcd.png)|

The following table will present the evaluation between the different parameters for both algorithms:
| P.O.C | No_of_Iterations| Processing Time | MSE | Rotation Error | Translation Error |
|:-----:| :--------------:| :--------------:|:---:|:--------------:|:-----------------:|
| **Icp**<br/>Rot: 0<br />trans:20m|100|2.1347 s|20784.8|10.0217|37.3933|
| **Tricp**<br/>Rot: 0<br />trans:20m|37|0.489893 s|11.5845|9.80781|29.6088|
| **Icp**<br/>Rot: 5<br />trans:200m|100|2.10667 s|20785.6|9.87661|41.5358|
| **Tricp**<br/>Rot: 5<br />trans:200m|38|0.493496 s|11.5845|9.88219|39.1379|
| **Icp**<br/>Rot: 10<br />trans:50m|100|2.09044 s|20785.7|9.69424|37.8298|
| **Tricp**<br/>Rot: 10<br />trans:50m|36|0.468165 s|11.5941|9.79966|29.5348|
| **Icp**<br/>Rot: 20<br />trans:100m|100|2.12897 s|20785.8|8.54808|38.5195|
| **Tricp**<br/>Rot: 20<br />trans:100m|65|0.84948 s|11.5703|9.11486|28.368|

The number of points in the horse point cloud are 2040 where as shown in the previous table that the **icp** took more time than the **tricp** also, the mean square error of the icp is very high but the recostruction of the point cloud is good. On the other hand, the mean square error of tricp for **60%** matching factor is very low. Also in each case, the rotation and translation error for the tricp is smaller than icp.

### Lion Point Cloud
___

| P.O.C | Ground Truth | ICP | Tricp |
|:-----:|:-----------: | :------:|:----:|
| Rot: 0<br />trans:20m|![lion_0_20](https://user-images.githubusercontent.com/74091020/216841453-686fc103-2b2c-4c95-b09e-254c1fafc947.png)|![icp_lion_0_20](https://user-images.githubusercontent.com/74091020/216841486-3c222172-ac42-410a-91a7-5e1d58fb3860.png)|![tricp_lion_0_20](https://user-images.githubusercontent.com/74091020/216841498-e0e9d283-4c16-4b38-beee-fd63adb34eb1.png)|
| Rot: 5<br />trans:50m|![lion_5_50](https://user-images.githubusercontent.com/74091020/216841515-d83dd3b1-8b20-4e31-99aa-887c5b1dfef7.png)|![icp_lion_5_50](https://user-images.githubusercontent.com/74091020/216841521-798cea62-760e-4102-b0fc-252cf28e8b47.png)|![tricp_lion_5_50](https://user-images.githubusercontent.com/74091020/216841524-b64fe4ec-f54d-42dd-8dc3-f8af7fb40967.png)|
| Rot: 10<br />trans:50m|![lion_10_50](https://user-images.githubusercontent.com/74091020/216841656-748ad7bf-a902-4992-b314-de6fc54942ed.png)|![icp_lion_10_50](https://user-images.githubusercontent.com/74091020/216841647-c17a7bd4-066e-42ad-b1bc-f619e451863e.png)|![tricp_lion_10_50](https://user-images.githubusercontent.com/74091020/216841674-f6a26b9f-9f62-4228-b5b7-d4ada9375301.png)|
| Rot: 20<br />trans:0m|![lion_20_0](https://user-images.githubusercontent.com/74091020/216842300-5ba96d43-398f-4f17-a7c5-aa8678ab57d5.png)|![icp_lion_20_0](https://user-images.githubusercontent.com/74091020/216842298-962ea574-7653-481c-9239-09bf92ae2eee.png)|![tricp_lion_20_0](https://user-images.githubusercontent.com/74091020/216842301-db75ae34-0e62-4fec-a545-136bc3a611b7.png)|

The following table will present the evaluation between the different parameters for both algorithms:
| P.O.C | No_of_Iterations| Processing Time | MSE | Rotation Error | Translation Error |
|:-----:| :--------------:| :--------------:|:---:|:--------------:|:-----------------:|
| **Icp**<br/>Rot: 0<br />trans:20m|47|30.7702 s|47.5876|45.2883|12.6881|
| **Tricp**<br/>Rot: 0<br />trans:20m|100|58.4768 s| 4.6819|68.3162|10.8677|
| **Icp**<br/>Rot: 5<br />trans:50m|67|50.4527 s|60.402|84.5864|38.9136|
| **Tricp**<br/>Rot: 5<br />trans:50m|100|65.484 s|7.26803|34.025|25.0846|
| **Icp**<br/>Rot: 10<br />trans:50m|71|53.5863 s|60.4014|84.7578|36.3768|
| **Tricp**<br/>Rot: 10<br />trans:50m|100|68.1844 s|10.6484|35.7319|28.6678|
| **Icp**<br/>Rot: 20<br />trans:0m|60|38.7912 s|47.5876|43.9368|11.3095|
| **Tricp**<br/>Rot: 20<br />trans:0m|88|43.5056 s|4.55607|11.7506|3.88247|

This time a test implemented on two different sizes of point clouds; the first one has 62145 point clouds and the second one has 37787. Due to the large number of points, the algorithms took too much time in order to give the result. But as expected that the mean square error in icp is much more greater than the tricp. But as shown from the pictures that when the translation is small the tricp would have the ability to reconstruct a perfect point cloud despite the rotation but when large translation used; the algorithm was not capable to give a good alignment for the point clouds.
