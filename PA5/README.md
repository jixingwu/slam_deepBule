###第一题
- 取图像的像素值
```bash
m01 += v * image.at<uchar>(y+v, x+u);   //获得单个像素值image.at<uchar>(y,x)
```
- 对于```cv::Mat& image```和```vector<cv::KeyPoint> &keypoints```传入的参数，可以使用下列方式操作
```
cv::Point2f p_r(cos_*ORB_pattern[4*i+0] - sin_*ORB_pattern[4*i+1],
                sin_*ORB_pattern[4*i+0] + cos_*ORB_pattern[4*i+1]);
               cv::Point2f q_r(cos_*ORB_pattern[4*i+2] - sin_*ORB_pattern[4*i+3],
               sin_*ORB_pattern[4*i+2] + cos_*ORB_pattern[4*i+3]);   
cv::Point2f p(kp.pt+p_r); //获取p‘和q’的真实坐标才能获得像素值
cv::Point2f q(kp.pt+q_r);
```
