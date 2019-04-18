# ellipseDetect
Algorithms for Ellipsoid detection

*author : Konstantinos Lagogiannis 2017
 
I ve been looking for ellipse detection algorithms and I came across an interesting paper for which however I could not find code in C.
 So, here is my, *yet unpolished* implementation of the IEEE paper  "A NEW EFFICIENT ELLIPSE DETECTION METHOD", Yonghong Xie & Qiang Ji  2002.
\note Minor Customization have been added so as to improve detection ellipsoids in low-res images, but also show debug output of points used on an image - using OpenCV and Gcc.

## Summary :
 Algorithm creates list of point pairs, and then checks/scores for candidate ellipse with major axis between a chosen  pair of test points, it
  then estimates minor axis by testing all 3rd points and uses a voting procedure to check for possible minor axis and ellipse
 
*Example usage code  :
```cv::Canny( imgIn_thres, imgEdge_local, gi_CannyThresSmall,gi_CannyThres  ); //Use an/any edge detection algorithm 
 getEdgePoints(imgEdge_local,vedgePoints_all); //Pass edge image, return list of points which we attempt to fit the ellipsoid.
 detectEllipse(vedgePoints_all,qEllipsoids); //Run Ellipsoid fitting Algorithm returns detected ellipsoids
 ```
 
 
## The steps of the algorithm Are :
 1. Store all edge pixels in a one dimensional array.
 2. Clear the accumulator array .
 3. For each pixel (x1, y1 ), carry out the following steps from (4) to (14).
 4. For each other pixel (x2, y2), if the distance between (x1, y1) and (x 2, y2)
 is greater than the required least distance  for  a  pair  of  pixels  to  be  considered  then
 carry out the following steps from (5) to (14).

 5. From  the  pair  of  pixels  (x1,  y1) and  (x2,  y2),  using
 equations   (1)   to   (4)   to   calculate   the   center,
 orientation and length of major axis for the assumed ellipse.

 6. For  each  third  pixel  (x,  y),  if  the  distance  between
 (x,  y)  and  (x0,  y0)   is  ?greater?  than  the  required  least
 distance  for  a  pair  of  pixels  to  be  considered  :
  *"The distance between (x, y) and (x_0 , y_0 ) should be less than the distance between (x_1 , y_1 ) and (x_0 ,y_0 ) or between (x_2 , y_2 ) and (x_0 , y_0 ) ."
 *found in MATlab implementation : ie 3rd point distance <= a; % (otherwise the formulae in paper do not work)
  then carry out the following steps from (7) to (9).
  
 7.  Using  equations  (5)  and  (6)  to  calculate  the  length  of minor axis.
 8.  Increment  the  accumulator  for  this  length  of  minor  axis by 1.
 9.  Loop  until  all  pixels  are  computed  for  this  pair  of  pixels.
 10. Find the maxium element in accumulator array.
 The related  length  is  the  possible  length  of  minor  axis
 for  assumed  ellipse.  If  the  vote  is  greater  than  the
 required   least   number   for   assumed   ellipse,   one  ellipse is detected.
 11.   Output ellipse parameters.
 12.   Remove the pixels on the detected ellipse from edge pixel array.
 13.   Clear accumulator array.
 14.   Loop until all pairs of pixels are computed.
 15.   Superimpose   detected   ellipses   on   the   original  image.
 16.   End.


## Eqns:
1. x 0 = (x 1 + x 2 )/2 
2. y 0 = (y 1 + y 2 )/2 
3. a = [(x 2 – x 1 ) + (y 2 – y 1 ) ] /2
4. α = atan [(y 2 – y 1 )/(x 2 – x 1 )],
5. b2 = (a 2 d 2 sin 2 τ)/( a 2 -d 2 cos 2 τ )
6. cos τ = ( a 2 + d 2 – f 2 )/(2ad) 

----
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
