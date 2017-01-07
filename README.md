# lane-detection
Lane and vehicle detection software for an autonomous vehicle

<p>This project is still under development and was built for educational purposes. An application was designed following the model-view-controller architecture to enable multiple autonomous vehicle algorithms to be simulated in different views and to allow th input parameters to be altered in run time e.g. adaptive threshold parameters, coordinates for inverse perspective mapping, number of sample points etc. The code will run slower due to the MVC architecture.</p>

<p>A model-view-controller architecture was utilised so that the project can eventually form an application where different input parameters can be altered e.g. adaptive threshold parameters, coordinates for inverse perspective mapping, number of sample points etc. It will also enable different autonomous vehicle algorithms to be implemented.</p>

<p><b>main.cpp</b> is the view, <b>laneDetectorController.hpp</b> is the controller for the lane detection and <b>vehicleDetectorController.hpp</b> is the controller for the vehicle detection (both controllers are derived from the base <b>Controller</b> class in <b>controller.hpp</b>).</p>

<h2>Lane Detection</h2>

<h3>Files/Classes</h3>
<p><b>laneDetector.hpp</b>  acts as the model for the lane detection and contains the <b>LaneDetector</b> class.</p>

<p>The lane detection utilises the <b>LineFinder</b> class within <b>lineFinder.hpp</b> which is capable of finding and drawing lines using the Hough Transform and the Probabilistic Hough Transfrom.</p>

<p><b>IPM.hpp</b> contains the <b>IPM</b> class for inverse perspective mapping, based on that seen in Marco Nieto's Blog (https://marcosnietoblog.wordpress.com/2014/02/22/source-code-inverse-perspective-mapping-c-opencv/). The <b>IPM</b> class is capable of converting the vehicle's front view to a birds eye view.</p>

<p><b>laneTracker.hpp</b> contains the <b>LaneTracker</b> class which tracks and predicts the lane markers using a Kalman filter.</p>

<h3>Algorithm</h3>
<p>The lane detection algorithm utilises image enhancement techniques including temporal blurring and inverse perspective mapping before splitting the image into two halves for further processing. Instances of the <b>LaneDetector</b> class are then created for each image in order to detect the lane markers. An adaptive threshold is then used to detect the edges within each image. The <b>LineFinder</b> class is then used to detect lines using both the Hough Transform and Probabilistic Hough Transform. Any lines which do not resemble lane markers are removed (incorrect orientation etc). A bitwise AND operation of the two results is then used to combine the results for optimal line selection. The Hough Transform function within the <b>LineFinder</b> class is then used to detect the 10 most probable lines within the resulting image.</p>

<p>The image is then sampled along it’s height and linear least squares regression is used to calculate the best fit line for the lane marker. The lines rho (distance from the coordinate origin) and theta (the line rotation angle in radians) are then calculated.</p>

<p>Two instances of the <b>LaneTracker</b> class are then created in order to track the lane marker in each image. The line parameters (rho, theta) are used as inputs to the Kalman Filter. If no best fit line is detected then the Kalman Filter predicts the parameters.</p>

<h2>Vehicle Detection</h2> 
<h3>Files/Classes</h3>
<p><b>vehicleDetector.cpp</b> acts as the model for the vehicle detection and contains the <b>VehicleDetector</b> class.</p>

<h3>Algorithm</h3>
The vehicle detection algorithm utilises a Haar Cascade which is trained using an MIT vehicle dataset. The input frame is converted to grayscale before undergoing histogram equalisation. Objects of different sizes are then detected using the Haar Cascade and stored in a list of rectangles. The detected cars are then marked on the output image.  

