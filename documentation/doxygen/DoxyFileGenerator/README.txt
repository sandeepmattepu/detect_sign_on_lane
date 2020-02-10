1. Doxygen is used to generate html documentation for C++ code. Install Doxygen from http://www.doxygen.nl/manual/install.html
2. Doxygen utilizes Doxyfile to generate the documentation.
3. Unfortunately it is not easy to change the paths of doxygen support files in Doxyfile.
4. A short code is written to solve this which generates a Doxyfile from Doxyfile_Template file. Follow the steps to generate the html code in your system.

i) Open terminal and navigate to bin folder. Then run DoxyFileGenerator binary by providing 4 arguments.
	a) Path to detect_sign_on_lane package.
	b) Path where you want to generate final html documentation.
	c) Path where Doxyfile_Template file is present.
	d) Path where a new Doxyfile will be generated.
	ex:- ./DoxyFileGenerator ~/Documents/ros_workspace/src/detect_sign_on_lane/ ~/Desktop/Html_Docs/ ~/Documents/ros_workspace/src/detect_sign_on_lane/documentation/doxygen/DoxyFileGenerator/Doxyfile_Template ~/Desktop/

ii) Now you will get a new Doxyfile to generate the html documentation.

iii) In the terminal type "doxygen /path/to/generated/Doxyfile". This should generate html documentation.
