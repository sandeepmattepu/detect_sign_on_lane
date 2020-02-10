#include <iostream>
#include <string>
#include <fstream>

int main(int argc, char** argv)
{
	if(argc < 5)
	{
		std::cout << "Usage " << argv[0] << " [Full path to the detect_sign_on_lane package] [Full path where html to generate]" 
		<< " [Full path to Doxyfile_Template] [Full path where final DoxyFile will be generated]" << std::endl;
		return 1;
	}
	std::string packageDirectory = argv[1];
	if(packageDirectory.at(packageDirectory.length()-1) == '/')
	{
		packageDirectory.pop_back();
	}
	std::string htmlOutputDirectory = argv[2];
	std::string doxyFileDirectory = argv[3];
	std::string resultDoxyFileDirectory = argv[4];
	resultDoxyFileDirectory += "Doxyfile";
	std::ifstream rfile;
	std::ofstream ofile;
	std::string line;
	rfile.open(doxyFileDirectory);
	ofile.open(resultDoxyFileDirectory);
	if(rfile.is_open())
	{
		while(std::getline(rfile,line))
		{
			std::string htmlOutputKey = "${OUTPUT_FULL_PATH}";
			std::size_t found = line.rfind(htmlOutputKey);
			if(found != std::string::npos)
			{
				line.replace(found, htmlOutputKey.length(), htmlOutputDirectory);
			}

			std::string pathToPackageKey = "${FULL_PATH_TO_PACKAGE}";
			found = line.rfind(pathToPackageKey);
			while(found != std::string::npos)
			{
				line.replace(found, pathToPackageKey.length(), packageDirectory);
				found = line.rfind(pathToPackageKey);
			}
			ofile << line << std::endl;
		}
		rfile.close();
	}
	ofile.close();
	return 0;
}