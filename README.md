<H1> Route Optimization Project instructions on Testing and Set-up </h1>

Utilized the Bridges API to create visualizations of the A-star and the Dijkstra's algorithm that take in user input for a city, and specific starting and ending coordinates to find the most optimal path. Visual and time methods are used to compare the algorithms regarding the user input. In the Bridges visualization, both of the algorithm maps will be displayed on the same link (either link works), and will create a joint visual that you are able to play as a video, or alternate between Dijkstra's or A-star with the buttons provided at the top.

When attempting to run code, Bridges bridges line in the main must be uncommented and userID and APIKey must be replaced with Bridges credentials. Documentation on testing the project is provided in the readme file and the provided links below.

<H2>Windows/Visual Studio Set-up</H2>
Create a Bridges API account and download the Bridges folder and the source files to the computer. Change the system variables path to include the bin folder in the Bridges 3.3.0 folder. Open up Visual Studio and create a new Console App project in C++.  Drop the PathfinderAlgorithm.cpp and the Algorithms.cpp and .h files into the source files. From there, open Project/Properties, and in active(debug) add the include folder to the additional include directories in tab C/C++. In Linker additional library directories you will also add the path for the lib folder, and finally, in Linker/input you will add the Additional Dependency of libcurl.lib.

<H2>macOS/Xcode Set-up</H2>

Start off by installing the CURL library to your computer from the link provided. Then install Xcode onto mac from Apple developer site, then install Xcode command line tools from the command line in Terminal using xcode-select --install. Create a Bridges API account and download the Mac OS X Bridges folder and the source files to the computer. Open Xcode, then select Create new project, when prompted for a template select command Line Tool under OS X/Application. Once in your project, transfer the source files into Xcode, and click on Project/Project Settings/Build Settings. Here you will select All and Levels, then scroll down until you find Header Search Paths/Debug, in which you will add a path to the bridges/include file, and the curl750/include file. After this, search for Library Search Paths, and you will add the path for curl750/lib file. After this, scroll to the top of settings and click on Build Phases, and select Link Binary With Libraries. Select Add Other, and add the library liburl.dylib to the project.

Documentation:
https://bridgesuncc.github.io/bridges_setup.html

CURL installation:
https://curl.se/download.html

Xcode Set-up Tutorial:
https://bridgesuncc.github.io/bridges_setup_cxx_xcode.html

Visual Studio Set-up Video Tutorial:
https://www.youtube.com/watch?v=WR47WGPOK54
