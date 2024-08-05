<H1> Route Optimization Project instructions on Testing and Set-up </h1>

Utilized the Bridges API to create visualizations of the A-star and the Dijkstra's algorithm that take in user input for a city, and specific starting and ending coordinates to find the most optimal path. Visual and time methods are used to compare the algorithms regarding the user input.

When attempting to run code, Bridges bridges line in the main must be uncommented and userID and APIKey must be replaced with Bridges credentials. Documentation on testing the project is provided in the readme file and the provided links below. A brief instruction would be to create a Bridges API account and download the Bridges file and the source files to the computer. Change the system variables path to include the bin folder in the Bridges 3.3.0 folder. Open up Visual Studio and create a new Console App project in C++.  Drop the PathfinderAlgorithm.cpp and the Algorithms.cpp and .h files into the source files. From there, open Project/Properties, and in active(debug) add the include folder to the additional include directories in tab C/C++. In Linker additional library directories you will also add the path for the lib folder, and finally, in Linker/input you will add the Additional Dependency of libcurl.lib. In the Bridges visualization, both of the algorithm maps will be displayed on the same link (either link works), and will create a joint visual that you are able to play as a video, or alternate between Dijkstra's or A-star with the buttons provided at the top.

Documentation:
https://bridgesuncc.github.io/bridges_setup.html

Video Tutorial:
https://www.youtube.com/watch?v=WR47WGPOK54
